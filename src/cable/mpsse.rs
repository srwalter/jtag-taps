//! Implement the `Cable` trait for "jtagkey" compatible hardware adapters like the Bus Blaster
use crate::cable::Cable;

use std::time::Duration;

use libftd2xx::{Ft2232h, Ftdi, FtdiMpsse, MpsseCmdBuilder, MpsseCmdExecutor, FtdiCommon};
use ftdi_mpsse::{ClockTMSOut, ClockTMS};
use libftd2xx::{ClockData, ClockDataOut, ClockBits, ClockBitsOut};

const MAX_BUFFER_SIZE: usize = 4096;

pub struct Mpsse<T> {
    ft: T,
    // Data to send to the adapter
    buffer: Vec<u8>,
    // Data we have read from the adapter and not yet returned
    queued_reads: Vec<u8>,
    // (bits, bytes, write, pause)
    queued_read_state: Vec<(usize, usize, bool, bool)>,
}

impl<T: FtdiMpsse + MpsseCmdExecutor> Mpsse<T>
    where <T as MpsseCmdExecutor>::Error: std::fmt::Debug
{
    pub fn new(mut ft: T, clock: u32) -> Self
    {
        ft.initialize_mpsse_default().expect("init");
        ft.set_clock(clock).expect("set clock");

        let builder = MpsseCmdBuilder::new()
            .disable_3phase_data_clocking()
            .disable_adaptive_data_clocking();
        ft.send(builder.as_slice()).expect("send");

        Self {
            ft,
            buffer: vec![],
            queued_reads: vec![],
            queued_read_state: vec![],
        }
    }
}

impl<T: FtdiMpsse + MpsseCmdExecutor> Cable for Mpsse<T>
    where <T as MpsseCmdExecutor>::Error: std::fmt::Debug
{
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        let mut count = 0;
        let mut buf = 0;
        let mut builder = MpsseCmdBuilder::new();

        for x in tms {
            if *x != 0 {
                buf |= 1 << count;
            }
            count += 1;

            if count == 7 {
                builder = builder.clock_tms_out(ClockTMSOut::NegEdge, buf, tdo, count);
                count = 0;
                buf = 0;
            }
        }
        builder = builder.clock_tms_out(ClockTMSOut::NegEdge, buf, tdo, count);
        let len = builder.as_slice().len();
        if len + self.buffer.len() > MAX_BUFFER_SIZE {
            self.flush();
        }
        self.buffer.append(&mut builder.as_slice().to_vec());
    }

    fn queue_read(&mut self, mut bits: usize) -> bool
    {
        let orig_bits = bits;

        let mut bytes = bits / 8;
        let mut builder = MpsseCmdBuilder::new();
        if bytes > 0 {
            bits -= bytes * 8;
            builder = builder.clock_data(ClockData::LsbPosIn, &vec![0xff; bytes]);
        }

        if bits > 0 {
            builder = builder.clock_bits(ClockBits::LsbPosIn, 0xff, bits as u8);
            bytes += 1;
        }

        let len = builder.as_slice().len();
        if len + self.buffer.len() > MAX_BUFFER_SIZE {
            self.flush();
        }

        let total_bytes = bytes + self.queued_read_state.iter()
            .map(|x| (x.0 + 7) / 8)
            .sum::<usize>();

        if total_bytes < MAX_BUFFER_SIZE {
            self.queued_read_state.push((orig_bits, bytes, false, false));
            self.buffer.append(&mut builder.as_slice().to_vec());
            true
        } else {
            false
        }
    }

    fn finish_read(&mut self, mut bits: usize) -> Vec<u8>
    {
        let (orig_bits, bytes, pause_after, write) = self.queued_read_state.remove(0);
        assert_eq!(bits, orig_bits);

        if self.queued_reads.is_empty() {
            // Read all of the pending bytes
            let total_bytes = bytes + self.queued_read_state.iter()
                .map(|x| x.1)
                .sum::<usize>();
            self.queued_reads.resize(total_bytes, 0);
            self.ft.xfer(&self.buffer, &mut self.queued_reads).expect("send");
            self.buffer.clear();
        }

        let mut buf = self.queued_reads.split_off(bytes);
        // split_off returns the second half of the vec, but we want the first half
        std::mem::swap(&mut buf, &mut self.queued_reads);

        if pause_after {
            buf.pop();
        }

        if write {
            let len = buf.len();
            buf[len-1] >>= 7;

            bits -= 1;
            if bits >= 1 {
                // Shift the bits from clock_bits
                buf[len-2] >>= 8 - (bits % 8);

                // Need to repack the bit from clock_tms into the bits from clock_bits
                let last_recv = buf[len-1] & 1;
                buf[len-2] |=  last_recv << (bits % 8);
                buf.pop();
            }
        } else {
            if bits % 8 != 0 {
                let last_idx = buf.len()-1;
                buf[last_idx] >>= 8 - (bits % 8);
            }
        }
        buf
    }

    fn read_data(&mut self, bits: usize) -> Vec<u8>
    {
        assert!(self.queued_read_state.is_empty());
        self.queue_read(bits);
        self.finish_read(bits)
    }

    fn write_data(&mut self, data: &[u8], mut bits: u8, pause_after: bool)
    {
        let mut builder = MpsseCmdBuilder::new();
        assert!(bits <= 8);
        assert!(bits != 0);

        // We will send the last bit using clock_tms
        bits -= 1;

        if data.len() > 1 {
            builder = builder.clock_data_out(ClockDataOut::LsbNeg, &data[..data.len()-1]);
        }
        let last_byte = data[data.len()-1];
        if bits >= 1 {
            builder = builder.clock_bits_out(ClockBitsOut::LsbNeg, last_byte, bits);
        }
        let last_bit = last_byte & (1 << bits) != 0;
        // Change to pause state
        if pause_after {
            builder = builder.clock_tms_out(ClockTMSOut::NegEdge, 1, last_bit, 2);
        } else {
            builder = builder.clock_tms_out(ClockTMSOut::NegEdge, 0, last_bit, 1);
        }

        let len = builder.as_slice().len();
        if len + self.buffer.len() > MAX_BUFFER_SIZE {
            self.flush();
        }
        self.buffer.append(&mut builder.as_slice().to_vec());
    }

    fn queue_read_write(&mut self, data: &[u8], mut bits: u8, pause_after: bool) -> bool {
        let total_bits = (data.len()-1) * 8 + bits as usize;
        let mut read_bytes = 1;
        let mut builder = MpsseCmdBuilder::new();

        assert!(bits <= 8);
        assert!(bits != 0);

        // We will send the last bit using clock_tms
        bits -= 1;

        if data.len() > 1 {
            builder = builder.clock_data(ClockData::LsbPosIn, &data[..data.len()-1]);
            read_bytes += data.len()-1;
        }
        let last_byte = data[data.len()-1];
        if bits >= 1 {
            builder = builder.clock_bits(ClockBits::LsbPosIn, last_byte, bits);
            read_bytes += 1;
        }
        let last_bit = last_byte & (1 << bits) != 0;

        // Change to pause state
        if pause_after {
            builder = builder.clock_tms(ClockTMS::NegTMSPosTDO, 1, last_bit, 1);
            read_bytes += 1;
        }
        builder = builder.clock_tms(ClockTMS::NegTMSPosTDO, 0, last_bit, 1);

        let len = builder.as_slice().len();
        if len + self.buffer.len() > MAX_BUFFER_SIZE {
            self.flush();
        }

        let total_bytes = read_bytes + self.queued_read_state.iter()
            .map(|x| x.1)
            .sum::<usize>();

        if total_bytes < MAX_BUFFER_SIZE {
            self.queued_read_state.push((total_bits, read_bytes, true, pause_after));
            self.buffer.append(&mut builder.as_slice().to_vec());
            true
        } else {
            false
        }
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        assert!(self.queued_read_state.is_empty());
        self.queue_read_write(data, bits, pause_after);
        let total_bits = (data.len()-1) * 8 + bits as usize;
        self.finish_read(total_bits)
    }

    fn flush(&mut self) {
        self.ft.send(&self.buffer).expect("flush");
        self.buffer.clear();
    }
}

// Lower pins
const PIN_TCK: u8 = 1;
const PIN_TDI: u8 = 1 << 1;
//const PIN_TDO: u8 = 1 << 2;
const PIN_TMS: u8 = 1 << 3;
const PIN_N_OE: u8 = 1 << 4;
const LOWER_OUTPUT_PINS: u8 = PIN_TCK | PIN_TDI | PIN_TMS | PIN_N_OE;

// Upper pins
const PIN_N_TRST: u8 = 1;
const PIN_N_SRST: u8 = 1 << 1;
const PIN_N_TRST_OE: u8 = 1 << 2;
const PIN_N_SRST_OE: u8 = 1 << 3;
const UPPER_OUTPUT_PINS: u8 = PIN_N_TRST | PIN_N_SRST | PIN_N_TRST_OE | PIN_N_SRST_OE;

pub struct JtagKey {
    ft: Mpsse<Ft2232h>,
}

impl JtagKey {
    /// Create a new JtagKey.  FT2232-based adapters like JtagKey have both an "A" interface and a
    /// "B" interface.  `primary` controls which to use. `clock` controls the speed of TCLK in hertz.
    pub fn new(clock: u32, primary: bool) -> Self {
        let description = if primary {
            "Dual RS232-HS A"
        } else {
            "Dual RS232-HS B"
        };
        let ft = Ftdi::with_description(description).expect("new");
        let ft = Ft2232h::try_from(ft).expect("try");
        let mut ft = Mpsse::new(ft, clock);
        ft.ft.set_latency_timer(Duration::from_millis(0)).expect("latency");
        ft.ft.set_gpio_upper(PIN_N_TRST | PIN_N_SRST, UPPER_OUTPUT_PINS).expect("pins");

        let builder = MpsseCmdBuilder::new()
            .set_gpio_lower(PIN_TMS, LOWER_OUTPUT_PINS);
        ft.ft.send(builder.as_slice()).expect("send");

        JtagKey {
            ft,
        }
    }

    pub fn new_olimex(clock: u32) -> Self {
        libftd2xx::set_vid_pid(0x15ba, 0x2a).expect("vid");
        let description = "Olimex OpenOCD JTAG ARM-USB-TINY-H A";
        let ft = Ftdi::with_description(description).expect("new");
        let ft = Ft2232h::try_from(ft).expect("try");
        let mut ft = Mpsse::new(ft, clock);
        ft.ft.set_latency_timer(Duration::from_millis(0)).expect("latency");
        ft.ft.set_gpio_upper(PIN_N_TRST | PIN_N_SRST, PIN_N_TRST).expect("pins");

        let builder = MpsseCmdBuilder::new()
            .set_gpio_lower(PIN_TMS, LOWER_OUTPUT_PINS);
        ft.ft.send(builder.as_slice()).expect("send");

        JtagKey {
            ft,
        }
    }

    /// JtagKey adapters implement the option SRST signal.  This function puts the system in reset.
    pub fn assert_srst(&mut self) {
        self.ft.ft.set_gpio_upper(PIN_N_TRST, UPPER_OUTPUT_PINS).expect("pins");
    }

    /// JtagKey adapters implement the option SRST signal.  This function takes the system out of
    /// reset.
    pub fn dessert_srst(&mut self) {
        self.ft.ft.set_gpio_upper(PIN_N_TRST | PIN_N_SRST, UPPER_OUTPUT_PINS).expect("pins");
    }
}

impl Cable for JtagKey {
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        self.ft.change_mode(tms, tdo)
    }

    fn read_data(&mut self, bits: usize) -> Vec<u8> {
        self.ft.read_data(bits)
    }

    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) {
        self.ft.write_data(data, bits, pause_after)
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        self.ft.read_write_data(data, bits, pause_after)
    }

    fn queue_read_write(&mut self, data: &[u8], bits: u8, pause_after: bool) -> bool {
        self.ft.queue_read_write(data, bits, pause_after)
    }

    fn flush(&mut self) {
        self.ft.flush();
    }

    fn queue_read(&mut self, bits: usize) -> bool {
        self.ft.queue_read(bits)
    }

    fn finish_read(&mut self, bits: usize) -> Vec<u8> {
        self.ft.finish_read(bits)
    }
}

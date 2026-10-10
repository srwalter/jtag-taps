//! Implement the `Cable` trait for FTDI RS232R-based adapters
use crate::cable::Cable;

use libftd2xx::{Ftdi, FtdiCommon, BitMode};

pub struct Ft232r {
    ft: Ftdi,
    tdi: u8,
    tdo: u8,
    tms: u8,
    clk: u8,
    read_queue: Vec<Vec<u8>>,
}

impl Ft232r {
    /// Create a new Ft232r.  `description` is the value passed to `Ftd::with_description` to
    /// select which hardware to use.
    pub fn easyflash3(baud: u32) -> Self {
        #[cfg(unix)]
        libftd2xx::set_vid_pid(0x0403, 0x8738).expect("vid");
        Ft232r::new("EasyFlash 3", baud, 3, 0, 1, 2)
    }

    /// Create a new Ft232r.  `description` is the value passed to `Ftd::with_description` to
    /// select which hardware to use.
    pub fn new(description: &str, baud: u32, tdi: u8, tdo: u8, tms: u8, clk: u8) -> Self {
        let mut ft = Ftdi::with_description(description).expect("new");
        ft.set_baud_rate(baud).expect("set_baud_rate");
        ft.set_bit_mode(1 << tdo | 1 << tms | 1 << clk, BitMode::SyncBitbang).expect("set bit mode");
        ft.purge_all().expect("purge");

        Self {
            ft,
            tdi,
            tdo,
            tms,
            clk,
            read_queue: vec![],
        }
    }

    fn select_bit(mut recv: Vec<u8>, tdi: u8) -> Vec<u8> {
        let mut recv_bits = vec![];
        let mut byte = 0_u8;
        let mut bit = 0;

        // convert bytes to bits
        loop {
            // Second byte sampled when CLK was high
            recv.remove(0);
            // First byte sampled when CLK was low
            let x = recv.remove(0);

            if x & (1 << tdi) != 0 {
                byte |= 1 << bit;
            }
            bit += 1;

            if bit == 8 {
                recv_bits.push(byte);
                byte = 0;
                bit = 0;
            }

            if recv.is_empty() {
                break;
            }
        }

        if bit > 0 {
            recv_bits.push(byte);
        }

        recv_bits
    }
}

impl Cable for Ft232r {
    fn change_mode(&mut self, tms: &[usize], tdo: bool) {
        let mut buf = vec![];
        let tdo = if tdo {
            1
        } else {
            0
        };

        for x in tms {
            let x = if *x != 0 {
                1
            } else {
                0
            };
            buf.push(x << self.tms | tdo << self.tdo);
            buf.push(x << self.tms | tdo << self.tdo | 1 << self.clk);
        }
        let mut recv = vec![0; buf.len()];
        self.ft.write(&buf).expect("send");
        self.ft.read(&mut recv).expect("send");
    }

    fn read_data(&mut self, bits: usize) -> Vec<u8>
    {
        let mut buf = vec![];
        for _ in 0..bits {
            buf.push(1 << self.tdo);
            buf.push(1 << self.tdo | 1 << self.clk);
        }

        let mut recv = vec![0; buf.len()];
        self.ft.write(&buf).expect("send");
        self.ft.read(&mut recv).expect("send");
        Self::select_bit(recv, self.tdi)
    }

    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool)
    {
        self.read_write_data(data, bits, pause_after);
    }

    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        let mut buf = vec![];

        assert!(bits <= 8);
        assert!(bits != 0);

        for x in &data[0..data.len()-1] {
            for bit in 0..8 {
                let tdo = (x >> bit) & 1;
                buf.push(tdo << self.tdo);
                buf.push(tdo << self.tdo | 1 << self.clk);
            }
        }

        // handle last byte
        let x = data[data.len()-1];
        for bit in 0..bits {
            let tdo = (x >> bit) & 1;
            if bit == bits-1 && pause_after {
                buf.push(1 << self.tms | tdo << self.tdo);
                buf.push(1 << self.tms | tdo << self.tdo | 1 << self.clk);
            }
            buf.push(tdo << self.tdo);
            buf.push(tdo << self.tdo | 1 << self.clk);
        }

        let mut recv = vec![0; buf.len()];
        self.ft.write(&buf).expect("send");
        self.ft.read(&mut recv).expect("send");
        Self::select_bit(recv, self.tdi)
    }

    fn queue_read(&mut self, bits: usize) -> bool {
        let data = self.read_data(bits);
        self.read_queue.push(data);
        true
    }

    fn queue_read_write(&mut self, data: &[u8], bits: u8, pause_after: bool) -> bool {
        let result = self.read_write_data(data, bits, pause_after);
        self.read_queue.push(result);
        true
    }

    fn finish_read(&mut self, _bits: usize) -> Vec<u8> {
        self.read_queue.remove(0)
    }
}

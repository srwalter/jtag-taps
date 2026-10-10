//! Implement the `Cable` trait for Altera USB Blaster and clones
use crate::cable::Cable;

use libftd2xx::{Ftdi, FtdiCommon};

pub struct UsbBlaster {
    ft: Ftdi,
    tdi: u8,
    tdo: u8,
    tms: u8,
    clk: u8,
    read_queue: Vec<Vec<u8>>,
}

const READ_CMD: u8 = 1 << 6;

impl Default for UsbBlaster {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbBlaster {
    /// Create a new UsbBlaster.
    pub fn new() -> Self {
        #[cfg(unix)]
        libftd2xx::set_vid_pid(0x16c0, 0x06ad).expect("vid");
        let mut ft = Ftdi::with_description("USB-JTAG-IF").expect("new");
        ft.purge_all().expect("purge");

        Self {
            ft,
            tdi: 0,
            tdo: 4,
            tms: 1,
            clk: 0,
            read_queue: vec![],
        }
    }

    fn select_bit(mut recv: Vec<u8>, tdi: u8) -> Vec<u8> {
        let mut recv_bits = vec![];
        let mut byte = 0_u8;
        let mut bit = 0;

        // convert bytes to bits
        loop {
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

impl Cable for UsbBlaster {
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
        self.ft.write(&buf).expect("send");
    }

    fn read_data(&mut self, mut bits: usize) -> Vec<u8>
    {
        let bytes = (bits + 7) / 8;
        let buf = vec![0xff; bytes];

        bits %= 8;
        if bits == 0 {
            bits = 8;
        }
        self.read_write_data(&buf, bits as u8, false)
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
                buf.push(READ_CMD | tdo << self.tdo | 1 << self.clk);
            }
        }

        // handle last byte
        let x = data[data.len()-1];
        for bit in 0..bits {
            let tdo = (x >> bit) & 1;
            if bit == bits-1 && pause_after {
                buf.push(1 << self.tms | tdo << self.tdo);
                buf.push(READ_CMD | 1 << self.tms | tdo << self.tdo | 1 << self.clk);
            }
            buf.push(tdo << self.tdo);
            buf.push(READ_CMD | tdo << self.tdo | 1 << self.clk);
        }

        let mut recv = vec![0; buf.len()/2];
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

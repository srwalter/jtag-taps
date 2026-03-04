//! Implementations for different JTAG hardware adapters will here.  Hardware adapters should
//! implement the `Cable` trait.
pub mod mpsse;
pub mod ft232r;
pub mod usbblaster;
pub mod jlink;

pub trait Cable {
    /// Clock out a series of TMS values to change the state of the JTAG chain.  Each element of
    /// `tms` determines the value of the TMS line, zero for low and any other value for high.
    /// `tdo` controls the state of the TDI line during mode changes.
    fn change_mode(&mut self, tms: &[usize], tdo: bool);
    /// Shift in bits from the TDO line.  `bits` is the total number of bits to read.  Should be
    /// called with state = ShiftIR or ShiftDR, and will remain in that state.  Should clock out
    /// all ones.
    fn read_data(&mut self, bits: usize) -> Vec<u8>;
    /// Shift out bits on the TDI line.  `bits` is the number of bits to send from the last byte.
    /// Should be called with state = ShiftIR or ShiftDR.  State won't change unless `pause_after`
    /// is true, in which case it will be PauseIR or PauseDR on exit.
    fn write_data(&mut self, data: &[u8], bits: u8, pause_after: bool);

    /// Shift out bits on the TDI line.  `bits` is the number of bits to send from the last byte.
    /// Should be called with state = ShiftIR or ShiftDR.  State won't change unless `pause_after`
    /// is true, in which case it will be PauseIR or PauseDR on exit.  Also captures and returns
    /// the bits that were shifted in from TDO
    fn read_write_data(&mut self, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8>;

    /// If the cable implements any queueing, flush to hardware.
    fn flush(&mut self) {
    }

    /// Request that data be read without immediately returning the data.  This allows for multiple
    /// read requests to be queued, which can allow for better performance.  Returns false if the
    /// adapter doesn't have any more queue space.
    fn queue_read(&mut self, bits: usize) -> bool;

    /// Shift out bits on the TDI line.  `bits` is the number of bits to send from the last byte.
    /// Should be called with state = ShiftIR or ShiftDR.  State won't change unless `pause_after`
    /// is true, in which case it will be PauseIR or PauseDR on exit.  Also captures
    /// the bits that were shifted in from TDO, which can be retrieved with a queue to
    /// `finish_read()`.  Returns false if the adapter doesn't have any more queue space.
    fn queue_read_write(&mut self, data: &[u8], bits: u8, pause_after: bool) -> bool;

    /// Return the data from a previously queued read.  `bits` must exactly match the corresponding
    /// call to `queue_read()`, otherwise the behavior is undefined.  Once you finish a read, you
    /// must finish all the queued reads by calling `finish_read()` as many times as `queue_read()`
    /// was called.
    fn finish_read(&mut self, bits: usize) -> Vec<u8>;
}

/// Helper function for constructing a cable from a string.  This is expected to be used by CLI
/// utilities where the cable is passed in as an argument, rather than constructed by code.
pub fn new_from_string(name: &str, clock: u32) -> Result<Box<dyn Cable>,String> {
    match name {
        "jtagkey" => Ok(Box::new(mpsse::JtagKey::new(clock, true))),
        "olimex-tiny-h" => Ok(Box::new(mpsse::JtagKey::new_olimex(clock))),
        "ef3" => Ok(Box::new(ft232r::Ft232r::easyflash3(clock))),
        "usbblaster" => Ok(Box::new(usbblaster::UsbBlaster::new())),
        "jlink" => Ok(Box::new(jlink::JLink::new(clock))),
        _ => Err(format!("unknown cable type: {}", name)),
    }
}

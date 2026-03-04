use jtag_taps::cable;
use jtag_taps::statemachine::JtagSM;
use jtag_taps::taps::Taps;

fn main() {
    let cable = cable::new_from_string("olimex-tiny-h", 1 << 20).expect("cable");
    let jtag = JtagSM::new(cable);
    let mut taps = Taps::new(jtag);
    taps.detect();

    let ir = vec![235, 0];
    taps.select_tap(0, &ir);
    let readback = taps.read_ir();
    print!("ir: ");
    for x in readback {
        print!("{:x} ", x);
    }
    println!();

    taps.write_ir(&ir);
    let buf = vec![
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    ];
    taps.write_dr(&buf, 8);
    let dr = taps.read_dr(256);
    print!("dr: ");
    for x in dr {
        print!("{:x} ", x);
    }
    println!();
}

//! This provides a higher-level interface than the `Cable` trait.  Specifically, it keeps track of
//! the state of the JTAG state machine, and allows setting the state to any desired state.
//! `JtagSM` will get to that state by the most efficient path, based on the current state.
use crate::cable::Cable;

#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Register {
    Data,
    Instruction
}

#[derive(Clone,Copy,Debug,PartialEq)]
pub enum JtagState {
    Reset = 0,
    Idle = 1,
    SelectDR = 2,
    CaptureDR = 3,
    ShiftDR = 4,
    Exit1DR = 5,
    PauseDR = 6,
    Exit2DR = 7,
    UpdateDR = 8,
    SelectIR = 9,
    CaptureIR = 10,
    ShiftIR = 11,
    Exit1IR = 12,
    PauseIR = 13,
    Exit2IR = 14,
    UpdateIR = 15,
}

struct Node {
    edges: Vec<usize>,
}

impl Node {
    fn new() -> Self {
        Self {
            edges: Vec::new(),
        }
    }
}

#[derive(Clone)]
struct Path {
    path: Vec<usize>,
    state: usize,
}

impl Path {
    fn new(state: usize) -> Self {
        Self {
            state,
            path: Vec::new()
        }
    }
}

pub struct JtagSM<T> {
    pub cable: T,
    state: JtagState,
    graph: Vec<Node>,
}

impl<T, U> JtagSM<T>
    where T: std::ops::DerefMut<Target=U>,
          U: Cable + ?Sized
{
    /// Create a JTAG state machine using an existing `Cable`
    pub fn new(mut cable: T) -> Self {
        let mut reset = Node::new();
        let mut idle = Node::new();
        let mut selectdr = Node::new();
        let mut capturedr = Node::new();
        let mut shiftdr = Node::new();
        let mut exit1dr = Node::new();
        let mut pausedr = Node::new();
        let mut exit2dr = Node::new();
        let mut updatedr = Node::new();
        let mut selectir = Node::new();
        let mut captureir = Node::new();
        let mut shiftir = Node::new();
        let mut exit1ir = Node::new();
        let mut pauseir = Node::new();
        let mut exit2ir = Node::new();
        let mut updateir = Node::new();

        reset.edges     = vec![JtagState::Idle as usize, JtagState::Reset as usize];
        idle.edges      = vec![JtagState::Idle as usize, JtagState::SelectDR as usize];
        selectdr.edges  = vec![JtagState::CaptureDR as usize,
                               JtagState::SelectIR as usize];
        capturedr.edges = vec![JtagState::ShiftDR as usize, JtagState::Exit1DR as usize];
        shiftdr.edges   = vec![JtagState::ShiftDR as usize, JtagState::Exit1DR as usize];
        exit1dr.edges   = vec![JtagState::PauseDR as usize, JtagState::UpdateDR as usize];
        pausedr.edges   = vec![JtagState::PauseDR as usize, JtagState::Exit2DR as usize];
        exit2dr.edges   = vec![JtagState::ShiftDR as usize, JtagState::UpdateDR as usize];
        updatedr.edges  = vec![JtagState::Idle as usize, JtagState::SelectDR as usize];

        selectir.edges  = vec![JtagState::CaptureIR as usize,
                               JtagState::Reset as usize];
        captureir.edges = vec![JtagState::ShiftIR as usize, JtagState::Exit1IR as usize];
        shiftir.edges   = vec![JtagState::ShiftIR as usize, JtagState::Exit1IR as usize];
        exit1ir.edges   = vec![JtagState::PauseIR as usize, JtagState::UpdateIR as usize];
        pauseir.edges   = vec![JtagState::PauseIR as usize, JtagState::Exit2IR as usize];
        exit2ir.edges   = vec![JtagState::ShiftIR as usize, JtagState::UpdateIR as usize];
        updateir.edges  = vec![JtagState::Idle as usize, JtagState::SelectDR as usize];

        let graph = vec![reset, idle,
            selectdr, capturedr, shiftdr, exit1dr, pausedr, exit2dr, updatedr,
            selectir, captureir, shiftir, exit1ir, pauseir, exit2ir, updateir,
        ];

        cable.change_mode(&[1, 1, 1, 1, 1, 0], true);

        Self {
            cable,
            state: JtagState::Reset,
            graph,
        }
    }

    /// Reset the scan chain by driving TMS high for 5 clocks
    pub fn mode_reset(&mut self)
    {
        self.cable.change_mode(&[1, 1, 1, 1, 1, 0], true);
        self.state = JtagState::Reset;
    }

    fn get_path(&mut self, state: JtagState) -> Vec<usize> {
        if self.state == state {
            return Vec::new();
        }

        let mut paths = Vec::new();

        let mut p = Path::new(self.graph[self.state as usize].edges[0]);
        p.path = vec![0];
        if p.state == state as usize {
            return p.path;
        }
        paths.push(p);

        let mut p = Path::new(self.graph[self.state as usize].edges[1]);
        p.path = vec![1];
        if p.state == state as usize {
            return p.path;
        }
        paths.push(p);

        loop {
            let mut newpaths = Vec::new();

            for p in paths {
                let mut p1 = p.clone();
                p1.state = self.graph[p.state].edges[0];
                p1.path.push(0);

                if p1.state == state as usize {
                    return p1.path
                }
                newpaths.push(p1);

                let mut p2 = p.clone();
                p2.state = self.graph[p.state].edges[1];
                p2.path.push(1);

                if p2.state == state as usize {
                    return p2.path
                }
                newpaths.push(p2);
            }

            paths = newpaths;
        }
    }

    /// Use TMS to get into `state` by the most efficient path
    pub fn change_mode(&mut self, state: JtagState) {
        if self.state == state {
            return;
        }

        let path = self.get_path(state);
        //println!("Path from {:?} to {:?}: {:?}", self.state, state, path);
        self.cable.change_mode(&path, true);
        self.state = state;
    }

    /// Read `bits` from either the instruction or data register
    pub fn read_reg(&mut self, reg: Register, bits: usize) -> Vec<u8> {
        if reg == Register::Data {
            self.change_mode(JtagState::ShiftDR);
        } else {
            self.change_mode(JtagState::ShiftIR);
        }
        self.cable.read_data(bits)
    }

    /// Read `bits` from either the instruction or data register
    pub fn queue_read(&mut self, reg: Register, bits: usize) -> bool {
        if reg == Register::Data {
            self.change_mode(JtagState::ShiftDR);
        } else {
            self.change_mode(JtagState::ShiftIR);
        }
        self.cable.queue_read(bits)
    }

    /// Write `data` into either the instruction or data register.  `bits` indicates how many bits
    /// of the last byte should be written (8 indicates that the entire byte should be written).
    /// The mode will either be ShiftIR / ShiftDR if `pause_after` is false, or PauseIR / PauseDR
    /// if `pause_after` is true.  This allows for setting the register with multiple calls to
    /// `write_reg`, which may be more convenient than manual bit-shifting.
    pub fn write_reg(&mut self, reg: Register, data: &[u8], bits: u8, pause_after: bool) {
        if reg == Register::Data {
            self.change_mode(JtagState::ShiftDR);
        } else {
            self.change_mode(JtagState::ShiftIR);
        }
        self.cable.write_data(data, bits, pause_after);
        if pause_after {
            if reg == Register::Data {
                self.state = JtagState::PauseDR;
            } else {
                self.state = JtagState::PauseIR;
            }
        }
    }

    /// Write `data` into either the instruction or data register.  `bits` indicates how many bits
    /// of the last byte should be written (8 indicates that the entire byte should be written).
    /// The mode will either be ShiftIR / ShiftDR if `pause_after` is false, or PauseIR / PauseDR
    /// if `pause_after` is true.  This allows for setting the register with multiple calls to
    /// `read_write_reg`, which may be more convenient than manual bit-shifting.
    ///
    /// Similar to `write_reg` except it returns the bits that were shifted out during writing.
    pub fn read_write_reg(&mut self, reg: Register, data: &[u8], bits: u8, pause_after: bool) -> Vec<u8> {
        if reg == Register::Data {
            self.change_mode(JtagState::ShiftDR);
        } else {
            self.change_mode(JtagState::ShiftIR);
        }
        let data = self.cable.read_write_data(data, bits, pause_after);
        if pause_after {
            if reg == Register::Data {
                self.state = JtagState::PauseDR;
            } else {
                self.state = JtagState::PauseIR;
            }
        }
        data
    }

    pub fn queue_read_write(&mut self, reg: Register, data: &[u8], bits: u8, pause_after: bool) -> bool {
        if reg == Register::Data {
            self.change_mode(JtagState::ShiftDR);
        } else {
            self.change_mode(JtagState::ShiftIR);
        }
        let data = self.cable.queue_read_write(data, bits, pause_after);
        if pause_after {
            if reg == Register::Data {
                self.state = JtagState::PauseDR;
            } else {
                self.state = JtagState::PauseIR;
            }
        }
        data
    }
}

use substrate::Io;
use substrate::io::{InOut, Signal};

pub mod vco;

#[derive(Debug, Default, Copy, Clone, Io)]
pub struct PowerIo {
    pub vdd: InOut<Signal>,
    pub vss: InOut<Signal>,
}
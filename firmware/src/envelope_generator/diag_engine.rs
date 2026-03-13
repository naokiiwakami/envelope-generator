use super::config::EgConfig;
use super::definitions::VoiceParams;

use crate::input_reader::{InputReaderInfo, PotKind};

pub struct DiagEgEngine {
    ascending: bool,
    current: u16,
    pub out_buf: u16,
}

impl DiagEgEngine {
    pub fn new() -> Self {
        Self {
            ascending: false,
            current: 0,
            out_buf: 0,
        }
    }

    pub fn update_params(
        &mut self,
        _voice_index: usize,
        _config: &EgConfig,
        _input: &InputReaderInfo,
    ) {
    }

    pub fn gate_on(&mut self, _params: &VoiceParams) {}

    pub fn gate_off(&mut self) {}

    /// Generates triangular wave
    pub fn update(&mut self, _params: &VoiceParams) -> u16 {
        if self.ascending {
            if self.current + 256 >= 0xfff {
                self.ascending = false;
                self.current -= 256;
            } else {
                self.current += 256;
            }
        } else {
            if self.current == 0 {
                self.ascending = true;
                self.current += 256;
            } else {
                self.current -= 256;
            }
        };
        self.out_buf = self.current;
        self.out_buf
    }
}

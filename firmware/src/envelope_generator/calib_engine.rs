use crate::envelope_generator::OutputPolarity;
use crate::input_reader::InputReaderInfo;

use super::config::EgConfig;
use super::definitions::{Engine, VoiceParams};

pub struct CalibEngine {
    specified_value: u16,
    polarity: OutputPolarity,
    is_gate_on: bool,
    count: usize,
}

impl Engine for CalibEngine {
    fn new() -> Self {
        Self {
            specified_value: 0,
            polarity: OutputPolarity::Positive,
            is_gate_on: false,
            count: 0,
        }
    }

    fn initialize(&mut self, _voice_index: usize, _config: &EgConfig) {}

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, _input: &InputReaderInfo) {
        self.polarity = config.out_polarity(voice_index);
    }

    fn gate_on(&mut self, _params: &VoiceParams) {
        self.is_gate_on = true;
    }

    fn gate_off(&mut self) {
        self.is_gate_on = false;
    }

    /// Generates triangular wave
    fn update(&mut self, params: &VoiceParams) -> u16 {
        let out = params.output_level + params.out_zero_point;
        if self.count % 16384 == 0 {
            defmt::debug!("out={}", out);
        }
        self.count += 1;
        out
    }
}

use crate::envelope_generator::OutputPolarity;
use crate::input_reader::InputReaderInfo;

use super::config::EgConfig;
use super::definitions::{Engine, VoiceParams};

pub struct CalibEngine {
    specified_value: u16,
    polarity: OutputPolarity,
}

impl Engine for CalibEngine {
    fn new() -> Self {
        Self {
            specified_value: 0,
            polarity: OutputPolarity::Positive,
        }
    }

    fn initialize(&mut self, _voice_index: usize, _config: &EgConfig) {}

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, _input: &InputReaderInfo) {
        self.polarity = config.out_polarity(voice_index);
        self.specified_value = config.note_scaling_depth(voice_index);
    }

    fn gate_on(&mut self, _params: &VoiceParams) {}

    fn gate_off(&mut self) {}

    /// Generates triangular wave
    fn update(&mut self, params: &VoiceParams) -> u16 {
        match self.polarity {
            OutputPolarity::Positive => self.specified_value + params.out_zero_point,
            OutputPolarity::Negative => params.out_zero_point - self.specified_value,
        }
    }
}

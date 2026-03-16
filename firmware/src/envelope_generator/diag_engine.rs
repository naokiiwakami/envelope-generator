use super::config::EgConfig;
use super::definitions::{Engine, VoiceParams};

use crate::input_reader::PotKind;

pub struct DiagEngine {
    ascending: bool,
    current: u16,
    pub out_buf: u16,
}

impl Engine for DiagEngine {
    fn new() -> Self {
        Self {
            ascending: false,
            current: 0,
            out_buf: 0,
        }
    }

    fn initialize(&mut self, voice_index: usize, config: &EgConfig) {
        self.update_params(voice_index, config, &PotKind::Attack);
        self.update_params(voice_index, config, &PotKind::Decay);
        self.update_params(voice_index, config, &PotKind::Sustain);
        self.update_params(voice_index, config, &PotKind::Release);
        self.update_params(voice_index, config, &PotKind::Extra1);
        self.update_params(voice_index, config, &PotKind::Extra2);
    }

    fn update_params(&mut self, _voice_index: usize, _config: &EgConfig, _updated_pot: &PotKind) {}

    fn gate_on(&mut self, _params: &VoiceParams) {}

    fn gate_off(&mut self) {}

    /// Generates triangular wave
    fn update(&mut self, _params: &VoiceParams) -> u16 {
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

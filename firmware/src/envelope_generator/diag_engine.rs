use crate::{definitions::PotKind, input_reader::InputReaderInfo};

use super::config::EgConfig;
use super::definitions::{Engine, VoiceParams};

pub struct DiagEngine {
    ascending: bool,
    current: u32,
    delta: u32,
}

impl Engine for DiagEngine {
    fn new() -> Self {
        Self {
            ascending: true,
            current: 0,
            delta: 1,
        }
    }

    fn initialize(&mut self, _voice_index: usize, _config: &EgConfig) {}

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo) {
        match input.pot_info.kind {
            PotKind::Attack => {
                if voice_index == 0 {
                    self.delta = (config.attack(voice_index) >> 12) as u32 + 1;
                }
            }
            PotKind::Release => {
                if voice_index == 1 {
                    self.delta = (config.release(voice_index) >> 5) as u32 + 1;
                }
            }
            _ => {} // TODO interpret CV1_DEPTH and CV2_DEPTH
        }
    }

    fn gate_on(&mut self, _params: &VoiceParams) {}

    fn gate_off(&mut self) {}

    /// Generates triangular wave
    fn update(&mut self, _params: &VoiceParams) -> u16 {
        if self.ascending {
            if self.current + self.delta >= 0xffff {
                self.ascending = false;
                self.current = 0xffff;
            } else {
                self.current += self.delta;
            }
        } else {
            if self.current <= self.delta {
                self.ascending = true;
                self.current = 0;
            } else {
                self.current -= self.delta;
            }
        };
        (self.current >> 4) as u16
    }
}

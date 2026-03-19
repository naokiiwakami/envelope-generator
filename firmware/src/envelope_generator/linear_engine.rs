/// Default EG voice engine
use defmt;
use fixed::types::I32F32;

use crate::input_reader::{InputReaderInfo, PotKind};

use super::config::EgConfig;
use super::definitions::{Engine, VoiceParams};

#[derive(Debug, defmt::Format)]
enum EnginePhase {
    Initial,
    Attack,
    Decay,
    Sustain,
    Release,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct LinearEngine {
    // Parameters translated by the EG configuration.
    attack_ratio: I32F32,
    decay_ratio: I32F32,
    sustain_level: I32F32,
    release_ratio: I32F32,

    // State
    current_value: I32F32,
    target_value: I32F32,
    start_value: I32F32,
    level: I32F32,

    phase: EnginePhase,
}

impl LinearEngine {}

impl Engine for LinearEngine {
    fn new() -> Self {
        Self {
            attack_ratio: I32F32::from_num(0),
            decay_ratio: I32F32::from_num(0),
            sustain_level: I32F32::from_num(1),
            release_ratio: I32F32::from_num(0),

            current_value: I32F32::from_num(0),
            target_value: I32F32::from_num(0),
            start_value: I32F32::from_num(0),
            level: I32F32::from_num(0),

            phase: EnginePhase::Initial,
        }
    }

    fn initialize(&mut self, voice_index: usize, config: &EgConfig) {
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Attack));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Decay));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Sustain));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Release));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Extra1));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Extra2));
        self.current_value = I32F32::from_num(0i32);
        self.phase = EnginePhase::Initial;
    }

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo) {
        match input.pot_info.kind {
            PotKind::Attack => {
                let t = I32F32::from_num(config.attack[voice_index] as u32);
                let tc = I32F32::from_num(2) + I32F32::from_num(8.52e-9) * t * t * t;
                self.attack_ratio = I32F32::from_num(1) / tc;
            }
            PotKind::Decay => {
                let t = I32F32::from_num(config.decay[voice_index] as u32);
                let tc = I32F32::from_num(2) + I32F32::from_num(1.7e-8) * t * t * t;
                self.decay_ratio = I32F32::from_num(1) / tc;
            }
            PotKind::Sustain => {
                let s = config.sustain[voice_index] as u64;
                let bits = ((s >> 1) + 32768) * s;
                self.sustain_level = I32F32::from_bits(bits as i64);
            }
            PotKind::Release => {
                let t = I32F32::from_num(config.release[voice_index] as u32);
                let tc = I32F32::from_num(2) + I32F32::from_num(1.7e-8) * t * t * t;
                self.release_ratio = I32F32::from_num(1) / tc;
            }
            _ => {}
        }
    }

    /// Handles a gate-on event.
    /// The method forces changing the phase to Attack regardless the current phase.
    fn gate_on(&mut self, params: &VoiceParams) {
        let velocity = params.velocity as u64;
        let level_bits = ((velocity * velocity) * 31 + 0xffffffff) >> 6;

        self.level = I32F32::from_bits(level_bits as i64);
        self.target_value = self.level;
        self.start_value = I32F32::from_num(0);

        self.phase = EnginePhase::Attack;
    }

    /// Handles a gate-off event.
    fn gate_off(&mut self) {
        self.target_value = I32F32::from_num(0);
        self.phase = EnginePhase::Release;
    }

    /// Updates the current envelope generator value and returns it in 12 bit range.
    fn update(&mut self, _params: &VoiceParams) -> u16 {
        match self.phase {
            EnginePhase::Initial => {}
            EnginePhase::Attack => {
                let delta = self.attack_ratio * self.level;
                self.current_value += delta;

                if self.current_value >= self.target_value {
                    self.current_value = self.target_value;
                    self.phase = EnginePhase::Decay;
                }
            }
            EnginePhase::Decay => {
                self.target_value = self.level * self.sustain_level;
                let delta = self.decay_ratio * self.level;
                let diff = self.current_value - self.target_value;
                if self.current_value >= self.target_value {
                    if diff > delta {
                        self.current_value -= delta;
                    } else {
                        self.current_value = self.target_value;
                        self.phase = EnginePhase::Sustain;
                    }
                } else {
                    if diff > delta {
                        self.current_value += delta;
                    } else {
                        self.current_value = self.target_value;
                        self.phase = EnginePhase::Sustain;
                    }
                }
            }
            EnginePhase::Sustain => {
                self.current_value = self.level * self.sustain_level;
            }
            EnginePhase::Release => {
                let delta = self.release_ratio * self.level;

                if self.current_value > delta {
                    self.current_value -= delta;
                } else {
                    self.current_value = I32F32::from_num(0);
                    self.phase = EnginePhase::Initial;
                }
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        let bits = self.current_value.to_bits();

        // clamp negative
        let bits = bits & !(bits >> 63);

        (bits as u64 >> 19) as u16
    }
}

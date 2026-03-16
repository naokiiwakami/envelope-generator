/// Default EG voice engine
use defmt;

use super::config::EgConfig;
use super::definitions::VoiceParams;

use crate::envelope_generator::EngineType;
use crate::input_reader::PotKind;

#[derive(Debug, defmt::Format)]
enum LinearEnginePhase {
    Initial,
    Attack,
    Decay,
    Sustain,
    Release,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct LinearEgEngine {
    // Values are UQ32.32 fixed point integers.
    // This equal integer and fractional assignment is convenient for EG curve
    // calculation as most of values are in range of [0:1) that fit within
    // 32-bit LSB part. Multiplying these vlaues never overflows in UQ32.32 format.

    // Parameters translated by the EG configuration.
    attack_ratio: u64,
    decay_ratio: u64,
    sustain_level: u64,
    release_ratio: u64,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ32.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: u64,
    target_value: u64,
    start_value: u64,
    level: u64,

    phase: LinearEnginePhase,

    // The 31-bit (0..0x7fffffff) current value is scaled down to this output buffer
    // of range 0..0xfff for each value update so that the parent EgVoice picks up the
    // value and put it in the buffer for DAC.
    pub out_buf: u16,
}

impl LinearEgEngine {
    pub fn new() -> Self {
        Self {
            attack_ratio: 0,
            decay_ratio: 0,
            sustain_level: 0xffffffff,
            release_ratio: 0,

            current_value: 0,
            target_value: 0,
            start_value: 0,
            level: 0,
            phase: LinearEnginePhase::Initial,

            out_buf: 0,
        }
    }

    pub fn initialize(&mut self, _engine_type: &EngineType, voice_index: usize, config: &EgConfig) {
        self.update_params(voice_index, config, &PotKind::Attack);
        self.update_params(voice_index, config, &PotKind::Decay);
        self.update_params(voice_index, config, &PotKind::Sustain);
        self.update_params(voice_index, config, &PotKind::Release);
        self.update_params(voice_index, config, &PotKind::Extra1);
        self.update_params(voice_index, config, &PotKind::Extra2);
        self.current_value = 0;
        self.phase = LinearEnginePhase::Initial;
    }

    pub fn update_params(&mut self, voice_index: usize, config: &EgConfig, updated_pot: &PotKind) {
        match updated_pot {
            PotKind::Attack => {
                let time_param = config.attack[voice_index] as f64;
                let attack_time_constant: f64 =
                    2.0 + 8.52e-9 * time_param * time_param * time_param;
                self.attack_ratio = 0xffffffff / attack_time_constant as u64;
            }
            PotKind::Decay => {
                let time_param = config.decay[voice_index] as f64;
                let decay_time_constant = 2.0 + 1.7e-8 * time_param * time_param * time_param;
                self.decay_ratio = 0xffffffff / decay_time_constant as u64;
            }
            PotKind::Sustain => {
                let sustain_level = config.sustain[voice_index] as u64;
                self.sustain_level = ((sustain_level >> 1) + 32768) * sustain_level;
            }
            PotKind::Release => {
                let time_param = config.release[voice_index] as f64;
                let release_time_constant = 2.0 + 1.7e-8 * time_param * time_param * time_param;
                self.release_ratio = 0xffffffff / release_time_constant as u64;
            }
            PotKind::Extra1 => {}
            PotKind::Extra2 => {}
            _ => {} // TODO interpret CV1_DEPTH and CV2_DEPTH
        }
    }

    /// Handles a gate-on event.
    /// The method forces changing the phase to Attack regardless the current phase.
    pub fn gate_on(&mut self, params: &VoiceParams) {
        // Add an offset of 1/32 level to avoid silence with low velocity
        let velocity = params.velocity as u64;
        self.level = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
        self.target_value = self.level;
        self.start_value = 0;
        self.phase = LinearEnginePhase::Attack;
    }

    /// Handles a gate-off event.
    pub fn gate_off(&mut self) {
        self.target_value = 0;
        self.phase = LinearEnginePhase::Release;
    }

    /// Updates the current envelope generator value and returns it in 12 bit range.
    pub fn update(&mut self, _params: &VoiceParams) -> u16 {
        match self.phase {
            LinearEnginePhase::Initial => {}
            LinearEnginePhase::Attack => {
                let delta = (self.attack_ratio * self.level) >> 32;
                self.current_value += delta;
                if self.current_value >= self.target_value {
                    self.current_value = self.target_value;
                    self.phase = LinearEnginePhase::Decay;
                }
            }
            LinearEnginePhase::Decay => {
                self.target_value = (self.level * self.sustain_level) >> 32;
                let delta = (self.decay_ratio * self.level) >> 32;
                if self.current_value >= self.target_value {
                    if self.current_value - self.target_value > delta {
                        self.current_value -= delta;
                    } else {
                        self.current_value = self.target_value;
                        self.phase = LinearEnginePhase::Sustain;
                    }
                } else {
                    if self.target_value - self.current_value > delta {
                        self.current_value += delta;
                    } else {
                        self.current_value = self.target_value;
                        self.phase = LinearEnginePhase::Sustain;
                    }
                }
            }
            LinearEnginePhase::Sustain => {
                self.current_value = (self.level * self.sustain_level) >> 32;
            }
            LinearEnginePhase::Release => {
                let delta = (self.release_ratio * self.level) >> 32;
                if self.current_value > delta {
                    self.current_value -= delta;
                } else {
                    self.current_value = 0;
                    self.phase = LinearEnginePhase::Initial;
                }
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        self.out_buf = (self.current_value >> 19) as u16;

        self.out_buf
    }
}

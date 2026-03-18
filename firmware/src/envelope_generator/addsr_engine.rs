/// Default EG voice engine
use defmt;
use fixed::types::U32F32;

use crate::input_reader::{InputReaderInfo, PotKind};

use super::config::EgConfig;
use super::definitions::VoiceParams;
use super::{EngineType, definitions::Engine};

#[derive(Debug, defmt::Format)]
enum EnginePhase {
    Released,
    Attack,
    InitialDecay,
    Decay,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct AddsrEngine {
    engine_type: EngineType,

    // Values are UQ32.32 fixed point integers.
    // This equal integer and fractional assignment is convenient for EG curve
    // calculation as most of values are in range of [0:1) that fit within
    // 32-bit LSB part. Multiplying these vlaues never overflows in UQ32.32 format.

    // Parameters translated by the EG configuration.
    attack_ratio: u64,
    decay_ratio: u64,
    sustain_level: u64,
    release_ratio: u64,
    initial_decay_ratio: u64,
    decay_switch_level: u64,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ32.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: u64,
    // The engine simulates RC charging/discharging for this target value.
    // Different transient ratio (attack_ratio, decay_ratio, or release_ratio) is used
    // according to the current phase.
    target_value: u64,
    // The peak value is used for switching phases between attack and decay. When the
    // current value reaches the peak value during attack phase, the engine swithces its
    // phase to decay.
    peak_value: u64,

    phase: EnginePhase,
}

impl AddsrEngine {}

impl Engine for AddsrEngine {
    fn new() -> Self {
        Self {
            engine_type: EngineType::ADDSR,

            attack_ratio: 0,
            decay_ratio: 0,
            sustain_level: 0xffffffff,
            release_ratio: 0,
            initial_decay_ratio: 0,
            decay_switch_level: 0xffffffff,

            current_value: 0,
            target_value: 0,
            peak_value: 0,
            phase: EnginePhase::Released,
        }
    }

    fn initialize(&mut self, voice_index: usize, config: &EgConfig) {
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Attack));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Decay));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Sustain));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Release));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Extra1));
        self.update_params(voice_index, config, &InputReaderInfo::new(PotKind::Extra2));
        self.current_value = 0;
        self.target_value = 0;
        self.phase = EnginePhase::Released;
    }

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo) {
        match input.pot_info.kind {
            PotKind::Attack => {
                let attack_time = U32F32::from_num(config.attack[voice_index] as u32);
                let attack_time_constant = U32F32::from_num(1u32)
                    + (U32F32::from_num(3u32) / U32F32::from_num(2_000_000_000u32))
                        * attack_time
                        * attack_time
                        * attack_time;
                let attack_ratio = U32F32::from_num(1u32) / attack_time_constant;
                self.attack_ratio = attack_ratio.to_bits();
            }
            PotKind::Decay => {
                let decay_time = U32F32::from_num(config.decay[voice_index] as u32);
                let decay_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * decay_time
                        * decay_time
                        * decay_time;
                let decay_ratio = U32F32::from_num(1u32) / decay_time_constant;
                self.decay_ratio = decay_ratio.to_bits();
            }
            PotKind::Sustain => {
                let sustain_level = config.sustain[voice_index] as u64;
                self.sustain_level = ((sustain_level >> 1) + 32768) * sustain_level;
            }
            PotKind::Release => {
                let release_time = U32F32::from_num(config.release[voice_index] as u32);
                let release_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * release_time
                        * release_time
                        * release_time;
                let release_ratio = U32F32::from_num(1u32) / release_time_constant;
                self.release_ratio = release_ratio.to_bits();
            }
            PotKind::Extra1 => {
                if matches!(self.engine_type, EngineType::ADDSR) {
                    let extra1_time = U32F32::from_num(config.extra1[voice_index] as u32);
                    let decay_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                        + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                            * extra1_time
                            * extra1_time
                            * extra1_time;
                    let initial_decay_ratio = U32F32::from_num(1u32) / decay_time_constant;
                    self.initial_decay_ratio = initial_decay_ratio.to_bits();
                }
            }
            PotKind::Extra2 => {
                if matches!(self.engine_type, EngineType::ADDSR) {
                    let level = config.extra2[voice_index] as u64;
                    self.decay_switch_level = ((level >> 1) + 32768) * level;
                }
            }
            _ => {} // TODO interpret CV1_DEPTH and CV2_DEPTH
        }
    }

    /// Handles a gate-on event.
    /// The method forces changing the phase to Attack regardless the current phase.
    /// Also
    fn gate_on(&mut self, params: &VoiceParams) {
        // Add an offset of 1/32 level to avoid silence with low velocity
        let velocity = params.velocity as u64;
        let level = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
        // target = level * 1.2
        self.target_value = level * 6;
        self.target_value /= 5;
        self.peak_value = level;
        self.phase = EnginePhase::Attack;
    }

    /// Handles a gate-off event.
    fn gate_off(&mut self) {
        self.target_value = 0;
        self.peak_value = self.current_value;
        self.phase = EnginePhase::Released;
    }

    /// Updates the current envelope generator value and returns it in 12 bit range.
    fn update(&mut self, _params: &VoiceParams) -> u16 {
        match self.phase {
            EnginePhase::Attack => {
                let mut delta = self.target_value - self.current_value;
                delta *= self.attack_ratio;
                self.current_value += delta >> 32;
                if self.current_value >= self.peak_value {
                    self.phase = EnginePhase::InitialDecay;
                    self.current_value = self.peak_value;
                }
            }
            EnginePhase::InitialDecay => {
                // update the target value every cycle as the sustain level may have changed.
                let switch_value = (self.peak_value * self.decay_switch_level) >> 32;
                let mut current_value: i64 = self.current_value as i64;
                let target_value: i64 =
                    (switch_value as i64 - self.peak_value as i64) * 6 / 5 + self.peak_value as i64;
                if target_value < current_value {
                    let mut delta = current_value - target_value;
                    delta *= self.initial_decay_ratio as i64;
                    delta >>= 32;
                    current_value -= delta;
                    if current_value > switch_value as i64 {
                        self.current_value = current_value as u64;
                    } else {
                        self.current_value = switch_value;
                        self.phase = EnginePhase::Decay;
                    }
                } else {
                    let mut delta = target_value - current_value;
                    delta *= self.initial_decay_ratio as i64;
                    delta >>= 32;
                    current_value += delta;
                    if current_value < switch_value as i64 {
                        self.current_value = current_value as u64;
                    } else {
                        self.current_value = switch_value;
                        self.phase = EnginePhase::Decay;
                    }
                }
            }
            EnginePhase::Decay => {
                // update the target value every cycle as the sustain level may have changed.
                self.target_value = (self.peak_value * self.sustain_level) >> 32;
                if self.target_value < self.current_value {
                    let mut delta = self.current_value - self.target_value;
                    delta *= self.decay_ratio;
                    self.current_value -= delta >> 32;
                } else {
                    let mut delta = self.target_value - self.current_value;
                    delta *= self.decay_ratio;
                    self.current_value += delta >> 32;
                };
            }
            EnginePhase::Released => {
                let delta = self.current_value * self.release_ratio;
                self.current_value -= delta >> 32;
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        (self.current_value >> 19) as u16
    }
}

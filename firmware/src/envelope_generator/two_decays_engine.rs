/// Default EG voice engine
use defmt;
use fixed::types::{I32F32, U32F32};

use crate::input_reader::{InputReaderInfo, PotKind};

use super::config::EgConfig;
use super::definitions::Engine;
use super::definitions::VoiceParams;

#[derive(Debug, defmt::Format)]
enum EnginePhase {
    Released,
    Attack,
    InitialDecay,
    Decay,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct TwoDecaysEngine {
    // Parameters translated by the EG configuration.
    attack_ratio: I32F32,
    decay_ratio: I32F32,
    release_ratio: I32F32,
    initial_decay_ratio: I32F32,

    sustain_level: I32F32,
    decay_switch_level: I32F32,

    // Precomputed constant
    initial_decay_scale_factor: I32F32,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ32.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: I32F32,
    // The engine simulates RC charging/discharging for this target value.
    // Different transient ratio (attack_ratio, decay_ratio, or release_ratio) is used
    // according to the current phase.
    target_value: I32F32,
    // The peak value is used for switching phases between attack and decay. When the
    // current value reaches the peak value during attack phase, the engine swithces its
    // phase to decay.
    peak_value: I32F32,

    phase: EnginePhase,
}

const SIX_FIFTHS: I32F32 = I32F32::from_bits(((6i64 << 32) / 5) as i64);

impl Engine for TwoDecaysEngine {
    fn new() -> Self {
        Self {
            attack_ratio: I32F32::from_num(0),
            decay_ratio: I32F32::from_num(0),
            release_ratio: I32F32::from_num(0),
            initial_decay_ratio: I32F32::from_num(0),

            sustain_level: I32F32::from_num(1),
            decay_switch_level: I32F32::from_num(1),

            initial_decay_scale_factor: SIX_FIFTHS,

            current_value: I32F32::from_num(0),
            target_value: I32F32::from_num(0),
            peak_value: I32F32::from_num(0),

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
        self.current_value = I32F32::from_num(0i32);
        self.target_value = I32F32::from_num(0i32);
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
                let ratio_u = U32F32::from_num(1u32) / attack_time_constant;
                self.attack_ratio = I32F32::from_bits(ratio_u.to_bits() as i64);
            }
            PotKind::Decay => {
                let decay_time = U32F32::from_num(config.decay[voice_index] as u32);
                let decay_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * decay_time
                        * decay_time
                        * decay_time;
                let ratio_u = U32F32::from_num(1u32) / decay_time_constant;
                self.decay_ratio = I32F32::from_bits(ratio_u.to_bits() as i64);
            }
            PotKind::Sustain => {
                let sustain_level = config.sustain[voice_index] as u64;
                let level_u = U32F32::from_bits(((sustain_level >> 1) + 32768) * sustain_level);
                self.sustain_level = I32F32::from_bits(level_u.to_bits() as i64);
            }
            PotKind::Release => {
                let release_time = U32F32::from_num(config.release[voice_index] as u32);
                let release_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * release_time
                        * release_time
                        * release_time;
                let ratio_u = U32F32::from_num(1u32) / release_time_constant;
                self.release_ratio = I32F32::from_bits(ratio_u.to_bits() as i64);
            }
            PotKind::Extra1 => {
                let extra1_time = U32F32::from_num(config.extra1[voice_index] as u32);
                let decay_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * extra1_time
                        * extra1_time
                        * extra1_time;
                let ratio_u = U32F32::from_num(1u32) / decay_time_constant;
                self.initial_decay_ratio = I32F32::from_bits(ratio_u.to_bits() as i64);
            }
            PotKind::Extra2 => {
                let level = config.extra2[voice_index] as u64;
                self.decay_switch_level =
                    I32F32::from_bits((((level >> 1) + 32768) * level) as i64);
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
        let level_bits = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
        let level = I32F32::from_bits(level_bits as i64);
        self.target_value = level * SIX_FIFTHS;
        self.peak_value = level;
        self.phase = EnginePhase::Attack;
    }

    /// Handles a gate-off event.
    fn gate_off(&mut self) {
        self.target_value = I32F32::from_num(0i32);
        self.peak_value = self.current_value;
        self.phase = EnginePhase::Released;
    }

    /// Updates the current envelope generator value and returns it in 12 bit range.
    fn update(&mut self, _params: &VoiceParams) -> u16 {
        match self.phase {
            EnginePhase::Attack => {
                let mut delta = self.target_value - self.current_value;
                delta *= self.attack_ratio;
                self.current_value += delta;
                if self.current_value >= self.peak_value {
                    self.phase = EnginePhase::InitialDecay;
                    self.current_value = self.peak_value;
                }
            }
            EnginePhase::InitialDecay => {
                let peak = self.peak_value;
                let switch = peak * self.decay_switch_level;

                let target = peak + (switch - peak) * self.initial_decay_scale_factor;

                let delta = (target - self.current_value) * self.initial_decay_ratio;
                self.current_value += delta;

                let crossed = if delta >= I32F32::from_num(0) {
                    self.current_value >= switch
                } else {
                    self.current_value <= switch
                };

                if crossed {
                    self.current_value = switch;
                    self.phase = EnginePhase::Decay;
                }
            }
            EnginePhase::Decay => {
                // update the target value every cycle as the sustain level may have changed.
                self.target_value = self.peak_value * self.sustain_level;

                let delta = (self.target_value - self.current_value) * self.decay_ratio;
                self.current_value += delta;

                // clamp to zero (branchless-ish)
                let bits = self.current_value.to_bits();
                self.current_value = I32F32::from_bits(bits & !(bits >> 63));
            }
            EnginePhase::Released => {
                let delta = self.current_value * self.release_ratio;
                self.current_value -= delta;
            }
        }

        // Generate the final output
        let bits = self.current_value.to_bits();
        // clamp negative → 0
        let bits = bits & !(bits >> 63);
        // scale to 12-bit
        (bits as u64 >> 19) as u16
    }
}

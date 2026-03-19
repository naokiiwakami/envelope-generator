/// Default EG voice engine
use defmt;
use fixed::types::{I32F32, U32F32};

use super::config::EgConfig;
use super::definitions::Engine;
use super::definitions::VoiceParams;

use crate::input_reader::{InputReaderInfo, PotKind};

#[derive(Debug, defmt::Format)]
enum EnginePhase {
    Released,
    Attack,
    Decay,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct AdsrDdEngine {
    // Parameters translated by the EG configuration.
    attack_ratio: I32F32,
    decay_ratio: I32F32,
    sustain_level: I32F32,
    release_ratio: I32F32,

    strum_decay_ratio: I32F32,

    decay_time_scale: U32F32,
    decay_time_constant: U32F32,

    speed: I32F32,
    depth: I32F32,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ32.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: I32F32,
    first_strum: I32F32,
    strum_value: I32F32,
    distortion: I32F32,
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
const SEVEN_POINT_FIVE: U32F32 = U32F32::from_bits(0x7_7fffffff);

impl Engine for AdsrDdEngine {
    fn new() -> Self {
        Self {
            attack_ratio: I32F32::from_num(0i32),
            decay_ratio: I32F32::from_num(0i32),
            sustain_level: I32F32::from_bits(0x0),
            release_ratio: I32F32::from_num(0i32),

            strum_decay_ratio: I32F32::from_bits(0x0),

            decay_time_scale: U32F32::from_num(0.0000000025f32),
            decay_time_constant: U32F32::from_bits(0x0),

            speed: I32F32::from_num(0i32),
            depth: I32F32::from_num(0.5f32),

            current_value: I32F32::from_bits(0x0),
            first_strum: I32F32::from_bits(0x0),
            strum_value: I32F32::from_bits(0x0),
            distortion: I32F32::from_num(0i32),

            target_value: I32F32::from_num(0i32),
            peak_value: I32F32::from_num(0i32),
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
                // let decay_time = U32F32::from_num(config.decay[voice_index] as u32);
                let decay_time = U32F32::from_bits((config.decay[voice_index] as u64) << 16);

                self.decay_time_constant = SEVEN_POINT_FIVE
                    + U32F32::from_num(700000) * decay_time * decay_time * decay_time;
                // SEVEN_POINT_FIVE + self.decay_time_scale * decay_time * decay_time * decay_time;
                let ratio_u = U32F32::from_bits(0x1_00000000) / self.decay_time_constant;
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
                let decay_time = U32F32::from_bits((config.extra1[voice_index] as u64) << 16);

                let decay_time_constant = SEVEN_POINT_FIVE
                    + U32F32::from_num(700000) * decay_time * decay_time * decay_time;
                // SEVEN_POINT_FIVE + self.decay_time_scale * decay_time * decay_time * decay_time;
                let ratio_u = U32F32::from_bits(0x1_00000000) / decay_time_constant;
                self.strum_decay_ratio = I32F32::from_bits(ratio_u.to_bits() as i64);
            }
            PotKind::Extra2 => {
                self.depth = I32F32::from_bits((config.extra2[voice_index] as i64) << 16);
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
                    self.phase = EnginePhase::Decay;
                    self.current_value = self.peak_value;
                    self.first_strum = self.peak_value;
                    self.strum_value = self.peak_value;
                    /*
                    self.distortion = I32F32::from_bits(0x1_00000000)
                        + I32F32::from_bits(0x4_00000000) * self.depth;
                        */
                }
            }
            EnginePhase::Decay => {
                // let ratio_u = U32F32::from_bits(0x1_00000000) / self.decay_time_constant;
                // self.decay_ratio = I32F32::from_bits(ratio_u.to_bits() as i64);

                self.target_value = self.peak_value * self.sustain_level;

                let delta = (self.target_value - self.first_strum) * self.decay_ratio;
                self.first_strum += delta;

                let delta = (self.target_value - self.strum_value) * self.strum_decay_ratio;
                self.strum_value += delta;

                let current_value = self.first_strum + self.strum_value;

                self.current_value = I32F32::from_bits(current_value.to_bits() >> 1);
                /*

                // clamp to zero (branchless-ish)
                let bits = current_value.to_bits() >> 1;
                // let out = I32F32::from_bits(bits & !(bits >> 63));
                return (bits >> 19) as u16;
                */
            }
            EnginePhase::Released => {
                let delta = self.current_value * self.release_ratio;
                self.current_value -= delta;
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        (self.current_value.to_bits() >> 19) as u16
    }
}

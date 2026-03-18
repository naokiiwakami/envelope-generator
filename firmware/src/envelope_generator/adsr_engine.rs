/// Default EG voice engine
use defmt;
use fixed::types::U32F32;

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
pub struct AdsrEngine {
    // Values are UQ32.32 fixed point integers.
    // This equal integer and fractional assignment is convenient for EG curve
    // calculation as most of values are in range of [0:1) that fit within
    // 32-bit LSB part. Multiplying these values never overflows in UQ32.32 format.

    // Parameters translated by the EG configuration.
    attack_ratio: U32F32,
    decay_ratio: U32F32,
    sustain_level: U32F32,
    release_ratio: U32F32,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ32.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: U32F32,
    // The engine simulates RC charging/discharging for this target value.
    // Different transient ratio (attack_ratio, decay_ratio, or release_ratio) is used
    // according to the current phase.
    target_value: U32F32,
    // The peak value is used for switching phases between attack and decay. When the
    // current value reaches the peak value during attack phase, the engine swithces its
    // phase to decay.
    peak_value: U32F32,

    phase: EnginePhase,
}

impl AdsrEngine {}

impl Engine for AdsrEngine {
    fn new() -> Self {
        Self {
            attack_ratio: U32F32::from_num(0u32),
            decay_ratio: U32F32::from_num(0u32),
            sustain_level: U32F32::from_bits(0xffffffff),
            release_ratio: U32F32::from_num(0u32),

            current_value: U32F32::from_num(0u32),
            target_value: U32F32::from_num(0u32),
            peak_value: U32F32::from_num(0u32),
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
        self.current_value = U32F32::from_num(0u32);
        self.target_value = U32F32::from_num(0u32);
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
                self.attack_ratio = U32F32::from_num(1u32) / attack_time_constant;
            }
            PotKind::Decay => {
                let decay_time = U32F32::from_num(config.decay[voice_index] as u32);
                let decay_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * decay_time
                        * decay_time
                        * decay_time;
                self.decay_ratio = U32F32::from_num(1u32) / decay_time_constant;
            }
            PotKind::Sustain => {
                let sustain_level = config.sustain[voice_index] as u64;
                self.sustain_level =
                    U32F32::from_bits(((sustain_level >> 1) + 32768) * sustain_level);
            }
            PotKind::Release => {
                let release_time = U32F32::from_num(config.release[voice_index] as u32);
                let release_time_constant = (U32F32::from_num(15u32) / U32F32::from_num(2u32))
                    + (U32F32::from_num(5u32) / U32F32::from_num(2_000_000_000u32))
                        * release_time
                        * release_time
                        * release_time;
                self.release_ratio = U32F32::from_num(1u32) / release_time_constant;
            }
            PotKind::Extra1 => {
                // TBD
            }
            PotKind::Extra2 => {
                // TBD
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
        let level = U32F32::from_bits(level_bits);
        // target = level * 1.2
        self.target_value = level * U32F32::from_num(6u32) / U32F32::from_num(5u32);
        self.peak_value = level;
        self.phase = EnginePhase::Attack;
    }

    /// Handles a gate-off event.
    fn gate_off(&mut self) {
        self.target_value = U32F32::from_num(0u32);
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
                }
            }
            EnginePhase::Decay => {
                // update the target value every cycle as the sustain level may have changed.
                self.target_value = self.peak_value * self.sustain_level;

                let current_signed = fixed::types::I32F32::from_num(self.current_value);
                let target_signed = fixed::types::I32F32::from_num(self.target_value);
                let diff = target_signed - current_signed;
                let delta = diff * fixed::types::I32F32::from_num(self.decay_ratio);
                let next_value = current_signed + delta;
                let next_bits = next_value.to_bits();
                self.current_value = if next_bits < 0 {
                    U32F32::from_num(0u32)
                } else {
                    U32F32::from_bits(next_bits as u64)
                };
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

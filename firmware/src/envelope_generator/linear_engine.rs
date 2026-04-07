/// Default EG voice engine
use defmt;

use crate::{
    definitions::PotKind,
    envelope_generator::utils::{
        calculate_linear_charging_ratio, calculate_linear_discharging_ratio,
        calculate_sustain_level,
    },
    input_reader::InputReaderInfo,
    utils::mul_uq0_32,
};

use super::{
    config::EgConfig,
    definitions::{Engine, VoiceParams},
    utils::note_to_scale,
};

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
    attack_ratio: u32,
    decay_ratio: u32,
    sustain_level: u32,
    release_ratio: u32,

    // note scaling factor, represented in UQ8.24
    note_scale: u32,
    // note scaling depth, Q0.32
    note_scale_depth: u32,

    // State
    current_value: u32,
    target_value: u32,
    start_value: u32,
    level: u32,

    phase: EnginePhase,
}

impl LinearEngine {}

impl Engine for LinearEngine {
    fn new() -> Self {
        Self {
            attack_ratio: 0,
            decay_ratio: 0,
            sustain_level: 0,
            release_ratio: 0,

            note_scale: 0x1000000,
            note_scale_depth: 0,

            current_value: 0,
            target_value: 0,
            start_value: 0,
            level: 0,

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
        self.current_value = 0;
        self.phase = EnginePhase::Initial;
    }

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo) {
        match input.pot_info.kind {
            PotKind::Attack => {
                self.attack_ratio = calculate_linear_charging_ratio(config.attack[voice_index]);
            }
            PotKind::Decay => {
                self.decay_ratio = calculate_linear_discharging_ratio(config.decay[voice_index]);
            }
            PotKind::Sustain => {
                self.sustain_level = calculate_sustain_level(config.sustain[voice_index], false, 0);
            }
            PotKind::Release => {
                self.release_ratio =
                    calculate_linear_discharging_ratio(config.release[voice_index]);
            }
            PotKind::Extra2 => {
                self.note_scale_depth = (config.extra2[voice_index] as u32) << 16;
            }
            _ => {}
        }
    }

    /// Handles a gate-on event.
    /// The method forces changing the phase to Attack regardless the current phase.
    fn gate_on(&mut self, params: &VoiceParams) {
        let velocity = params.velocity as u64;
        let level_uq32_32 = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
        self.level = level_uq32_32 as u32;
        self.target_value = self.level;
        self.start_value = 0;
        self.note_scale = note_to_scale(params.note, self.note_scale_depth);
        self.phase = EnginePhase::Attack;
    }

    /// Handles a gate-off event.
    fn gate_off(&mut self) {
        self.target_value = 0;
        self.phase = EnginePhase::Release;
    }

    /// Updates the current envelope generator value and returns it in 12 bit range.
    fn update(&mut self, params: &VoiceParams) -> u16 {
        match self.phase {
            EnginePhase::Initial => {}
            EnginePhase::Attack => {
                let ratio: u64 = (self.attack_ratio as u64 * self.note_scale as u64) >> 24;
                let delta_uq32_32 = mul_uq0_32(ratio.min(0xffffffff) as u32, self.level) as u64;
                let current_value_q32_32: u64 = self.current_value as u64 + delta_uq32_32;

                if current_value_q32_32 >= self.target_value as u64 {
                    self.current_value = self.target_value;
                    self.phase = EnginePhase::Decay;
                } else {
                    self.current_value = current_value_q32_32 as u32;
                }
            }
            EnginePhase::Decay => {
                self.target_value = mul_uq0_32(self.level, self.sustain_level);
                let ratio: u64 = (self.decay_ratio as u64 * self.note_scale as u64) >> 24;
                let delta_uq0_32 = mul_uq0_32(ratio.min(0xffffffff) as u32, self.level);
                if self.current_value >= self.target_value {
                    // going down
                    if self.current_value - self.target_value > delta_uq0_32 {
                        self.current_value -= delta_uq0_32;
                    } else {
                        self.current_value = self.target_value;
                        self.phase = EnginePhase::Sustain;
                    }
                } else {
                    // going up0
                    if self.target_value - self.current_value > delta_uq0_32 {
                        self.current_value += delta_uq0_32;
                    } else {
                        self.current_value = self.target_value;
                        self.phase = EnginePhase::Sustain;
                    }
                }
            }
            EnginePhase::Sustain => {
                self.current_value = mul_uq0_32(self.level, self.sustain_level);
            }
            EnginePhase::Release => {
                let ratio: u64 = (self.release_ratio as u64 * self.note_scale as u64) >> 24;
                let delta = mul_uq0_32(ratio.min(0xffffffff) as u32, self.level);
                if self.current_value > delta {
                    self.current_value -= delta;
                } else {
                    self.current_value = 0;
                    self.phase = EnginePhase::Initial;
                }
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        (*params.value_to_output)(self.current_value, params.out_zero_point)
    }
}

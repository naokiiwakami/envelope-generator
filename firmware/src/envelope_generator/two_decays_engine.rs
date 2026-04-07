/// Default EG voice engine
use defmt;

use crate::{
    definitions::PotKind,
    envelope_generator::utils::{
        calculate_charging_ratio, calculate_discharging_ratio, calculate_sustain_level,
    },
    input_reader::InputReaderInfo,
    utils::{fraction_uq32_32, mul_uq0_32},
};

use super::{
    config::EgConfig,
    definitions::{Engine, VoiceParams},
    utils::note_to_scale,
};

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
    attack_ratio: u32,
    decay_ratio: u32,
    release_ratio: u32,
    initial_decay_ratio: u32,

    sustain_level: u32,
    decay_switch_level: u32,

    // Precomputed constant
    initial_decay_scale_factor: u64,

    // note scaling factor, represented in UQ8.24
    note_scale: u32,
    // note scaling depth, Q0.32
    note_scale_depth: u32,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ32.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: u32,
    // The engine simulates RC charging/discharging for this target value.
    // Different transient ratio (attack_ratio, decay_ratio, or release_ratio) is used
    // according to the current phase.
    target_value: u32,
    // The peak value is used for switching phases between attack and decay. When the
    // current value reaches the peak value during attack phase, the engine swithces its
    // phase to decay.
    peak_value: u32,

    phase: EnginePhase,
}

impl Engine for TwoDecaysEngine {
    fn new() -> Self {
        Self {
            attack_ratio: 0,
            decay_ratio: 0,
            release_ratio: 0,
            initial_decay_ratio: 0,

            sustain_level: 0,
            decay_switch_level: 0,

            initial_decay_scale_factor: fraction_uq32_32(6, 5),

            note_scale: 0x1000000,
            note_scale_depth: 0x40000000, // 0.25

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
                self.attack_ratio = calculate_charging_ratio(config.attack[voice_index]);
            }
            PotKind::Decay => {
                self.decay_ratio = calculate_discharging_ratio(config.decay[voice_index]);
            }
            PotKind::Sustain => {
                self.sustain_level = calculate_sustain_level(config.sustain[voice_index], false, 0);
            }
            PotKind::Release => {
                self.release_ratio = calculate_discharging_ratio(config.release[voice_index]);
            }
            PotKind::Extra1 => {
                self.initial_decay_ratio = calculate_discharging_ratio(config.extra1[voice_index]);
            }
            PotKind::Extra2 => {
                let level = config.extra2[voice_index] as u32;
                self.decay_switch_level = ((level >> 1) + 32768) * level;
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
        let peak_uq32_32 = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
        // target = peak * 1.2, assuming peak does not exceed 1.0
        let target_uq32_32: u64 = (peak_uq32_32 * self.initial_decay_scale_factor) >> 32;
        // UQ32.32 to UQ0.32
        self.target_value = target_uq32_32 as u32;

        self.note_scale = note_to_scale(params.note, self.note_scale_depth);

        self.peak_value = peak_uq32_32 as u32;
        self.phase = EnginePhase::Attack;
    }

    /// Handles a gate-off event.
    fn gate_off(&mut self) {
        self.target_value = 0;
        self.peak_value = self.current_value;
        self.phase = EnginePhase::Released;
    }

    /// Updates the current envelope generator value and returns it in 12 bit range.
    fn update(&mut self, params: &VoiceParams) -> u16 {
        match self.phase {
            EnginePhase::Attack => {
                let diff = self.target_value - self.current_value;
                let ratio: u64 = (self.attack_ratio as u64 * self.note_scale as u64) >> 24;
                let delta = mul_uq0_32(diff, ratio.min(0xffffffff) as u32);
                self.current_value += delta;
                if self.current_value >= self.peak_value {
                    self.phase = EnginePhase::InitialDecay;
                    self.current_value = self.peak_value;
                }
            }
            EnginePhase::InitialDecay => {
                let peak_q32_32 = self.peak_value as i64;
                let switch_q32_32 = mul_uq0_32(self.peak_value, self.decay_switch_level) as i64;
                let mut current_q32_32 = self.current_value as i64;
                let target_q32_32 = (switch_q32_32 - peak_q32_32) * 6 / 5 + peak_q32_32;

                let ratio: u64 = (self.initial_decay_ratio as u64 * self.note_scale as u64) >> 24;
                let ratio: u32 = ratio.min(0xffffffff) as u32;

                let crossed = if target_q32_32 < current_q32_32 {
                    let diff_uq0_32 = (current_q32_32 - target_q32_32) as u32;
                    let delta_uq0_32 = mul_uq0_32(diff_uq0_32, ratio);
                    current_q32_32 -= delta_uq0_32 as i64;
                    current_q32_32 <= switch_q32_32
                } else {
                    let diff_uq0_32 = (target_q32_32 - current_q32_32) as u32;
                    let delta_uq0_32 = mul_uq0_32(diff_uq0_32, ratio);
                    current_q32_32 += delta_uq0_32 as i64;
                    current_q32_32 >= switch_q32_32
                };

                if crossed {
                    self.current_value = switch_q32_32 as u32;
                    self.phase = EnginePhase::Decay;
                } else {
                    self.current_value = current_q32_32 as u32;
                }
            }
            EnginePhase::Decay => {
                // update the target value every cycle as the sustain level may have changed.
                self.target_value = mul_uq0_32(self.peak_value, self.sustain_level);
                let ratio: u64 = (self.decay_ratio as u64 * self.note_scale as u64) >> 24;
                let ratio: u32 = ratio.min(0xffffffff) as u32;
                if self.target_value < self.current_value {
                    let delta = mul_uq0_32(self.current_value - self.target_value, ratio);
                    self.current_value -= delta;
                } else {
                    let delta = mul_uq0_32(self.target_value - self.current_value, ratio);
                    self.current_value += delta;
                }
            }
            EnginePhase::Released => {
                let ratio: u64 = (self.release_ratio as u64 * self.note_scale as u64) >> 24;
                let delta = mul_uq0_32(self.current_value, ratio.min(0xffffffff) as u32);
                self.current_value -= delta;
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        (*params.value_to_output)(self.current_value, params.out_zero_point)
    }
}

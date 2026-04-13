/// Default EG voice engine
use defmt;

use crate::{
    definitions::PotKind,
    envelope_generator::utils::{
        calculate_charging_ratio, calculate_discharging_ratio, calculate_sustain_level,
    },
    input_reader::InputReaderInfo,
    utils::{mul_i16_uq0_16, mul_uq0_32},
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
    Peak,
    Decay,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct AdsrEngine {
    // Parameters translated by the EG configuration.
    attack_ratio: u32,
    decay_ratio: u32,
    sustain_level: u32,
    release_ratio: u32,

    peak_length: u64,

    // note scaling factor, represented in UQ8.24
    note_scale: u32,
    // note scaling depth, Q0.32
    note_scale_depth: u32,

    // modulations
    cv_depth_a: u32, // Q0.32, 0 to 1
    cv_depth_b: u32, // Q0.32, 0 to 1

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ0.32 representation
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

    peak_count: u64,

    phase: EnginePhase,
}

impl AdsrEngine {
    fn update_params_for_pot(
        &mut self,
        voice_index: usize,
        config: &EgConfig,
        pot_kind: &PotKind,
        mod_amount: i16,
    ) {
        match pot_kind {
            PotKind::Attack => {
                self.attack_ratio =
                    calculate_charging_ratio(config.attack(voice_index), mod_amount);
            }
            PotKind::Decay => {
                self.decay_ratio =
                    calculate_discharging_ratio(config.decay(voice_index), mod_amount);
            }
            PotKind::Sustain => {
                self.sustain_level =
                    calculate_sustain_level(config.sustain(voice_index), mod_amount);
            }
            PotKind::Release => {
                self.release_ratio =
                    calculate_discharging_ratio(config.release(voice_index), mod_amount);
            }
            PotKind::Extra1 => {
                let param = config.extra_1(voice_index) as u64;
                self.peak_length = (param * param) >> 21;
            }
            PotKind::Extra2 => {
                let value = config.extra_2(voice_index);
                config.set_note_scaling_depth(voice_index, value);
                self.note_scale_depth = (value as u32) << 16;
            }
            PotKind::CvDepthA => {
                self.cv_depth_a = (config.cv_depth_a() as u32) << 16;
            }
            PotKind::CvDepthB => {
                self.cv_depth_b = (config.cv_depth_b() as u32) << 16;
            }
        }
    }
}

impl Engine for AdsrEngine {
    fn new() -> Self {
        Self {
            attack_ratio: 0,
            decay_ratio: 0,
            sustain_level: 0,
            release_ratio: 0,
            peak_length: 0,

            note_scale: 0x1000000,
            note_scale_depth: 0,

            cv_depth_a: 0,
            cv_depth_b: 0,

            current_value: 0,
            target_value: 0,
            peak_value: 0,
            peak_count: 0,
            phase: EnginePhase::Released,
        }
    }

    fn initialize(&mut self, index: usize, config: &EgConfig) {
        self.update_params(index, config, &InputReaderInfo::new(PotKind::Attack));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::Decay));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::Sustain));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::Release));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::Extra1));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::Extra2));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::CvDepthA));
        self.update_params(index, config, &InputReaderInfo::new(PotKind::CvDepthB));
        self.note_scale_depth = (config.note_scaling_depth(index) as u32) << 16;
        self.current_value = 0;
        self.target_value = 0;
        self.phase = EnginePhase::Released;
    }

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo) {
        let pot_kind = input.pot_info.kind;
        let mod_a = mul_i16_uq0_16(input.cv_info.cv_a, config.cv_depth_a());
        let mod_b = mul_i16_uq0_16(input.cv_info.cv_b, config.cv_depth_b());
        let (mod_amount, mod_a_covered, mod_b_covered) = if config.cv_destination_a() == pot_kind {
            (mod_a, true, false)
        } else if config.cv_destination_b() == pot_kind {
            (mod_b, false, true)
        } else {
            (0, false, false)
        };
        self.update_params_for_pot(voice_index, config, &pot_kind, mod_amount);
        if !mod_a_covered {
            self.update_params_for_pot(voice_index, config, &config.cv_destination_a(), mod_a);
        }
        if !mod_b_covered {
            self.update_params_for_pot(voice_index, config, &config.cv_destination_b(), mod_b);
        }
    }

    /// Handles a gate-on event.
    /// The method forces changing the phase to Attack regardless the current phase.
    fn gate_on(&mut self, params: &VoiceParams) {
        // Add an offset of 1/32 level to avoid silence with low velocity
        let velocity = params.velocity as u64;
        let level = ((velocity * velocity) * 31 + 0xffffffff) >> 6;
        // target = level * 1.2
        let mut target_value: u64 = level * 6;
        target_value /= 5;
        self.target_value = target_value as u32;

        self.note_scale = note_to_scale(params.note, self.note_scale_depth);

        self.peak_value = level as u32;
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
                    self.current_value = self.peak_value;
                    self.peak_count = 0;
                    self.phase = if self.peak_length == 0 {
                        EnginePhase::Decay
                    } else {
                        EnginePhase::Peak
                    };
                }
            }
            EnginePhase::Peak => {
                self.peak_count += 1;
                if self.peak_count >= self.peak_length {
                    self.phase = EnginePhase::Decay;
                }
            }
            EnginePhase::Decay => {
                // update the target value every cycle as the sustain level may have changed.
                self.target_value = mul_uq0_32(self.peak_value, self.sustain_level);
                let ratio: u64 = (self.decay_ratio as u64 * self.note_scale as u64) >> 24;
                let ratio: u32 = ratio.min(0xffffffff) as u32;
                if self.target_value < self.current_value {
                    let delta = mul_uq0_32(self.current_value - self.target_value, ratio as u32);
                    self.current_value -= delta;
                } else {
                    let delta = mul_uq0_32(self.target_value - self.current_value, ratio as u32);
                    self.current_value += delta;
                }
            }
            EnginePhase::Released => {
                // let ratio: u64 = self.release_ratio as u64;
                let ratio: u64 = (self.release_ratio as u64 * self.note_scale as u64) >> 24;
                let delta = mul_uq0_32(self.current_value, ratio.min(0xffffffff) as u32);
                self.current_value -= delta;
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        (*params.value_to_output)(self.current_value, params.out_zero_point)
    }
}

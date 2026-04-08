/// Default EG voice engine
use defmt;

use crate::{
    definitions::PotKind,
    envelope_generator::utils::{
        calculate_charging_ratio, calculate_discharging_ratio, calculate_sustain_level,
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
    Released,
    Attack,
    Decay,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct ParaDecaysEngine {
    // Parameters translated by the EG configuration.
    attack_ratio: u32,
    decay_ratio: u32,
    sustain_level: u32,
    release_ratio: u32,

    strum_decay_ratio: u32,

    balance: u32,

    // note scaling factor, represented in UQ8.24
    note_scale: u32,
    // note scaling depth, Q0.32
    note_scale_depth: u32,

    // Values that represent current EG state

    // Current values are calculated to fit within range [0:0.5) in UQ0.32 representation
    // where actual range is [0..0x7fffffff].
    current_value: u32,
    strum: u32,
    main_decay: u32,
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

impl Engine for ParaDecaysEngine {
    fn new() -> Self {
        Self {
            attack_ratio: 0,
            decay_ratio: 0,
            sustain_level: 0,
            release_ratio: 0,

            strum_decay_ratio: 0,

            balance: 0x7fffffff,

            note_scale: 0x1000000,
            note_scale_depth: 0x40000000, // 0.25

            current_value: 0,
            strum: 0,
            main_decay: 0,

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
        self.note_scale_depth = (config.note_scaling_depth(voice_index) as u32) << 16;
        self.current_value = 0;
        self.target_value = 0;
        self.phase = EnginePhase::Released;
    }

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo) {
        match input.pot_info.kind {
            PotKind::Attack => {
                self.attack_ratio = calculate_charging_ratio(config.attack(voice_index), 0);
            }
            PotKind::Decay => {
                self.decay_ratio = calculate_discharging_ratio(config.decay(voice_index), 0);
            }
            PotKind::Sustain => {
                self.sustain_level = calculate_sustain_level(config.sustain(voice_index), 0);
            }
            PotKind::Release => {
                self.release_ratio = calculate_discharging_ratio(config.release(voice_index), 0);
            }
            PotKind::Extra1 => {
                self.strum_decay_ratio =
                    calculate_discharging_ratio(config.extra_1(voice_index), 0);
            }
            PotKind::Extra2 => {
                self.balance = (config.extra_2(voice_index) as u32) << 16;
            }
            _ => {} // TODO interpret CV1_DEPTH and CV2_DEPTH
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
                    self.phase = EnginePhase::Decay;
                    self.current_value = self.peak_value;
                    self.main_decay = self.current_value;
                    self.strum = self.current_value;
                }
            }
            EnginePhase::Decay => {
                self.target_value = mul_uq0_32(self.peak_value, self.sustain_level);

                // update the main decay curve
                let decay_ratio: u64 = (self.decay_ratio as u64 * self.note_scale as u64) >> 24;
                let decay_ratio: u32 = decay_ratio.min(0xffffffff) as u32;
                if self.target_value < self.main_decay {
                    let delta = mul_uq0_32(self.main_decay - self.target_value, decay_ratio);
                    self.main_decay -= delta;
                } else {
                    let delta = mul_uq0_32(self.target_value - self.main_decay, decay_ratio);
                    self.main_decay += delta;
                }

                // update the strum curve
                let strum_decay_ratio: u64 =
                    (self.strum_decay_ratio as u64 * self.note_scale as u64) >> 24;
                let strum_decay_ratio: u32 = strum_decay_ratio.min(0xffffffff) as u32;
                if self.target_value < self.strum {
                    let delta = mul_uq0_32(self.strum - self.target_value, strum_decay_ratio);
                    self.strum -= delta;
                } else {
                    let delta = mul_uq0_32(self.target_value - self.strum, strum_decay_ratio);
                    self.strum += delta;
                }

                // mix two
                let strum_depth = u32::MAX - self.balance;
                self.current_value =
                    mul_uq0_32(self.main_decay, self.balance) + mul_uq0_32(self.strum, strum_depth);
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

/// Default EG voice engine
use defmt;

use crate::input_reader::{InputReaderInfo, PotKind};

use super::{
    config::EgConfig,
    definitions::{Engine, VoiceParams},
    utils::{mul_uq0_32, note_to_scale},
};

#[derive(Debug, defmt::Format)]
enum EnginePhase {
    Released,
    Attack,
    Decay,
}

/// The fundamental envelope EG voice engine that generates traditional ADSR curve.
pub struct AdsrEngine {
    // Parameters translated by the EG configuration.
    attack_ratio: u32,
    decay_ratio: u32,
    sustain_level: u32,
    release_ratio: u32,

    // note scaling factor, represented in UQ8.24
    note_scale: u32,
    // note scaling depth, Q0.32
    note_scale_depth: u32,

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

    phase: EnginePhase,
}

impl Engine for AdsrEngine {
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
                let attack_param = config.attack[voice_index] as u64;
                // approximately 1 + 1.5e-9 * attack_param^3
                let attack_time: u64 = 1 + ((7 * attack_param * attack_param * attack_param) >> 32);
                self.attack_ratio = 0xffffffff / attack_time as u32;
            }
            PotKind::Decay => {
                let decay_param = config.decay[voice_index] as u64;
                // approximately 7 + 2.5e-9 * decay_param^3
                let decay_time = 7 + ((11 * decay_param * decay_param * decay_param) >> 32);
                self.decay_ratio = 0xffffffff / decay_time as u32;
            }
            PotKind::Sustain => {
                let sustain_param = config.sustain[voice_index] as u32;
                self.sustain_level = ((sustain_param >> 1) + 32768) * sustain_param;
            }
            PotKind::Release => {
                let release_param = config.release[voice_index] as u64;
                // approximately 7 + 2.5e-9 * decay_param^3
                let release_time = 7 + ((11 * release_param * release_param * release_param) >> 32);
                self.release_ratio = 0xffffffff / release_time as u32;
            }
            PotKind::Extra1 => {
                // TBD
            }
            PotKind::Extra2 => {
                self.note_scale_depth = (config.extra2[voice_index] as u32) << 16;
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
                let ratio: u32 = ratio.min(0xffffffff) as u32;
                let delta = mul_uq0_32(self.current_value, ratio as u32);
                self.current_value -= delta;
            }
        }

        // scale range of 31 bit (0..7fffffff) down to 12 bit (0..fff).
        (*params.value_to_output)(self.current_value, params.out_zero_point)
    }
}

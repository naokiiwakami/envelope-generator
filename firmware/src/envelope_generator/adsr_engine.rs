/// Default EG voice engine
use defmt;

use super::config::EgConfig;
use super::definitions::Engine;
use super::definitions::VoiceParams;

use crate::input_reader::PotKind;

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
    // The engine simulates RC charging/discharging for this target value.
    // Different transient ratio (attack_ratio, decay_ratio, or release_ratio) is used
    // according to the current phase.
    target_value: u64,
    // The peak value is used for switching phases between attack and decay. When the
    // current value reaches the peak value during attack phase, the engine swithces its
    // phase to decay.
    peak_value: u64,

    phase: EnginePhase,

    // The 31-bit (0..0x7fffffff) current value is scaled down to this output buffer
    // of range 0..0xfff for each value update so that the parent EgVoice picks up the
    // value and put it in the buffer for DAC.
    pub out_buf: u16,
}

impl AdsrEngine {}

impl Engine for AdsrEngine {
    fn new() -> Self {
        Self {
            attack_ratio: 0,
            decay_ratio: 0,
            sustain_level: 0xffffffff,
            release_ratio: 0,

            current_value: 0,
            target_value: 0,
            peak_value: 0,
            phase: EnginePhase::Released,

            out_buf: 0,
        }
    }

    fn initialize(&mut self, voice_index: usize, config: &EgConfig) {
        self.update_params(voice_index, config, &PotKind::Attack);
        self.update_params(voice_index, config, &PotKind::Decay);
        self.update_params(voice_index, config, &PotKind::Sustain);
        self.update_params(voice_index, config, &PotKind::Release);
        self.update_params(voice_index, config, &PotKind::Extra1);
        self.update_params(voice_index, config, &PotKind::Extra2);
        self.current_value = 0;
        self.target_value = 0;
        self.phase = EnginePhase::Released;
    }

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, updated_pot: &PotKind) {
        match updated_pot {
            PotKind::Attack => {
                let attack_time = config.attack[voice_index] as f64;
                let attack_time_constant: f64 =
                    1.0 + 1.5e-9 * attack_time * attack_time * attack_time;
                self.attack_ratio = 0xffffffff / attack_time_constant as u64;
            }
            PotKind::Decay => {
                let new_decay_time = config.decay[voice_index] as f64;
                let decay_time_constant =
                    7.5 + 2.5e-9 * new_decay_time * new_decay_time * new_decay_time;
                self.decay_ratio = 0xffffffff / decay_time_constant as u64;
            }
            PotKind::Sustain => {
                let sustain_level = config.sustain[voice_index] as u64;
                self.sustain_level = ((sustain_level >> 1) + 32768) * sustain_level;
            }
            PotKind::Release => {
                let new_release_time = config.release[voice_index] as f64;
                let release_time_constant =
                    7.5 + 2.5e-9 * new_release_time * new_release_time * new_release_time;
                self.release_ratio = 0xffffffff / release_time_constant as u64;
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
                    self.phase = EnginePhase::Decay;
                    self.current_value = self.peak_value;
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
        self.out_buf = (self.current_value >> 19) as u16;

        self.out_buf
    }
}

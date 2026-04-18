use defmt;

use super::config::EgConfig;

use crate::{
    definitions::{AtomicEnumRepr, CvKind, PotKind},
    envelope_generator::utils::uq0_32_to_12bit_positive,
    input_reader::{InputReaderInfo, PotInfo},
};

pub const DEFAULT_VOICE_IDS: [u16; 2] = [0x101, 0x102];
pub const DEFAULT_ENGINE_TYPE: EngineType = EngineType::ParaDecays;
pub const DEFAULT_NOTE_SCALING_DEPTH: u16 = 0x4000;

/// The zero point should be at the center of value range if the circuit is perfect.
pub const DEFAULT_OUT_ZERO_POINT: u16 = 0x800;

/// Envelope Generator operation modes
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub enum Mode {
    Normal,
    Calibration,
    Diagnose,
}

/// Parameters shared between the EG voice controller and EG engine
pub struct VoiceParams {
    pub voice_index: usize,
    pub note: u8,
    pub velocity: u16,
    pub out_zero_point: u16,
    pub value_to_output: &'static dyn Fn(u32, u16) -> u16,

    pub physical_gate_enabled: bool,
    pub operation_mode: Mode,
}

impl VoiceParams {
    pub fn new(voice_index: usize) -> Self {
        Self {
            voice_index,
            note: 60, // middle C
            velocity: 0,
            out_zero_point: DEFAULT_OUT_ZERO_POINT,
            value_to_output: &uq0_32_to_12bit_positive,
            physical_gate_enabled: false,
            operation_mode: Mode::Normal,
        }
    }
}

/// Request for the envelope generator
pub enum EgRequest {
    /// Notifies the EnvelopeGenerator a physical gate event.
    GateEvent {
        id: VoiceId,
        event: GateEventType,
    },
    /// Requests to switch the engine type.
    SwitchEngine {
        engine_type: EngineType,
        send_notif: bool,
    },
    /// Requests to change output polarities
    ChangeOutputPolarities {
        polarity_1: OutputPolarity,
        polarity_2: OutputPolarity,
        send_notif: bool,
    },
    /// Requests to change CV destination
    ChangeCvDestination {
        source: CvKind,
        destination: PotKind,
    },
    ChangeNoteScalingDepth {
        depth: u16,
        save: bool,
    },
    /// Requests to toggle the operation mode.
    /// The EnvelopeGenerator switches operation mode if the requested mode is different
    /// from the current one, otherwise switches to the Normal mode.
    ToggleMode {
        mode: Mode,
    },
    /// Requests to update output zero points.
    UpdateZeroPoints {
        value_1: u16,
        value_2: u16,
        save: bool,
    },
    /// Set output to a certain value.
    /// Valid only in Calibration mode.
    SetOutput {
        voice_id: VoiceId,
        value: u16,
        polarity: OutputPolarity,
    },
}

/// EG engine types
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
#[repr(u8)]
pub enum EngineType {
    ParaDecays = 0,
    Adsr = 1,
    TwoPhases = 2,
    Linear = 3,
}

impl EngineType {
    pub fn name(&self) -> &'static str {
        match self {
            EngineType::ParaDecays => "ParaDecays",
            EngineType::TwoPhases => "ADDSR",
            EngineType::Adsr => "ADSR",
            EngineType::Linear => "Linear",
        }
    }

    #[inline]
    pub fn index(self) -> usize {
        (self as u8) as usize
    }
}

impl TryFrom<u8> for EngineType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::ParaDecays),
            1 => Ok(Self::Adsr),
            2 => Ok(Self::TwoPhases),
            3 => Ok(Self::Linear),
            _ => Err(()),
        }
    }
}

impl AtomicEnumRepr for EngineType {
    fn to_u8(self) -> u8 {
        self as u8
    }
}

/// Gate identifiers
#[derive(Clone, Copy, Debug, defmt::Format)]
#[repr(u8)]
pub enum VoiceId {
    Voice1 = 0,
    Voice2 = 1,
}

/// Gate event types
#[derive(Clone, Debug, defmt::Format)]
pub enum GateEventType {
    PhysicalGateEnabled,
    PhysicalGateDisabled,
    GateOn { velocity: u16 },
    GateOff,
}

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum OutputPolarity {
    Positive = 0,
    Negative = 1,
}

impl TryFrom<u8> for OutputPolarity {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Positive),
            1 => Ok(Self::Negative),
            _ => Err(()),
        }
    }
}

impl AtomicEnumRepr for OutputPolarity {
    fn to_u8(self) -> u8 {
        self as u8
    }
}

/// Event occurred in the EnvelopeGenerator
#[derive(Clone)]
pub enum EgEvent {
    EngineSwitched(EngineType),
    PolarityChanged((OutputPolarity, OutputPolarity)), // polarity_1, polarity_2
    PotMoved(PotInfo),
}

pub trait Engine {
    fn new() -> Self;

    fn initialize(&mut self, voice_index: usize, config: &EgConfig);

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo);

    fn gate_on(&mut self, params: &VoiceParams);

    fn gate_off(&mut self);

    // Calculate the next sample and return value in range 0..0xfff.
    fn update(&mut self, params: &VoiceParams) -> u16;
}

/*
// This looks similar to PotKind for now, but we may add more destinations in the future.
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum CvDestination {
    Attack,
    Decay,
    Sustain,
    Release,
    Extra1,
    Extra2,
}
*/

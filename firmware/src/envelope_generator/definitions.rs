use defmt;

use super::config::EgConfig;

use crate::input_reader::InputReaderInfo;

pub const DEFAULT_VOICE_IDS: [u16; 2] = [0x101, 0x102];
pub const DEFAULT_ENGINE_TYPE: EngineType = EngineType::ParaDecays;

/// Parameters shared between the EG voice controller and EG engine
pub struct VoiceParams {
    pub voice_index: usize,
    pub note: u8,
    pub velocity: u16,
    pub out_zero_point: u16,
    pub value_to_output: &'static dyn Fn(u32, u16) -> u16,
}

/// Request for the envelope generator
pub enum EgRequest {
    GateEvent {
        id: GateId,
        event: GateEventType,
    },
    SwitchEngine {
        engine_type: EngineType,
        send_notif: bool,
        save: bool,
    },
    UpdateZeroPoint {
        value_1: u16,
        value_2: u16,
        save: bool,
    },
}

/// EG engine types
#[derive(Clone, PartialEq)]
#[repr(u8)]
pub enum EngineType {
    ParaDecays = 0,
    Addsr = 1,
    Adsr = 2,
    Linear = 3,
    Diag = 4,
}

impl EngineType {
    pub fn name(&self) -> &'static str {
        match self {
            EngineType::ParaDecays => "ParaDecays",
            EngineType::Addsr => "ADDSR",
            EngineType::Adsr => "ADSR",
            EngineType::Linear => "Linear",
            EngineType::Diag => "Diagnose",
        }
    }
}

impl TryFrom<u8> for EngineType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(EngineType::ParaDecays),
            1 => Ok(EngineType::Addsr),
            2 => Ok(EngineType::Adsr),
            3 => Ok(EngineType::Linear),
            4 => Ok(EngineType::Diag),
            _ => Err(()),
        }
    }
}

/// Gate identifiers
#[derive(Clone, Debug, defmt::Format)]
pub enum GateId {
    Gate1,
    Gate2,
}

/// Gate event types
#[derive(Clone, Debug, defmt::Format)]
pub enum GateEventType {
    AnalogGateEnabled,
    AnalogGateDisabled,
    GateOn { velocity: u16 },
    GateOff,
}

/// Event occurred in the EnvelopeGenerator
#[derive(Clone)]
pub enum EgEvent {
    EngineSwitched(EngineType),
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

/// The zero point should be at the center of value range if the circuit is perfect.
pub const DEFAULT_OUT_ZERO_POINT: u16 = 0x800;

/*
/// Converts a Q0.32 value of range [0..0.5) to 12-bit negative output.
/// The function does not check boundary intentionally for performance.
/// The call should ensure the input is less than 0x80000000.
pub fn uq0_32_to_output_negative(value: u32, zero_point: u16) -> u16 {
    zero_point - (value >> 20) as u16
}
*/

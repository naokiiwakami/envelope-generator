use defmt;

use super::config::EgConfig;

use crate::input_reader::InputReaderInfo;

/// Parameters shared between the EG voice controller and EG engine
pub struct VoiceParams {
    pub voice_index: usize,
    pub note: u8,
    pub velocity: u16,
}

/// Request for the envelope generator
pub enum EgRequest {
    GateEvent { id: GateId, event: GateEventType },
    SwitchEngine(EngineType),
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

// Fixed point arithmetic /////////////////////////////////////////

/// Multiplies two UQ0.32 numbers and returns UQ0.32 result.
#[inline(always)]
pub fn mul_uq0_32(a: u32, b: u32) -> u32 {
    ((a as u64 * b as u64) >> 32) as u32
}

/// Calculate a UQ32.32 fraction.
#[inline(always)]
pub fn fraction_uq32_32(numerator: u32, denominator: u32) -> u64 {
    ((numerator as u64) << 32) / denominator as u64
}

// Engine helper functions ////////////////////////////////////////

/// Converts a UQ0.32 value of range [0..0.5) to 12-bit positive output.
/// Zero value is 0x800.
/// The function does not check boundary intentionally for performance.
/// The call should ensure the input is less than 0x80000000.
pub fn uq0_32_to_output_positive(value: u32) -> u16 {
    (value >> 20) as u16 + 0x800
}

/// Converts a Q0.32 value of range [0..0.5) to 12-bit negative output.
/// Zero value is 0x800
/// The function does not check boundary intentionally for performance.
/// The call should ensure the input is less than 0x80000000.
pub fn uq0_32_to_output_negative(value: u32) -> u16 {
    0x800 - (value >> 20) as u16
}

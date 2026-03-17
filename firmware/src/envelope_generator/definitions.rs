use defmt;

use super::config::EgConfig;

use crate::input_reader::InputReaderInfo;

/// Parameters shared between the EG voice controller and EG engine
pub struct VoiceParams {
    pub voice_index: usize,
    pub note: u8,
    pub velocity: u16,
}

/// Event for the envelope generator
pub enum EgEvent {
    GateEvent { id: GateId, event: GateEventType },
    SwitchEngineRequested(EngineType),
}

/// EG engine types
#[derive(Clone, Debug, defmt::Format)]
#[repr(u8)]
pub enum EngineType {
    ADSR = 0,
    ADDSR = 1,
    Linear = 2,
    Diag = 3,
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

pub trait Engine {
    fn new() -> Self;

    fn initialize(&mut self, voice_index: usize, config: &EgConfig);

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, input: &InputReaderInfo);

    fn gate_on(&mut self, params: &VoiceParams);

    fn gate_off(&mut self);

    // Calculate the next sample and return value in range 0..0xfff.
    fn update(&mut self, params: &VoiceParams) -> u16;
}

use defmt;

use super::config::EgConfig;

use crate::input_reader::PotKind;

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
#[derive(Clone)]
pub enum EngineType {
    ADSR,
    ADDSR,
    Linear,
    Diag,
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

    fn update_params(&mut self, voice_index: usize, config: &EgConfig, updated_pot: &PotKind);

    fn gate_on(&mut self, params: &VoiceParams);

    fn gate_off(&mut self);

    // Calculate the next sample and return value in range 0..0xfff.
    fn update(&mut self, params: &VoiceParams) -> u16;
}

use defmt;

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
    Default,
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

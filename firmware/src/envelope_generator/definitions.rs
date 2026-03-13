pub enum EngineType {
    Default,
    Diag,
}

pub struct VoiceParams {
    pub voice_index: usize,
    pub note: u8,
    pub velocity: u16,
}

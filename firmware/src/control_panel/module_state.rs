use core::sync::atomic::AtomicU16;

use crate::definitions::AtomicEnum;
use crate::envelope_generator::{EngineType, OutputPolarity};

pub struct ModuleState {
    // potentiometers
    /*
    pub attack: AtomicU16,
    pub decay: AtomicU16,
    pub sustain: AtomicU16,
    pub release: AtomicU16,
    pub extra_1: AtomicU16,
    pub extra_2: AtomicU16,
    */
    pub engine_type: AtomicEnum<EngineType>,
    pub polarity_1: AtomicEnum<OutputPolarity>,
    pub polarity_2: AtomicEnum<OutputPolarity>,
}

impl ModuleState {
    pub const fn new() -> Self {
        Self {
            /*
            attack: AtomicU16::new(0),
            decay: AtomicU16::new(0),
            sustain: AtomicU16::new(0),
            release: AtomicU16::new(0),
            extra_1: AtomicU16::new(0),
            extra_2: AtomicU16::new(0),
            */
            engine_type: AtomicEnum::new(EngineType::Adsr as u8),
            polarity_1: AtomicEnum::new(OutputPolarity::Positive as u8),
            polarity_2: AtomicEnum::new(OutputPolarity::Positive as u8),
        }
    }
}

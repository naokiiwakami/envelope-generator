use analog3::{
    definitions::{A3_PROP_ID_NAME, MAX_PROP_VECTOR_LENGTH, Value},
    property::{PropRequest, Property},
};
use heapless::Vec;

use crate::{
    definitions::PotKind,
    envelope_generator::definitions::{DEFAULT_VOICE_IDS, OutputPolarity},
    input_reader::PotInfo,
};

use super::{EngineType, definitions::DEFAULT_ENGINE_TYPE};

static mut CONFIG_DATA: ConfigData = ConfigData::new();

struct ConfigData {
    pub voice_id: [u16; 2],

    pub engine_type: [EngineType; 2],
    pub attack: [u16; 2],
    pub decay: [u16; 2],
    pub sustain: [u16; 2],
    pub release: [u16; 2],
    pub extra_1: [u16; 2],
    pub extra_2: [u16; 2],
    pub cv_a_destination: PotKind,
    pub cv_a_depth: u16,
    pub cv_b_destination: PotKind,
    pub cv_b_depth: u16,
    pub out_polarity: [OutputPolarity; 2],

    pub note_scaling_depth: [u16; 2],
}

impl ConfigData {
    pub const fn new() -> Self {
        Self {
            voice_id: [DEFAULT_VOICE_IDS[0], DEFAULT_VOICE_IDS[1]],

            engine_type: [DEFAULT_ENGINE_TYPE, DEFAULT_ENGINE_TYPE],
            attack: [0; 2],
            decay: [0; 2],
            sustain: [0; 2],
            release: [0; 2],
            extra_1: [0; 2],
            extra_2: [0; 2],
            cv_a_destination: PotKind::Decay,
            cv_a_depth: 0,
            cv_b_destination: PotKind::Sustain,
            cv_b_depth: 0,
            out_polarity: [OutputPolarity::Positive; 2],

            note_scaling_depth: [0x4000; 2], // 0.25
        }
    }
}

// Getters to the ConfigData members ////////////////////

#[inline(always)]
fn get_voice_id(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.voice_id[voice_index] }
}

#[inline(always)]
fn get_engine_type(voice_index: usize) -> EngineType {
    unsafe { CONFIG_DATA.engine_type[voice_index] }
}

#[inline(always)]
fn get_attack(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.attack[voice_index] }
}

#[inline(always)]
fn get_decay(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.decay[voice_index] }
}

#[inline(always)]
fn get_sustain(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.sustain[voice_index] }
}

#[inline(always)]
fn get_release(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.release[voice_index] }
}

#[inline(always)]
fn get_extra_1(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.extra_1[voice_index] }
}

#[inline(always)]
fn get_extra_2(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.extra_2[voice_index] }
}

#[inline(always)]
fn get_cv_a_destination() -> PotKind {
    unsafe { CONFIG_DATA.cv_a_destination }
}

#[inline(always)]
fn get_cv_a_depth() -> u16 {
    unsafe { CONFIG_DATA.cv_a_depth }
}

#[inline(always)]
fn get_cv_b_destination() -> PotKind {
    unsafe { CONFIG_DATA.cv_b_destination }
}

#[inline(always)]
fn get_cv_b_depth() -> u16 {
    unsafe { CONFIG_DATA.cv_b_depth }
}

#[inline(always)]
fn get_out_polarity(voice_index: usize) -> OutputPolarity {
    unsafe { CONFIG_DATA.out_polarity[voice_index] }
}

#[inline(always)]
fn get_note_scaling_depth(voice_index: usize) -> u16 {
    unsafe { CONFIG_DATA.note_scaling_depth[voice_index] }
}

#[derive(Clone)]
#[repr(u8)]
enum EgProperty {
    NumVoices = 3,
    VoiceId = 4,
    EnvelopeGenerationType = 5,
    AttackTime = 6,
    DecayTime = 7,
    SustainLevel = 8,
    ReleaseTime = 9,
    Extra1 = 10,
    Extra2 = 11,
    CvADestination = 12,
    CvADepth = 13,
    CvBDestination = 14,
    CvBDepth = 15,
    OutputPolarity = 16,
}

pub struct EgConfig {
    // TODO: Eliminate these
    pub prop_id: u8,
    pub value: u32,
}

impl EgConfig {
    const PROPS: [EgProperty; 14] = [
        EgProperty::NumVoices,
        EgProperty::VoiceId,
        EgProperty::EnvelopeGenerationType,
        EgProperty::AttackTime,
        EgProperty::DecayTime,
        EgProperty::SustainLevel,
        EgProperty::ReleaseTime,
        EgProperty::Extra1,
        EgProperty::Extra2,
        EgProperty::CvADestination,
        EgProperty::CvADepth,
        EgProperty::CvBDestination,
        EgProperty::CvBDepth,
        EgProperty::OutputPolarity,
    ];

    pub fn new() -> Self {
        Self {
            prop_id: A3_PROP_ID_NAME + 1,
            value: 0x1337c0de,
        }
    }

    #[inline(always)]
    pub fn voice_id(&self, voice_index: usize) -> u16 {
        get_voice_id(voice_index)
    }

    #[inline]
    pub fn set_voice_id(&self, voice_index: usize, voice_id: u16) {
        unsafe {
            CONFIG_DATA.voice_id[voice_index] = voice_id;
        }
    }

    #[inline(always)]
    pub fn engine_type(&self, voice_index: usize) -> EngineType {
        get_engine_type(voice_index)
    }

    #[inline(always)]
    pub fn set_engine_type(&self, voice_index: usize, engine_type: EngineType) {
        unsafe {
            CONFIG_DATA.engine_type[voice_index] = engine_type;
        }
    }

    #[inline(always)]
    pub fn attack(&self, voice_index: usize) -> u16 {
        get_attack(voice_index)
    }

    #[inline(always)]
    pub fn set_attack(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.attack[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn decay(&self, voice_index: usize) -> u16 {
        get_decay(voice_index)
    }

    #[inline(always)]
    pub fn set_decay(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.decay[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn sustain(&self, voice_index: usize) -> u16 {
        get_sustain(voice_index)
    }

    #[inline(always)]
    pub fn set_sustain(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.sustain[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn release(&self, voice_index: usize) -> u16 {
        get_release(voice_index)
    }

    #[inline(always)]
    pub fn set_release(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.release[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn extra_1(&self, voice_index: usize) -> u16 {
        get_extra_1(voice_index)
    }

    #[inline(always)]
    pub fn set_extra_1(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.extra_1[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn extra_2(&self, voice_index: usize) -> u16 {
        get_extra_2(voice_index)
    }

    #[inline(always)]
    pub fn set_extra_2(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.extra_2[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn cv_a_destination(&self) -> PotKind {
        get_cv_a_destination()
    }

    #[inline(always)]
    pub fn set_cv_a_destination(&self, value: PotKind) {
        unsafe {
            CONFIG_DATA.cv_a_destination = value;
        }
    }

    #[inline(always)]
    pub fn cv_a_depth(&self) -> u16 {
        get_cv_a_depth()
    }

    #[inline(always)]
    pub fn set_cv_a_depth(&self, value: u16) {
        unsafe {
            CONFIG_DATA.cv_a_depth = value;
        }
    }

    #[inline(always)]
    pub fn cv_b_destination(&self) -> PotKind {
        get_cv_b_destination()
    }

    #[inline(always)]
    pub fn set_cv_b_destination(&self, value: PotKind) {
        unsafe {
            CONFIG_DATA.cv_b_destination = value;
        }
    }

    #[inline(always)]
    pub fn cv_b_depth(&self) -> u16 {
        get_cv_b_depth()
    }

    #[inline(always)]
    pub fn set_cv_b_depth(&self, value: u16) {
        unsafe {
            CONFIG_DATA.cv_b_depth = value;
        }
    }

    #[inline(always)]
    pub fn out_polarity(&self, voice_index: usize) -> OutputPolarity {
        get_out_polarity(voice_index)
    }

    #[inline(always)]
    pub fn set_out_polarity(&self, voice_index: usize, polarity: OutputPolarity) {
        unsafe {
            CONFIG_DATA.out_polarity[voice_index] = polarity;
        }
    }

    #[inline(always)]
    pub fn note_scaling_depth(&self, voice_index: usize) -> u16 {
        get_note_scaling_depth(voice_index)
    }

    #[inline(always)]
    pub fn set_note_scaling_depth(&self, voice_index: usize, value: u16) {
        unsafe {
            CONFIG_DATA.note_scaling_depth[voice_index] = value;
        }
    }

    #[inline(always)]
    pub fn set_count(&mut self, value: u32) {
        self.value = value;
    }

    /// Handles a prop request
    pub fn handle_request(&mut self, prop_request: PropRequest) {
        match prop_request {
            PropRequest::GetNumProperties { reply } => {
                reply.signal(Some(Property::new(0, Value::U8(Self::PROPS.len() as u8))));
            }
            PropRequest::GetProperty { index, reply } => {
                let property = self.get_property(index as usize);
                reply.signal(property);
            }
            PropRequest::SetProperty {
                prop_id,
                length: _,
                value,
            } => {
                if prop_id == self.prop_id {
                    let value = u32::from_be_bytes(value[..4].try_into().unwrap());
                    self.set_count(value);
                }
            }
        }
    }

    fn make_vec8(src: &[u8]) -> Vec<u8, MAX_PROP_VECTOR_LENGTH> {
        let mut vec = Vec::<u8, MAX_PROP_VECTOR_LENGTH>::new();
        vec.extend_from_slice(&src).unwrap();
        vec
    }

    fn make_vec16(src: &[u16]) -> Vec<u16, MAX_PROP_VECTOR_LENGTH> {
        let mut vec = Vec::<u16, MAX_PROP_VECTOR_LENGTH>::new();
        vec.extend_from_slice(&src).unwrap();
        vec
    }

    fn get_property(&self, index: usize) -> Option<Property> {
        if index >= Self::PROPS.len() {
            return None;
        }
        let prop = &Self::PROPS[index];
        let prop_id = prop.clone() as u8;
        let value = match prop {
            EgProperty::NumVoices => Value::U8(2),
            EgProperty::VoiceId => {
                Value::VectorU16(Self::make_vec16(&[self.voice_id(0), self.voice_id(1)]))
            }
            EgProperty::EnvelopeGenerationType => {
                let type_1 = self.engine_type(0) as u8;
                let type_2 = self.engine_type(1) as u8;
                Value::VectorU8(Self::make_vec8(&[type_1, type_2]))
            }
            EgProperty::AttackTime => {
                Value::VectorU16(Self::make_vec16(&[self.attack(0), self.attack(1)]))
            }
            EgProperty::DecayTime => {
                Value::VectorU16(Self::make_vec16(&[self.decay(0), self.decay(1)]))
            }
            EgProperty::SustainLevel => {
                Value::VectorU16(Self::make_vec16(&[self.sustain(0), self.sustain(1)]))
            }
            EgProperty::ReleaseTime => {
                Value::VectorU16(Self::make_vec16(&[self.release(0), self.release(1)]))
            }
            EgProperty::Extra1 => {
                Value::VectorU16(Self::make_vec16(&[self.extra_1(0), self.extra_1(1)]))
            }
            EgProperty::Extra2 => {
                Value::VectorU16(Self::make_vec16(&[self.extra_2(0), self.extra_2(1)]))
            }
            EgProperty::CvADestination => Value::U8(self.cv_a_destination() as u8),
            EgProperty::CvADepth => Value::U16(self.cv_a_depth()),
            EgProperty::CvBDestination => Value::U8(self.cv_b_destination() as u8),
            EgProperty::CvBDepth => Value::U16(self.cv_b_depth()),
            EgProperty::OutputPolarity => {
                let polarity_1 = self.out_polarity(0) as u8;
                let polarity_2 = self.out_polarity(1) as u8;
                Value::VectorU8(Self::make_vec8(&[polarity_1, polarity_2]))
            }
        };
        Some(Property::new(prop_id, value))
    }

    /// Translates data from the input reader into the config values.
    pub fn translate(&mut self, pot_info: &PotInfo) {
        let pot_value = pot_info.value;
        match pot_info.kind {
            PotKind::Attack => {
                self.set_attack(0, pot_value);
                self.set_attack(1, pot_value);
            }
            PotKind::Decay => {
                self.set_decay(0, pot_value);
                self.set_decay(1, pot_value);
            }
            PotKind::Sustain => {
                self.set_sustain(0, pot_value);
                self.set_sustain(1, pot_value);
            }
            PotKind::Release => {
                self.set_release(0, pot_value);
                self.set_release(1, pot_value);
            }
            PotKind::Extra1 => {
                self.set_extra_1(0, pot_value);
                self.set_extra_1(1, pot_value);
            }
            PotKind::Extra2 => {
                self.set_extra_2(0, pot_value);
                self.set_extra_2(1, pot_value);
            }
            PotKind::CvADepth => self.set_cv_a_depth(pot_value),
            PotKind::CvBDepth => self.set_cv_b_depth(pot_value),
        }
    }
}

pub struct ConfigReader;

impl ConfigReader {
    pub fn new() -> Self {
        Self {}
    }

    #[inline(always)]
    pub fn voice_id(&self, voice_index: usize) -> u16 {
        get_voice_id(voice_index)
    }

    #[inline(always)]
    pub fn engine_type(&self, voice_index: usize) -> EngineType {
        get_engine_type(voice_index)
    }

    #[inline(always)]
    pub fn attack(&self, voice_index: usize) -> u16 {
        get_attack(voice_index)
    }

    #[inline(always)]
    pub fn decay(&self, voice_index: usize) -> u16 {
        get_decay(voice_index)
    }

    #[inline(always)]
    pub fn sustain(&self, voice_index: usize) -> u16 {
        get_sustain(voice_index)
    }

    #[inline(always)]
    pub fn release(&self, voice_index: usize) -> u16 {
        get_release(voice_index)
    }

    #[inline(always)]
    pub fn extra_1(&self, voice_index: usize) -> u16 {
        get_extra_1(voice_index)
    }

    #[inline(always)]
    pub fn extra_2(&self, voice_index: usize) -> u16 {
        get_extra_2(voice_index)
    }

    #[inline(always)]
    pub fn cv_a_destination(&self) -> PotKind {
        get_cv_a_destination()
    }

    #[inline(always)]
    pub fn cv_a_depth(&self) -> u16 {
        get_cv_a_depth()
    }

    #[inline(always)]
    pub fn cv_b_destination(&self) -> PotKind {
        get_cv_b_destination()
    }

    #[inline(always)]
    pub fn cv_b_depth(&self) -> u16 {
        get_cv_b_depth()
    }

    #[inline(always)]
    pub fn out_polarity(&self, voice_index: usize) -> OutputPolarity {
        get_out_polarity(voice_index)
    }

    #[inline(always)]
    pub fn note_scaling_depth(&self, voice_index: usize) -> u16 {
        get_note_scaling_depth(voice_index)
    }
}

use analog3::{
    definitions::{A3_PROP_ID_NAME, MAX_PROP_VECTOR_LENGTH, Value},
    property::{PropRequest, Property},
};
use heapless::Vec;

use crate::{
    definitions::PotKind, envelope_generator::definitions::OutputPolarity, input_reader::PotInfo,
};

use super::{
    EngineType,
    definitions::{DEFAULT_ENGINE_TYPE, DEFAULT_VOICE_IDS},
};

pub struct EgConfig {
    pub prop_id: u8,
    pub value: u32,

    pub voice_id: [u16; 2],

    pub engine_type: [EngineType; 2],
    pub attack: [u16; 2],
    pub decay: [u16; 2],
    pub sustain: [u16; 2],
    pub release: [u16; 2],
    pub extra1: [u16; 2],
    pub extra2: [u16; 2],
    pub cv_a_destination: PotKind,
    pub cv_a_depth: u16,
    pub cv_b_destination: PotKind,
    pub cv_b_depth: u16,
    pub out_polarity: [OutputPolarity; 2],
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

            voice_id: DEFAULT_VOICE_IDS.clone(),

            engine_type: [DEFAULT_ENGINE_TYPE.clone(), DEFAULT_ENGINE_TYPE.clone()],
            attack: [0; 2],
            decay: [0; 2],
            sustain: [0; 2],
            release: [0; 2],
            extra1: [0; 2],
            extra2: [0; 2],
            cv_a_destination: PotKind::Decay,
            cv_a_depth: 0,
            cv_b_destination: PotKind::Sustain,
            cv_b_depth: 0,
            out_polarity: [OutputPolarity::Positive; 2],
        }
    }

    #[inline]
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
            EgProperty::VoiceId => Value::VectorU16(Self::make_vec16(&self.voice_id)),
            EgProperty::EnvelopeGenerationType => {
                let type_1 = self.engine_type[0].clone() as u8;
                let type_2 = self.engine_type[1].clone() as u8;
                Value::VectorU8(Self::make_vec8(&[type_1, type_2]))
            }
            EgProperty::AttackTime => Value::VectorU16(Self::make_vec16(&self.attack)),
            EgProperty::DecayTime => Value::VectorU16(Self::make_vec16(&self.decay)),
            EgProperty::SustainLevel => Value::VectorU16(Self::make_vec16(&self.sustain)),
            EgProperty::ReleaseTime => Value::VectorU16(Self::make_vec16(&self.release)),
            EgProperty::Extra1 => Value::VectorU16(Self::make_vec16(&self.extra1)),
            EgProperty::Extra2 => Value::VectorU16(Self::make_vec16(&self.extra2)),
            EgProperty::CvADestination => Value::U8(self.cv_a_destination as usize as u8),
            EgProperty::CvADepth => Value::U16(self.cv_a_depth),
            EgProperty::CvBDestination => Value::U8(self.cv_b_destination as usize as u8),
            EgProperty::CvBDepth => Value::U16(self.cv_b_depth),
            EgProperty::OutputPolarity => {
                let polarity_1 = self.out_polarity[0].clone() as u8;
                let polarity_2 = self.out_polarity[1].clone() as u8;
                Value::VectorU8(Self::make_vec8(&[polarity_1, polarity_2]))
            }
        };
        Some(Property::new(prop_id, value))
    }

    /// Translates data from the input reader into the config values.
    pub fn translate(&mut self, pot_info: &PotInfo) {
        let pot_value = pot_info.value;
        let destination_params: Option<&mut [u16; 2]> = match pot_info.kind {
            PotKind::Attack => Some(&mut self.attack),
            PotKind::Decay => Some(&mut self.decay),
            PotKind::Sustain => Some(&mut self.sustain),
            PotKind::Release => Some(&mut self.release),
            PotKind::Extra1 => Some(&mut self.extra1),
            PotKind::Extra2 => Some(&mut self.extra2),
            PotKind::CvADepth => {
                self.cv_a_depth = pot_value;
                None
            }
            PotKind::CvBDepth => {
                self.cv_b_depth = pot_value;
                None
            }
        };
        destination_params.and_then(|params| {
            params[0] = pot_value;
            params[1] = pot_value;
            Some(())
        });
    }
}

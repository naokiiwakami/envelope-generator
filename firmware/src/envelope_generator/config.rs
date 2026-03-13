use crate::analog3::definitions::*;

use crate::input_reader::{PotInfo, PotKind};

pub struct EgConfig {
    pub prop_id: u8,
    pub value: u32,

    pub voice_id: [u16; 2],

    pub attack: [u16; 2],
    pub decay: [u16; 2],
    pub sustain: [u16; 2],
    pub release: [u16; 2],
    pub extra1: [u16; 2],
    pub extra2: [u16; 2],
    pub cv1_depth: [u16; 2],
    pub cv2_depth: [u16; 2],
}

impl EgConfig {
    pub fn new(voice0_id: u16, voice1_id: u16) -> Self {
        Self {
            prop_id: A3_PROP_ID_NAME + 1,
            value: 0x1337c0de,

            voice_id: [voice0_id, voice1_id],

            attack: [0; 2],
            decay: [0; 2],
            sustain: [0; 2],
            release: [0; 2],
            extra1: [0; 2],
            extra2: [0; 2],
            cv1_depth: [0; 2],
            cv2_depth: [0; 2],
        }
    }

    #[inline]
    pub fn get_count(&self) -> u32 {
        self.value
    }

    #[inline]
    pub fn set_count(&mut self, value: u32) {
        self.value = value;
    }

    pub fn translate(&mut self, pot_info: &PotInfo) {
        // Pots pick up noise so their values do not drop to zero at the bottoms.
        // It causes noticable slight level at the edge of the configuration.
        // We subtract 4 from the original value to mitigate this problem.
        let value = if pot_info.value >= 4 {
            (pot_info.value - 4) << 4
        } else {
            0
        };
        let destination_params: &mut [u16; 2] = match pot_info.kind {
            PotKind::Attack => &mut self.attack,
            PotKind::Decay => &mut self.decay,
            PotKind::Sustain => &mut self.sustain,
            PotKind::Release => &mut self.release,
            PotKind::Extra1 => &mut self.extra1,
            PotKind::Extra2 => &mut self.extra2,
            PotKind::Cv1Depth => &mut self.cv1_depth,
            PotKind::Cv2Depth => &mut self.cv2_depth,
        };
        destination_params[0] = value;
        destination_params[1] = value;
    }
}

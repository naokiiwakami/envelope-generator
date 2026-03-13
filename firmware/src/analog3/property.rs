use core::cmp::min;
use embassy_stm32::can::frame::FdFrame;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};

use crate::analog3::{A3_MAX_PROP_DATA_SIZE, Value};

#[non_exhaustive]
#[derive(Clone)]
pub struct LV {
    pub length: usize,
    pub value: [u8; 8],
}

impl LV {
    pub fn from_frame(frame: &FdFrame) -> Self {
        let mut value = [0u8; 8];
        let len = min(frame.header().len() as usize, 8);
        value[..len].copy_from_slice(&frame.data()[..len]);
        Self {
            length: frame.data().len(),
            value,
        }
    }
}

#[non_exhaustive]
pub struct Property {
    pub prop_id: u8,
    pub value: Value,
}

impl Property {
    pub fn new(prop_id: u8, value: Value) -> Self {
        Self { prop_id, value }
    }
}

#[allow(unused)]
pub enum PropRequest<'a> {
    GetNumProperties {
        reply: &'a Signal<ThreadModeRawMutex, Option<Property>>,
    },
    GetProperty {
        index: u8,
        reply: &'a Signal<ThreadModeRawMutex, Option<Property>>,
    },
    SetProperty {
        prop_id: u8,
        length: usize,
        value: [u8; A3_MAX_PROP_DATA_SIZE],
    },
}

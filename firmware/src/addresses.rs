#![allow(unused)]
use analog3::{addresses_common::A3_ADDR_MODULE_SPECIFIC_BASE, storage::PAGE_SIZE};

const CALIBRATION_PARAMETER_ADDRESS_RANGE: u16 = 0x20; // we have quota of 32 bytes
const SAVED_CONFIG_BASE: u16 = A3_ADDR_MODULE_SPECIFIC_BASE + CALIBRATION_PARAMETER_ADDRESS_RANGE;

// calibration parameters
pub const ADDR_CV_OFFSET_A: u16 = A3_ADDR_MODULE_SPECIFIC_BASE; // U16 -> i16
pub const ADDR_CV_OFFSET_B: u16 = ADDR_CV_OFFSET_A + 2; // U16 -> i16
pub const ADDR_OUT_ZERO_POINT_1: u16 = ADDR_CV_OFFSET_B + 2; // U16
pub const ADDR_OUT_ZERO_POINT_2: u16 = ADDR_OUT_ZERO_POINT_1 + 2; // U16
pub const ADDR_GATE_TRIGGER_POINT_1: u16 = ADDR_OUT_ZERO_POINT_2 + 2; // U16
pub const ADDR_GATE_TRIGGER_POINT_2: u16 = ADDR_GATE_TRIGGER_POINT_1 + 2; // U16
// ensure the last address is within the allocated area.
const _: () = {
    assert!(ADDR_GATE_TRIGGER_POINT_2 + 2 <= SAVED_CONFIG_BASE);
};

// saved config
pub const ADDR_VOICE_ID_1: u16 = SAVED_CONFIG_BASE; // U16
pub const ADDR_VOICE_ID_2: u16 = ADDR_VOICE_ID_1 + 2; // U16
pub const ADDR_EG_TYPE_1: u16 = ADDR_VOICE_ID_2 + 2; // U8
pub const ADDR_EG_TYPE_2: u16 = ADDR_EG_TYPE_1 + 1; // U8
pub const ADDR_OUTPUT_POLARITY_1: u16 = ADDR_EG_TYPE_2 + 1; // U8
pub const ADDR_OUTPUT_POLARITY_2: u16 = ADDR_OUTPUT_POLARITY_1 + 1; // U8

// ensure the last address is within the page.
const _: () = {
    assert!(ADDR_OUTPUT_POLARITY_2 + 1 <= PAGE_SIZE as u16);
};

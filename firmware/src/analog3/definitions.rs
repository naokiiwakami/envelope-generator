#![allow(dead_code)]

use embassy_stm32::can::frame::FdFrame;
use embedded_can::{Frame, Id};
use heapless::{String, Vec};

// ID assignments /////////////////////////////////
pub const A3_ID_UNASSIGNED: u16 = 0xFFFF;

pub const A3_ID_MIDI_TIMING_CLOCK: u16 = 0x100;
pub const A3_ID_MIDI_VOICE_BASE: u16 = 0x101;
pub const A3_ID_MIDI_REAL_TIME: u16 = 0x140;

pub const A3_ID_ADMIN_WIRES_BASE: u16 = 0x680;

pub const A3_ID_MISSION_CONTROL: u16 = 0x700;
pub const A3_ID_IM_BASE: u16 = 0x700;

// Message opcodes //////////////////////////////////

/* MIDI channel voice messages */
pub const A3_VOICE_MSG_SET_NOTE: u8 = 0x07;
pub const A3_VOICE_MSG_GATE_OFF: u8 = 0x08;
pub const A3_VOICE_MSG_GATE_ON: u8 = 0x09;
pub const A3_VOICE_MSG_POLY_KEY_PRESSURE: u8 = 0x0A;

/* MIDI channel messages */
pub const A3_VOICE_MSG_CONTROL_CHANGE: u8 = 0x0B;
pub const A3_VOICE_MSG_PROGRAM_CHANGE: u8 = 0x0C;
pub const A3_VOICE_MSG_CHANNEL_PRESSURE: u8 = 0x0D;
pub const A3_VOICE_MSG_PITCH_BEND: u8 = 0x0E;

/* Module administration opcodes */
pub const A3_ADMIN_SIGN_IN: u8 = 0x01;
pub const A3_ADMIN_NOTIFY_ID: u8 = 0x02;
pub const A3_ADMIN_REQ_UID_CANCEL: u8 = 0x03;

/* Mission control opcodes */
pub const A3_MC_NO_OPCODE: u8 = 0x0;
pub const A3_MC_SIGN_IN: u8 = 0x01;
pub const A3_MC_ASSIGN_MODULE_ID: u8 = 0x02;
pub const A3_MC_PING: u8 = 0x03;
pub const A3_MC_REQUEST_NAME: u8 = 0x04;
pub const A3_MC_REQUEST_CONFIG: u8 = 0x05;
pub const A3_MC_MODIFY_CONFIG: u8 = 0x08;

/* Individual module opcodes */
pub const A3_IM_PING_REPLY: u8 = 0x01;
pub const A3_IM_ID_ASSIGN_ACK: u8 = 0x02;
// pub const A3_IM_MODIFY_CONFIG_REPLY: u8 = 0x04;

// A3 message over wire //////////////////

pub enum A3DatagramId {
    Standard(u16),
    Extended(u32),
}

pub struct A3Datagram {
    pub id: A3DatagramId,
    pub data: [u8; 8],
    pub size: usize,
}

impl A3Datagram {
    pub fn from_fdcan(frame: &FdFrame) -> Self {
        let id = match frame.id() {
            Id::Standard(id) => A3DatagramId::Standard(id.as_raw()),
            Id::Extended(id) => A3DatagramId::Extended(id.as_raw()),
        };
        let mut data = [0u8; 8];
        data[0..frame.dlc()].copy_from_slice(&frame.data()[0..frame.dlc()]);
        Self {
            id,
            data,
            size: frame.dlc(),
        }
    }
}

// Stream ////////////////////////////////

#[derive(Clone /*, IntoPrimitive , TryFromPrimitive*/)]
#[repr(u8)]
pub enum StreamStatus {
    Ready = 0x0,
    Busy = 0x1,
    NotSupported = 0x2,
    NoSuchStream = 0x3,
}

pub const A3_STREAM_PAYLOAD_SIZE: usize = 8;

// Properties /////////////////////////////////////

pub const A3_MAX_PROP_DATA_SIZE: usize = 63;

/* Common property IDs */
pub const A3_PROP_ID_MODULE_UID: u8 = 0;
pub const A3_PROP_ID_MODULE_TYPE: u8 = 1;
pub const A3_PROP_ID_NAME: u8 = 2;

pub const MAX_PROP_VECTOR_LENGTH: usize = 16;

#[allow(unused)]
#[derive(Debug, Clone)]
pub enum Value {
    U8(u8),
    U16(u16),
    U32(u32),
    Text(String<A3_MAX_PROP_DATA_SIZE>),
    Boolean(bool),
    VectorU8(Vec<u8, MAX_PROP_VECTOR_LENGTH>),
    VectorU16(Vec<u16, MAX_PROP_VECTOR_LENGTH>),
}

#[allow(unused)]
#[derive(Debug, Clone)]
pub enum ValueType {
    U8,
    U16,
    U32,
    VectorU8,
    VectorU16,
    Text,
    Boolean,
}

// Known modules /////////////////////////////////

pub const MODULE_TYPE_CV_DEPOT: u16 = 1;
pub const MODULE_TYPE_AMPS: u16 = 2;
pub const MODULE_TYPE_HUMPS: u16 = 3;

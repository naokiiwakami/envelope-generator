/*
 * definitions.h
 *
 *  Created on: Jul 28, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_ANALOG3_DEFINITIONS_H_
#define INCLUDE_ANALOG3_DEFINITIONS_H_

// ID assignments /////////////////////////////////
#define A3_ID_UNASSIGNED          0x0
#define A3_ID_MIDI_TIMING_CLOCK 0x100
#define A3_ID_MIDI_VOICE_BASE   0x101
#define A3_ID_MIDI_REAL_TIME    0x140

#define A3_ID_ADMIN_WIRES_BASE  0x680

#define A3_ID_MISSION_CONTROL   0x700
#define A3_ID_IM_BASE           0x700

#define A3_ID_INVALID      0xffffffff

// Message types //////////////////////////////////

/* MIDI channel voice messages */
#define A3_VOICE_MSG_SET_NOTE          0x07
#define A3_VOICE_MSG_GATE_OFF          0x08
#define A3_VOICE_MSG_GATE_ON           0x09
#define A3_VOICE_MSG_POLY_KEY_PRESSURE 0x0A

/* MIDI channel messages */
#define A3_VOICE_MSG_CONTROL_CHANGE    0x0B
#define A3_VOICE_MSG_PROGRAM_CHANGE    0x0C
#define A3_VOICE_MSG_CHANNEL_PRESSURE  0x0D
#define A3_VOICE_MSG_PITCH_BEND        0x0E

/* Module administration opcodes */
#define A3_ADMIN_SIGN_IN 0x01
#define A3_ADMIN_NOTIFY_ID 0x02
#define A3_ADMIN_REQ_UID_CANCEL 0x03

/* Mission control opcodes */
#define A3_MC_SIGN_IN 0x01
#define A3_MC_ASSIGN_MODULE_ID 0x02
#define A3_MC_PING 0x03
#define A3_MC_REQUEST_NAME 0x04
#define A3_MC_CONTINUE_NAME 0x05
#define A3_MC_REQUEST_CONFIG 0x06
#define A3_MC_CONTINUE_CONFIG 0x07
#define A3_MC_MODIFY_CONFIG 0x08

/* Individual module opcodes */
#define A3_IM_PING_REPLY 0x01
#define A3_IM_ID_ASSIGN_ACK 0x02

#define CAN_STD_DATA_LENGTH 8
#define A3_MAX_CONFIG_DATA_LENGTH 64

// Types of modules /////////////////////////////////

#define MODULE_TYPE_CV_DEPOT 1
#define MODULE_TYPE_AMPS 2
#define MODULE_TYPE_HUMPS 3

#endif /* INCLUDE_ANALOG3_DEFINITIONS_H_ */

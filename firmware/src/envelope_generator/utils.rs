use crate::{
    envelope_generator::definitions::OutputPolarity,
    utils::{mul_uq0_32, mul_uq8_24},
};

/// Calculates the ratio for exponential charger.
/// The input is a Q0.16 value ranging between 0 and 1.0
/// The output ia a Q0.32 growth ratio calculated for 40 kHz sampling
#[inline(always)]
pub fn calculate_charging_ratio(attack: u16, mod_amount: i16) -> u32 {
    let modulated_param = attack as i32 + mod_amount as i32;
    let attack_param = modulated_param.clamp(0, u16::MAX as i32) as u64;
    // approximately 1 + 1.5e-9 * attack_param^3
    let attack_time: u64 = 1 + ((7 * attack_param * attack_param * attack_param) >> 32);
    0xffffffff / attack_time as u32
}

/// Calculates the ratio for exponential discharger.
/// The input is a Q0.16 value ranging between 0 and 1.0
/// The output ia a Q0.32 decay/release ratio calculated for 40 kHz sampling
#[inline(always)]
pub fn calculate_discharging_ratio(decay: u16, mod_amount: i16) -> u32 {
    let modulated_param = decay as i32 + mod_amount as i32;
    let decay_param = modulated_param.clamp(0, u16::MAX as i32) as u64;
    // approximately 7 + 2.5e-9 * decay_param^3
    let decay_time = 7 + ((11 * decay_param * decay_param * decay_param) >> 32);
    0xffffffff / decay_time as u32
}

/// Calculates sustain level.
/// The input is a Q0.16 value ranging between 0 and 1.0
/// The output is a Q0.32 level also ranging between 0 and 1.0.
#[inline(always)]
pub fn calculate_sustain_level(sustain: u16, mod_amount: i16) -> u32 {
    let modulated_param = sustain as i32 + mod_amount as i32 * 2;
    let sustain_param = modulated_param.clamp(0, u16::MAX as i32) as u32;
    ((sustain_param >> 1) + 32768) * sustain_param
}

/// Calculates the ratio for linear charger.
/// The input is a Q0.16 value ranging between 0 and 1.0
/// The output ia a Q0.32 growth ratio calculated for 40 kHz sampling
#[inline(always)]
pub fn calculate_linear_charging_ratio(attack: u16, mod_amount: i16) -> u32 {
    let modulated_param = attack as i32 + mod_amount as i32;
    let attack_param = modulated_param.clamp(0, u16::MAX as i32) as u64;
    // approximately 2 + 8.5e-9 * attack_param^3
    let time_constant: u64 = 2 + ((9 * attack_param * attack_param * attack_param) >> 30);
    0xffffffff / time_constant as u32
}

/// Calculates the ratio for linear discharger.
/// The input is a Q0.16 value ranging between 0 and 1.0
/// The output ia a Q0.32 decay/release ratio calculated for 40 kHz sampling
#[inline(always)]
pub fn calculate_linear_discharging_ratio(decay: u16, mod_amount: i16) -> u32 {
    let modulated_param = decay as i32 + mod_amount as i32;
    let decay_param = modulated_param.clamp(0, u16::MAX as i32) as u64;
    // approximately 2 + 1.7e-8 * attack_param^3
    let time_constant: u64 = 2 + ((9 * decay_param * decay_param * decay_param) >> 29);
    0xffffffff / time_constant as u32
}

// Note scale calculator ////////////////////////////////////////////////////

// Q8.24 constants to calculate quadratic curve for note tracking
const A: u32 = (9u32 << 24) / 8;
const B: u32 = (3u32 << 24) / 8;
const C: u32 = (1u32 << 24) / 4;

/// Gets note scale. Input is ranging between 0 to 65535 where
/// 32768 is the mid point that gives 1.0 scale. The input 65535 gives
/// the maximum scale that is 4.0. The input 0 gives the minimum scale
/// that is 1/4.
///
/// The output is in Q8.24 format.
#[inline]
pub fn calculate_note_scale(x: u16) -> u32 {
    // t = x / 32768 in UQ8.24
    let t: u32 = (x as u32) << 9;

    // calculate 1.125 * t^2 - 0.375 * t + 0.25
    let square = mul_uq8_24(A, mul_uq8_24(t, t));
    let linear = mul_uq8_24(B, t);

    // Q8.24
    square - linear + C
}

/// Get the note tracking scale factor from note number and depth.
///
/// Args:
/// - note: Note number in range 0..128
/// - depth: Depth in UQ0.32 format that determines sensitivity of the note.
///   depth 0 takes no effect, i.e. output is always 1.0 regardless the note.
///   depth u32.MAX gives maximum scaling.
///
/// The return value is UQ8.24 scale factor.
#[inline]
pub fn note_to_scale(note: u8, depth: u32) -> u32 {
    let mut scale_input: u32 = (note as u32) << 9; // 7bit to 16 bit
    if scale_input >= 0x8000 {
        scale_input = 0x8000 + mul_uq0_32(scale_input - 0x8000, depth);
    } else {
        scale_input = 0x8000 - mul_uq0_32(0x8000 - scale_input, depth);
    }
    calculate_note_scale(scale_input as u16)
}

// UQ0.32 to DAC value converters /////////////////////////////////////////

/// Converts a UQ0.32 value of range [0..0.5) to 12-bit positive output.
/// The function does not check boundary intentionally for performance.
/// The caller should ensure the input is less than 0x80000000.
pub fn uq0_32_to_12bit_positive(value: u32, zero_point: u16) -> u16 {
    (value >> 20) as u16 + zero_point
}

/// Converts a Q0.32 value of range [0..0.5) to 12-bit negative output.
/// The function does not check boundary intentionally for performance.
/// The caller should ensure the input is less than 0x80000000.
pub fn uq0_32_to_12bit_negative(value: u32, zero_point: u16) -> u16 {
    zero_point - (value >> 20) as u16
}

/// Chooses the output converter that transforms a Q0.32 value of range [0..0.5) to 12-bit
/// value to be used for DAC.  The DAC should be able to output both polarities - output value
/// 0x800 with given u16 zero_point should give the 0V DAC output.
///
/// If the parameter `polarity` is positive, the returned function maps the input value ranging
///  0 to 0.5 to 0 to positive maximum output while the returned function for negative polarity maps
/// 0 to 0.5 input to 0 to negative maximum output.  We may want to provide both-polarities function
/// in the future, but it's not supported yet.
pub fn choose_output_converter(polarity: &OutputPolarity) -> &'static dyn Fn(u32, u16) -> u16 {
    match polarity {
        OutputPolarity::Positive => &uq0_32_to_12bit_positive,
        OutputPolarity::Negative => &uq0_32_to_12bit_negative,
    }
}

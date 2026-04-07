// Fixed point arithmetic /////////////////////////////////////////

/// Multiplies two UQ0.32 numbers and returns UQ0.32 result.
#[inline(always)]
pub fn mul_uq0_32(a: u32, b: u32) -> u32 {
    ((a as u64 * b as u64) >> 32) as u32
}

/// Calculate a UQ32.32 fraction.
#[inline(always)]
pub fn fraction_uq32_32(numerator: u32, denominator: u32) -> u64 {
    ((numerator as u64) << 32) / denominator as u64
}

/// Multiplies two UQ8.24 numbers with avoiding overflow
#[inline(always)]
pub fn mul_uq8_24(a: u32, b: u32) -> u32 {
    ((a as u64 >> 4) * (b as u64 >> 4) >> 16) as u32
}

#[inline(always)]
pub fn mul_i16_uq0_16(a: i16, b: u16) -> i16 {
    let prod = ((a as i32) * (b as i32)).clamp(i16::MIN as i32, i16::MAX as i32);
    (prod >> 16) as i16
}

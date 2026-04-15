use super::definitions::{BOTTOM, LEFT};

pub const CURVE_WIDE: i32 = 35;
pub const CURVE_NARROW: i32 = 30;

#[inline]
pub(super) fn attack_pos(attack: u16, width: i32) -> i32 {
    ((width * (distort(attack) as i32 + 1)) >> 16) + LEFT
}

#[inline]
pub(super) fn decay_pos(decay: u16, attack: i32, width: i32) -> i32 {
    ((width * (distort(decay) as i32 + 1)) >> 16) + attack
}

#[inline]
pub(super) fn sustain_pos(sustain: u16) -> i32 {
    // sustain should not drop to the bottom as we want to show the release curve
    // even at sustain = 0
    BOTTOM - ((BOTTOM * ((sustain as i32 * 3) / 4 + 16384)) >> 16)
}

#[inline]
pub(super) fn release_pos(release: u16, width: i32) -> i32 {
    125 - ((width * (distort(release) as i32 + 1)) >> 16)
}

#[inline]
pub(super) fn mirroring_pos(param: u16) -> i32 {
    let param = !param as u32;
    let pos = (param * 28) >> 16;
    pos as i32
}

#[inline]
fn distort(input: u16) -> u16 {
    let reverse = (!input) as u32;
    !(((((((reverse * reverse) >> 16) * reverse) >> 16) * reverse) >> 16) as u16)
}

#[inline]
pub fn distort2(input: u16) -> u16 {
    let reverse = (!input) as u32;
    !(((reverse * reverse) >> 16) as u16)
}

use super::{
    definitions::{BOTTOM, LEFT, RIGHT, TOP},
    in_operation_mode::InOperationMode,
};

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
fn distort(input: u16) -> u16 {
    let reverse = (!input) as u32;
    !(((((((reverse * reverse) >> 16) * reverse) >> 16) * reverse) >> 16) as u16)
}

pub async fn draw_attack<'a>(parent: &mut InOperationMode<'a>, attack: i32) {
    parent.erase_x_range(LEFT, attack).await;
    parent.draw_curve((LEFT, BOTTOM), (attack, TOP)).await;
}

pub async fn draw_decay_and_sustain<'a>(
    parent: &mut InOperationMode<'a>,
    start: (i32, i32),
    decay: i32,
    sustain_level: i32,
    sustain_end: i32,
) {
    parent.erase_x_range(start.0, sustain_end).await;
    parent.draw_curve(start, (decay, sustain_level)).await;
    parent
        .draw_line((decay, sustain_level), (sustain_end, sustain_level))
        .await;
}

pub async fn draw_release<'a>(parent: &mut InOperationMode<'a>, start: (i32, i32)) {
    parent.erase_x_range(start.0, RIGHT).await;
    parent.draw_curve(start, (RIGHT, BOTTOM)).await;
}

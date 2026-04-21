use crate::{definitions::PotKind, input_reader::PotInfo};

use super::{
    definitions::{BOTTOM, LEFT, RIGHT, TOP},
    home_page_helpers::{CURVE_NARROW, attack_pos, decay_pos, release_pos, sustain_pos},
    in_operation_mode::InOperationMode,
};

pub async fn show_home_page<'a>(parent: &mut InOperationMode<'a>) {
    // prepare parameters
    let attack = parent.eg_config.attack(0);
    let decay = parent.eg_config.decay(0);
    let sustain = parent.eg_config.sustain(0);
    let release = parent.eg_config.release(0);
    let decay0 = parent.eg_config.extra_1(0);
    let sustain0 = parent.eg_config.extra_2(0);

    parent.attack = attack_pos(attack, CURVE_NARROW);
    parent.extra_1 = decay_pos(decay0, parent.attack, CURVE_NARROW);
    parent.extra_2 = sustain_pos(sustain0);
    parent.decay = decay_pos(decay, parent.extra_1, CURVE_NARROW);
    parent.sustain = sustain_pos(sustain);
    parent.release = release_pos(release, CURVE_NARROW);

    parent.display.clear(false, false).await;

    // draw the entire curves
    let end_attack = get_end_attack(parent);
    parent.draw_curve((LEFT, BOTTOM), end_attack).await;

    let end_decay_0 = get_end_decay_0(parent);
    parent.draw_curve(end_attack, end_decay_0).await;
    draw_node(parent, end_decay_0, false).await;

    let end_decay = get_end_decay(parent);
    parent.draw_curve(end_decay_0, end_decay).await;
    draw_node(parent, end_decay, false).await;

    let end_sustain = get_end_sustain(parent);
    parent.draw_line(end_decay, end_sustain).await;
    parent.draw_curve(end_sustain, (RIGHT, BOTTOM)).await;

    parent.display.driver.flush().await;
}

pub async fn update_pot<'a>(parent: &mut InOperationMode<'a>, pot_info: PotInfo) {
    match pot_info.kind {
        PotKind::Attack => {
            let next_attack = attack_pos(pot_info.value, CURVE_NARROW);
            if next_attack == parent.attack {
                return;
            }
            let delta = next_attack - parent.attack;
            parent.extra_1 += delta;
            parent.decay += delta;
            parent.attack = next_attack;
            parent.erase_x_range(LEFT, parent.release).await;

            let end_attack = get_end_attack(parent);
            parent.draw_curve((LEFT, BOTTOM), end_attack).await;

            let end_decay_0 = get_end_decay_0(parent);
            parent.draw_curve(end_attack, end_decay_0).await;
            draw_node(parent, end_decay_0, false).await;

            let end_decay = get_end_decay(parent);
            parent.draw_curve(end_decay_0, end_decay).await;
            draw_node(parent, end_decay, false).await;

            let end_sustain = get_end_sustain(parent);
            parent.draw_line(end_decay, end_sustain).await;
            draw_release(parent, end_sustain).await;
        }
        PotKind::Extra1 => {
            let next_decay0 = decay_pos(pot_info.value, parent.attack, CURVE_NARROW);
            if next_decay0 == parent.extra_1 {
                return;
            }

            let end_decay_0 = get_end_decay_0(parent);
            draw_node(parent, end_decay_0, true).await;

            parent.decay += next_decay0 - parent.extra_1;
            parent.extra_1 = next_decay0;

            parent.erase_x_range(parent.attack, parent.decay).await;
            draw_decay_and_sustain(
                parent,
                (parent.extra_1, parent.extra_2),
                parent.decay,
                parent.sustain,
                parent.release,
            )
            .await;
            let end_attack = get_end_attack(parent);
            parent
                .draw_curve(end_attack, (parent.extra_1, parent.extra_2))
                .await;
            if parent.attack < LEFT + 2 {
                parent.draw_curve((LEFT, BOTTOM), end_attack).await;
            }
        }
        PotKind::Extra2 => {
            let next_sustain0 = sustain_pos(pot_info.value);
            if next_sustain0 == parent.extra_2 {
                return;
            }

            let end_decay_0 = get_end_decay_0(parent);
            draw_node(parent, end_decay_0, true).await;

            parent.extra_2 = next_sustain0;

            parent.erase_x_range(parent.attack, parent.decay).await;
            let end_attack = get_end_attack(parent);
            parent.draw_curve((LEFT, BOTTOM), end_attack).await;
            let end_decay_0 = get_end_decay_0(parent);
            parent.draw_curve(end_attack, end_decay_0).await;
            draw_node(parent, end_decay_0, false).await;

            let end_decay = get_end_decay(parent);
            parent.draw_curve(end_decay_0, end_decay).await;
            draw_node(parent, end_decay, false).await;
        }
        PotKind::Decay => {
            let next_decay = decay_pos(pot_info.value, parent.extra_1, CURVE_NARROW);
            if next_decay == parent.decay {
                return;
            }

            let end_decay = get_end_decay(parent);
            draw_node(parent, end_decay, true).await;

            parent.decay = next_decay;

            draw_decay_and_sustain(
                parent,
                (parent.extra_1, parent.extra_2),
                parent.decay,
                parent.sustain,
                parent.release,
            )
            .await;

            let end_attack = get_end_attack(parent);
            if parent.extra_1 < parent.attack + 2 {
                parent
                    .draw_curve(end_attack, (parent.extra_1, parent.extra_2))
                    .await;
            }
        }
        PotKind::Sustain => {
            let next_sustain = sustain_pos(pot_info.value);
            if next_sustain == parent.sustain {
                return;
            }

            let end_decay = get_end_decay(parent);
            draw_node(parent, end_decay, true).await;

            parent.sustain = next_sustain;

            draw_decay_and_sustain(
                parent,
                (parent.extra_1, parent.extra_2),
                parent.decay,
                parent.sustain,
                parent.release,
            )
            .await;

            let end_attack = get_end_attack(parent);
            if parent.extra_1 < parent.attack + 2 {
                parent
                    .draw_curve(end_attack, (parent.extra_1, parent.extra_2))
                    .await;
            }
        }
        PotKind::Release => {
            let next_release = release_pos(pot_info.value, CURVE_NARROW);
            if next_release == parent.release {
                return;
            }
            parent.release = next_release;

            parent.erase_x_range(parent.decay, RIGHT).await;
            let end_decay = get_end_decay(parent);
            let end_sustain = get_end_sustain(parent);
            parent.draw_line(end_decay, end_sustain).await;
            draw_release(parent, end_sustain).await;
            draw_node(parent, end_decay, false).await;
        }
        _ => {}
    }
    parent.display.driver.flush().await;
}

#[inline]
fn get_end_attack<'a>(parent: &mut InOperationMode<'a>) -> (i32, i32) {
    (parent.attack, TOP)
}

#[inline]
fn get_end_decay_0<'a>(parent: &mut InOperationMode<'a>) -> (i32, i32) {
    (parent.extra_1, parent.extra_2)
}

#[inline]
fn get_end_decay<'a>(parent: &mut InOperationMode<'a>) -> (i32, i32) {
    (parent.decay, parent.sustain)
}

#[inline]
fn get_end_sustain<'a>(parent: &mut InOperationMode<'a>) -> (i32, i32) {
    (parent.release, parent.sustain)
}

async fn draw_decay_and_sustain<'a>(
    parent: &mut InOperationMode<'a>,
    start: (i32, i32),
    decay: i32,
    sustain_level: i32,
    sustain_end: i32,
) {
    parent.erase_x_range(start.0, RIGHT).await;
    parent.draw_curve(start, (decay, sustain_level)).await;
    draw_node(parent, start, false).await;
    parent
        .draw_line((decay, sustain_level), (sustain_end, sustain_level))
        .await;
    draw_node(parent, (decay, sustain_level), false).await;
    draw_release(parent, (sustain_end, sustain_level)).await;
}

async fn draw_release<'a>(parent: &mut InOperationMode<'a>, start: (i32, i32)) {
    parent.draw_curve(start, (RIGHT, BOTTOM)).await;
}

async fn draw_node<'a>(parent: &mut InOperationMode<'a>, center: (i32, i32), erase: bool) {
    let diameter = 3;
    let top_left_x = center.0 - diameter as i32 / 2;
    let top_left_y = center.1 - diameter as i32 / 2;
    parent
        .display
        .driver
        .draw_circle(
            (top_left_x, top_left_y),
            diameter,
            if erase {
                parent.erase_area
            } else {
                parent.fill_area
            },
        )
        .await;
}

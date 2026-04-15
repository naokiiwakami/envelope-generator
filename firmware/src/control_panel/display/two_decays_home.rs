use crate::{definitions::PotKind, input_reader::PotInfo};

use super::{
    definitions::{BOTTOM, LEFT, RIGHT, TOP},
    home_page_helpers::{
        CURVE_NARROW, attack_pos, decay_pos, draw_attack, draw_decay_and_sustain, draw_release,
        release_pos, sustain_pos,
    },
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
    parent
        .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
        .await;
    parent
        .draw_curve((parent.attack, TOP), (parent.extra_1, parent.extra_2))
        .await;
    parent
        .draw_curve(
            (parent.extra_1, parent.extra_2),
            (parent.decay, parent.sustain),
        )
        .await;
    parent
        .draw_line(
            (parent.decay, parent.sustain),
            (parent.release, parent.sustain),
        )
        .await;
    parent
        .draw_curve((parent.release, parent.sustain), (RIGHT, BOTTOM))
        .await;

    parent.display.driver.flush().await;
}

pub async fn update_pot<'a>(parent: &mut InOperationMode<'a>, pot_info: PotInfo) {
    match pot_info.kind {
        PotKind::Attack => {
            let next_attack = attack_pos(pot_info.value, CURVE_NARROW);
            if next_attack == parent.attack {
                return;
            }
            parent.attack = next_attack;
            draw_attack(parent, parent.attack).await;
        }
        PotKind::Extra1 => {
            let next_decay0 = decay_pos(pot_info.value, parent.attack, CURVE_NARROW);
            if next_decay0 == parent.extra_1 {
                return;
            }
            parent.decay = parent.decay + next_decay0 - parent.extra_1;
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
            parent
                .draw_curve((parent.attack, TOP), (parent.extra_1, parent.extra_2))
                .await;
            if parent.attack < LEFT + 2 {
                parent
                    .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
                    .await;
            }
        }
        PotKind::Extra2 => {
            let next_sustain0 = sustain_pos(pot_info.value);
            if next_sustain0 == parent.extra_2 {
                return;
            }
            parent.extra_2 = next_sustain0;
            parent.erase_x_range(parent.attack, parent.decay).await;
            parent
                .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
                .await;
            parent
                .draw_curve((parent.attack, TOP), (parent.extra_1, parent.extra_2))
                .await;
            parent
                .draw_curve(
                    (parent.extra_1, parent.extra_2),
                    (parent.decay, parent.sustain),
                )
                .await;
        }
        PotKind::Decay => {
            let next_decay = decay_pos(pot_info.value, parent.extra_1, CURVE_NARROW);
            if next_decay == parent.decay {
                return;
            }
            parent.decay = next_decay;
            draw_decay_and_sustain(
                parent,
                (parent.extra_1, parent.extra_2),
                parent.decay,
                parent.sustain,
                parent.release,
            )
            .await;

            if parent.extra_1 < parent.attack + 2 {
                parent
                    .draw_curve((parent.attack, TOP), (parent.extra_1, parent.extra_2))
                    .await;
            }
        }
        PotKind::Sustain => {
            let next_sustain = sustain_pos(pot_info.value);
            if next_sustain == parent.sustain {
                return;
            }
            parent.sustain = next_sustain;

            draw_decay_and_sustain(
                parent,
                (parent.extra_1, parent.extra_2),
                parent.decay,
                parent.sustain,
                parent.release,
            )
            .await;

            draw_release(parent, (parent.release, parent.sustain)).await;

            if parent.extra_1 < parent.attack + 2 {
                parent
                    .draw_curve((parent.attack, TOP), (parent.extra_1, parent.extra_2))
                    .await;
            }
        }
        PotKind::Release => {
            let next_release = release_pos(pot_info.value, CURVE_NARROW);
            if next_release == parent.release {
                return;
            }
            parent.release = next_release;

            parent.erase_x_range(parent.decay, parent.release).await;
            parent
                .draw_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;

            draw_release(parent, (parent.release, parent.sustain)).await;
        }
        _ => {}
    }
    parent.display.driver.flush().await;
}

use crate::{definitions::PotKind, input_reader::PotInfo};

use super::{
    definitions::{BOTTOM, LEFT, RIGHT, TOP},
    in_operation_mode::InOperationMode,
};

pub async fn show_home_page<'a>(parent: &mut InOperationMode<'a>) {
    // prepare parameters
    let attack = parent.eg_config.attack(0);
    let decay = parent.eg_config.decay(0);
    let sustain = parent.eg_config.sustain(0);
    let release = parent.eg_config.release(0);
    let punch = parent.eg_config.extra_1(0);

    parent.attack = parent.attack_pos(attack);
    parent.decay = parent.decay_pos(decay, parent.attack);
    parent.sustain = parent.sustain_pos(sustain);
    parent.release = parent.release_pos(release);
    parent.extra_1 = punch as i32;
    parent.extra_2 = parent.eg_config.note_scaling_depth(0) as i32;

    // start drawing
    parent.display.clear(false, false).await;

    parent
        .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
        .await;
    parent
        .draw_curve((parent.attack, TOP), (parent.decay, parent.sustain))
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

    parent.draw_note_scaling_bar(parent.extra_2 as u16).await;

    parent.draw_punch(parent.extra_1 as u16).await;

    parent.display.driver.flush().await;
}

pub async fn update_pot<'a>(parent: &mut InOperationMode<'a>, pot_info: PotInfo) {
    match pot_info.kind {
        PotKind::Attack => {
            let next_attack = parent.attack_pos(pot_info.value);
            if next_attack == parent.attack {
                return;
            }
            parent.attack = next_attack;

            parent.erase_x_range(LEFT, parent.attack).await;
            parent
                .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
                .await;
        }
        PotKind::Decay => {
            let next_decay = parent.decay_pos(pot_info.value, parent.attack);
            if next_decay == parent.decay {
                return;
            }
            parent.decay = next_decay;

            parent.erase_x_range(parent.attack, parent.release).await;
            parent
                .draw_curve((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;
            parent
                .draw_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            if parent.attack < LEFT + 2 {
                parent
                    .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
                    .await;
            }
        }
        PotKind::Sustain => {
            let next_sustain = parent.sustain_pos(pot_info.value);
            if next_sustain == parent.sustain {
                return;
            }
            parent.sustain = next_sustain;

            parent.erase_x_range(parent.attack, RIGHT).await;
            parent
                .draw_curve((parent.attack, TOP), (parent.decay, parent.sustain))
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
            if parent.attack < LEFT + 2 {
                parent
                    .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
                    .await;
            }
        }
        PotKind::Release => {
            let next_release = parent.release_pos(pot_info.value);
            if next_release == parent.release {
                return;
            }
            parent.release = next_release;

            parent.erase_x_range(parent.decay, RIGHT).await;
            parent
                .draw_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .draw_curve((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;
        }
        PotKind::Extra1 => {
            let new_value = pot_info.value;
            if new_value == parent.extra_1 as u16 {
                return;
            }
            parent.extra_1 = new_value as i32;
            parent.draw_punch(new_value).await;
        }
        PotKind::Extra2 => {
            parent
                .draw_note_scaling_bar(parent.eg_config.note_scaling_depth(0))
                .await;
        }
        _ => {}
    }
    parent.display.driver.flush().await;
}

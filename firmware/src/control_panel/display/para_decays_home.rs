use crate::{definitions::PotKind, input_reader::PotInfo};

use super::{
    definitions::{BOTTOM, LEFT, N_BOTTOM, RIGHT, TOP},
    in_operation_mode::InOperationMode,
};

pub async fn show_home_page<'a>(parent: &mut InOperationMode<'a>) {
    parent.display.clear(false, false).await;

    // attack curve
    parent
        .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
        .await;
    parent
        .draw_curve((LEFT, BOTTOM), (parent.attack, N_BOTTOM + 1))
        .await;
    // decay curve
    parent
        .draw_curve((parent.attack, TOP), (parent.decay, parent.sustain))
        .await;
    parent
        .draw_line(
            (parent.decay, parent.sustain),
            (parent.release, parent.sustain),
        )
        .await;
    // strum curve
    let n_sustain = N_BOTTOM - parent.sustain;
    parent
        .draw_curve((parent.attack, N_BOTTOM), (parent.extra_1, n_sustain))
        .await;
    parent
        .draw_line((parent.extra_1, n_sustain), (parent.release, n_sustain))
        .await;

    // release curve
    parent
        .draw_curve((parent.release, parent.sustain), (RIGHT, BOTTOM))
        .await;

    parent
        .draw_curve((parent.release, n_sustain), (RIGHT, BOTTOM))
        .await;

    parent.display.driver.flush().await;
}

pub async fn update_pot<'a>(parent: &mut InOperationMode<'a>, pot_info: PotInfo) {
    match pot_info.kind {
        PotKind::Attack => {
            let next_attack = parent.attack_pos(pot_info.value);
            if next_attack == parent.attack {
                return;
            }

            parent
                .display
                .clear_rectangle(
                    (TOP, LEFT),
                    parent.attack as u32 + 1,
                    N_BOTTOM as u32 + 1,
                    false,
                )
                .await;
            parent.attack = next_attack;
            parent
                .draw_curve((LEFT, BOTTOM), (parent.attack, TOP))
                .await;
            parent
                .draw_curve((LEFT, BOTTOM), (parent.attack, BOTTOM * 2))
                .await;
        }
        PotKind::Decay => {
            let next_decay = parent.decay_pos(pot_info.value, parent.attack);
            if next_decay == parent.decay {
                return;
            }

            parent.decay = next_decay;

            /*
            parent
                .display
                .clear_rectangle(
                    (TOP, parent.attack),
                    (parent.release - parent.attack) as u32 + 1,
                    BOTTOM as u32 + 1,
                    false,
                )
                .await;
            */
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
        PotKind::Extra1 => {}
        PotKind::Extra2 => {}
        _ => {}
    }
    parent.display.driver.flush().await;
}

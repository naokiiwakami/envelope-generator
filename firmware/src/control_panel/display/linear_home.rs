use crate::{definitions::PotKind, input_reader::PotInfo};

use super::{
    definitions::{BOTTOM, LEFT, RIGHT, TOP},
    in_operation_mode::InOperationMode,
};

pub async fn show_home_page<'a>(parent: &mut InOperationMode<'a>) {
    parent.display.clear(false, false).await;

    parent.draw_line((LEFT, BOTTOM), (parent.attack, TOP)).await;
    parent
        .draw_line((parent.attack, TOP), (parent.decay, parent.sustain))
        .await;
    parent
        .draw_line(
            (parent.decay, parent.sustain),
            (parent.release, parent.sustain),
        )
        .await;
    parent
        .draw_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
        .await;

    parent
        .draw_note_scaling_bar(parent.eg_config.note_scaling_depth(0))
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
                .erase_line((LEFT, BOTTOM), (parent.attack, TOP))
                .await;
            parent
                .erase_line((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;

            parent.attack = next_attack;

            parent.draw_line((LEFT, BOTTOM), (parent.attack, TOP)).await;
            parent
                .draw_line((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;
        }
        PotKind::Decay => {
            let next_decay = parent.decay_pos(pot_info.value, parent.attack);
            if next_decay == parent.decay {
                return;
            }

            parent
                .erase_line((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;
            parent
                .erase_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .erase_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;

            parent.decay = next_decay;

            parent.draw_line((LEFT, BOTTOM), (parent.attack, TOP)).await;
            parent
                .draw_line((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;
            parent
                .draw_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .draw_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;
        }
        PotKind::Sustain => {
            let next_sustain = parent.sustain_pos(pot_info.value);
            if next_sustain == parent.sustain {
                return;
            }

            parent
                .erase_line((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;
            parent
                .erase_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .erase_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;

            parent.sustain = next_sustain;

            parent
                .draw_line((parent.attack, TOP), (parent.decay, parent.sustain))
                .await;
            parent
                .draw_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .draw_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;
        }
        PotKind::Release => {
            let next_release = parent.release_pos(pot_info.value);
            if next_release == parent.release {
                return;
            }

            parent
                .erase_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .erase_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;

            parent.release = next_release;

            parent
                .draw_line(
                    (parent.decay, parent.sustain),
                    (parent.release, parent.sustain),
                )
                .await;
            parent
                .draw_line((parent.release, parent.sustain), (RIGHT, BOTTOM))
                .await;
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

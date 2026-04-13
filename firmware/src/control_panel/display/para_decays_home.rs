use embedded_graphics::{pixelcolor::BinaryColor, primitives::PrimitiveStyleBuilder};

use crate::{definitions::PotKind, input_reader::PotInfo};

use super::{
    definitions::{BOTTOM, LEFT, N_BOTTOM, RIGHT, TOP},
    in_operation_mode::InOperationMode,
};

const LEFT_MARGIN: i32 = 8;

pub async fn show_home_page<'a>(parent: &mut InOperationMode<'a>) {
    // prepare parameters
    let attack = parent.eg_config.attack(0);
    let decay = parent.eg_config.decay(0);
    let sustain = parent.eg_config.sustain(0);
    let release = parent.eg_config.release(0);
    let extra_1 = parent.eg_config.extra_1(0);
    let extra_2 = parent.eg_config.extra_2(0);

    parent.attack = parent.attack_pos(attack) + LEFT_MARGIN;
    parent.decay = parent.decay_pos(decay, parent.attack);
    parent.sustain = parent.sustain_pos(sustain);
    parent.release = parent.release_pos(release);
    parent.extra_1 = parent.decay_pos(extra_1, parent.attack);
    parent.extra_2 = parent.mirroring_pos(extra_2);

    parent.display.clear(false, false).await;

    // let center: i32 = parent.extra_2;
    let center = BOTTOM;

    // helper closure to transform points
    let map = |(x, y): (i32, i32)| (x, remap_y(y, center));

    // attack curve
    parent
        .draw_curve(map((LEFT + LEFT_MARGIN, BOTTOM)), map((parent.attack, TOP)))
        .await;
    parent
        .draw_curve(
            map((LEFT + LEFT_MARGIN, BOTTOM)),
            map((parent.attack, N_BOTTOM)),
        )
        .await;

    // decay curve
    parent
        .draw_curve(
            map((parent.attack, TOP)),
            map((parent.decay, parent.sustain)),
        )
        .await;
    parent
        .draw_line(
            map((parent.decay, parent.sustain)),
            map((parent.release, parent.sustain)),
        )
        .await;

    // strum curve
    let n_sustain = N_BOTTOM - parent.sustain;
    parent
        .draw_curve(
            map((parent.attack, N_BOTTOM)),
            map((parent.extra_1, n_sustain)),
        )
        .await;
    parent
        .draw_line(
            map((parent.extra_1, n_sustain)),
            map((parent.release, n_sustain)),
        )
        .await;

    // release curve
    parent
        .draw_curve(map((parent.release, parent.sustain)), map((RIGHT, BOTTOM)))
        .await;

    parent
        .draw_curve(map((parent.release, n_sustain)), map((RIGHT, BOTTOM)))
        .await;

    parent
        .display
        .driver
        .draw_line(
            (LEFT + 1, parent.extra_2),
            (LEFT + 1, parent.extra_2 + BOTTOM),
            PrimitiveStyleBuilder::new()
                .stroke_width(3)
                .stroke_color(BinaryColor::On)
                .build(),
        )
        .await;

    parent.display.driver.flush().await;
}

pub async fn update_pot<'a>(parent: &mut InOperationMode<'a>, pot_info: PotInfo) {
    // let center: i32 = parent.extra_2;
    let center = BOTTOM;

    let map = |(x, y): (i32, i32)| (x, remap_y(y, center));

    // helper for rectangle Y transform
    let map_y = |y: i32| remap_y(y, center);

    match pot_info.kind {
        PotKind::Attack => {
            let next_attack = parent.attack_pos(pot_info.value) + LEFT_MARGIN;
            if next_attack == parent.attack {
                return;
            }

            let n_sustain = N_BOTTOM - parent.sustain;

            parent
                .display
                .clear_rectangle(
                    (LEFT + LEFT_MARGIN, map_y(TOP)),
                    (parent.release + 1 - LEFT_MARGIN) as u32,
                    (map_y(N_BOTTOM) - map_y(TOP) + 1) as u32,
                    false,
                )
                .await;

            let delta = next_attack - parent.attack;
            parent.decay += delta;
            parent.extra_1 += delta;
            parent.attack = next_attack;

            parent
                .draw_curve(map((LEFT + LEFT_MARGIN, BOTTOM)), map((parent.attack, TOP)))
                .await;
            parent
                .draw_curve(
                    map((LEFT + LEFT_MARGIN, BOTTOM)),
                    map((parent.attack, N_BOTTOM)),
                )
                .await;
            parent
                .draw_curve(
                    map((parent.attack, TOP)),
                    map((parent.decay, parent.sustain)),
                )
                .await;
            parent
                .draw_line(
                    map((parent.decay, parent.sustain)),
                    map((parent.release, parent.sustain)),
                )
                .await;
            parent
                .draw_curve(
                    map((parent.attack, N_BOTTOM)),
                    map((parent.extra_1, n_sustain)),
                )
                .await;
            parent
                .draw_line(
                    map((parent.extra_1, n_sustain)),
                    map((parent.release, n_sustain)),
                )
                .await;
        }

        PotKind::Decay => {
            let next_decay = parent.decay_pos(pot_info.value, parent.attack);
            if next_decay == parent.decay {
                return;
            }

            parent.decay = next_decay;

            parent
                .display
                .clear_rectangle(
                    (parent.attack, map_y(TOP)),
                    (parent.release - parent.attack) as u32,
                    (map_y(BOTTOM) - map_y(TOP) + 1) as u32,
                    false,
                )
                .await;

            parent
                .draw_curve(
                    map((parent.attack, TOP)),
                    map((parent.decay, parent.sustain)),
                )
                .await;
            parent
                .draw_line(
                    map((parent.decay, parent.sustain)),
                    map((parent.release, parent.sustain)),
                )
                .await;

            if parent.attack < LEFT + LEFT_MARGIN + 3 {
                parent
                    .draw_curve(map((LEFT + LEFT_MARGIN, BOTTOM)), map((parent.attack, TOP)))
                    .await;
            }
        }

        PotKind::Sustain => {
            let next_sustain = parent.sustain_pos(pot_info.value);
            if next_sustain == parent.sustain {
                return;
            }
            parent.sustain = next_sustain;

            parent
                .display
                .clear_rectangle(
                    (parent.attack, map_y(TOP)),
                    (RIGHT - parent.attack) as u32,
                    (map_y(N_BOTTOM) - map_y(TOP) + 1) as u32,
                    false,
                )
                .await;

            // top
            parent
                .draw_curve(
                    map((parent.attack, TOP)),
                    map((parent.decay, parent.sustain)),
                )
                .await;
            parent
                .draw_line(
                    map((parent.decay, parent.sustain)),
                    map((parent.release, parent.sustain)),
                )
                .await;
            parent
                .draw_curve(map((parent.release, parent.sustain)), map((RIGHT, BOTTOM)))
                .await;

            // bottom
            let n_sustain = N_BOTTOM - parent.sustain;
            parent
                .draw_curve(
                    map((parent.attack, N_BOTTOM)),
                    map((parent.extra_1, n_sustain)),
                )
                .await;
            parent
                .draw_line(
                    map((parent.extra_1, n_sustain)),
                    map((parent.release, n_sustain)),
                )
                .await;
            parent
                .draw_curve(map((parent.release, n_sustain)), map((RIGHT, BOTTOM)))
                .await;

            if parent.attack < LEFT + LEFT_MARGIN + 3 {
                parent
                    .draw_curve(map((LEFT + LEFT_MARGIN, BOTTOM)), map((parent.attack, TOP)))
                    .await;
                parent
                    .draw_curve(
                        map((LEFT + LEFT_MARGIN, BOTTOM)),
                        map((parent.attack, N_BOTTOM)),
                    )
                    .await;
            }
        }

        PotKind::Release => {
            let next_release = parent.release_pos(pot_info.value);
            if next_release == parent.release {
                return;
            }
            parent.release = next_release;

            let n_sustain = N_BOTTOM - parent.sustain;

            parent
                .display
                .clear_rectangle(
                    (parent.decay, map_y(TOP)),
                    (RIGHT - parent.decay + 1) as u32,
                    (map_y(N_BOTTOM) - map_y(TOP) + 1) as u32,
                    false,
                )
                .await;

            parent
                .draw_line(
                    map((parent.decay, parent.sustain)),
                    map((parent.release, parent.sustain)),
                )
                .await;
            parent
                .draw_curve(map((parent.release, parent.sustain)), map((RIGHT, BOTTOM)))
                .await;
            parent
                .draw_line(
                    map((parent.extra_1, n_sustain)),
                    map((parent.release, n_sustain)),
                )
                .await;
            parent
                .draw_curve(map((parent.release, n_sustain)), map((RIGHT, BOTTOM)))
                .await;
        }

        PotKind::Extra1 => {
            let next_extra_1 = parent.decay_pos(pot_info.value, parent.attack);
            if next_extra_1 == parent.extra_1 {
                return;
            }
            parent.extra_1 = next_extra_1;

            let n_sustain = N_BOTTOM - parent.sustain;

            parent
                .display
                .clear_rectangle(
                    (parent.attack, map_y(BOTTOM)),
                    (parent.release - parent.attack + 1) as u32,
                    (map_y(N_BOTTOM) - map_y(BOTTOM) + 1) as u32,
                    false,
                )
                .await;

            parent
                .draw_curve(
                    map((parent.attack, N_BOTTOM)),
                    map((parent.extra_1, n_sustain)),
                )
                .await;
            parent
                .draw_line(
                    map((parent.extra_1, n_sustain)),
                    map((parent.release, n_sustain)),
                )
                .await;

            if parent.attack < LEFT + LEFT_MARGIN + 3 {
                parent
                    .draw_curve(
                        map((LEFT + LEFT_MARGIN, BOTTOM)),
                        map((parent.attack, N_BOTTOM)),
                    )
                    .await;
            }
        }

        PotKind::Extra2 => {
            let next_balance_pos = parent.mirroring_pos(pot_info.value);
            if next_balance_pos == parent.extra_2 {
                return;
            }
            parent
                .display
                .driver
                .draw_line(
                    (LEFT + 1, parent.extra_2),
                    (LEFT + 1, parent.extra_2 + BOTTOM),
                    PrimitiveStyleBuilder::new()
                        .stroke_width(3)
                        .stroke_color(BinaryColor::Off)
                        .build(),
                )
                .await;

            parent.extra_2 = next_balance_pos;

            parent
                .display
                .driver
                .draw_line(
                    (LEFT + 1, parent.extra_2),
                    (LEFT + 1, parent.extra_2 + BOTTOM),
                    PrimitiveStyleBuilder::new()
                        .stroke_width(3)
                        .stroke_color(BinaryColor::On)
                        .build(),
                )
                .await;
        }
        _ => {}
    }

    // parent.draw_line((LEFT, center), (RIGHT, center)).await;
    parent.display.driver.flush().await;
}

#[inline]
fn remap_y(y: i32, center: i32) -> i32 {
    if y <= BOTTOM {
        // Upper half: BOTTOM..TOP  → center..TOP
        center + (y - BOTTOM) * (TOP - center) / (TOP - BOTTOM)
    } else {
        // Lower half: BOTTOM..N_BOTTOM → center..N_BOTTOM
        center + (y - BOTTOM) * (N_BOTTOM - center) / (N_BOTTOM - BOTTOM)
    }
}

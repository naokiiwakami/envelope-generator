use crate::{definitions::PotKind, input_reader::PotInfo};

use super::in_operation_mode::InOperationMode;

pub struct LinearHome {
    attack: i32,
    decay: i32,
    sustain: i32,
    release: i32,
}

impl LinearHome {
    pub fn new() -> Self {
        Self {
            attack: 0,
            decay: 0,
            sustain: 0,
            release: 0,
        }
    }

    pub async fn show_home_page<'a>(
        &mut self,
        parent: &mut InOperationMode<'a>,
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        _extra_1: u16,
        _extra_2: u16,
    ) {
        parent.display.clear(false, false).await;

        self.attack = parent.attack_pos(attack);
        self.decay = parent.decay_pos(decay, self.attack);
        self.sustain = parent.sustain_pos(sustain);
        self.release = parent.release_pos(release);

        parent.draw_line((LEFT, BOTTOM), (self.attack, TOP)).await;
        parent
            .draw_line((self.attack, TOP), (self.decay, self.sustain))
            .await;
        parent
            .draw_line((self.decay, self.sustain), (self.release, self.sustain))
            .await;
        parent
            .draw_line((self.release, self.sustain), (RIGHT, BOTTOM))
            .await;

        parent
            .draw_note_scaling_bar(parent.eg_config.note_scaling_depth(0))
            .await;

        parent.display.driver.flush().await;
    }

    pub async fn update_pot<'a>(&mut self, parent: &mut InOperationMode<'a>, pot_info: PotInfo) {
        match pot_info.kind {
            PotKind::Attack => {
                let next_attack = parent.attack_pos(pot_info.value);
                if next_attack == self.attack {
                    return;
                }

                parent.erase_line((LEFT, BOTTOM), (self.attack, TOP)).await;
                parent
                    .erase_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;

                self.attack = next_attack;

                parent.draw_line((LEFT, BOTTOM), (self.attack, TOP)).await;
                parent
                    .draw_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
            }
            PotKind::Decay => {
                let next_decay = parent.decay_pos(pot_info.value, self.attack);
                if next_decay == self.decay {
                    return;
                }

                parent
                    .erase_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                parent
                    .erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                parent.erase_line((self.release, self.sustain), (RIGHT, BOTTOM)).await;

                self.decay = next_decay;

                parent.draw_line((LEFT, BOTTOM), (self.attack, TOP)).await;
                parent
                    .draw_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                parent
                    .draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                parent.draw_line((self.release, self.sustain), (RIGHT, BOTTOM)).await;
            }
            PotKind::Sustain => {
                let next_sustain = parent.sustain_pos(pot_info.value);
                if next_sustain == self.sustain {
                    return;
                }

                parent
                    .erase_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                parent
                    .erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                parent.erase_line((self.release, self.sustain), (RIGHT, BOTTOM)).await;

                self.sustain = next_sustain;

                parent
                    .draw_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                parent
                    .draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                parent.draw_line((self.release, self.sustain), (RIGHT, BOTTOM)).await;
            }
            PotKind::Release => {
                let next_release = parent.release_pos(pot_info.value);
                if next_release == self.release {
                    return;
                }

                parent
                    .erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                parent.erase_line((self.release, self.sustain), (RIGHT, BOTTOM)).await;

                self.release = next_release;

                parent
                    .draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                parent.draw_line((self.release, self.sustain), (RIGHT, BOTTOM)).await;
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
}

const LEFT: i32 = 0;
const RIGHT: i32 = 127;
const TOP: i32 = 0;
const BOTTOM: i32 = 28;

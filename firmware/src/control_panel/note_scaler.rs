use defmt;
use embassy_stm32::gpio::Level;
use embassy_time::{Instant, Timer};

use super::{ControlPanel, DisplayRequest};

const NOTE_SCALER_SAMPLE_PERIOD_MS: u64 = 10;
const NOTE_SCALER_VELOCITY_WINDOW_MS: u64 = 40;
const NOTE_SCALER_HISTORY_LEN: usize =
    (NOTE_SCALER_VELOCITY_WINDOW_MS / NOTE_SCALER_SAMPLE_PERIOD_MS) as usize;

pub struct NoteScaler<'a> {
    control_panel: &'a mut ControlPanel,
    current_depth: u16,
}

impl<'a> NoteScaler<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        let current_depth = control_panel.eg_config.note_scaling_depth(0);
        Self {
            control_panel,
            current_depth,
        }
    }

    pub async fn execute(&mut self) {
        defmt::debug!("NoteScaler.execute()");
        self.control_panel.smash_counter();
        self.control_panel.ind_red.set_high();
        self.control_panel.ind_green.set_high();

        defmt::debug!("sending UpdateNoteScaling");
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::UpdateNoteScaling {
                depth: self.current_depth,
            })
            .await;

        let mut last_raw = self.control_panel.encoder_last_raw;
        let mut velocity_history = [0i16; NOTE_SCALER_HISTORY_LEN];
        let mut velocity_history_index = 0;
        let mut velocity_history_len = 0;
        let mut accumulated_counts: i16 = 0;

        loop {
            Timer::after_millis(NOTE_SCALER_SAMPLE_PERIOD_MS).await;

            let button_level = self.control_panel.button.get_level();
            if button_level == Level::Low {
                if self.control_panel.button_pressed_at.is_none() {
                    self.control_panel.button_pressed_at = Some(Instant::now());
                }
                continue;
            }

            if self.control_panel.button_pressed_at.is_some() {
                self.control_panel.button_pressed_at = None;
                break;
            }

            let raw = self.control_panel.encoder.count() as i16;
            let count_delta = raw - last_raw;
            let velocity_step = if count_delta.abs() < 4 && count_delta != 0 {
                count_delta.signum()
            } else {
                count_delta / 4
            };
            last_raw = raw;
            if velocity_step == 0 {
                continue;
            }

            accumulated_counts += count_delta;
            velocity_history[velocity_history_index] = velocity_step;
            velocity_history_index = (velocity_history_index + 1) % NOTE_SCALER_HISTORY_LEN;
            if velocity_history_len < NOTE_SCALER_HISTORY_LEN {
                velocity_history_len += 1;
            }

            let sum: i32 = velocity_history[..velocity_history_len]
                .iter()
                .map(|&v| v as i32)
                .sum();
            let avg_velocity = if velocity_history_len > 0 {
                let mut average =
                    (sum.abs() + velocity_history_len as i32 / 2) / velocity_history_len as i32;
                if average == 0 && sum != 0 {
                    average = 1;
                }
                sum.signum() * average
            } else {
                0
            };

            let clicks = accumulated_counts / 4;
            if clicks == 0 {
                continue;
            }
            accumulated_counts -= clicks * 4;

            let velocity_magnitude = avg_velocity.abs() as u16;
            let scaled_speed = velocity_magnitude.saturating_mul(velocity_magnitude) - 1;
            let depth_delta = 16i32 * clicks as i32 * (1 + scaled_speed as i32 * 4);

            let new_depth =
                (self.current_depth as i32 + depth_delta).clamp(0, u16::MAX as i32) as u16;
            if new_depth != self.current_depth {
                self.current_depth = new_depth;
                self.control_panel
                    .display_request_sender
                    .send(DisplayRequest::UpdateNoteScaling {
                        depth: self.current_depth,
                    })
                    .await;
            }

            self.control_panel.encoder_last_raw = raw;
        }

        self.control_panel.ind_red.set_low();
        self.control_panel.ind_green.set_low();
    }
}

use defmt;
use embassy_stm32::gpio::Level;
use embassy_time::{Duration, Instant, Timer};

use crate::envelope_generator::{EgRequest, OutputPolarity};

use super::{ControlPanel, DisplayRequest, POLARITY_CHANGE_TARGET_ITEMS};

pub struct PolarityChanger<'a> {
    control_panel: &'a mut ControlPanel,
    targets: u8,
    polarity_1: OutputPolarity,
    polarity_2: OutputPolarity,
}

impl<'a> PolarityChanger<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        let polarity_1 = control_panel.eg_config.out_polarity(0);
        let polarity_2 = control_panel.eg_config.out_polarity(1);

        Self {
            control_panel,
            targets: POLARITY_CHANGE_TARGET_ITEMS[0].selection,
            polarity_1,
            polarity_2,
        }
    }

    pub async fn execute(&mut self) {
        defmt::debug!("PolarityChanger.execute()");
        self.choose_change_target().await;
        self.change_polarity().await;
    }

    async fn choose_change_target(&mut self) {
        self.control_panel.smash_counter();
        self.control_panel.ind_red.set_low();
        self.control_panel.ind_green.set_low();

        let mut current_index = 0;
        let mut toggle_time = Instant::now().saturating_add(Duration::from_millis(500));
        self.targets = POLARITY_CHANGE_TARGET_ITEMS[current_index].selection;
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::SetPolarityChangeTargets {
                targets: self.targets,
            })
            .await;

        loop {
            Timer::after_millis(10).await;
            let button_level = self.control_panel.button.get_level();
            if button_level == Level::Low {
                if self.control_panel.button_pressed_at.is_none() {
                    self.control_panel.button_pressed_at = Some(Instant::now());
                }
            } else if self.control_panel.button_pressed_at.is_some() {
                self.control_panel.button_pressed_at = None;
                self.targets = POLARITY_CHANGE_TARGET_ITEMS[current_index].selection;
                self.control_panel
                    .display_request_sender
                    .send(DisplayRequest::SetPolarityChangeTargets {
                        targets: self.targets,
                    })
                    .await;
                self.control_panel.ind_red.set_low();
                self.control_panel.ind_green.set_low();
                return;
            } else {
                let raw = self.control_panel.encoder.count() as i16;
                let delta = (raw - self.control_panel.encoder_last_raw) / 4;
                if delta != 0 {
                    let mut next_index = current_index as i32 + delta as i32;
                    let len = POLARITY_CHANGE_TARGET_ITEMS.len() as i32;
                    next_index %= len;
                    if next_index < 0 {
                        next_index += len;
                    }
                    current_index = next_index as usize;
                    self.targets = POLARITY_CHANGE_TARGET_ITEMS[current_index].selection;
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::SetPolarityChangeTargets {
                            targets: self.targets,
                        })
                        .await;
                    self.control_panel.encoder_last_raw = raw;
                }
            }

            if Instant::now().ge(&toggle_time) {
                if self.control_panel.ind_red.is_set_low() {
                    self.control_panel.ind_red.set_high();
                    self.control_panel.ind_green.set_high();
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::SetPolarityChangeTargets {
                            targets: self.targets,
                        })
                        .await;
                } else {
                    self.control_panel.ind_red.set_low();
                    self.control_panel.ind_green.set_low();
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::SetPolarityChangeTargets { targets: 0 })
                        .await;
                }
                toggle_time = toggle_time.saturating_add(Duration::from_millis(500));
            }
        }
    }

    async fn change_polarity(&mut self) {
        self.control_panel.smash_counter();
        self.control_panel.ind_red.set_low();
        self.control_panel.ind_green.set_low();

        let mut toggle_time = Instant::now().saturating_add(Duration::from_millis(500));
        let mut should_draw = true;
        let voice_1 = (self.targets & 0x1) != 0;
        let voice_2 = (self.targets & 0x2) != 0;

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::UpdatePolarities {
                targets: self.targets,
                polarity_1: self.polarity_1,
                polarity_2: self.polarity_2,
                is_draw: true,
            })
            .await;

        loop {
            Timer::after_millis(10).await;
            let button_level = self.control_panel.button.get_level();
            if button_level == Level::Low {
                if self.control_panel.button_pressed_at.is_none() {
                    self.control_panel.button_pressed_at = Some(Instant::now());
                }
            } else if self.control_panel.button_pressed_at.is_some() {
                self.control_panel.button_pressed_at = None;
                self.control_panel.ind_red.set_low();
                self.control_panel.ind_green.set_low();
                self.control_panel
                    .eg_request_sender
                    .send(EgRequest::ChangeOutputPolarities {
                        polarity_1: self.polarity_1,
                        polarity_2: self.polarity_2,
                        send_notif: false,
                    })
                    .await;
                self.control_panel
                    .show_polarity(self.polarity_1, self.polarity_2)
                    .await;
                return;
            } else {
                let raw = self.control_panel.encoder.count() as i16;
                let delta = (raw - self.control_panel.encoder_last_raw) / 4;
                if delta != 0 {
                    if voice_1 {
                        self.polarity_1 = self.next_polarity(self.polarity_1, delta);
                    }
                    if voice_2 {
                        self.polarity_2 = self.next_polarity(self.polarity_2, delta);
                    }
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::UpdatePolarities {
                            targets: self.targets,
                            polarity_1: self.polarity_1,
                            polarity_2: self.polarity_2,
                            is_draw: true,
                        })
                        .await;
                    self.control_panel.encoder_last_raw = raw;
                    should_draw = true;
                }
            }

            if Instant::now().ge(&toggle_time) {
                if should_draw {
                    self.control_panel.ind_red.set_high();
                    self.control_panel.ind_green.set_high();
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::UpdatePolarities {
                            targets: self.targets,
                            polarity_1: self.polarity_1,
                            polarity_2: self.polarity_2,
                            is_draw: true,
                        })
                        .await;
                } else {
                    self.control_panel.ind_red.set_low();
                    self.control_panel.ind_green.set_low();
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::UpdatePolarities {
                            targets: self.targets,
                            polarity_1: OutputPolarity::Positive,
                            polarity_2: OutputPolarity::Positive,
                            is_draw: false,
                        })
                        .await;
                }
                should_draw = !should_draw;
                toggle_time = toggle_time.saturating_add(Duration::from_millis(500));
            }
        }
    }

    fn next_polarity(&self, current: OutputPolarity, delta: i16) -> OutputPolarity {
        let new_value = current as i8 + (delta % 2) as i8;
        let new_value = ((new_value % 2) + 2) % 2;
        OutputPolarity::try_from(new_value as u8).unwrap()
    }
}

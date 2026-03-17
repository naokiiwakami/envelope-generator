use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::prelude::Point;

use crate::{
    envelope_generator::{EgRequest, EngineType},
    input_reader::get_reader_info_receiver,
};

use super::{ControlPanel, DisplayRequest, display::FontSize};

pub struct Diagnoser<'a> {
    control_panel: &'a mut ControlPanel,
}

impl<'a> Diagnoser<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        Self { control_panel }
    }

    pub async fn diagnose(&mut self) {
        self.control_panel
            .display_text(
                "DIAGNOSING...",
                true,
                true,
                FontSize::Medium,
                Point::new(10, 25),
            )
            .await;
        self.control_panel
            .eg_request_sender
            .send(EgRequest::SwitchEngine(EngineType::Diag))
            .await;
        crate::analog3::trigger_diagnose().await;
        Timer::after_millis(6500).await;
        self.control_panel.blink_leds().await;
        Timer::after_millis(500).await;
        self.diagnose_patch_controller().await;
        self.diagnose_pots().await;
        self.diagnose_cv().await;
        self.control_panel.switch_engine_type().await;
        self.control_panel
            .display_text("DONE!", true, true, FontSize::Medium, Point::new(40, 25))
            .await;
        Timer::after_secs(1).await;
        self.control_panel.show_initial_screen().await;
    }

    async fn diagnose_patch_controller(&mut self) {
        crate::patch_controller::diagnose_leds().await;
        self.control_panel
            .display_text(
                "Press button",
                false,
                true,
                FontSize::Medium,
                Point::new(17, 25),
            )
            .await;
        crate::patch_controller::diagnose_button().await;
    }

    async fn diagnose_pots(&mut self) {
        let mut receiver = get_reader_info_receiver().await;
        let mut now = Instant::now();
        self.control_panel.button_pressed_at = if self.control_panel.button.is_low() {
            Some(now.clone())
        } else {
            None
        };
        let mut next = now.saturating_add(Duration::from_millis(10));
        let mut wakeup_count = 0;

        let mut first_one_covered = false;
        loop {
            let sleep_time = next.duration_since(now);
            match select(receiver.changed(), Timer::after(sleep_time)).await {
                Either::First(reader_info) => {
                    let pot_info = reader_info.pot_info;
                    let index = pot_info.kind.clone() as usize;
                    self.control_panel
                        .display_request_sender
                        .send(DisplayRequest::UpdatePotValue { pot_info })
                        .await;
                    if index == 0 {
                        first_one_covered = true;
                    } else if index == 7 && first_one_covered {
                        self.control_panel
                            .display_request_sender
                            .send(DisplayRequest::Flush)
                            .await;
                    }
                }
                Either::Second(()) => {}
            }
            now = Instant::now();
            if now.ge(&next) {
                if self.control_panel.button.is_low() {
                    self.control_panel.button_pressed_at.replace(now.clone());
                } else {
                    let to_exit = self.control_panel.button_pressed_at.is_some();
                    self.control_panel.button_pressed_at = None;
                    if to_exit {
                        self.control_panel.ind_red.set_low();
                        self.control_panel.ind_green.set_low();
                        break;
                    }
                }
                if wakeup_count % 50 == 0 {
                    self.control_panel.ind_red.toggle();
                    self.control_panel.ind_green.toggle();
                }
                wakeup_count += 1;
                while now.ge(&next) {
                    next = next.saturating_add(Duration::from_millis(10));
                }
            }
        }
    }

    async fn diagnose_cv(&mut self) {
        let mut receiver = get_reader_info_receiver().await;
        let mut now = Instant::now();
        self.control_panel.button_pressed_at = if self.control_panel.button.is_low() {
            Some(now.clone())
        } else {
            None
        };
        let mut next = now.saturating_add(Duration::from_millis(10));
        let mut wakeup_count = 0;

        let mut last_update_time = Instant::now();
        loop {
            let sleep_time = next.duration_since(now);
            match select(receiver.changed(), Timer::after(sleep_time)).await {
                Either::First(reader_info) => {
                    let cv_info = reader_info.cv_info;
                    if last_update_time.elapsed().as_millis() >= 30 {
                        self.control_panel
                            .display_request_sender
                            .send(DisplayRequest::UpdateCvValues { cv_info })
                            .await;
                        last_update_time = Instant::now();
                    }
                }
                Either::Second(()) => {}
            }
            now = Instant::now();
            if now.ge(&next) {
                if self.control_panel.button.is_low() {
                    self.control_panel.button_pressed_at.replace(now.clone());
                } else {
                    let to_exit = self.control_panel.button_pressed_at.is_some();
                    self.control_panel.button_pressed_at = None;
                    if to_exit {
                        self.control_panel.ind_red.set_low();
                        self.control_panel.ind_green.set_low();
                        break;
                    }
                }
                if wakeup_count % 50 == 0 {
                    self.control_panel.ind_red.toggle();
                    self.control_panel.ind_green.toggle();
                }
                wakeup_count += 1;
                while now.ge(&next) {
                    next = next.saturating_add(Duration::from_millis(10));
                }
            }
        }
    }
}

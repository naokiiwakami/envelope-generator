use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use ssd1306_lite::{FontSize, TextBox};

use crate::{envelope_generator::EngineType, input_reader::get_reader_info_receiver};

use super::{ControlPanel, DisplayRequest};

// signal to receive nudges.
static SIGNAL_REPLY: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub struct Calibrator<'a> {
    control_panel: &'a mut ControlPanel,
}

impl<'a> Calibrator<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        Self { control_panel }
    }

    pub async fn execute(&mut self) {
        self.control_panel
            .display_text(
                "PLUG OFF",
                TextBox::top_center().build(),
                FontSize::Medium,
                true,
            )
            .await;
        let orig_engine_type = self.control_panel.current_engine_type.clone();
        self.control_panel
            .request_switching_engine(&EngineType::Diag)
            .await;
        crate::analog3::diagnose(&SIGNAL_REPLY).await;
        Timer::after_millis(500).await;
        self.control_panel.blink_leds().await;
        Timer::after_millis(500).await;
        self.diagnose_pots().await;
        self.diagnose_cv().await;
        self.control_panel
            .request_switching_engine(&orig_engine_type)
            .await;
        self.control_panel
            .switch_engine_type(orig_engine_type)
            .await;
        self.control_panel.clear_screen(true, false).await;
        Timer::after_secs(1).await;
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

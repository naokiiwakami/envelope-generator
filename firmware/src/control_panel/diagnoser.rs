use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::{
    pixelcolor::BinaryColor, prelude::Point, primitives::PrimitiveStyleBuilder,
};
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    envelope_generator::EngineType,
    input_reader::get_reader_info_receiver,
    patch_controller::{diagnose_button, diagnose_leds},
};

use super::{ControlPanel, DisplayRequest};

// signal to receive nudges.
static SIGNAL_REPLY: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub struct Diagnoser<'a> {
    control_panel: &'a mut ControlPanel,
}

impl<'a> Diagnoser<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        Self { control_panel }
    }

    pub async fn execute(&mut self) {
        let text_box = TextBox::center().fg_color(BinaryColor::Off).build();
        self.control_panel.clear_screen(true, false).await;
        self.control_panel
            .display_text("Diagnosing...", text_box.clone(), FontSize::Medium, true)
            .await;
        let orig_engine_type = self.control_panel.current_engine_type.clone();
        self.control_panel
            .request_switching_engine(&EngineType::Diag, false)
            .await;
        crate::analog3::diagnose(&SIGNAL_REPLY).await;
        Timer::after_millis(500).await;
        self.control_panel.blink_leds().await;
        Timer::after_millis(500).await;
        self.diagnose_patch_controller().await;
        self.diagnose_pots().await;
        self.diagnose_cv().await;
        self.control_panel
            .request_switching_engine(&orig_engine_type, false)
            .await;
        self.control_panel
            .switch_engine_type(orig_engine_type)
            .await;
        self.control_panel.clear_screen(true, false).await;
        self.control_panel
            .display_text("DONE!", text_box, FontSize::Medium, true)
            .await;
        Timer::after_secs(1).await;
    }

    async fn diagnose_patch_controller(&mut self) {
        diagnose_leds(&SIGNAL_REPLY).await;
        let text_box = TextBox::top_center().build();
        self.control_panel.clear_screen(false, false).await;
        self.control_panel
            .display_text("Press the button", text_box, FontSize::Medium, false)
            .await;
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawLine {
                start: Point::new(95, 39),
                end: Point::new(122, 58),
                style: PrimitiveStyleBuilder::new()
                    .stroke_color(BinaryColor::On)
                    .stroke_width(2)
                    .build(),
                flush: false,
            })
            .await;
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawTriangle {
                vertex1: Point::new(122, 58),
                vertex2: Point::new(113, 57),
                vertex3: Point::new(118, 50),
                style: PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build(),
                flush: true,
            })
            .await;
        diagnose_button(&SIGNAL_REPLY).await;
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

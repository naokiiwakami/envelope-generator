use defmt::debug;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal, watch};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{Point, Size},
    primitives::PrimitiveStyleBuilder,
};
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    control_panel::display::Mode,
    envelope_generator::{DEFAULT_OUT_ZERO_POINT, EngineType, get_eg_request_sender},
    input_reader::{
        InputReaderInfo, InputReaderRequest, get_reader_info_receiver, get_reader_request_sender,
    },
};

use super::{ControlPanel, DisplayRequest};

// signal to receive nudges.
static SIGNAL_REPLY: Signal<ThreadModeRawMutex, ()> = Signal::new();

pub struct Calibrator<'a> {
    control_panel: &'a mut ControlPanel,
}

const JACK_POSITIONS: [Point; 6] = [
    Point::new(1, 17),   // GATE 1
    Point::new(1, 45),   // GATE 2
    Point::new(37, 45),  // CV A
    Point::new(74, 45),  // CV B
    Point::new(109, 17), // OUT 1
    Point::new(109, 45), // OUT 2
];

impl<'a> Calibrator<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        Self { control_panel }
    }

    pub async fn execute(&mut self) {
        self.prepare().await;

        let mut reader_info_receiver = get_reader_info_receiver().await;
        self.calibrate_cv(&mut reader_info_receiver).await;
        self.calibrate_output(&mut reader_info_receiver).await;

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

    async fn prepare(&mut self) {
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::SwitchMode {
                mode: Mode::Fundamental,
            })
            .await;
        self.control_panel.clear_screen(false, false).await;
        self.display_title("PLUG OFF").await;

        // Draw initial jacks
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(2)
            .build();

        for position in JACK_POSITIONS {
            self.control_panel
                .display_request_sender
                .send(DisplayRequest::DrawCircle {
                    top_left: position,
                    diameter: 18,
                    style,
                    flush: false,
                })
                .await;
        }

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::Flush)
            .await;

        self.wait_for_button_pressed().await;
    }

    async fn calibrate_cv(
        &mut self,
        reader_info_receiver: &mut watch::Receiver<'_, ThreadModeRawMutex, InputReaderInfo, 2>,
    ) {
        self.display_title("CV: MEASURING").await;

        let request_sender = get_reader_request_sender();

        request_sender
            .send(InputReaderRequest::SetCvOffsets {
                offset_a: 0,
                offset_b: 0,
                save: false,
            })
            .await;

        // wait for the offset change being applied to the InputReader
        Timer::after_millis(100).await;

        let mut offset_a = 0i32;
        let mut offset_b = 0i32;
        let repeat = 256;

        // measure offset
        for _ in 0..repeat {
            let reader_info = reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_a -= cv_info.cv_a as i32;
            offset_b -= cv_info.cv_b as i32;
        }
        offset_a /= repeat;
        offset_b /= repeat;
        debug!("offset: A={}, B={}", offset_a, offset_b);

        self.display_title("CV: VERIFYING").await;

        // change offsets
        request_sender
            .send(InputReaderRequest::SetCvOffsets {
                offset_a: offset_a as i16,
                offset_b: offset_b as i16,
                save: true,
            })
            .await;
        // wait for the offset change being applied to the InputReader
        Timer::after_millis(100).await;

        // verify
        offset_a = 0;
        offset_b = 0;
        for _ in 0..repeat {
            let reader_info = reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_a -= cv_info.cv_a as i32;
            offset_b -= cv_info.cv_b as i32;
        }
        offset_a /= repeat;
        offset_b /= repeat;
        debug!("offset after calib: A={}, B={}", offset_a, offset_b);

        Timer::after_millis(1000).await;
    }

    async fn calibrate_output(
        &mut self,
        reader_info_receiver: &mut watch::Receiver<'_, ThreadModeRawMutex, InputReaderInfo, 2>,
    ) {
        let eg_request_sender = get_eg_request_sender();
        eg_request_sender
            .send(crate::envelope_generator::EgRequest::UpdateZeroPoint {
                value_1: DEFAULT_OUT_ZERO_POINT,
                value_2: DEFAULT_OUT_ZERO_POINT,
                save: false,
            })
            .await;

        self.display_title("PLUG...").await;
        self.wait_for_button_pressed().await;
        self.display_title("OUT: MEASURING").await;

        let mut offset_1 = 0i32;
        let mut offset_2 = 0i32;
        let repeat = 256;

        // measure offset
        for _ in 0..repeat {
            let reader_info = reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_1 -= cv_info.cv_a as i32;
            offset_2 -= cv_info.cv_b as i32;
        }
        let scale_16_to_12 = 16; // 4 bit
        offset_1 /= repeat * scale_16_to_12;
        offset_2 /= repeat * scale_16_to_12;

        let value_1 = (DEFAULT_OUT_ZERO_POINT as i32 + offset_1) as u16;
        let value_2 = (DEFAULT_OUT_ZERO_POINT as i32 + offset_2) as u16;

        debug!("zero_points: 1={:#x}, 2={:#x}", value_1, value_2);

        eg_request_sender
            .send(crate::envelope_generator::EgRequest::UpdateZeroPoint {
                value_1,
                value_2,
                save: true,
            })
            .await;
        self.display_title("CV: VERIFYING").await;
        offset_1 = 0;
        offset_2 = 0;
        for _ in 0..repeat {
            let reader_info = reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_1 += cv_info.cv_a as i32;
            offset_2 += cv_info.cv_b as i32;
        }
        offset_1 /= repeat;
        offset_2 /= repeat;
        debug!("drift after calib: 1={}, 2={}", offset_1, offset_2);

        self.display_title("OUT: DONE").await;

        self.wait_for_button_pressed().await;
    }

    async fn display_title(&mut self, text: &str) {
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawRectangle {
                top_left: Point::zero(),
                size: Size::new(128, 16),
                style: PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::Off)
                    .build(),
                flush: false,
            })
            .await;
        self.control_panel
            .display_text(text, TextBox::top_center().build(), FontSize::Medium, true)
            .await;
    }

    async fn wait_for_button_pressed(&mut self) {
        let mut wakeup_count = 0;
        loop {
            Timer::after_millis(10).await;
            let now = Instant::now();
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
        }
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

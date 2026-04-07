use core::ops::Add;

use defmt::debug;
use embassy_time::{Instant, Timer};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{Point, Size},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder},
};
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    control_panel::display::Mode,
    envelope_generator::{DEFAULT_OUT_ZERO_POINT, EngineType, get_eg_request_sender},
    input_reader::{InputReaderRequest, get_reader_request_sender},
};

use super::{ControlPanel, DisplayRequest, display::Mode as DisplayMode};

pub struct Calibrator<'a> {
    control_panel: &'a mut ControlPanel,
}

const JACK_POSITIONS: [Point; 6] = [
    Point::new(0, 16),   // GATE 1
    Point::new(0, 46),   // GATE 2
    Point::new(36, 46),  // CV A
    Point::new(75, 46),  // CV B
    Point::new(110, 16), // OUT 1
    Point::new(110, 46), // OUT 2
];

// const INDEX_GATE_1: usize = 0;
// const INDEX_GATE_2: usize = 1;
const INDEX_CV_A: usize = 2;
const INDEX_CV_B: usize = 3;
const INDEX_OUT_1: usize = 4;
const INDEX_OUT_2: usize = 5;

const JACK_DIAMETER: u32 = 18;

impl<'a> Calibrator<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        Self { control_panel }
    }

    pub async fn execute(&mut self) {
        self.prepare().await;

        self.calibrate_cv().await;
        self.calibrate_output().await;
        self.wrap_up().await;
    }

    async fn prepare(&mut self) {
        self.control_panel
            .switch_display_mode(DisplayMode::Fundamental)
            .await;
        self.control_panel
            .switch_engine_type(EngineType::Adsr)
            .await;
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::SwitchMode {
                mode: Mode::Fundamental,
            })
            .await;
        self.control_panel.clear_screen(false, false).await;
        self.display_title("UNPLUG...").await;

        // Draw initial jacks
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .build();

        for index in 0..JACK_POSITIONS.len() {
            self.draw_jack(index, 0, style, false).await;
        }

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::Flush)
            .await;

        self.wait_for_button_pressed().await;
    }

    async fn calibrate_cv(&mut self) {
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
        let step = repeat / JACK_DIAMETER * 2;
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .build();

        // measure offset
        for i in 0..repeat {
            let reader_info = self.control_panel.reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_a -= cv_info.cv_a as i32;
            offset_b -= cv_info.cv_b as i32;
            if i % step == 0 {
                let shift = i / step;
                self.draw_jack(INDEX_CV_A, shift, stroke, false).await;
                self.draw_jack(INDEX_CV_B, shift, stroke, true).await;
            }
        }
        offset_a /= repeat as i32;
        offset_b /= repeat as i32;
        debug!("CV offsets: A={}, B={}", offset_a, offset_b);

        self.display_title("VERIFYING").await;

        self.draw_arrow(
            Point::new(45, 16),
            Point::new(45, 40),
            Point::new(43, 34),
            Point::new(47, 34),
            BinaryColor::On,
            false,
        )
        .await;

        self.draw_arrow(
            Point::new(84, 16),
            Point::new(84, 40),
            Point::new(82, 34),
            Point::new(86, 34),
            BinaryColor::On,
            true,
        )
        .await;

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
        let step = repeat / (JACK_DIAMETER - 6) * 2;
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::Off)
            .stroke_width(1)
            .build();

        for i in 0..repeat {
            let reader_info = self.control_panel.reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_a -= cv_info.cv_a as i32;
            offset_b -= cv_info.cv_b as i32;
            if i % step == 0 {
                let shift = JACK_DIAMETER / 2 - i / step;
                self.draw_jack(INDEX_CV_A, shift, stroke, false).await;
                self.draw_jack(INDEX_CV_B, shift, stroke, true).await;
            }
        }
        offset_a /= repeat as i32;
        offset_b /= repeat as i32;
        debug!("CV offsets after calib: A={}, B={}", offset_a, offset_b);

        // TODO: check offsets actually

        self.display_title("OK").await;

        self.clear_courtyard(true).await;

        Timer::after_millis(2000).await;
    }

    async fn calibrate_output(&mut self) {
        let eg_request_sender = get_eg_request_sender();
        eg_request_sender
            .send(crate::envelope_generator::EgRequest::UpdateZeroPoints {
                value_1: DEFAULT_OUT_ZERO_POINT,
                value_2: DEFAULT_OUT_ZERO_POINT,
                save: false,
            })
            .await;

        self.display_title("PLUG...").await;

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawArc {
                center: Point::new(100, 80),
                radius: 57,
                start_degree: 214,
                end_degree: 279,
                color: BinaryColor::On,
                flush: false,
            })
            .await;

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawArc {
                center: Point::new(101, 64),
                radius: 16,
                start_degree: 239,
                end_degree: 303,
                color: BinaryColor::On,
                flush: true,
            })
            .await;

        self.wait_for_button_pressed().await;
        self.display_title("OUT: MEASURING").await;

        let mut offset_1 = 0i32;
        let mut offset_2 = 0i32;
        let repeat = 256;
        let step = repeat / JACK_DIAMETER * 2;
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(1)
            .build();

        // measure offset
        for i in 0..repeat {
            let reader_info = self.control_panel.reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_1 += cv_info.cv_a as i32;
            offset_2 += cv_info.cv_b as i32;
            if i % step == 0 {
                let shift = i / step;
                self.draw_jack(INDEX_OUT_1, shift, stroke, false).await;
                self.draw_jack(INDEX_OUT_2, shift, stroke, true).await;
            }
        }
        offset_1 /= repeat as i32;
        offset_2 /= repeat as i32;
        debug!("drift before calib: 1={}, 2={}", offset_1, offset_2);
        let scale_16_to_12 = 16; // 4 bit
        offset_1 /= scale_16_to_12;
        offset_2 /= scale_16_to_12;

        let value_1 = (DEFAULT_OUT_ZERO_POINT as i32 - offset_1) as u16;
        let value_2 = (DEFAULT_OUT_ZERO_POINT as i32 - offset_2) as u16;

        debug!("zero_points: 1={:#x}, 2={:#x}", value_1, value_2);

        eg_request_sender
            .send(crate::envelope_generator::EgRequest::UpdateZeroPoints {
                value_1,
                value_2,
                save: true,
            })
            .await;

        Timer::after_millis(100).await;

        self.display_title("VERIFYING").await;

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawArc {
                center: Point::new(100, 80),
                radius: 57,
                start_degree: 214,
                end_degree: 279,
                color: BinaryColor::Off,
                flush: false,
            })
            .await;

        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawArc {
                center: Point::new(101, 64),
                radius: 16,
                start_degree: 239,
                end_degree: 303,
                color: BinaryColor::Off,
                flush: false,
            })
            .await;

        self.draw_arrow(
            Point::new(45, 16),
            Point::new(105, 47),
            Point::new(99, 47),
            Point::new(101, 42),
            BinaryColor::On,
            false,
        )
        .await;

        self.draw_arrow(
            Point::new(84, 16),
            Point::new(105, 23),
            Point::new(99, 24),
            Point::new(100, 19),
            BinaryColor::On,
            true,
        )
        .await;

        offset_1 = 0;
        offset_2 = 0;
        let step = repeat / (JACK_DIAMETER - 6) * 2;
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::Off)
            .stroke_width(1)
            .build();

        for i in 0..repeat {
            let reader_info = self.control_panel.reader_info_receiver.changed().await;
            let cv_info = reader_info.cv_info;
            offset_1 += cv_info.cv_a as i32;
            offset_2 += cv_info.cv_b as i32;
            if i % step == 0 {
                let shift = JACK_DIAMETER / 2 - i / step;
                self.draw_jack(INDEX_OUT_1, shift, stroke, false).await;
                self.draw_jack(INDEX_OUT_2, shift, stroke, true).await;
            }
        }
        offset_1 /= repeat as i32;
        offset_2 /= repeat as i32;
        debug!("drift after calib: 1={}, 2={}", offset_1, offset_2);

        // TODO: check offsets actually

        self.display_title("OK").await;

        self.draw_arrow(
            Point::new(45, 16),
            Point::new(105, 47),
            Point::new(99, 47),
            Point::new(101, 42),
            BinaryColor::Off,
            false,
        )
        .await;

        self.draw_arrow(
            Point::new(84, 16),
            Point::new(105, 23),
            Point::new(99, 24),
            Point::new(100, 19),
            BinaryColor::Off,
            true,
        )
        .await;

        Timer::after_millis(2000).await;
    }

    async fn wrap_up(&mut self) {
        self.control_panel.clear_screen(true, false).await;
        self.control_panel
            .display_text(
                "DONE!",
                TextBox::center().fg_color(BinaryColor::Off).build(),
                FontSize::Large,
                true,
            )
            .await;
        Timer::after_millis(2000).await;
        self.control_panel
            .switch_engine_type(self.control_panel.eg_config.engine_type(0))
            .await;
    }

    // Utilities ////////////////////////////////////////////////

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

    fn shift(position: &Point, delta: i32) -> Point {
        position.add(Point::new(delta, delta))
    }

    async fn draw_jack(
        &mut self,
        index: usize,
        shift: u32,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    ) {
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawCircle {
                top_left: Self::shift(&JACK_POSITIONS[index], shift as i32),
                diameter: JACK_DIAMETER - shift * 2,
                style,
                flush,
            })
            .await;
    }

    async fn draw_arrow(
        &mut self,
        start: Point,
        end: Point,
        arrow_left: Point,
        arrow_right: Point,
        color: BinaryColor,
        flush: bool,
    ) {
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_color(color)
            .stroke_width(1)
            .build();
        let fill = PrimitiveStyleBuilder::new().fill_color(color).build();
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawLine {
                start,
                end,
                style: stroke,
                flush: false,
            })
            .await;
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawTriangle {
                vertex1: end,
                vertex2: arrow_left,
                vertex3: arrow_right,
                style: fill,
                flush,
            })
            .await;
    }

    async fn clear_courtyard(&mut self, flush: bool) {
        let erase = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::Off)
            .build();
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::DrawRectangle {
                top_left: Point::new(18, 16),
                size: Size::new(92, 30),
                style: erase,
                flush,
            })
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
}

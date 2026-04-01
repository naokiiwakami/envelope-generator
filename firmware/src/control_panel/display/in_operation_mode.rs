use defmt::{debug, error};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{
        Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
};
use ssd1306_lite::{Angle, FontSize, Ssd1306Lite, TextBox};

use crate::{
    envelope_generator::EngineType,
    input_reader::{PotInfo, PotKind},
};

use super::{Display, ENGINE_TYPE_MENU_ITEMS, Mode, Request};

pub struct InOperationMode<'a> {
    display: &'a mut Display,
}

impl<'a> InOperationMode<'a> {
    pub fn new(display: &'a mut Display) -> Self {
        Self { display }
    }

    pub async fn run(&mut self) {
        debug!("In InOperation mode");
        while matches!(self.display.mode, Mode::InOperation) {
            let request = self.display.fetch_request().await;
            /*
            debug!(
                "[InOperation]: request: {} mode: {}",
                request.name(),
                self.display.mode
            );
            */
            match request {
                Request::SwitchMode { .. }
                | Request::Clear { .. }
                | Request::Flush
                | Request::DrawLine { .. }
                | Request::DrawCircle { .. }
                | Request::DrawRectangle { .. }
                | Request::DrawTriangle { .. }
                | Request::DrawArc { .. }
                | Request::DisplayText { .. } => self.display.handle_generic_request(request).await,
                Request::GoToOpHome {
                    engine_type,
                    attack,
                    decay,
                    sustain,
                    release,
                    extra_1,
                    extra_2,
                } => {
                    self.show_initial_screen(
                        engine_type,
                        attack,
                        decay,
                        sustain,
                        release,
                        extra_1,
                        extra_2,
                    )
                    .await
                }
                Request::UpdatePot { pot_info } => self.update_pot(pot_info).await,
                Request::ShowPolarity {
                    polarity_1,
                    polarity_2,
                } => self.show_polarity(polarity_1, polarity_2).await,
                _ => {
                    self.display
                        .switch_mode(request.mode(), Some(request))
                        .await
                }
            }
        }
        // debug!("[InOperation]: out to {}", self.display.mode);
    }

    pub async fn show_initial_screen(
        &mut self,
        engine_type: EngineType,
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        extra_1: u16,
        extra_2: u16,
    ) {
        debug!("engine type to {}", engine_type);
        self.display.current_engine_type = engine_type;
        match self.display.current_engine_type {
            EngineType::Adsr => {
                self.show_initial_screen_adsr(attack, decay, sustain, release, extra_1, extra_2)
                    .await
            }
            EngineType::Linear => {
                self.show_initial_screen_linear(attack, decay, sustain, release, extra_1, extra_2)
                    .await
            }
            EngineType::Diag => {}
            _ => {
                self.show_default_initial_screen(attack, decay, sustain, release, extra_1, extra_2)
                    .await
            }
        }
    }

    async fn show_default_initial_screen(
        &mut self,
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        extra_1: u16,
        extra_2: u16,
    ) {
        self.display.clear(false, false).await;
        let name =
            ENGINE_TYPE_MENU_ITEMS[(self.display.current_engine_type.clone() as u8) as usize].name;
        let text_box = TextBox::center().build();
        self.display
            .driver
            .draw_string(name, text_box, FontSize::Large)
            .await;
        self.display.driver.flush().await;
    }

    async fn update_pot(&mut self, pot_info: PotInfo) {
        match self.display.current_engine_type {
            EngineType::Adsr => self.update_pot_adsr(pot_info).await,
            _ => {}
        }
    }

    // ADSR ///////////////////////////////////////////////////////////////////////////////////

    async fn show_initial_screen_adsr(
        &mut self,
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        extra_1: u16,
        extra_2: u16,
    ) {
        self.display.clear(false, false).await;
        let fill = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .build();
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(0, 20), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(33, 0), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(67, 0), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(104, 20), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_string("ADSR", TextBox::center().build(), FontSize::Large)
            .await;

        let stroke = PrimitiveStyleBuilder::new()
            .stroke_width(3)
            .stroke_alignment(StrokeAlignment::Center)
            .stroke_color(BinaryColor::On)
            .build();

        let line_length: i32 = (extra_2 >> 12) as i32;
        self.display
            .driver
            .draw_styled(
                Line::new(Point::new(90, 60), Point::new(90 + line_length, 60)).into_styled(stroke),
            )
            .await;

        self.display.driver.flush().await;
    }

    async fn update_pot_adsr(&mut self, pot_info: PotInfo) {
        match pot_info.kind {
            PotKind::Extra2 => {
                let erase = PrimitiveStyleBuilder::new()
                    .stroke_width(3)
                    .stroke_alignment(StrokeAlignment::Center)
                    .stroke_color(BinaryColor::Off)
                    .build();

                let stroke = PrimitiveStyleBuilder::new()
                    .stroke_width(3)
                    .stroke_alignment(StrokeAlignment::Center)
                    .stroke_color(BinaryColor::On)
                    .build();

                let line_length: i32 = (pot_info.value >> 12) as i32;
                self.display
                    .driver
                    .draw_styled(
                        Line::new(Point::new(90, 60), Point::new(90 + 16, 60)).into_styled(erase),
                    )
                    .await;
                self.display
                    .driver
                    .draw_styled(
                        Line::new(Point::new(90, 60), Point::new(90 + line_length, 60))
                            .into_styled(stroke),
                    )
                    .await;
            }
            _ => {}
        }
        self.display.driver.flush().await;
    }

    // Linear //////////////////////////////////////////////////////////////////////////////////////////

    async fn show_initial_screen_linear(
        &mut self,
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        extra_1: u16,
        extra_2: u16,
    ) {
        self.display.clear(false, false).await;
        let fill = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .build();
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(0, 20), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(33, 0), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(67, 0), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(104, 20), 12).into_styled(fill))
            .await;
        self.display
            .driver
            .draw_string("LINEAR", TextBox::center().build(), FontSize::Large)
            .await;
        self.display.driver.flush().await;
    }

    // Polarity management ///////////////////////////////////////////////////////////////////////

    async fn show_polarity(&mut self, polarity_1: i8, polarity_2: i8) {
        self.display.clear(false, false).await;
        self.display
            .driver
            .draw_string(
                "POLARITIES",
                TextBox::simple(0, 0, BinaryColor::On),
                FontSize::Medium,
            )
            .await;

        // out 1
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_width(1)
            .stroke_color(BinaryColor::On)
            .build();
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(100, 5), 21).into_styled(stroke))
            .await;
        self.display
            .driver
            .draw_styled(Line::new(Point::new(103, 15), Point::new(117, 15)).into_styled(stroke))
            .await;
        if polarity_1 > 0 {
            self.display
                .driver
                .draw_styled(Line::new(Point::new(110, 8), Point::new(110, 22)).into_styled(stroke))
                .await;
        }

        // out 2
        self.display
            .driver
            .draw_styled(Circle::new(Point::new(100, 37), 21).into_styled(stroke))
            .await;
        self.display
            .driver
            .draw_styled(Line::new(Point::new(103, 47), Point::new(117, 47)).into_styled(stroke))
            .await;
        if polarity_2 > 0 {
            self.display
                .driver
                .draw_styled(
                    Line::new(Point::new(110, 38), Point::new(110, 52)).into_styled(stroke),
                )
                .await;
        }

        self.display.driver.flush().await;
    }
}

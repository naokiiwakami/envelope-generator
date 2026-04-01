use defmt::{debug, error};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Triangle},
};
use ssd1306_lite::{Angle, FontSize, Ssd1306Lite, TextBox};

use crate::envelope_generator::EngineType;

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
            debug!(
                "[InOperation]: request: {} mode: {}",
                request.name(),
                self.display.mode
            );
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
                Request::GoToOpHome { engine_type } => self.show_initial_screen(engine_type).await,
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
        debug!("[InOperation]: out to {}", self.display.mode);
    }

    pub async fn show_initial_screen(&mut self, engine_type: EngineType) {
        debug!("engine type to {}", engine_type);
        self.display.current_engine_type = engine_type;
        match self.display.current_engine_type {
            EngineType::Adsr => self.show_adsr_initial_screen().await,
            EngineType::Linear => self.show_linear_initial_screen().await,
            EngineType::Diag => {}
            _ => self.show_default_initial_screen().await,
        }
    }

    async fn show_adsr_initial_screen(&mut self) {
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
        self.display.driver.flush().await;
    }

    async fn show_linear_initial_screen(&mut self) {
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
        self.display.driver.flush().await;
    }

    async fn show_default_initial_screen(&mut self) {
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

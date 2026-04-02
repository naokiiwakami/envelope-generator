use defmt::debug;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    primitives::{PrimitiveStyleBuilder, StrokeAlignment},
};
use ssd1306_lite::{FontSize, TextBox};

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
                    self.show_home_page(
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

    pub async fn show_home_page(
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
                self.show_home_page_adsr(attack, decay, sustain, release, extra_1, extra_2)
                    .await
            }
            EngineType::Linear => {
                self.show_home_page_linear(attack, decay, sustain, release, extra_1, extra_2)
                    .await
            }
            _ => {
                self.show_default_home_page(attack, decay, sustain, release, extra_1, extra_2)
                    .await
            }
        }
    }

    async fn show_default_home_page(
        &mut self,
        _attack: u16,
        _decay: u16,
        _sustain: u16,
        _release: u16,
        _extra_1: u16,
        _extra_2: u16,
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

    async fn show_home_page_adsr(
        &mut self,
        _attack: u16,
        _decay: u16,
        _sustain: u16,
        _release: u16,
        _extra_1: u16,
        extra_2: u16,
    ) {
        self.display.clear(false, false).await;
        let fill = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .build();
        self.display.driver.draw_circle((0, 20), 12, fill).await;
        self.display.driver.draw_circle((33, 0), 12, fill).await;
        self.display.driver.draw_circle((67, 0), 12, fill).await;
        self.display.driver.draw_circle((104, 20), 12, fill).await;
        self.display
            .driver
            .draw_string("ADSR", TextBox::center().build(), FontSize::Large)
            .await;

        self.draw_note_scaling_bar(extra_2).await;

        self.display.driver.flush().await;
    }

    async fn update_pot_adsr(&mut self, pot_info: PotInfo) {
        match pot_info.kind {
            PotKind::Extra2 => {
                self.draw_note_scaling_bar(pot_info.value).await;
            }
            _ => {}
        }
        self.display.driver.flush().await;
    }

    /// Draws a note scaling bar
    async fn draw_note_scaling_bar(&mut self, depth: u16) {
        let erase = PrimitiveStyleBuilder::new()
            .stroke_width(0)
            .fill_color(BinaryColor::Off)
            .build();
        let fill = PrimitiveStyleBuilder::new()
            .stroke_width(0)
            .fill_color(BinaryColor::On)
            .build();

        let left: i32 = 80;
        let width: u32 = 40;
        let right: i32 = left + width as i32;
        let bottom_y: i32 = 63;
        let max_bar_thickness: u32 = 8;
        let min_bar_thickness: u32 = 2;
        let max_triangle_height: u32 = 15;

        let thickness =
            max_bar_thickness - ((depth as u32 * (max_bar_thickness - min_bar_thickness)) >> 16);
        let triangle_height = (depth as u32 * (max_triangle_height - min_bar_thickness)) >> 16;

        let bar_top_y = bottom_y + 1 - thickness as i32;
        let triangle_base_y = bar_top_y.min(bottom_y);
        let triangle_top_y = triangle_base_y - triangle_height as i32;

        self.display
            .driver
            .draw_rectangle(
                (left, bottom_y - max_triangle_height as i32 + 1),
                width + 1,
                max_triangle_height,
                erase,
            )
            .await;

        if thickness > 0 {
            self.display
                .driver
                .draw_rectangle((left, bar_top_y), width, thickness, fill)
                .await;
        }

        if triangle_height > 0 {
            self.display
                .driver
                .draw_triangle(
                    (left, triangle_top_y),
                    (left, triangle_base_y),
                    (right, triangle_base_y),
                    fill,
                )
                .await;
        }
    }

    // Linear //////////////////////////////////////////////////////////////////////////////////////////

    async fn show_home_page_linear(
        &mut self,
        _attack: u16,
        _decay: u16,
        _sustain: u16,
        _release: u16,
        _extra_1: u16,
        _extra_2: u16,
    ) {
        self.display.clear(false, false).await;
        let fill = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .build();
        self.display.driver.draw_circle((0, 20), 12, fill).await;
        self.display.driver.draw_circle((33, 0), 12, fill).await;
        self.display.driver.draw_circle((67, 0), 12, fill).await;
        self.display.driver.draw_circle((104, 20), 12, fill).await;
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
        self.display.driver.draw_circle((100, 5), 21, stroke).await;
        self.display
            .driver
            .draw_line((103, 15), (117, 15), stroke)
            .await;
        if polarity_1 > 0 {
            self.display
                .driver
                .draw_line((110, 8), (110, 22), stroke)
                .await;
        }

        // out 2
        self.display.driver.draw_circle((100, 37), 21, stroke).await;
        self.display
            .driver
            .draw_line((103, 47), (117, 47), stroke)
            .await;
        if polarity_2 > 0 {
            self.display
                .driver
                .draw_line((110, 40), (110, 54), stroke)
                .await;
        }

        self.display.driver.flush().await;
    }
}

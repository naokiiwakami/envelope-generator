use defmt::debug;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::Point,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, StrokeAlignment},
};
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    envelope_generator::EngineType,
    input_reader::{PotInfo, PotKind},
};

use super::{Display, ENGINE_TYPE_MENU_ITEMS, Mode, Request};

pub struct InOperationMode<'a> {
    display: &'a mut Display,

    attack: i32,
    decay: i32,
    sustain: i32,
    gate_off: i32,
    release: i32,

    // frequently used styles
    erase_fill: PrimitiveStyle<BinaryColor>,
    fill: PrimitiveStyle<BinaryColor>,
    erase_stroke: PrimitiveStyle<BinaryColor>,
    stroke: PrimitiveStyle<BinaryColor>,
}

const NODE_SIZE: u32 = 3;

impl<'a> InOperationMode<'a> {
    pub fn new(display: &'a mut Display) -> Self {
        Self {
            display,
            attack: 0,
            decay: 0,
            sustain: 0,
            gate_off: 0,
            release: 0,
            erase_fill: PrimitiveStyleBuilder::new()
                .stroke_width(0)
                .fill_color(BinaryColor::Off)
                .build(),
            fill: PrimitiveStyleBuilder::new()
                .stroke_width(0)
                .fill_color(BinaryColor::On)
                .build(),
            erase_stroke: PrimitiveStyleBuilder::new()
                .stroke_width(1)
                .stroke_color(BinaryColor::Off)
                .build(),
            stroke: PrimitiveStyleBuilder::new()
                .stroke_width(1)
                .stroke_color(BinaryColor::On)
                .build(),
        }
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
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        _extra_1: u16,
        extra_2: u16,
    ) {
        self.display.clear(false, false).await;

        self.attack = self.attack_pos(attack);
        self.decay = self.decay_pos(decay);
        self.sustain = self.sustain_pos(sustain);
        // self.gate_off = self.gate_off_pos();
        self.release = self.release_pos(release);

        // self.draw_node(0, 28).await;
        // self.draw_node(self.attack, 0).await;
        self.draw_line((0, 28), (self.attack, 0)).await;
        // self.draw_node(self.decay, self.sustain).await;
        self.draw_line((self.attack, 0), (self.decay, self.sustain))
            .await;
        // self.draw_node(self.gate_off, self.sustain).await;
        self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
            .await;
        // self.draw_node(self.release, 28).await;
        self.draw_line((self.release, self.sustain), (127, 28))
            .await;

        self.draw_note_scaling_bar(extra_2).await;

        self.display.driver.flush().await;
    }

    async fn update_pot_adsr(&mut self, pot_info: PotInfo) {
        match pot_info.kind {
            PotKind::Attack => {
                let next_attack = self.attack_pos(pot_info.value);
                if next_attack == self.attack {
                    return;
                }
                // self.erase_node(self.attack, 0).await;
                self.erase_line((0, 28), (self.attack, 0)).await;
                self.erase_line((self.attack, 0), (self.decay, self.sustain))
                    .await;

                self.attack = next_attack;

                // self.draw_node(self.attack, 0).await;
                self.draw_line((0, 28), (self.attack, 0)).await;
                self.draw_line((self.attack, 0), (self.decay, self.sustain))
                    .await;
            }
            PotKind::Decay => {
                let next_decay = self.decay_pos(pot_info.value);
                if next_decay == self.decay {
                    return;
                }

                // self.erase_node(self.decay, self.sustain).await;
                // self.erase_node(self.gate_off, self.sustain).await;
                self.erase_line((self.attack, 0), (self.decay, self.sustain))
                    .await;
                self.erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.erase_line((self.release, self.sustain), (127, 28))
                    .await;

                self.decay = next_decay;
                // self.gate_off = self.gate_off_pos();

                // self.draw_node(self.decay, self.sustain).await;
                // self.draw_node(self.gate_off, self.sustain).await;
                self.draw_line((self.attack, 0), (self.decay, self.sustain))
                    .await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_line((self.release, self.sustain), (127, 28))
                    .await;
            }
            PotKind::Sustain => {
                let next_sustain = self.sustain_pos(pot_info.value);
                if next_sustain == self.sustain {
                    return;
                }
                // self.erase_node(self.decay, self.sustain).await;
                // self.erase_node(self.gate_off, self.sustain).await;
                self.erase_line((self.attack, 0), (self.decay, self.sustain))
                    .await;
                self.erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.erase_line((self.release, self.sustain), (127, 28))
                    .await;

                self.sustain = next_sustain;

                // self.draw_node(self.decay, self.sustain).await;
                // self.draw_node(self.gate_off, self.sustain).await;
                self.draw_line((self.attack, 0), (self.decay, self.sustain))
                    .await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_line((self.release, self.sustain), (127, 28))
                    .await;
            }
            PotKind::Release => {
                let next_release = self.release_pos(pot_info.value);
                if next_release == self.release {
                    return;
                }
                // self.erase_node(self.release, 28).await;
                self.erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.erase_line((self.release, self.sustain), (127, 28))
                    .await;

                self.release = next_release;

                // self.draw_node(self.release, 28).await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_line((self.release, self.sustain), (127, 28))
                    .await;
            }
            PotKind::Extra2 => {
                self.draw_note_scaling_bar(pot_info.value).await;
            }
            _ => {}
        }
        self.display.driver.flush().await;
    }

    fn attack_pos(&self, attack: u16) -> i32 {
        ((24 * (attack as i32 + 1)) >> 16) + NODE_SIZE as i32
    }

    fn decay_pos(&self, decay: u16) -> i32 {
        ((30 * (decay as i32 + 1)) >> 16) + NODE_SIZE as i32 + self.attack
    }

    fn sustain_pos(&self, sustain: u16) -> i32 {
        28 - ((28 * (sustain as i32 + 1)) >> 16)
    }

    fn gate_off_pos(&self) -> i32 {
        self.decay + 32 + NODE_SIZE as i32
    }

    fn release_pos(&self, release: u16) -> i32 {
        125 - ((30 * (release as i32 + 1)) >> 16)
    }

    #[inline(always)]
    async fn draw_node(&mut self, top_left_x: i32, top_left_y: i32) {
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), NODE_SIZE, self.fill)
            .await;
    }

    #[inline(always)]
    async fn erase_node(&mut self, top_left_x: i32, top_left_y: i32) {
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), NODE_SIZE, self.erase_fill)
            .await;
    }

    #[inline(always)]
    async fn draw_line(&mut self, start: (i32, i32), end: (i32, i32)) {
        self.display.driver.draw_line(start, end, self.stroke).await;
    }

    #[inline(always)]
    async fn erase_line(&mut self, start: (i32, i32), end: (i32, i32)) {
        self.display
            .driver
            .draw_line(start, end, self.erase_stroke)
            .await;
    }

    /// Draws a note scaling bar
    async fn draw_note_scaling_bar(&mut self, depth: u16) {
        let left: i32 = 80;
        let width: u32 = 40;
        let right: i32 = left + width as i32;
        let bottom_y: i32 = 63;
        let max_bar_thickness: u32 = 11;
        let min_bar_thickness: u32 = 2;
        let max_triangle_height: u32 = 20;

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
                self.erase_fill,
            )
            .await;

        self.draw_line((left, bottom_y), (right, bottom_y)).await;
        self.draw_line((right, bottom_y), (right, bar_top_y)).await;
        self.draw_line((right, bar_top_y), (left, triangle_top_y))
            .await;
        self.draw_line((left, triangle_top_y), (left, bottom_y))
            .await;

        /*
        if thickness > 0 {
            self.display
                .driver
                .draw_rectangle((left, bar_top_y), width, thickness, self.fill)
                .await;
        }

        if triangle_height > 0 {
            self.display
                .driver
                .draw_triangle(
                    (left, triangle_top_y),
                    (left, triangle_base_y),
                    (right, triangle_base_y),
                    self.fill,
                )
                .await;
        }
        */
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

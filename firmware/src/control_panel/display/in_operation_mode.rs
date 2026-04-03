use defmt::debug;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder},
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
    release: i32,

    // frequently used styles
    erase_area: PrimitiveStyle<BinaryColor>,
    _fill_area: PrimitiveStyle<BinaryColor>,
    erase_stroke: PrimitiveStyle<BinaryColor>,
    stroke: PrimitiveStyle<BinaryColor>,
}

const _NODE_SIZE: u32 = 3;

// window edges
const LEFT: i32 = 0;
const RIGHT: i32 = 127;
const TOP: i32 = 0;
const BOTTOM: i32 = 28;

impl<'a> InOperationMode<'a> {
    pub fn new(display: &'a mut Display) -> Self {
        Self {
            display,
            attack: 0,
            decay: 0,
            sustain: 0,
            release: 0,
            erase_area: PrimitiveStyleBuilder::new()
                .stroke_width(1)
                .stroke_color(BinaryColor::Off)
                .fill_color(BinaryColor::Off)
                .build(),
            _fill_area: PrimitiveStyleBuilder::new()
                .stroke_width(0)
                // .stroke_color(BinaryColor::On)
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
            EngineType::Linear => self.update_pot_linear(pot_info).await,
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
        self.release = self.release_pos(release);

        self.draw_curve((LEFT, BOTTOM), (self.attack, TOP)).await;
        self.draw_curve((self.attack, TOP), (self.decay, self.sustain))
            .await;
        self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
            .await;
        self.draw_curve((self.release, self.sustain), (RIGHT, BOTTOM))
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

                self.attack = next_attack;

                self.erase_x_range(LEFT, self.attack).await;
                self.draw_curve((LEFT, BOTTOM), (self.attack, TOP)).await;
            }
            PotKind::Decay => {
                let next_decay = self.decay_pos(pot_info.value);
                if next_decay == self.decay {
                    return;
                }

                self.decay = next_decay;

                self.erase_x_range(self.attack, self.release).await;

                self.draw_curve((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                if self.attack < LEFT + 2 {
                    self.draw_curve((LEFT, BOTTOM), (self.attack, TOP)).await;
                }
            }
            PotKind::Sustain => {
                let next_sustain = self.sustain_pos(pot_info.value);
                if next_sustain == self.sustain {
                    return;
                }

                self.sustain = next_sustain;

                self.erase_x_range(self.attack, RIGHT).await;

                self.draw_curve((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_curve((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;
                if self.attack < LEFT + 2 {
                    self.draw_curve((LEFT, BOTTOM), (self.attack, TOP)).await;
                }
            }
            PotKind::Release => {
                let next_release = self.release_pos(pot_info.value);
                if next_release == self.release {
                    return;
                }

                self.release = next_release;

                self.erase_x_range(self.decay, RIGHT).await;

                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_curve((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;
            }
            PotKind::Extra2 => {
                self.draw_note_scaling_bar(pot_info.value).await;
            }
            _ => {}
        }
        self.display.driver.flush().await;
    }

    // Linear ///////////////////////////////////////////////////////////////////////

    async fn show_home_page_linear(
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
        self.release = self.release_pos(release);

        self.draw_line((LEFT, BOTTOM), (self.attack, TOP)).await;
        self.draw_line((self.attack, TOP), (self.decay, self.sustain))
            .await;
        self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
            .await;
        self.draw_line((self.release, self.sustain), (RIGHT, BOTTOM))
            .await;

        self.draw_note_scaling_bar(extra_2).await;

        self.display.driver.flush().await;
    }

    async fn update_pot_linear(&mut self, pot_info: PotInfo) {
        match pot_info.kind {
            PotKind::Attack => {
                let next_attack = self.attack_pos(pot_info.value);
                if next_attack == self.attack {
                    return;
                }
                self.erase_line((LEFT, BOTTOM), (self.attack, TOP)).await;
                self.erase_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;

                self.attack = next_attack;

                self.draw_line((LEFT, BOTTOM), (self.attack, TOP)).await;
                self.draw_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
            }
            PotKind::Decay => {
                let next_decay = self.decay_pos(pot_info.value);
                if next_decay == self.decay {
                    return;
                }

                self.erase_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                self.erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.erase_line((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;

                self.decay = next_decay;

                self.draw_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_line((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;
            }
            PotKind::Sustain => {
                let next_sustain = self.sustain_pos(pot_info.value);
                if next_sustain == self.sustain {
                    return;
                }
                self.erase_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                self.erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.erase_line((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;

                self.sustain = next_sustain;

                self.draw_line((self.attack, TOP), (self.decay, self.sustain))
                    .await;
                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_line((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;
            }
            PotKind::Release => {
                let next_release = self.release_pos(pot_info.value);
                if next_release == self.release {
                    return;
                }
                self.erase_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.erase_line((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;

                self.release = next_release;

                self.draw_line((self.decay, self.sustain), (self.release, self.sustain))
                    .await;
                self.draw_line((self.release, self.sustain), (RIGHT, BOTTOM))
                    .await;
            }
            PotKind::Extra2 => {
                self.draw_note_scaling_bar(pot_info.value).await;
            }
            _ => {}
        }
        self.display.driver.flush().await;
    }

    // Note scaling //////////////////////////////////////////////////////////////////////////////

    /// Draws a note scaling bar
    async fn draw_note_scaling_bar(&mut self, depth: u16) {
        let left: i32 = 80;
        let width: u32 = 40;
        let right: i32 = left + width as i32;
        let bottom_y: i32 = 63;
        let max_bar_thickness: u32 = 11;
        let min_bar_thickness: u32 = 2;
        let max_triangle_height: u32 = 20;

        let degree = distort(depth);

        let thickness =
            max_bar_thickness - ((degree as u32 * (max_bar_thickness - min_bar_thickness)) >> 16);
        let triangle_height = (degree as u32 * (max_triangle_height - min_bar_thickness)) >> 16;

        let bar_top_y = bottom_y + 1 - thickness as i32;
        let triangle_base_y = bar_top_y.min(bottom_y);
        let triangle_top_y = triangle_base_y - triangle_height as i32;

        self.display
            .driver
            .draw_rectangle(
                (left, bottom_y - max_triangle_height as i32 + 1),
                width + 1,
                max_triangle_height,
                self.erase_area,
            )
            .await;

        self.draw_line((left, bottom_y), (right, bottom_y)).await;
        self.draw_line((right, bottom_y), (right, bar_top_y)).await;
        self.draw_line((right, bar_top_y), (left, triangle_top_y))
            .await;
        self.draw_line((left, triangle_top_y), (left, bottom_y))
            .await;
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

    // Utils /////////////////////////////////////////////////////////////////////////////////////

    #[inline]
    fn attack_pos(&self, attack: u16) -> i32 {
        ((35 * (distort(attack) as i32 + 1)) >> 16) + LEFT
    }

    #[inline]
    fn decay_pos(&self, decay: u16) -> i32 {
        ((35 * (distort(decay) as i32 + 1)) >> 16) + self.attack + 1
    }

    #[inline]
    fn sustain_pos(&self, sustain: u16) -> i32 {
        // sustain should not drop to the bottom as we want to show the release curve
        // even at sustain = 0
        BOTTOM - ((BOTTOM * ((sustain as i32 * 3) / 4 + 16384)) >> 16)
    }

    #[inline]
    fn release_pos(&self, release: u16) -> i32 {
        125 - ((35 * (distort(release) as i32 + 1)) >> 16)
    }

    #[inline(always)]
    async fn _draw_node(&mut self, top_left_x: i32, top_left_y: i32) {
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), _NODE_SIZE, self._fill_area)
            .await;
    }

    #[inline(always)]
    async fn _erase_node(&mut self, top_left_x: i32, top_left_y: i32) {
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), _NODE_SIZE, self.erase_area)
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

    #[inline(always)]
    async fn erase_x_range(&mut self, left: i32, right: i32) {
        self.display
            .driver
            .draw_rectangle(
                (left, TOP),
                (right - left + 1) as u32,
                (BOTTOM + 1) as u32,
                self.erase_area,
            )
            .await;
    }

    #[inline(always)]
    async fn draw_curve(&mut self, start: (i32, i32), end: (i32, i32)) {
        let p2_x = (start.0 + end.0) / 2;
        let p2_y = (end.1 * 2 + start.1) / 3;
        self.display
            .driver
            .draw_3p_curve(start, (p2_x, p2_y), end, BinaryColor::On)
            .await;
    }
}

#[inline]
fn distort(input: u16) -> u16 {
    let reverse = (!input) as u32;
    !(((reverse * reverse) >> 16) as u16)
}

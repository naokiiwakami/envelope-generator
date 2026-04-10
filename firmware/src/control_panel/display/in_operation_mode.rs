use defmt::debug;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::{Point, Size},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, StrokeAlignment},
};
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    definitions::{CvKind, PotKind},
    envelope_generator::{ConfigReader, EngineType, OutputPolarity},
    input_reader::PotInfo,
};

use super::{Display, ENGINE_TYPE_MENU_ITEMS, Mode, Request};

// CV assignment display constants
const POS_ATTACK: (i32, i32) = (7, 18);
const POS_DECAY: (i32, i32) = (43, 6);
const POS_SUSTAIN: (i32, i32) = (84, 6);
const POS_RELEASE: (i32, i32) = (121, 18);
const POS_EXTRA_1: (i32, i32) = (43, 30);
const POS_EXTRA_2: (i32, i32) = (84, 30);

const POS_CV_A: (i32, i32) = (49, 54);
const POS_CV_B: (i32, i32) = (78, 54);

const CV_A_TO_ATTACK: [(i32, i32); 4] = [POS_CV_A, (26, 40), (16, 30), POS_ATTACK];
const CV_A_TO_DECAY: [(i32, i32); 5] = [POS_CV_A, (34, 46), (27, 32), (32, 16), POS_DECAY];
const CV_A_TO_SUSTAIN: [(i32, i32); 4] = [POS_CV_A, (59, 36), (70, 21), POS_SUSTAIN];
const CV_A_TO_RELEASE: [(i32, i32); 5] = [POS_CV_A, (66, 45), (92, 43), (110, 33), POS_RELEASE];
const CV_A_TO_EXTRA_1: [(i32, i32); 3] = [POS_CV_A, (40, 42), POS_EXTRA_1];
const CV_A_TO_EXTRA_2: [(i32, i32); 3] = [POS_CV_A, (62, 41), POS_EXTRA_2];

const CV_KNOB_DIAMETER: u32 = 13;
const CV_JACK_DIAMETER: u32 = 9;

pub struct InOperationMode<'a> {
    display: &'a mut Display,

    eg_config: ConfigReader,

    // current displaying parameters
    attack: i32,
    decay: i32,
    sustain: i32,
    release: i32,

    cv_destination_a: PotKind,
    cv_destination_b: PotKind,

    // frequently used styles
    erase_area: PrimitiveStyle<BinaryColor>,
    fill_area: PrimitiveStyle<BinaryColor>,
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
            eg_config: ConfigReader::new(),
            attack: 0,
            decay: 0,
            sustain: 0,
            release: 0,
            cv_destination_a: PotKind::Decay,
            cv_destination_b: PotKind::Sustain,
            erase_area: PrimitiveStyleBuilder::new()
                .stroke_width(1)
                .stroke_color(BinaryColor::Off)
                .fill_color(BinaryColor::Off)
                .build(),
            fill_area: PrimitiveStyleBuilder::new()
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
                Request::GoToOpHome => self.show_home_page().await,
                Request::UpdatePot { pot_info } => self.update_pot(pot_info).await,
                Request::ShowPolarity {
                    polarity_1,
                    polarity_2,
                } => self.show_polarity(polarity_1, polarity_2).await,
                Request::SetPolarityChangeTargets { targets } => {
                    self.set_polarity_change_targets(targets).await
                }
                Request::UpdatePolarities {
                    targets,
                    polarity_1,
                    polarity_2,
                    is_draw,
                } => {
                    self.update_polarities(targets, polarity_1, polarity_2, is_draw)
                        .await
                }
                Request::ShowCvAssignment => self.show_cv_assignment().await,
                Request::UpdateCvAssignment {
                    source,
                    destination,
                } => self.update_cv_assignment(source, destination).await,
                Request::BlinkCvSource { source, turn_on } => {
                    self.blink_cv_source(source, turn_on).await
                }
                _ => {
                    self.display
                        .switch_mode(request.mode(), Some(request))
                        .await
                }
            }
        }
        // debug!("[InOperation]: out to {}", self.display.mode);
    }

    pub async fn show_home_page(&mut self) {
        let engine_type = self.eg_config.engine_type(0);
        let attack = self.eg_config.attack(0);
        let decay = self.eg_config.decay(0);
        let sustain = self.eg_config.sustain(0);
        let release = self.eg_config.release(0);
        let extra_1 = self.eg_config.extra_1(0);
        let extra_2 = self.eg_config.extra_2(0);

        debug!("showing {} home page", engine_type);

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
        let name = ENGINE_TYPE_MENU_ITEMS[(self.display.current_engine_type as u8) as usize].name;
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
        _extra_2: u16,
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

        self.draw_note_scaling_bar(self.eg_config.note_scaling_depth(0))
            .await;

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
                self.draw_note_scaling_bar(self.eg_config.note_scaling_depth(0))
                    .await;
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
        _extra_2: u16,
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

        self.draw_note_scaling_bar(self.eg_config.note_scaling_depth(0))
            .await;

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

                self.draw_line((LEFT, BOTTOM), (self.attack, TOP)).await;
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
                self.draw_note_scaling_bar(self.eg_config.note_scaling_depth(0))
                    .await;
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

    const POL_LEFT_EDGE: i32 = 97;
    const POL_TOP_1: i32 = 5;
    const POL_TOP_2: i32 = 37;
    const POL_DIAMETER: u32 = 21;

    async fn show_polarity(&mut self, polarity_1: OutputPolarity, polarity_2: OutputPolarity) {
        self.display.clear(false, false).await;
        self.display
            .driver
            .draw_string(
                "POLARITIES",
                TextBox::simple(0, 0, BinaryColor::On),
                FontSize::Medium,
            )
            .await;

        self.draw_polarity_jack(
            (Self::POL_LEFT_EDGE, Self::POL_TOP_1),
            Self::POL_DIAMETER,
            polarity_1,
        )
        .await;
        self.draw_polarity_jack(
            (Self::POL_LEFT_EDGE, Self::POL_TOP_2),
            Self::POL_DIAMETER,
            polarity_2,
        )
        .await;

        self.display.driver.flush().await;
    }

    async fn set_polarity_change_targets(&mut self, targets: u8) {
        let voice_1 = targets & 0x1 != 0;
        let voice_2 = targets & 0x2 != 0;
        self.display
            .clear_rectangle(Point::new(125, 0), Size::new(3, 64), false)
            .await;
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_alignment(StrokeAlignment::Center)
            .stroke_color(BinaryColor::On)
            .stroke_width(3)
            .build();
        if voice_1 {
            self.display
                .driver
                .draw_line((126, 5), (126, 26), stroke)
                .await;
        }
        if voice_2 {
            self.display
                .driver
                .draw_line((126, 37), (126, 58), stroke)
                .await;
        }
        self.display.driver.flush().await;
    }

    async fn update_polarities(
        &mut self,
        targets: u8,
        polarity_1: OutputPolarity,
        polarity_2: OutputPolarity,
        is_draw: bool,
    ) {
        let voice_1 = targets & 0x1 != 0;
        let voice_2 = targets & 0x2 != 0;

        if voice_1 {
            self.display
                .clear_rectangle(
                    Point::new(Self::POL_LEFT_EDGE, Self::POL_TOP_1),
                    Size::new(Self::POL_DIAMETER, Self::POL_DIAMETER),
                    false,
                )
                .await;
            if is_draw {
                self.draw_polarity_jack(
                    (Self::POL_LEFT_EDGE, Self::POL_TOP_1),
                    Self::POL_DIAMETER,
                    polarity_1,
                )
                .await;
            }
        }
        if voice_2 {
            self.display
                .clear_rectangle(
                    Point::new(Self::POL_LEFT_EDGE, Self::POL_TOP_2),
                    Size::new(Self::POL_DIAMETER, Self::POL_DIAMETER),
                    false,
                )
                .await;
            if is_draw {
                self.draw_polarity_jack(
                    (Self::POL_LEFT_EDGE, Self::POL_TOP_2),
                    Self::POL_DIAMETER,
                    polarity_2,
                )
                .await;
            }
        }
        self.display.driver.flush().await;
    }

    async fn draw_polarity_jack(
        &mut self,
        top_left: (i32, i32),
        diameter: u32,
        polarity: OutputPolarity,
    ) {
        let h_line_left = top_left.0 + 3;
        let h_line_right = top_left.0 + diameter as i32 - 4;
        let h_line_y = top_left.1 + (diameter / 2) as i32;

        let v_line_x = top_left.0 + (diameter / 2) as i32;
        let v_line_top = top_left.1 + 3;
        let v_line_bottom = top_left.1 + diameter as i32 - 4;

        // out 1
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_width(1)
            .stroke_color(BinaryColor::On)
            .build();
        self.display
            .driver
            .draw_circle(top_left, Self::POL_DIAMETER, stroke)
            .await;
        self.display
            .driver
            .draw_line((h_line_left, h_line_y), (h_line_right, h_line_y), stroke)
            .await;
        if matches!(polarity, OutputPolarity::Positive) {
            self.display
                .driver
                .draw_line((v_line_x, v_line_top), (v_line_x, v_line_bottom), stroke)
                .await;
        }
    }

    // CV Assignment ///////////////////////////////////////////////////////////

    async fn show_cv_assignment(&mut self) {
        self.display.clear(false, false).await;
        self.display
            .driver
            .draw_string(
                "CV",
                TextBox::simple(0, 50, BinaryColor::On),
                FontSize::Medium,
            )
            .await;

        let cv_destination_a = self.eg_config.cv_destination_a();
        let cv_destination_b = self.eg_config.cv_destination_b();
        self.cv_destination_a = cv_destination_a;
        self.cv_destination_b = cv_destination_b;
        let mut flags = [false; 8];
        flags[cv_destination_a as usize] = true;
        flags[cv_destination_b as usize] = true;

        self.draw_cv_pot(POS_ATTACK, flags[PotKind::Attack as usize])
            .await;
        self.draw_cv_pot(POS_DECAY, flags[PotKind::Decay as usize])
            .await;
        self.draw_cv_pot(POS_SUSTAIN, flags[PotKind::Sustain as usize])
            .await;
        self.draw_cv_pot(POS_RELEASE, flags[PotKind::Release as usize])
            .await;
        self.draw_cv_pot(POS_EXTRA_1, flags[PotKind::Extra1 as usize])
            .await;
        self.draw_cv_pot(POS_EXTRA_2, flags[PotKind::Extra2 as usize])
            .await;

        self.show_cv_source(POS_CV_A, true).await;
        self.show_cv_source(POS_CV_B, true).await;

        let mut mirror: [(i32, i32); 5] = [(0, 0); 5];

        if let Some(points) = path_from_a_to_dest(cv_destination_a) {
            self.display
                .driver
                .draw_spline(points, 18, BinaryColor::On)
                .await;
        };

        if let Some(points) = path_from_b_to_dest(cv_destination_b) {
            self.flip(points, &mut mirror);
            self.display
                .driver
                .draw_spline(&mirror[0..points.len()], 18, BinaryColor::On)
                .await;
        }

        self.display.driver.flush().await;
    }

    async fn update_cv_assignment(&mut self, source: CvKind, destination: PotKind) {
        defmt::debug!("update_cv_assignment: {:?} {:?}", source, destination);
        match source {
            CvKind::A => {
                if destination == self.cv_destination_a {
                    return;
                }
                if let Some(points) = path_from_a_to_dest(self.cv_destination_a) {
                    self.display
                        .driver
                        .draw_spline(points, 18, BinaryColor::Off)
                        .await;
                };
                if let Some(position) = pot_pos(self.cv_destination_a) {
                    self.draw_cv_node(*position, CV_KNOB_DIAMETER, false).await;
                }
                if let Some(position) = pot_pos(destination) {
                    self.draw_cv_node(*position, CV_KNOB_DIAMETER, true).await;
                }
                if let Some(points) = path_from_a_to_dest(destination) {
                    self.display
                        .driver
                        .draw_spline(points, 18, BinaryColor::On)
                        .await;
                }
                self.show_cv_source(POS_CV_A, true).await;
                // the erased line may have crossed with the other one. redraw.
                if let Some(points) = path_from_b_to_dest(self.cv_destination_b) {
                    let mut mirror: [(i32, i32); 5] = [(0, 0); 5];
                    self.flip(points, &mut mirror);
                    self.display
                        .driver
                        .draw_spline(&mirror[0..points.len()], 18, BinaryColor::On)
                        .await;
                }
                self.display.driver.flush().await;
                self.cv_destination_a = destination;
            }
            CvKind::B => {
                if destination == self.cv_destination_b {
                    return;
                }
                let mut mirror: [(i32, i32); 5] = [(0, 0); 5];
                if let Some(points) = path_from_b_to_dest(self.cv_destination_b) {
                    self.flip(points, &mut mirror);
                    self.display
                        .driver
                        .draw_spline(&mirror[0..points.len()], 18, BinaryColor::Off)
                        .await;
                };
                if let Some(position) = pot_pos(self.cv_destination_b) {
                    self.draw_cv_node(*position, CV_KNOB_DIAMETER, false).await;
                }
                if let Some(position) = pot_pos(destination) {
                    self.draw_cv_node(*position, CV_KNOB_DIAMETER, true).await;
                }
                if let Some(points) = path_from_b_to_dest(destination) {
                    self.flip(points, &mut mirror);
                    self.display
                        .driver
                        .draw_spline(&mirror[0..points.len()], 18, BinaryColor::On)
                        .await;
                }
                self.show_cv_source(POS_CV_B, true).await;
                // the erased line may have crossed with the other one. redraw.
                if let Some(points) = path_from_a_to_dest(self.cv_destination_a) {
                    self.display
                        .driver
                        .draw_spline(points, 18, BinaryColor::On)
                        .await;
                }
                self.display.driver.flush().await;
                self.cv_destination_b = destination;
            }
        }
    }

    async fn blink_cv_source(&mut self, source: CvKind, turn_on: bool) {
        match source {
            CvKind::A => self.show_cv_source(POS_CV_A, turn_on).await,
            CvKind::B => self.show_cv_source(POS_CV_B, turn_on).await,
        }
        self.display.driver.flush().await;
    }

    fn flip(&self, original: &[(i32, i32)], mirror: &mut [(i32, i32)]) {
        for i in 0..original.len() {
            mirror[i] = (127 - original[i].0, original[i].1);
        }
    }

    async fn draw_cv_pot(&mut self, center: (i32, i32), in_use: bool) {
        self.draw_cv_node(center, CV_KNOB_DIAMETER, in_use).await;
    }

    async fn draw_cv_node(&mut self, center: (i32, i32), diameter: u32, in_use: bool) {
        let top_left_x = center.0 - diameter as i32 / 2;
        let top_left_y = center.1 - diameter as i32 / 2;
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), diameter, self.erase_area)
            .await;
        let styled = PrimitiveStyleBuilder::new()
            .stroke_alignment(StrokeAlignment::Inside)
            .stroke_width(2)
            .stroke_color(BinaryColor::On)
            .build();
        self.display
            .driver
            .draw_circle(
                (top_left_x, top_left_y),
                diameter,
                if in_use { styled } else { self.fill_area },
            )
            .await;
        self.display
            .driver
            .draw_circle(
                (center.0 - 2, center.1 - 2),
                5,
                if in_use {
                    self.fill_area
                } else {
                    self.erase_area
                },
            )
            .await;
    }

    async fn show_cv_source(&mut self, center: (i32, i32), turn_on: bool) {
        let diameter = CV_JACK_DIAMETER;
        let top_left_x = center.0 - diameter as i32 / 2;
        let top_left_y = center.1 - diameter as i32 / 2;
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), diameter, self.erase_area)
            .await;
        self.display
            .driver
            .draw_circle((top_left_x, top_left_y), diameter, self.stroke)
            .await;
        if turn_on {
            self.display
                .driver
                .draw_circle((center.0 - 2, center.1 - 2), 5, self.fill_area)
                .await;
        }
    }

    // Utils /////////////////////////////////////////////////////////////////////////////////////

    #[inline]
    fn attack_pos(&self, attack: u16) -> i32 {
        ((35 * (distort(attack) as i32 + 1)) >> 16) + LEFT
    }

    #[inline]
    fn decay_pos(&self, decay: u16) -> i32 {
        ((35 * (distort(decay) as i32 + 1)) >> 16) + self.attack
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
    !(((((reverse * reverse) >> 16) * reverse) >> 16) as u16)
}

fn path_from_a_to_dest(destination: PotKind) -> Option<&'static [(i32, i32)]> {
    match destination {
        PotKind::Attack => Some(&CV_A_TO_ATTACK),
        PotKind::Decay => Some(&CV_A_TO_DECAY),
        PotKind::Sustain => Some(&CV_A_TO_SUSTAIN),
        PotKind::Release => Some(&CV_A_TO_RELEASE),
        PotKind::Extra1 => Some(&CV_A_TO_EXTRA_1),
        PotKind::Extra2 => Some(&CV_A_TO_EXTRA_2),
        _ => None,
    }
}

fn pot_pos(pot: PotKind) -> Option<&'static (i32, i32)> {
    match pot {
        PotKind::Attack => Some(&POS_ATTACK),
        PotKind::Decay => Some(&POS_DECAY),
        PotKind::Sustain => Some(&POS_SUSTAIN),
        PotKind::Release => Some(&POS_RELEASE),
        PotKind::Extra1 => Some(&POS_EXTRA_1),
        PotKind::Extra2 => Some(&POS_EXTRA_2),
        _ => None,
    }
}

fn path_from_b_to_dest(destination: PotKind) -> Option<&'static [(i32, i32)]> {
    match destination {
        PotKind::Attack => Some(&CV_A_TO_RELEASE),
        PotKind::Decay => Some(&CV_A_TO_SUSTAIN),
        PotKind::Sustain => Some(&CV_A_TO_DECAY),
        PotKind::Release => Some(&CV_A_TO_ATTACK),
        PotKind::Extra1 => Some(&CV_A_TO_EXTRA_2),
        PotKind::Extra2 => Some(&CV_A_TO_EXTRA_1),
        _ => None,
    }
}

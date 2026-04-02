mod in_operation_mode;
mod menu_mode;

use core::cmp::min;
use defmt::{debug, error};
use embassy_futures::yield_now;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{self, Channel},
};
use embassy_time::{Duration, Instant};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder},
};
use heapless::String;
use ssd1306_lite::{Angle, FontSize, Ssd1306Lite, TextBox};

use crate::{
    control_panel::{display::menu_mode::MenuMode, menu::ENGINE_TYPE_MENU_ITEMS},
    envelope_generator::EngineType,
    input_reader::{CvInfo, PotInfo},
};

use super::menu::ADMIN_MENU_ITEMS;

use self::in_operation_mode::InOperationMode;

pub const CHANNEL_LENGTH: usize = 4;
static CHANNEL_REQUEST: Channel<ThreadModeRawMutex, Request, CHANNEL_LENGTH> = Channel::new();

#[embassy_executor::task]
pub async fn run_display(mut eg_display: Display) {
    eg_display.run().await;
}

#[derive(PartialEq, Debug, defmt::Format)]
pub enum Mode {
    Any,
    Fundamental,
    InOperation,
    EngineTypeMenu,
    AdminMenu,
    PotsDiag,
    CvDiag,
}

pub enum Request {
    // Generic requests
    SwitchMode {
        mode: Mode,
    },
    Clear {
        reverse: bool,
        flush: bool,
    },
    Flush,
    DrawLine {
        start: Point,
        end: Point,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    },
    DrawCircle {
        top_left: Point,
        diameter: u32,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    },
    DrawRectangle {
        top_left: Point,
        size: Size,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    },
    DrawTriangle {
        vertex1: Point,
        vertex2: Point,
        vertex3: Point,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    },
    DrawArc {
        center: Point,
        radius: i32,
        start_degree: i32,
        end_degree: i32,
        color: bool,
        flush: bool,
    },
    // Fundamental requests
    GoToOpHome {
        engine_type: EngineType,
        attack: u16,
        decay: u16,
        sustain: u16,
        release: u16,
        extra_1: u16,
        extra_2: u16,
    },
    UpdatePot {
        pot_info: PotInfo,
    },
    ShowPolarity {
        polarity_1: i8,
        polarity_2: i8,
    },
    DisplayText {
        text: String<32>,
        text_box: TextBox,
        font_size: FontSize,
        flush: bool,
    },
    // Menu requests
    DisplayEngineTypeMenuItem {
        index: usize,
    },
    DisplayAdminMenuItem {
        index: usize,
    },
    // PotsDiag requests
    UpdatePotForDiag {
        pot_info: PotInfo,
    },
    // CvDiag requests
    UpdateCvValues {
        cv_info: CvInfo,
    },
}

impl Request {
    fn mode(&self) -> Mode {
        match self {
            Request::SwitchMode { .. }
            | Request::Clear { .. }
            | Request::Flush
            | Request::DrawLine { .. }
            | Request::DrawCircle { .. }
            | Request::DrawRectangle { .. }
            | Request::DrawTriangle { .. }
            | Request::DrawArc { .. }
            | Request::DisplayText { .. } => Mode::Any,
            Request::GoToOpHome { .. }
            | Request::UpdatePot { .. }
            | Request::ShowPolarity { .. } => Mode::InOperation,
            Request::DisplayEngineTypeMenuItem { .. } => Mode::EngineTypeMenu,
            Request::DisplayAdminMenuItem { .. } => Mode::AdminMenu,
            Request::UpdatePotForDiag { .. } => Mode::PotsDiag,
            Request::UpdateCvValues { .. } => Mode::CvDiag,
        }
    }

    fn name(&self) -> &str {
        match self {
            Request::SwitchMode { .. } => "SwitchMode",
            Request::Clear { .. } => "Clear",
            Request::Flush => "Flush",
            Request::DrawLine { .. } => "DrawLine",
            Request::DrawCircle { .. } => "DrawCircle",
            Request::DrawRectangle { .. } => "DrawRectangle",
            Request::DrawTriangle { .. } => "DrawTriangle",
            Request::DrawArc { .. } => "DrawArc",
            Request::DisplayText { .. } => "DisplayText",
            Request::GoToOpHome { .. } => "GoToOpHome",
            Request::UpdatePot { .. } => "UpdatePot",
            Request::ShowPolarity { .. } => "ShowPolarity",
            Request::DisplayEngineTypeMenuItem { .. } => "DisplayEngineTypeMenuItem",
            Request::DisplayAdminMenuItem { .. } => "DisplayAdminMenuItem",
            Request::UpdatePotForDiag { .. } => "UpdatePotForDiag",
            Request::UpdateCvValues { .. } => "UpdateCvValues",
        }
    }
}

pub fn get_request_sender() -> channel::Sender<'static, ThreadModeRawMutex, Request, CHANNEL_LENGTH>
{
    CHANNEL_REQUEST.sender()
}

pub struct Display {
    driver: Ssd1306Lite<I2c<'static, Async, Master>>,

    mode: Mode,
    current_engine_type: EngineType,
    request_receiver: channel::Receiver<'static, ThreadModeRawMutex, Request, CHANNEL_LENGTH>,
    pending_request: Option<Request>,
}

impl Display {
    pub fn new(i2c: I2c<'static, Async, Master>) -> Self {
        let mut driver = Ssd1306Lite::new(i2c);
        driver.set_yield_interval(Duration::from_micros(15));

        Self {
            driver,
            mode: Mode::Fundamental,
            current_engine_type: EngineType::Adsr,
            request_receiver: CHANNEL_REQUEST.receiver(),
            pending_request: None,
        }
    }

    pub async fn run(&mut self) {
        self.driver.initialize().await;
        debug!("initialized");
        loop {
            match self.mode {
                Mode::Any => {} // Generic requests don't have a specific mode
                Mode::Fundamental => self.run_fundamental_mode().await,
                Mode::InOperation => self.run_in_operation_mode().await,
                Mode::EngineTypeMenu => self.run_engine_type_menu_mode().await,
                Mode::AdminMenu => self.run_admin_menu_mode().await,
                Mode::PotsDiag => self.run_pots_diag_mode().await,
                Mode::CvDiag => self.run_cv_diag_mode().await,
            };
        }
    }

    async fn handle_generic_request(&mut self, request: Request) {
        match request {
            Request::SwitchMode { mode } => self.switch_mode(mode, None).await,
            Request::Clear { reverse, flush } => self.clear(reverse, flush).await,
            Request::Flush => self.driver.flush().await,
            Request::DrawLine {
                start,
                end,
                style,
                flush,
            } => self.draw_line(start, end, style, flush).await,
            Request::DrawCircle {
                top_left,
                diameter,
                style,
                flush,
            } => self.draw_circle(top_left, diameter, style, flush).await,
            Request::DrawRectangle {
                top_left,
                size,
                style,
                flush,
            } => self.draw_rectangle(top_left, size, style, flush).await,
            Request::DrawTriangle {
                vertex1,
                vertex2,
                vertex3,
                style,
                flush,
            } => {
                self.draw_triangle(vertex1, vertex2, vertex3, style, flush)
                    .await
            }
            Request::DrawArc {
                center,
                radius,
                start_degree,
                end_degree,
                color,
                flush,
            } => {
                self.draw_arc(center, radius, start_degree, end_degree, color, flush)
                    .await
            }
            Request::DisplayText {
                text,
                text_box,
                font_size,
                flush,
            } => {
                self.display_text(text.as_str(), text_box, font_size, flush)
                    .await
            }

            _ => {} // Other requests shouldn't reach here
        }
    }

    // fundamental mode ///////////////////////////////////////////////////////////

    async fn into_fundamental_mode(&mut self, pending_request: Option<Request>) {
        self.mode = Mode::Fundamental;
        self.pending_request = pending_request;
    }

    async fn run_fundamental_mode(&mut self) {
        debug!("In fundamental mode");
        while matches!(self.mode, Mode::Fundamental) {
            let request = self.fetch_request().await;
            debug!("[Fundamental]: request: {}", request.name());
            match request {
                Request::SwitchMode { .. }
                | Request::Clear { .. }
                | Request::Flush
                | Request::DrawLine { .. }
                | Request::DrawCircle { .. }
                | Request::DrawRectangle { .. }
                | Request::DrawTriangle { .. }
                | Request::DrawArc { .. }
                | Request::DisplayText { .. } => self.handle_generic_request(request).await,
                _ => self.switch_mode(request.mode(), Some(request)).await,
            }
        }
        debug!("[Fundamental] out to {}", self.mode);
    }

    // operational mode ///////////////////////////////////////////////////////////

    async fn into_in_operation_mode(&mut self, pending_request: Option<Request>) {
        self.mode = Mode::InOperation;
        self.pending_request = pending_request;
    }

    async fn run_in_operation_mode(&mut self) {
        let mut in_operation_mode = InOperationMode::new(self);
        in_operation_mode.run().await;
    }

    // op menu mode /////////////////////////////////////////////////////////

    async fn run_engine_type_menu_mode(&mut self) {
        let mut menu = MenuMode::new(self, "EG TYPE", &ENGINE_TYPE_MENU_ITEMS);
        menu.run().await;
    }

    // admin menu mode /////////////////////////////////////////////////////

    async fn run_admin_menu_mode(&mut self) {
        let mut menu = MenuMode::new(self, "ADMIN", &ADMIN_MENU_ITEMS);
        menu.run().await;
    }

    // pots diag mode //////////////////////////////////////////////////////

    async fn into_pots_diag_mode(&mut self, pending_request: Option<Request>) {
        self.clear(false, false).await;
        self.mode = Mode::PotsDiag;
        self.pending_request = pending_request;
    }

    async fn run_pots_diag_mode(&mut self) {
        let positions = [
            (Point::new(2, 2), Point::new(10, 10)),
            (Point::new(34, 2), Point::new(42, 10)),
            (Point::new(66, 2), Point::new(74, 10)),
            (Point::new(98, 2), Point::new(106, 10)),
            (Point::new(2, 34), Point::new(10, 42)),
            (Point::new(98, 34), Point::new(106, 42)),
            (Point::new(34, 34), Point::new(42, 42)),
            (Point::new(66, 34), Point::new(74, 42)),
        ];
        self.driver.clear(BinaryColor::Off).await;
        let erase = PrimitiveStyleBuilder::new()
            .stroke_width(5)
            .stroke_color(BinaryColor::Off)
            .build();
        let fill = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .stroke_width(5)
            .build();
        for index in 0..positions.len() {
            let (_, position) = positions[index];
            self.driver
                .draw_circle((position.x, position.y), 12, fill)
                .await;
        }

        while matches!(self.mode, Mode::PotsDiag) {
            let request = self.fetch_request().await;
            match request {
                Request::SwitchMode { .. }
                | Request::Clear { .. }
                | Request::Flush
                | Request::DrawLine { .. }
                | Request::DrawCircle { .. }
                | Request::DrawRectangle { .. }
                | Request::DrawTriangle { .. }
                | Request::DrawArc { .. }
                | Request::DisplayText { .. } => self.handle_generic_request(request).await,
                Request::UpdatePotForDiag { pot_info: info } => {
                    self.update_pot_value(info, &erase, &positions).await
                }
                _ => self.switch_mode(request.mode(), Some(request)).await,
            }
        }
    }

    async fn update_pot_value(
        &mut self,
        pot_info: PotInfo,
        erase: &PrimitiveStyle<BinaryColor>,
        positions: &[(Point, Point); 8],
    ) {
        let index = pot_info.kind.clone() as usize;
        if index >= positions.len() {
            error!("update_pot_value: Index out of bounds; index={}", index);
            return;
        }
        let (position, _) = positions[index];
        self.driver
            .draw_circle((position.x, position.y), 28, *erase)
            .await;

        let start = Angle::from_degrees(120);
        let mut sweep: u32 = 300 * pot_info.value as u32;
        sweep >>= 16;
        let end = Angle::from_degrees(120 + sweep as i32);
        self.driver
            .draw_arc(position.x + 14, position.y + 14, 14, start, end, true)
            .await;
    }

    // CV diag mode //////////////////////////////////////////////////////
    async fn into_cv_diag_mode(&mut self, pending_request: Option<Request>) {
        self.clear(false, false).await;
        self.display_text(
            "Put something into CV",
            TextBox::simple(0, 0, BinaryColor::On),
            FontSize::Small,
            true,
        )
        .await;
        self.mode = Mode::CvDiag;
        self.pending_request = pending_request;
    }

    async fn run_cv_diag_mode(&mut self) {
        let mut cv_points = CvPoints {
            data1: [0; 128],
            data2: [0; 128],
            head: 0,
            data_len: 0,
        };
        while matches!(self.mode, Mode::CvDiag) {
            let request = self.fetch_request().await;
            match request {
                Request::SwitchMode { .. }
                | Request::Clear { .. }
                | Request::Flush
                | Request::DrawLine { .. }
                | Request::DrawCircle { .. }
                | Request::DrawRectangle { .. }
                | Request::DrawTriangle { .. }
                | Request::DrawArc { .. }
                | Request::DisplayText { .. } => self.handle_generic_request(request).await,
                Request::UpdateCvValues { cv_info } => {
                    self.update_cv_info(cv_info, &mut cv_points).await
                }
                _ => self.switch_mode(request.mode(), Some(request)).await,
            }
        }
    }

    async fn update_cv_info(&mut self, cv_info: CvInfo, cv_points: &mut CvPoints) {
        let data_len = min(cv_points.data_len + 1, cv_points.data1.len());
        let erase_needed = data_len == cv_points.data1.len();
        let mut last_yield = Instant::now();
        for i in 0..data_len {
            if erase_needed {
                let erase_index = (i + cv_points.head) % data_len;
                let mut erase_y = 16u32 + 22u32 - cv_points.data1[erase_index] as u32;
                self.driver.unset_pixel(i as u32, erase_y);
                erase_y = 16u32 + 24u32 + 22u32 - cv_points.data2[erase_index] as u32;
                self.driver.unset_pixel(i as u32, erase_y);
            }
            let set_index = (i + 1 + cv_points.head) % data_len;
            if set_index == cv_points.head {
                // scale value of range [-32768..32767] to [0..19]
                cv_points.data1[set_index] =
                    (((cv_info.cv_a as i32 + 32768) as u32 * 304) >> 20) as u8;
                cv_points.data2[set_index] =
                    (((cv_info.cv_b as i32 + 32768) as u32 * 304) >> 20) as u8;
            }
            let mut set_y = 16u32 + 22u32 - cv_points.data1[set_index] as u32;
            self.driver.set_pixel(i as u32, set_y);
            set_y = 16u32 + 24u32 + 22u32 - cv_points.data2[set_index] as u32;
            self.driver.set_pixel(i as u32, set_y);
            if last_yield.elapsed().as_micros() > 15 {
                yield_now().await;
                last_yield = Instant::now();
            }
        }
        self.driver.flush().await;

        cv_points.data_len = data_len;
        cv_points.head = (cv_points.head + 1) % cv_points.data1.len();
    }

    // utils /////////////////////////////////////////////////////////////

    /// Switch into a menu mode.
    async fn into_menu_mode(&mut self, menu_mode: Mode, pending_request: Option<Request>) {
        self.clear(false, false).await;
        self.mode = menu_mode;
        self.pending_request = pending_request;
    }

    /// Fetches next request to process. The function returns immediately
    /// if there's a pending request, otherwise hangs on the request receiver
    /// for the next incoming request.
    async fn fetch_request(&mut self) -> Request {
        match self.pending_request.take() {
            Some(request) => request,
            None => self.request_receiver.receive().await,
        }
    }

    /// Switches display mode based on the request kind
    async fn switch_mode(&mut self, mode: Mode, request: Option<Request>) {
        debug!("switch_mode, Switching mode to {:?}", mode);
        match mode {
            Mode::Fundamental => self.into_fundamental_mode(request).await,
            Mode::InOperation => self.into_in_operation_mode(request).await,
            Mode::EngineTypeMenu => self.into_menu_mode(Mode::EngineTypeMenu, request).await,
            Mode::AdminMenu => self.into_menu_mode(Mode::AdminMenu, request).await,
            Mode::PotsDiag => self.into_pots_diag_mode(request).await,
            Mode::CvDiag => self.into_cv_diag_mode(request).await,
            Mode::Any => {} // Generic requests shouldn't reach here.
        }
    }

    async fn clear(&mut self, reverse: bool, flush: bool) {
        self.driver
            .clear(if reverse {
                BinaryColor::On
            } else {
                BinaryColor::Off
            })
            .await;
        if flush {
            self.driver.flush_full().await;
        } else {
            yield_now().await;
        }
    }

    async fn draw_line(
        &mut self,
        start: Point,
        end: Point,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    ) {
        self.driver
            .draw_line((start.x, start.y), (end.x, end.y), style)
            .await;
        if flush {
            self.driver.flush().await;
        }
    }

    async fn draw_circle(
        &mut self,
        top_left: Point,
        diameter: u32,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    ) {
        self.driver
            .draw_circle((top_left.x, top_left.y), diameter, style)
            .await;
        if flush {
            self.driver.flush().await;
        }
    }

    async fn draw_rectangle(
        &mut self,
        top_left: Point,
        size: Size,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    ) {
        self.driver
            .draw_rectangle((top_left.x, top_left.y), size.width, size.height, style)
            .await;
        if flush {
            self.driver.flush().await;
        }
    }

    async fn draw_triangle(
        &mut self,
        vertex1: Point,
        vertex2: Point,
        vertex3: Point,
        style: PrimitiveStyle<BinaryColor>,
        flush: bool,
    ) {
        self.driver
            .draw_triangle(
                (vertex1.x, vertex1.y),
                (vertex2.x, vertex2.y),
                (vertex3.x, vertex3.y),
                style,
            )
            .await;
        if flush {
            self.driver.flush().await;
        }
    }

    async fn draw_arc(
        &mut self,
        center: Point,
        radius: i32,
        start_degree: i32,
        end_degree: i32,
        color: bool,
        flush: bool,
    ) {
        self.driver
            .draw_arc(
                center.x,
                center.y,
                radius,
                Angle::from_degrees(start_degree),
                Angle::from_degrees(end_degree),
                color,
            )
            .await;
        if flush {
            self.driver.flush().await;
        }
    }

    async fn display_text(
        &mut self,
        text: &str,
        text_box: TextBox,
        font_size: FontSize,
        flush: bool,
    ) {
        self.driver.draw_string(text, text_box, font_size).await;
        if flush {
            self.driver.flush_full().await;
        } else {
            yield_now().await;
        }
    }
}

struct CvPoints {
    pub data1: [u8; 128],
    pub data2: [u8; 128],
    pub head: usize,
    pub data_len: usize,
}

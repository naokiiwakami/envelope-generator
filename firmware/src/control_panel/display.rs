mod in_operation_mode;
mod menu_mode;

use analog3::{
    IndicatorRequest,
    rng::{LocalRng, make_local_rng},
};
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
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder},
};
use heapless::String;
use ssd1306_lite::{Angle, FontSize, Ssd1306Lite, TextBox};

use crate::{
    control_panel::{display::menu_mode::MenuMode, menu::ENGINE_TYPE_MENU_ITEMS},
    envelope_generator::{EngineType, OutputPolarity},
    input_reader::{CvInfo, PotInfo},
    patch_controller::{LedColor, PatchControllerRequest, get_patch_controller_request_sender},
};

use super::menu::ADMIN_MENU_ITEMS;

use self::in_operation_mode::InOperationMode;

pub const CHANNEL_LENGTH: usize = 4;
static CHANNEL_REQUEST: Channel<ThreadModeRawMutex, Request, CHANNEL_LENGTH> = Channel::new();

#[embassy_executor::task]
pub async fn run_display(mut display: Display) {
    display.run().await;
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
        radius: u32,
        start_degree: i32,
        end_degree: i32,
        color: BinaryColor,
        flush: bool,
    },
    // Fundamental requests
    GoToOpHome,
    UpdatePot {
        pot_info: PotInfo,
    },
    ShowPolarity {
        polarity_1: OutputPolarity,
        polarity_2: OutputPolarity,
    },
    SetPolarityChangeTargets {
        targets: u8,
    },
    UpdatePolarities {
        targets: u8,
        polarity_1: OutputPolarity,
        polarity_2: OutputPolarity,
        is_draw: bool,
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
            Self::SwitchMode { .. }
            | Self::Clear { .. }
            | Self::Flush
            | Self::DrawLine { .. }
            | Self::DrawCircle { .. }
            | Self::DrawRectangle { .. }
            | Self::DrawTriangle { .. }
            | Self::DrawArc { .. }
            | Self::DisplayText { .. } => Mode::Any,
            Self::GoToOpHome { .. }
            | Self::UpdatePot { .. }
            | Self::ShowPolarity { .. }
            | Self::SetPolarityChangeTargets { .. }
            | Self::UpdatePolarities { .. } => Mode::InOperation,
            Self::DisplayEngineTypeMenuItem { .. } => Mode::EngineTypeMenu,
            Self::DisplayAdminMenuItem { .. } => Mode::AdminMenu,
            Self::UpdatePotForDiag { .. } => Mode::PotsDiag,
            Self::UpdateCvValues { .. } => Mode::CvDiag,
        }
    }

    fn name(&self) -> &str {
        match self {
            Self::SwitchMode { .. } => "SwitchMode",
            Self::Clear { .. } => "Clear",
            Self::Flush => "Flush",
            Self::DrawLine { .. } => "DrawLine",
            Self::DrawCircle { .. } => "DrawCircle",
            Self::DrawRectangle { .. } => "DrawRectangle",
            Self::DrawTriangle { .. } => "DrawTriangle",
            Self::DrawArc { .. } => "DrawArc",
            Self::DisplayText { .. } => "DisplayText",
            Self::GoToOpHome { .. } => "GoToOpHome",
            Self::UpdatePot { .. } => "UpdatePot",
            Self::ShowPolarity { .. } => "ShowPolarity",
            Self::SetPolarityChangeTargets { .. } => "SetPolarityChangeTargets",
            Self::UpdatePolarities { .. } => "UpdatePolarities",
            Self::DisplayEngineTypeMenuItem { .. } => "DisplayEngineTypeMenuItem",
            Self::DisplayAdminMenuItem { .. } => "DisplayAdminMenuItem",
            Self::UpdatePotForDiag { .. } => "UpdatePotForDiag",
            Self::UpdateCvValues { .. } => "UpdateCvValues",
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

    pub async fn initialize(&mut self) {
        self.driver.initialize().await;
        debug!("initialized");
    }

    pub async fn run(&mut self) {
        self.splash_screen().await;
        // Analog3 halts until someone gives a go. The control panel is more ideal
        // one to conduct the entire app behavior, but it's simpler to implement with
        // nudging here.
        analog3::start_operation().await;
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

    pub async fn splash_screen(&mut self) {
        let blank = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::Off)
            .build();
        let rng = make_local_rng();
        let mut patch_controller_blinker = InitialBlinker::new(&rng);
        let pc_request_sender = get_patch_controller_request_sender();
        let mut a3_blinker = InitialBlinker::new(&rng);
        let a3_request_sender = analog3::get_indicator_request_sender();
        for threshold in (0..16).rev() {
            // silently ignore what received while showing the splash screen
            let _ = self.request_receiver.try_receive();
            for y in 0..64 {
                for x in 0..32 {
                    let mut random = rng.random_u64();
                    for seg in 0..8 {
                        if random & 0x1f < threshold {
                            self.driver.set_pixel(x * 8 + seg, y);
                        } else {
                            self.driver.unset_pixel(x * 8 + seg, y);
                        }
                        random >>= 8;
                    }
                    if patch_controller_blinker.check() {
                        pc_request_sender
                            .send(PatchControllerRequest::OperateIndicator {
                                led_color: if patch_controller_blinker.index == 0 {
                                    LedColor::Red
                                } else {
                                    LedColor::Green
                                },
                                is_high: patch_controller_blinker.turn_on,
                            })
                            .await;
                    }
                    if a3_blinker.check() {
                        let request = if a3_blinker.turn_on {
                            if a3_blinker.index == 0 {
                                IndicatorRequest::SetRedLed
                            } else {
                                IndicatorRequest::SetBlueLed
                            }
                        } else {
                            if a3_blinker.index == 0 {
                                IndicatorRequest::ResetRedLed
                            } else {
                                IndicatorRequest::ResetBlueLed
                            }
                        };
                        a3_request_sender.send(request).await;
                    }
                }
                yield_now().await;
            }
            self.driver
                .draw_triangle((28, 50), (100, 50), (64, 14), blank)
                .await;
            self.driver.flush().await;
        }
        pc_request_sender
            .send(PatchControllerRequest::OperateIndicator {
                led_color: LedColor::Red,
                is_high: false,
            })
            .await;
        pc_request_sender
            .send(PatchControllerRequest::OperateIndicator {
                led_color: LedColor::Green,
                is_high: false,
            })
            .await;
        a3_request_sender.send(IndicatorRequest::ResetRedLed).await;
        a3_request_sender.send(IndicatorRequest::ResetBlueLed).await;
        Timer::after_millis(300).await;
        self.pending_request = Some(Request::GoToOpHome);
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
        let index = pot_info.kind as usize;
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
            .draw_arc(
                position.x + 14,
                position.y + 14,
                14,
                start,
                end,
                BinaryColor::On,
            )
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
                let mut erase_y = 16i32 + 22i32 - cv_points.data1[erase_index] as i32;
                self.driver.unset_pixel(i as i32, erase_y);
                erase_y = 16i32 + 24i32 + 22i32 - cv_points.data2[erase_index] as i32;
                self.driver.unset_pixel(i as i32, erase_y);
            }
            let set_index = (i + 1 + cv_points.head) % data_len;
            if set_index == cv_points.head {
                // scale value of range [-32768..32767] to [0..19]
                cv_points.data1[set_index] =
                    (((cv_info.cv_a as i32 + 32768) as u32 * 304) >> 20) as u8;
                cv_points.data2[set_index] =
                    (((cv_info.cv_b as i32 + 32768) as u32 * 304) >> 20) as u8;
            }
            let mut set_y = 16i32 + 22i32 - cv_points.data1[set_index] as i32;
            self.driver.set_pixel(i as i32, set_y);
            set_y = 16i32 + 24i32 + 22i32 - cv_points.data2[set_index] as i32;
            self.driver.set_pixel(i as i32, set_y);
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

    #[inline]
    async fn clear_rectangle(&mut self, top_left: Point, size: Size, flush: bool) {
        let erase = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::Off)
            .build();
        self.draw_rectangle(top_left, size, erase, flush).await;
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
        radius: u32,
        start_degree: i32,
        end_degree: i32,
        color: BinaryColor,
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

/// Controls initial blink
struct InitialBlinker<'a> {
    rng: &'a LocalRng,
    last_lit: Instant,
    interval_c: u64,
    interval: Duration,
    pub remaining: usize,
    pub index: usize,
    pub turn_on: bool,
}

impl<'a> InitialBlinker<'a> {
    pub fn new(rng: &'a LocalRng) -> Self {
        let interval_c = 100;
        let interval = Duration::from_millis(rng.random_u64() % interval_c + interval_c / 2);
        Self {
            rng,
            last_lit: Instant::now(),
            interval_c,
            interval,
            remaining: 6,
            index: 0,
            turn_on: false,
        }
    }

    /// Checks whether it's time to do any blinking action.
    /// The user should check properties `turn_on` and `index` to determine what to do.
    /// If `turn_on` is true, the user should turn on an indicator LED for the `index`.
    pub fn check(&mut self) -> bool {
        if self.remaining > 0 && self.last_lit.elapsed() >= self.interval {
            if self.turn_on {
                self.turn_on = false;
                self.interval = Duration::from_millis(
                    self.rng.random_u64() % self.interval_c + self.interval_c / 2,
                );
                self.remaining -= 1;
                return true;
            } else {
                self.turn_on = true;
                self.index = (self.rng.random_u64() % 2) as usize;
                self.interval_c = self.interval_c * 3 / 2;
                self.interval = Duration::from_millis(30);
                self.last_lit = Instant::now();
                return true;
            }
        }
        false
    }
}

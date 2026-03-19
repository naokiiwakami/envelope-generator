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
use embedded_graphics::{
    mono_font::{
        MonoTextStyleBuilder,
        ascii::{FONT_5X8, FONT_8X13, FONT_8X13_BOLD, FONT_10X20},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Arc, Circle, PrimitiveStyle, PrimitiveStyleBuilder},
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306::{
    I2CDisplayInterface, Ssd1306Async,
    mode::{BufferedGraphicsModeAsync, DisplayConfigAsync},
    prelude::{DisplayRotation, I2CInterface},
    size::DisplaySize128x64,
};

use crate::{
    control_panel::{display::menu_mode::MenuMode, menu::ENGINE_TYPE_MENU_ITEMS},
    envelope_generator::EngineType,
    input_reader::{CvInfo, PotInfo},
};

use super::menu::{ADMIN_MENU_ITEMS, OP_MENU_ITEMS};

pub const CHANNEL_LENGTH: usize = 4;
static CHANNEL_REQUEST: Channel<ThreadModeRawMutex, Request, CHANNEL_LENGTH> = Channel::new();

#[embassy_executor::task]
pub async fn run_display(mut eg_display: EgDisplay) {
    eg_display.run().await;
}

pub enum FontSize {
    Small,
    Medium,
    Large,
}

#[derive(PartialEq)]
enum Mode {
    Any,
    Fundamental,
    OpMenu,
    EngineTypeMenu,
    AdminMenu,
    PotsDiag,
    CvDiag,
}

pub enum Request {
    // Generic requests
    Clear {
        reverse: bool,
        flush: bool,
    },
    Flush,
    // Fundamental requests
    ShowInitialScreen {
        engine_type: EngineType,
    },
    DisplayText {
        reverse: bool,
        flush: bool,
        text: String<32>,
        font_size: FontSize,
        position: Point,
    },
    // Menu requests
    DisplayOpMenuItem {
        index: usize,
    },
    DisplayEngineTypeMenuItem {
        index: usize,
    },
    DisplayAdminMenuItem {
        index: usize,
    },
    // PotsDiag requests
    UpdatePotValue {
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
            Request::Clear { .. } | Request::Flush => Mode::Any,
            Request::ShowInitialScreen { .. } | Request::DisplayText { .. } => Mode::Fundamental,
            Request::DisplayOpMenuItem { .. } => Mode::OpMenu,
            Request::DisplayEngineTypeMenuItem { .. } => Mode::EngineTypeMenu,
            Request::DisplayAdminMenuItem { .. } => Mode::AdminMenu,
            Request::UpdatePotValue { .. } => Mode::PotsDiag,
            Request::UpdateCvValues { .. } => Mode::CvDiag,
        }
    }
}

pub fn get_request_sender() -> channel::Sender<'static, ThreadModeRawMutex, Request, CHANNEL_LENGTH>
{
    CHANNEL_REQUEST.sender()
}

pub struct EgDisplay {
    display: Ssd1306Async<
        I2CInterface<I2c<'static, Async, Master>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,

    mode: Mode,
    current_engine_type: EngineType,
    request_receiver: channel::Receiver<'static, ThreadModeRawMutex, Request, CHANNEL_LENGTH>,
    pending_request: Option<Request>,
}

impl EgDisplay {
    pub fn new(i2c: I2c<'static, Async, Master>) -> Self {
        let interface = I2CDisplayInterface::new(i2c);
        let display = Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        Self {
            display,
            mode: Mode::Fundamental,
            current_engine_type: EngineType::ADSR,
            request_receiver: CHANNEL_REQUEST.receiver(),
            pending_request: None,
        }
    }

    pub async fn run(&mut self) {
        self.display.init().await.unwrap();
        loop {
            match self.mode {
                Mode::Any => {} // Generic requests don't have a specific mode
                Mode::Fundamental => self.run_fundamental_mode().await,
                Mode::OpMenu => self.run_op_menu_mode().await,
                Mode::EngineTypeMenu => self.run_engine_type_menu_mode().await,
                Mode::AdminMenu => self.run_admin_menu_mode().await,
                Mode::PotsDiag => self.run_pots_diag_mode().await,
                Mode::CvDiag => self.run_cv_diag_mode().await,
            };
        }
    }

    async fn handle_generic_request(&mut self, request: Request) {
        match request {
            Request::Clear { reverse, flush } => {
                debug!(
                    "received clear request; reverse={}, flush={}",
                    reverse, flush
                );
                self.clear(reverse, flush).await
            }
            Request::Flush => self.display.flush().await.unwrap(),
            _ => {} // Other requests shouldn't reach here
        }
    }

    // fundamental mode ///////////////////////////////////////////////////////////

    async fn into_fundamental_mode(&mut self, pending_request: Request) {
        self.mode = Mode::Fundamental;
        self.pending_request = Some(pending_request);
    }

    async fn run_fundamental_mode(&mut self) {
        while matches!(self.mode, Mode::Fundamental) {
            let request = self.fetch_request().await;
            match request {
                Request::Clear { .. } | Request::Flush => {
                    self.handle_generic_request(request).await
                }
                Request::ShowInitialScreen { engine_type } => {
                    self.show_initial_screen(engine_type).await
                }
                Request::DisplayText {
                    reverse,
                    flush,
                    text,
                    font_size,
                    position,
                } => {
                    self.display_text(reverse, flush, text.as_str(), font_size, position)
                        .await
                }
                _ => self.switch_mode(request).await,
            }
        }
    }

    pub async fn show_initial_screen(&mut self, engine_type: EngineType) {
        self.mode = Mode::Fundamental;
        self.current_engine_type = engine_type;
        match self.current_engine_type {
            EngineType::ADSR => self.show_adsr_initial_screen().await,
            _ => self.show_default_initial_screen().await,
        }
    }

    async fn show_adsr_initial_screen(&mut self) {
        self.clear(false, false).await;
        yield_now().await;
        let fill = PrimitiveStyleBuilder::new()
            .fill_color(BinaryColor::On)
            .stroke_width(5)
            .build();
        yield_now().await;
        Circle::new(Point::new(0, 20), 24)
            .into_styled(fill)
            .draw(&mut self.display)
            .unwrap();
        yield_now().await;
        Circle::new(Point::new(33, 0), 24)
            .into_styled(fill)
            .draw(&mut self.display)
            .unwrap();
        yield_now().await;
        Circle::new(Point::new(67, 0), 24)
            .into_styled(fill)
            .draw(&mut self.display)
            .unwrap();
        yield_now().await;
        Circle::new(Point::new(104, 20), 24)
            .into_styled(fill)
            .draw(&mut self.display)
            .unwrap();
        self.display.flush().await.unwrap();
    }

    async fn show_default_initial_screen(&mut self) {
        self.clear(false, false).await;
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        let name = ENGINE_TYPE_MENU_ITEMS[(self.current_engine_type.clone() as u8) as usize].name;
        Text::with_baseline(name, Point::new(20, 20), text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        self.display.flush().await.unwrap();
    }

    // op menu mode /////////////////////////////////////////////////////////

    async fn run_op_menu_mode(&mut self) {
        let mut menu = MenuMode::new(self, "SETTINGS", &OP_MENU_ITEMS);
        menu.run().await;
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

    async fn into_pots_diag_mode(&mut self, pending_request: Request) {
        self.clear(false, false).await;
        self.mode = Mode::PotsDiag;
        self.pending_request = Some(pending_request);
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
        self.display.clear(BinaryColor::Off).unwrap();
        let stroke = PrimitiveStyleBuilder::new()
            .stroke_width(1)
            .stroke_color(BinaryColor::On)
            .build();
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
            Circle::new(position, 12)
                .into_styled(fill)
                .draw(&mut self.display)
                .unwrap();
        }

        while matches!(self.mode, Mode::PotsDiag) {
            let request = self.fetch_request().await;
            match request {
                Request::Clear { .. } | Request::Flush => {
                    self.handle_generic_request(request).await
                }
                Request::UpdatePotValue { pot_info: info } => {
                    self.update_pot_value(info, &stroke, &erase, &positions)
                        .await
                }
                _ => self.switch_mode(request).await,
            }
        }
    }

    async fn update_pot_value(
        &mut self,
        pot_info: PotInfo,
        stroke: &PrimitiveStyle<BinaryColor>,
        erase: &PrimitiveStyle<BinaryColor>,
        positions: &[(Point, Point); 8],
    ) {
        let index = pot_info.kind.clone() as usize;
        if index >= positions.len() {
            error!("update_pot_value: Index out of bounds; index={}", index);
            return;
        }
        let (position, _) = positions[index];
        Circle::new(position, 28)
            .into_styled(*erase)
            .draw(&mut self.display)
            .unwrap();
        yield_now().await;
        Arc::new(
            position,
            28,
            Angle::from_degrees(120.0),
            Angle::from_degrees((300.0 / 0xffff as f32) * pot_info.value as f32),
        )
        .into_styled(*stroke)
        .draw(&mut self.display)
        .unwrap();
        yield_now().await;
    }

    // CV diag mode //////////////////////////////////////////////////////
    async fn into_cv_diag_mode(&mut self, pending_request: Request) {
        self.clear(false, false).await;
        self.display_text(
            false,
            true,
            "Plug LFOs into CV jacks...",
            FontSize::Small,
            Point::zero(),
        )
        .await;
        self.mode = Mode::CvDiag;
        self.pending_request = Some(pending_request);
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
                Request::Clear { .. } | Request::Flush => {
                    self.handle_generic_request(request).await
                }
                Request::UpdateCvValues { cv_info } => {
                    self.update_cv_info(cv_info, &mut cv_points).await
                }
                _ => self.switch_mode(request).await,
            }
        }
    }

    async fn update_cv_info(&mut self, cv_info: CvInfo, cv_points: &mut CvPoints) {
        let data_len = min(cv_points.data_len + 1, cv_points.data1.len());
        let erase_needed = data_len == cv_points.data1.len();
        for i in 0..data_len {
            if erase_needed {
                let erase_index = (i + cv_points.head) % data_len;
                let mut erase_y = 16u32 + 22u32 - cv_points.data1[erase_index] as u32;
                self.display.set_pixel(i as u32, erase_y, false);
                erase_y = 16u32 + 24u32 + 22u32 - cv_points.data2[erase_index] as u32;
                self.display.set_pixel(i as u32, erase_y, false);
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
            self.display.set_pixel(i as u32, set_y, true);
            set_y = 16u32 + 24u32 + 22u32 - cv_points.data2[set_index] as u32;
            self.display.set_pixel(i as u32, set_y, true);
            yield_now().await;
        }
        self.display.flush().await.unwrap();

        cv_points.data_len = data_len;
        cv_points.head = (cv_points.head + 1) % cv_points.data1.len();
    }

    // utils /////////////////////////////////////////////////////////////

    /// Switch into a menu mode.
    async fn into_menu_mode(&mut self, menu_mode: Mode, pending_request: Request) {
        self.clear(false, false).await;
        self.mode = menu_mode;
        self.pending_request = Some(pending_request);
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
    async fn switch_mode(&mut self, request: Request) {
        match request.mode() {
            Mode::Fundamental => self.into_fundamental_mode(request).await,
            Mode::OpMenu => self.into_menu_mode(Mode::OpMenu, request).await,
            Mode::EngineTypeMenu => self.into_menu_mode(Mode::EngineTypeMenu, request).await,
            Mode::AdminMenu => self.into_menu_mode(Mode::AdminMenu, request).await,
            Mode::PotsDiag => self.into_pots_diag_mode(request).await,
            Mode::CvDiag => self.into_cv_diag_mode(request).await,
            Mode::Any => {} // Generic requests shouldn't reach here.
        }
    }

    async fn clear(&mut self, reverse: bool, flush: bool) {
        debug!("clear; reverse={}, flush={}", reverse, flush);
        self.display
            .clear(if reverse {
                BinaryColor::On
            } else {
                BinaryColor::Off
            })
            .unwrap();
        if flush {
            self.display.flush().await.unwrap();
        } else {
            yield_now().await;
        }
    }

    async fn display_text(
        &mut self,
        reverse: bool,
        flush: bool,
        text: &str,
        size: FontSize,
        position: Point,
    ) {
        let font = match size {
            FontSize::Small => &FONT_5X8,
            FontSize::Medium => {
                if reverse {
                    &FONT_8X13_BOLD
                } else {
                    &FONT_8X13
                }
            }
            FontSize::Large => &FONT_10X20,
        };
        let text_style = MonoTextStyleBuilder::new()
            .font(font)
            .text_color(if reverse {
                BinaryColor::Off
            } else {
                BinaryColor::On
            })
            .build();
        yield_now().await;
        Text::with_baseline(text, position, text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        if flush {
            self.display.flush().await.unwrap();
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

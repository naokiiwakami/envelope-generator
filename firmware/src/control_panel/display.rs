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
        ascii::{FONT_5X8, FONT_7X13, FONT_10X20},
    },
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Arc, Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StyledDrawable},
    text::{Baseline, Text},
};
use heapless::String;
use ssd1306::{
    I2CDisplayInterface, Ssd1306Async,
    mode::{BufferedGraphicsModeAsync, DisplayConfigAsync},
    prelude::{DisplayRotation, I2CInterface},
    size::DisplaySize128x64,
};

use crate::input_reader::{CvInfo, PotInfo};

use super::menu::ADMIN_MENU_ITEMS;

pub const CHANNEL_LENGTH: usize = 4;
static CHANNEL_REQUEST: Channel<ThreadModeRawMutex, Request, CHANNEL_LENGTH> = Channel::new();

#[embassy_executor::task]
pub async fn run_display(i2c: I2c<'static, Async, Master>) {
    let mut eg_display = EgDisplay::new(i2c);
    eg_display.run().await;
}

enum Mode {
    Any,
    Fundamental,
    Menu,
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
    ShowInitialScreen,
    DisplayText {
        reverse: bool,
        flush: bool,
        text: String<32>,
        size: u32,
        position: Point,
    },
    // Menu requests
    DisplayMenuItem {
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
            Request::ShowInitialScreen | Request::DisplayText { .. } => Mode::Fundamental,
            Request::DisplayMenuItem { .. } => Mode::Menu,
            Request::UpdatePotValue { .. } => Mode::PotsDiag,
            Request::UpdateCvValues { .. } => Mode::CvDiag,
        }
    }
}

pub fn get_request_sender() -> channel::Sender<'static, ThreadModeRawMutex, Request, CHANNEL_LENGTH>
{
    CHANNEL_REQUEST.sender()
}

struct EgDisplay {
    display: Ssd1306Async<
        I2CInterface<I2c<'static, Async, Master>>,
        DisplaySize128x64,
        BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,

    mode: Mode,
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
                Mode::Menu => self.run_menu_mode().await,
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
                Request::ShowInitialScreen => self.show_initial_screen().await,
                Request::DisplayText {
                    reverse,
                    flush,
                    text,
                    size,
                    position,
                } => {
                    self.display_text(reverse, flush, text.as_str(), size, position)
                        .await
                }
                _ => self.switch_mode(request).await,
            }
        }
    }

    pub async fn show_initial_screen(&mut self) {
        self.mode = Mode::Fundamental;
        self.display.clear(BinaryColor::Off).unwrap();
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline("Humps Rev.0", Point::new(12, 0), text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        Text::with_baseline("UNDER", Point::new(40, 21), text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        Text::with_baseline("CONSTRUCTION", Point::new(5, 42), text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();

        self.display.flush().await.unwrap();
    }

    // menu mode //////////////////////////////////////////////////////////

    async fn into_menu_mode(&mut self, pending_request: Request) {
        self.clear(false, false).await;
        self.mode = Mode::Menu;
        self.pending_request = Some(pending_request);
    }

    async fn run_menu_mode(&mut self) {
        while matches!(self.mode, Mode::Menu) {
            let request = self.fetch_request().await;
            match request {
                Request::Clear { .. } | Request::Flush => {
                    self.handle_generic_request(request).await
                }
                Request::DisplayMenuItem { index } => self.display_current_menu(index).await,
                _ => self.switch_mode(request).await,
            }
        }
    }

    async fn display_current_menu(&mut self, index: usize) {
        if index >= ADMIN_MENU_ITEMS.len() {
            error!("display_current_menu: Index out of bounds; index={}", index);
            return;
        }
        self.display.clear(BinaryColor::Off).unwrap();
        let current_item = &ADMIN_MENU_ITEMS[index];
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();
        Text::with_baseline(
            current_item.name,
            Point::new(0, 20),
            text_style,
            Baseline::Top,
        )
        .draw(&mut self.display)
        .unwrap();
        self.display.flush().await.unwrap();
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
            Angle::from_degrees((300.0 / 0xfff as f32) * pot_info.value as f32),
        )
        .into_styled(*stroke)
        .draw(&mut self.display)
        .unwrap();
        yield_now().await;
    }

    // CV diag mode //////////////////////////////////////////////////////
    async fn into_cv_diag_mode(&mut self, pending_request: Request) {
        self.clear(false, false).await;
        self.display_text(false, true, "Plug LFOs into CV jacks...", 0, Point::zero())
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
                // scale value of range [0..0xfff] to [0..19]
                cv_points.data1[set_index] = ((cv_info.cv_1 as u32 * 304) >> 16) as u8;
                cv_points.data2[set_index] = ((cv_info.cv_2 as u32 * 304) >> 16) as u8;
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
            Mode::Menu => self.into_menu_mode(request).await,
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
        size: u32,
        position: Point,
    ) {
        let font = match size {
            1 => &FONT_7X13,
            2 => &FONT_10X20,
            _ => &FONT_5X8,
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

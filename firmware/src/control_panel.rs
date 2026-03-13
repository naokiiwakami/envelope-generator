mod diagnoser;
mod display;
mod menu;

use defmt::debug;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_stm32::{
    gpio::{Input, Level, Output},
    i2c::{I2c, Master},
    mode::Async,
    peripherals::TIM3,
    timer::qei::Qei,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::prelude::Point;
use heapless::String;

use display::{
    CHANNEL_LENGTH as DISPLAY_CHANNEL_LENGTH, Request as DisplayRequest, get_request_sender,
};
use menu::{ADMIN_MENU_ITEMS, AdminAction as MenuAction};

use crate::{control_panel::diagnoser::Diagnoser, input_reader::get_reader_info_receiver};

pub fn start(
    spawner: Spawner,
    i2c: I2c<'static, Async, Master>,
    encoder: Qei<'static, TIM3>,
    encoder_button: Input<'static>,
    encoder_ind_red: Output<'static>,
    encoder_ind_green: Output<'static>,
) {
    spawner.spawn(display::run_display(i2c).unwrap());
    let control_panel =
        ControlPanel::new(encoder, encoder_button, encoder_ind_red, encoder_ind_green);
    spawner.spawn(run_control_panel(control_panel).unwrap());
}

#[embassy_executor::task]
async fn run_control_panel(mut control_panel: ControlPanel) {
    control_panel.run().await;
}

enum ControlPanelMode {
    Normal,
    AdminMenu,
    AdminMenuSelected,
}

struct ControlPanel {
    // display
    display_request_sender:
        channel::Sender<'static, ThreadModeRawMutex, DisplayRequest, DISPLAY_CHANNEL_LENGTH>,

    // rotary encoder
    encoder: Qei<'static, TIM3>,
    button: Input<'static>,
    ind_red: Output<'static>,
    ind_green: Output<'static>,

    button_pressed_at: Option<Instant>,
    mode: ControlPanelMode,

    encoder_origin: u16,
    menu_item_index: usize,
    toggle_time: Instant,
}

impl ControlPanel {
    pub fn new(
        encoder: Qei<'static, TIM3>,
        encoder_button: Input<'static>,
        encoder_ind_red: Output<'static>,
        encoder_ind_green: Output<'static>,
    ) -> Self {
        let display_request_sender = get_request_sender();
        let encoder_origin = encoder.count() / 4;
        Self {
            display_request_sender,
            encoder,
            button: encoder_button,
            ind_red: encoder_ind_red,
            ind_green: encoder_ind_green,
            button_pressed_at: Option::None,
            mode: ControlPanelMode::Normal,
            encoder_origin,
            menu_item_index: 0,
            toggle_time: Instant::now(),
        }
    }

    pub async fn run(&mut self) {
        self.show_initial_screen().await;
        // self.blink_leds().await;
        loop {
            Timer::after_millis(10).await;
            self.update().await;
        }
    }

    async fn update(&mut self) {
        let next_level = self.button.get_level();

        if next_level == Level::Low {
            match self.button_pressed_at {
                Some(button_pressed_at) => {
                    match self.mode {
                        ControlPanelMode::Normal => {
                            if button_pressed_at.elapsed().as_millis() > 2000 {
                                self.into_menu_mode().await;
                            }
                        }
                        _ => {} // do nothing
                    }
                }
                None => self.on_button_pressed(),
            }
        } else if self.button_pressed_at.is_some() {
            self.on_button_released().await;
        } else {
            match self.mode {
                ControlPanelMode::AdminMenu => self.update_admin_menu().await,
                _ => {}
            }
        }
    }

    fn on_button_pressed(&mut self) {
        self.button_pressed_at = Some(Instant::now());
        match self.mode {
            ControlPanelMode::Normal => {
                self.ind_red.set_high();
                self.ind_green.set_high();
            }
            ControlPanelMode::AdminMenu => {
                self.mode = ControlPanelMode::AdminMenuSelected;
            }
            _ => {}
        };
    }

    async fn on_button_released(&mut self) {
        match self.mode {
            ControlPanelMode::Normal => {
                self.ind_red.set_low();
                self.ind_green.set_low();
            }
            ControlPanelMode::AdminMenuSelected => {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.execute_admin_menu().await;
            }
            _ => {}
        }
        self.button_pressed_at = None;
    }

    async fn into_menu_mode(&mut self) {
        self.mode = ControlPanelMode::AdminMenu;
        self.encoder_origin = self.encoder.count() / 4;
        self.ind_red.set_high();
        self.ind_green.set_low();
        self.menu_item_index = 0;
        self.toggle_time = Instant::now().saturating_add(Duration::from_millis(500));
        self.display_current_menu().await;
    }

    async fn update_admin_menu(&mut self) {
        if Instant::now().ge(&self.toggle_time) {
            self.ind_red.toggle();
            self.toggle_time = self.toggle_time.saturating_add(Duration::from_millis(500));
        }
        let count = self.encoder.count() / 4;
        let next_index = if count > self.encoder_origin {
            count - self.encoder_origin
        } else {
            16384 - (self.encoder_origin - count)
        } as usize
            % ADMIN_MENU_ITEMS.len();
        if next_index == self.menu_item_index {
            return;
        }
        debug!(
            "count: {}, index: {}, origin: {}",
            count, next_index, self.encoder_origin
        );
        self.menu_item_index = next_index;
        self.display_current_menu().await;
    }

    async fn execute_admin_menu(&mut self) {
        let action = &ADMIN_MENU_ITEMS[self.menu_item_index].action;
        match action {
            MenuAction::Diagnose => {
                let mut diagnoser = Diagnoser::new(self);
                diagnoser.diagnose().await;
                // self.diagnose().await;
                self.mode = ControlPanelMode::Normal;
            }
            MenuAction::Exit => {
                self.mode = ControlPanelMode::Normal;
                self.show_initial_screen().await;
            }
        };
    }

    async fn blink_leds(&mut self) {
        for _ in 0..24 {
            Timer::after_millis(100).await;
            self.ind_red.toggle();
        }
        Timer::after_millis(500).await;
        for _ in 0..24 {
            Timer::after_millis(100).await;
            self.ind_green.toggle();
        }
    }

    async fn show_initial_screen(&self) {
        self.display_request_sender
            .send(DisplayRequest::ShowInitialScreen)
            .await;
    }

    async fn display_current_menu(&mut self) {
        let request = DisplayRequest::DisplayMenuItem {
            index: self.menu_item_index,
        };
        self.display_request_sender.send(request).await;
    }

    async fn display_text(&mut self, text: &str, reverse: bool) {
        debug!("display_text request; reverse={}", reverse);
        self.display_request_sender
            .send(DisplayRequest::Clear {
                reverse,
                flush: false,
            })
            .await;
        self.display_request_sender
            .send(DisplayRequest::DisplayText {
                reverse,
                flush: true,
                text: String::<32>::try_from(text).unwrap(),
                size: 1,
                position: Point::new(20, 20),
            })
            .await;
    }
}

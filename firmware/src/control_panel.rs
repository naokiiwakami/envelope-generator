mod diagnoser;
mod display;
mod menu;

use defmt::debug;
use embassy_executor::Spawner;
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
use menu::{ADMIN_MENU_ITEMS, AdminAction, OpAction};

use crate::control_panel::{diagnoser::Diagnoser, menu::OP_MENU_ITEMS};

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
    OpMenu,
    OpActionSelected,
    AdminMenu,
    AdminActionSelected,
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

    encoder_origin: i16,
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
        let encoder_origin = encoder.count() as i16 / 4;
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
            // swiched on
            match self.button_pressed_at {
                Some(button_pressed_at) => {
                    match self.mode {
                        ControlPanelMode::Normal => {
                            if button_pressed_at.elapsed().as_millis() > 2000 {
                                self.into_admin_menu_mode().await;
                            }
                        }
                        _ => {} // do nothing, next mode is not determined yet
                    }
                }
                None => self.on_button_pressed(),
            }
        } else if self.button_pressed_at.is_some() {
            self.on_button_released().await;
        } else {
            // normal "button off" status, do regular task for the mode
            match self.mode {
                ControlPanelMode::OpMenu => self.update_op_menu().await,
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
            ControlPanelMode::OpMenu => {
                self.mode = ControlPanelMode::OpActionSelected;
            }
            ControlPanelMode::AdminMenu => {
                self.mode = ControlPanelMode::AdminActionSelected;
            }
            _ => {}
        };
    }

    async fn on_button_released(&mut self) {
        match self.mode {
            ControlPanelMode::Normal => {
                self.ind_red.set_low();
                self.ind_green.set_high();
                self.into_op_menu_mode().await;
            }
            ControlPanelMode::OpActionSelected => {
                self.execute_op_action().await;
            }
            ControlPanelMode::AdminActionSelected => {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.execute_admin_action().await;
            }
            _ => {}
        }
        self.button_pressed_at = None;
    }

    async fn into_op_menu_mode(&mut self) {
        debug!("into_op_menu_mode");
        self.mode = ControlPanelMode::OpMenu;
        self.encoder_origin = self.encoder.count() as i16 / 4;
        self.ind_green.set_high();
        self.menu_item_index = 0;
        self.display_current_op_menu().await;
    }

    async fn into_admin_menu_mode(&mut self) {
        self.mode = ControlPanelMode::AdminMenu;
        self.encoder_origin = self.encoder.count() as i16 / 4;
        self.ind_red.set_high();
        self.ind_green.set_low();
        self.menu_item_index = 0;
        self.toggle_time = Instant::now().saturating_add(Duration::from_millis(500));
        self.display_current_admin_menu().await;
    }

    /// Checks the encoder value and update the index if there's any change.
    /// Returns true if the index has changed.
    fn update_menu_index(&mut self, menu_size: usize) -> bool {
        let raw_count = self.encoder.count();
        if raw_count % 4 != 0 {
            // the encoder is still moving
            return false;
        }
        let count = raw_count as i16 / 4;
        if count == self.encoder_origin {
            // no change in the counter
            return false;
        }
        let delta = count - self.encoder_origin;
        let mut idx: i32 = (self.menu_item_index as i32 + delta as i32) % (menu_size as i32);
        debug!(
            "count: {}, origin: {}, delta: {}, idx: {}",
            count, self.encoder_origin, delta, idx
        );
        if idx < 0 {
            idx += menu_size as i32;
        }
        self.menu_item_index = idx as usize;
        debug!(
            "count: {}, index: {}, origin: {}",
            count, self.menu_item_index, self.encoder_origin
        );
        self.encoder_origin = count;
        return true;
    }

    async fn update_op_menu(&mut self) {
        if self.update_menu_index(OP_MENU_ITEMS.len()) {
            self.display_current_op_menu().await;
        }
    }

    async fn update_admin_menu(&mut self) {
        if Instant::now().ge(&self.toggle_time) {
            self.ind_red.toggle();
            self.toggle_time = self.toggle_time.saturating_add(Duration::from_millis(500));
        }
        if self.update_menu_index(ADMIN_MENU_ITEMS.len()) {
            self.display_current_admin_menu().await;
        }
    }

    async fn execute_op_action(&mut self) {
        let action = &OP_MENU_ITEMS[self.menu_item_index].action;
        match action {
            OpAction::ChangeMode => {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.mode = ControlPanelMode::Normal;
                self.show_initial_screen().await;
            }
            OpAction::Exit => {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.mode = ControlPanelMode::Normal;
                self.show_initial_screen().await;
            }
        }
    }

    async fn execute_admin_action(&mut self) {
        let action = &ADMIN_MENU_ITEMS[self.menu_item_index].action;
        match action {
            AdminAction::Diagnose => {
                let mut diagnoser = Diagnoser::new(self);
                diagnoser.diagnose().await;
                self.mode = ControlPanelMode::Normal;
            }
            AdminAction::Exit => {
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

    async fn display_current_op_menu(&mut self) {
        let request = DisplayRequest::DisplayOpMenuItem {
            index: self.menu_item_index,
        };
        debug!(
            "sending display op menu item request, index={}",
            self.menu_item_index
        );
        self.display_request_sender.send(request).await;
    }

    async fn display_current_admin_menu(&mut self) {
        let request = DisplayRequest::DisplayAdminMenuItem {
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

mod calibrator;
mod diagnoser;
mod display;
mod menu;

use defmt::{debug, error};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    gpio::{Input, Level, Output},
    i2c::{I2c, Master},
    mode::Async,
    peripherals::TIM3,
    timer::qei::Qei,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel, pubsub};
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use ssd1306_lite::{FontSize, TextBox};

use crate::envelope_generator::{
    EG_CHANNEL_SIZE, EG_PUBS, EG_SUBS, EgEvent, EgRequest, EngineType, get_eg_event_subscriber,
    get_eg_request_sender,
};

use self::{
    calibrator::Calibrator,
    diagnoser::Diagnoser,
    display::{
        CHANNEL_LENGTH as DISPLAY_CHANNEL_LENGTH, Display, Mode as DisplayMode,
        Request as DisplayRequest, get_request_sender,
    },
    menu::{ADMIN_MENU_ITEMS, AdminAction, ENGINE_TYPE_MENU_ITEMS, OP_MENU_ITEMS, OpAction},
};

pub fn start(
    spawner: Spawner,
    i2c: I2c<'static, Async, Master>,
    encoder: Qei<'static, TIM3>,
    encoder_button: Input<'static>,
    encoder_ind_red: Output<'static>,
    encoder_ind_green: Output<'static>,
) {
    let eg_display = Display::new(i2c);
    spawner.spawn(display::run_display(eg_display).unwrap());
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
    ActionSelected,
    OpMenu,
    OpActionSelected,
    EngineTypeMenu,
    EngineTypeSelected,
    AdminMenu,
    AdminActionSelected,
}

enum OperationPage {
    Home,
    OutputPolarity,
    // CvAssignment,
    // VelocitySensitivity,
    // NoteScaling,
}

#[derive(Clone)]
enum Action {
    SelectEngineType,
    SetUpPolatiry,
}

struct ControlPanel {
    // display
    display_request_sender:
        channel::Sender<'static, ThreadModeRawMutex, DisplayRequest, DISPLAY_CHANNEL_LENGTH>,

    // EG
    eg_request_sender: channel::Sender<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE>,
    eg_event_subscriber:
        pubsub::Subscriber<'static, ThreadModeRawMutex, EgEvent, EG_CHANNEL_SIZE, EG_SUBS, EG_PUBS>,
    engine_type_index: usize,
    current_engine_type: EngineType,

    // rotary encoder
    encoder: Qei<'static, TIM3>,
    button: Input<'static>,
    ind_red: Output<'static>,
    ind_green: Output<'static>,

    button_pressed_at: Option<Instant>,
    mode: ControlPanelMode,
    page: OperationPage,
    next_action: Option<Action>,

    encoder_last_raw: i16,
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
        let encoder_last_raw = encoder.count() as i16;
        Self {
            display_request_sender,
            eg_request_sender: get_eg_request_sender(),
            eg_event_subscriber: get_eg_event_subscriber(),
            engine_type_index: 0,
            current_engine_type: EngineType::Adsr,
            encoder,
            button: encoder_button,
            ind_red: encoder_ind_red,
            ind_green: encoder_ind_green,
            button_pressed_at: Option::None,
            mode: ControlPanelMode::Normal,
            page: OperationPage::Home,
            next_action: None,
            encoder_last_raw,
            menu_item_index: 0,
            toggle_time: Instant::now(),
        }
    }

    pub async fn run(&mut self) {
        self.clear_screen(false, true).await;
        loop {
            match select(
                self.eg_event_subscriber.next_message_pure(),
                Timer::after_millis(10),
            )
            .await
            {
                Either::First(event) => self.handle_eg_event(event).await,
                Either::Second(()) => {}
            };
            self.update().await;
        }
    }

    async fn handle_eg_event(&mut self, event: EgEvent) {
        match event {
            EgEvent::EngineSwitched(engine_type) => self.switch_engine_type(engine_type).await,
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
            self.button_pressed_at = None;
            self.on_button_released().await;
        } else {
            // normal "button off" status, do regular task for the mode
            match self.mode {
                ControlPanelMode::OpMenu => self.update_op_menu().await,
                ControlPanelMode::EngineTypeMenu => self.update_engine_type_menu().await,
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
                self.next_action = match self.page {
                    OperationPage::Home => Some(Action::SelectEngineType),
                    OperationPage::OutputPolarity => Some(Action::SetUpPolatiry),
                };
                self.mode = ControlPanelMode::ActionSelected;
            }
            ControlPanelMode::OpMenu => {
                self.mode = ControlPanelMode::OpActionSelected;
            }
            ControlPanelMode::EngineTypeMenu => {
                self.mode = ControlPanelMode::EngineTypeSelected;
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
            ControlPanelMode::ActionSelected => match &self.next_action {
                Some(action) => self.execute_action(action.clone()).await,
                None => error!("No action set up -- shouldn't happen"),
            },
            ControlPanelMode::OpActionSelected => {
                self.execute_op_action().await;
            }
            ControlPanelMode::EngineTypeSelected => {
                match &ENGINE_TYPE_MENU_ITEMS[self.menu_item_index].selection {
                    Some(engine_type) => {
                        self.request_switching_engine(&engine_type, true).await;
                        self.switch_engine_type(engine_type.clone()).await;
                    }
                    None => {} // do not switch the engine type
                }
                self.into_normal_mode().await;
            }
            ControlPanelMode::AdminActionSelected => {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.execute_admin_action().await;
            }
            _ => {}
        }
    }

    async fn execute_action(&mut self, action: Action) {
        match action {
            Action::SelectEngineType => self.into_engine_type_menu_mode().await,
            Action::SetUpPolatiry => {}
        }
    }

    // Op menu mode //////////////////////////////////////////////////////////

    /// Transit the mode to OpMenu.
    async fn into_op_menu_mode(&mut self) {
        self.into_menu_mode(ControlPanelMode::OpMenu, 0, false, true, false)
            .await;
        self.display_current_op_menu().await;
    }

    /// Called periodically to update op menu state.
    async fn update_op_menu(&mut self) {
        if self.update_menu(OP_MENU_ITEMS.len(), false, true) {
            self.display_current_op_menu().await;
        }
    }

    async fn display_current_op_menu(&mut self) {
        let request = DisplayRequest::DisplayOpMenuItem {
            index: self.menu_item_index,
        };
        self.display_request_sender.send(request).await;
    }

    /// Called on button release in OpActionSelected mode to execute the next action.
    async fn execute_op_action(&mut self) {
        let action = &OP_MENU_ITEMS[self.menu_item_index].selection;
        match action {
            OpAction::EngineType => self.into_engine_type_menu_mode().await,
            OpAction::Cancel => self.into_normal_mode().await,
        }
    }

    // Engine type menu mode //////////////////////////////////////////////////////////

    /// Transit the mode to EngineTypeMenu.
    async fn into_engine_type_menu_mode(&mut self) {
        self.into_menu_mode(
            ControlPanelMode::EngineTypeMenu,
            self.engine_type_index,
            false,
            true,
            false,
        )
        .await;
        self.display_current_engine_type_menu().await;
    }

    /// Called periodically to update engine type menu state.
    async fn update_engine_type_menu(&mut self) {
        if self.update_menu(ENGINE_TYPE_MENU_ITEMS.len(), false, true) {
            self.display_current_engine_type_menu().await;
        }
    }

    async fn display_current_engine_type_menu(&mut self) {
        let request = DisplayRequest::DisplayEngineTypeMenuItem {
            index: self.menu_item_index,
        };
        self.display_request_sender.send(request).await;
    }

    /// Called to request EnvelopeGenerator to switch engine mode.
    async fn request_switching_engine(&mut self, engine_type: &EngineType, save: bool) {
        self.eg_request_sender
            .send(EgRequest::SwitchEngine {
                engine_type: engine_type.clone(),
                send_notif: false,
                save,
            })
            .await;
    }

    async fn switch_engine_type(&mut self, next_engine_type: EngineType) {
        self.engine_type_index = (next_engine_type.clone() as u8) as usize;
        self.current_engine_type = next_engine_type;
        self.go_to_op_home().await;
    }

    // Admin mode ////////////////////////////////////////////////////////////

    /// Transit the mode to AdminMenu.
    async fn into_admin_menu_mode(&mut self) {
        self.into_menu_mode(ControlPanelMode::AdminMenu, 0, true, false, true)
            .await;
        self.display_current_admin_menu().await;
    }

    /// Called periodically to update the menu state.
    async fn update_admin_menu(&mut self) {
        if self.update_menu(ADMIN_MENU_ITEMS.len(), true, false) {
            self.display_current_admin_menu().await;
        }
    }

    async fn display_current_admin_menu(&mut self) {
        let request = DisplayRequest::DisplayAdminMenuItem {
            index: self.menu_item_index,
        };
        self.display_request_sender.send(request).await;
    }

    /// Called on button release in AdminActionSelected mode to execute the next action.
    async fn execute_admin_action(&mut self) {
        let action = &ADMIN_MENU_ITEMS[self.menu_item_index].selection;
        match action {
            AdminAction::Calibrate => {
                let mut calibrator = Calibrator::new(self);
                calibrator.execute().await;
                self.mode = ControlPanelMode::Normal;
            }
            AdminAction::Diagnose => {
                let mut diagnoser = Diagnoser::new(self);
                diagnoser.execute().await;
                self.mode = ControlPanelMode::Normal;
            }
            AdminAction::Cancel => self.into_normal_mode().await,
        };
    }

    // Utilities /////////////////////////////////////////////////////////////

    /// Switch to the normal mode.
    async fn into_normal_mode(&mut self) {
        self.ind_red.set_low();
        self.ind_green.set_low();
        self.mode = ControlPanelMode::Normal;
        self.go_to_op_home().await;
    }

    /// Switch to a menu mode.
    async fn into_menu_mode(
        &mut self,
        mode: ControlPanelMode,
        index: usize,
        red: bool,
        green: bool,
        blink: bool,
    ) {
        self.mode = mode;
        self.encoder_last_raw = self.encoder.count() as i16;
        self.ind_red
            .set_level(if red { Level::High } else { Level::Low });
        self.ind_green
            .set_level(if green { Level::High } else { Level::Low });
        self.menu_item_index = index;
        if blink {
            self.toggle_time = Instant::now().saturating_add(Duration::from_millis(500));
        }
    }

    /// Updates menu state.
    /// Returns true when there's an update.
    fn update_menu(&mut self, menu_length: usize, toggle_red: bool, toggle_green: bool) -> bool {
        if Instant::now().ge(&self.toggle_time) {
            if toggle_red {
                self.ind_red.toggle();
            }
            if toggle_green {
                self.ind_red.toggle();
            }
            self.toggle_time = self.toggle_time.saturating_add(Duration::from_millis(500));
        }
        self.update_menu_index(menu_length)
    }

    /// Checks the encoder value and update the index if there's any change.
    /// Returns true if the index has changed.
    fn update_menu_index(&mut self, menu_size: usize) -> bool {
        let raw = self.encoder.count() as i16;
        let delta = (raw - self.encoder_last_raw) / 4;
        if delta == 0 {
            return false;
        }
        let mut idx: i32 = (self.menu_item_index as i32 + delta as i32) % (menu_size as i32);
        debug!(
            "count: {}, origin: {}, delta: {}, idx: {}",
            raw, self.encoder_last_raw, delta, idx
        );
        if idx < 0 {
            idx += menu_size as i32;
        }
        self.menu_item_index = idx as usize;
        debug!(
            "count: {}, index: {}, origin: {}",
            raw, self.menu_item_index, self.encoder_last_raw
        );
        self.encoder_last_raw = raw;
        return true;
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

    async fn go_to_op_home(&self) {
        self.display_request_sender
            .send(DisplayRequest::GoToOpHome {
                engine_type: self.current_engine_type.clone(),
            })
            .await;
    }

    async fn switch_display_mode(&mut self, mode: DisplayMode) {
        self.display_request_sender
            .send(DisplayRequest::SwitchMode { mode })
            .await;
    }

    async fn clear_screen(&mut self, reverse: bool, flush: bool) {
        self.display_request_sender
            .send(DisplayRequest::Clear { reverse, flush })
            .await;
    }

    async fn display_text(
        &mut self,
        text: &str,
        text_box: TextBox,
        font_size: FontSize,
        flush: bool,
    ) {
        self.display_request_sender
            .send(DisplayRequest::DisplayText {
                text: String::<32>::try_from(text).unwrap(),
                text_box,
                font_size,
                flush,
            })
            .await;
    }
}

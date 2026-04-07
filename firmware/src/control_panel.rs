mod calibrator;
mod diagnoser;
mod display;
mod menu;

use analog3::rng::make_local_rng;
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
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel, pubsub, watch};
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    control_panel::menu::POLARITY_CHANGE_TARGET_ITEMS,
    envelope_generator::{
        EG_CHANNEL_SIZE, EG_PUBS, EG_SUBS, EgEvent, EgRequest, EngineType, Mode as EgOperationMode,
        OutputPolarity, get_eg_event_subscriber, get_eg_request_sender,
    },
    input_reader::{InputReaderInfo, PotKind, get_reader_info_receiver},
};

use self::{
    calibrator::Calibrator,
    diagnoser::Diagnoser,
    display::{
        CHANNEL_LENGTH as DISPLAY_CHANNEL_LENGTH, Display, Mode as DisplayMode,
        Request as DisplayRequest, get_request_sender,
    },
    menu::{ADMIN_MENU_ITEMS, AdminAction, ENGINE_TYPE_MENU_ITEMS},
};

const PARA_DECAYS_PAGES: [OperationPage; 2] = [OperationPage::Home, OperationPage::OutputPolarity];
const ADDSR_PAGES: [OperationPage; 2] = [OperationPage::Home, OperationPage::OutputPolarity];
const ADSR_PAGES: [OperationPage; 2] = [OperationPage::Home, OperationPage::OutputPolarity];
const LINEAR_PAGES: [OperationPage; 2] = [OperationPage::Home, OperationPage::OutputPolarity];

const ALL_PAGES: [&[OperationPage]; 4] =
    [&PARA_DECAYS_PAGES, &ADDSR_PAGES, &ADSR_PAGES, &LINEAR_PAGES];

const _: () = {
    assert!(ALL_PAGES.len() == EngineType::Linear as u8 as usize + 1);
};

pub async fn start(
    spawner: Spawner,
    i2c: I2c<'static, Async, Master>,
    encoder: Qei<'static, TIM3>,
    encoder_button: Input<'static>,
    encoder_ind_red: Output<'static>,
    encoder_ind_green: Output<'static>,
) {
    let mut display = Display::new(i2c);
    display.initialize().await;
    spawner.spawn(display::run_display(display).unwrap());
    let control_panel =
        ControlPanel::new(encoder, encoder_button, encoder_ind_red, encoder_ind_green).await;
    spawner.spawn(run_control_panel(control_panel).unwrap());
}

#[embassy_executor::task]
async fn run_control_panel(mut control_panel: ControlPanel) {
    control_panel.run().await;
}

enum ControlPanelMode {
    Normal,
    ActionSelected,
    EngineTypeMenu,
    EngineTypeSelected,
    AdminMenu,
    AdminActionSelected,
    ChooseChangePolarityTarget,
    ChangePolarityTargetSelected,
    ChangePolarity,
    PolarityChanged,
}

#[derive(Clone)]
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
    SetupPolarity,
}

struct ControlPanel {
    // display
    display_request_sender:
        channel::Sender<'static, ThreadModeRawMutex, DisplayRequest, DISPLAY_CHANNEL_LENGTH>,

    // EG
    eg_request_sender: channel::Sender<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE>,
    eg_event_subscriber:
        pubsub::Subscriber<'static, ThreadModeRawMutex, EgEvent, EG_CHANNEL_SIZE, EG_SUBS, EG_PUBS>,

    // Input Reader
    reader_info_receiver: watch::Receiver<'static, ThreadModeRawMutex, InputReaderInfo, 2>,
    attack: u16,
    decay: u16,
    sustain: u16,
    release: u16,
    extra_1: u16,
    extra_2: u16,

    // EG states
    engine_type_index: usize,
    current_engine_type: EngineType,
    polarity_1: OutputPolarity,
    polarity_2: OutputPolarity,
    polarity_change_targets: u8,

    // rotary encoder
    encoder: Qei<'static, TIM3>,
    button: Input<'static>,
    ind_red: Output<'static>,
    ind_green: Output<'static>,

    button_pressed_at: Option<Instant>,
    mode: ControlPanelMode,
    page: OperationPage,
    page_index: usize,
    next_action: Option<Action>,

    encoder_last_raw: i16,
    menu_item_index: usize,
    toggle_time: Instant,
}

impl ControlPanel {
    pub async fn new(
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
            reader_info_receiver: get_reader_info_receiver().await,
            attack: 0,
            decay: 0,
            sustain: 0,
            release: 0,
            extra_1: 0,
            extra_2: 0,
            engine_type_index: 0,
            current_engine_type: EngineType::Adsr,
            polarity_1: OutputPolarity::Positive,
            polarity_2: OutputPolarity::Positive,
            polarity_change_targets: 0,
            encoder,
            button: encoder_button,
            ind_red: encoder_ind_red,
            ind_green: encoder_ind_green,
            button_pressed_at: Option::None,
            mode: ControlPanelMode::Normal,
            page: OperationPage::Home,
            page_index: 0,
            next_action: None,
            encoder_last_raw,
            menu_item_index: 0,
            toggle_time: Instant::now(),
        }
    }

    pub async fn run(&mut self) {
        self.clear_screen(false, true).await;
        {
            let mut leds = [&mut self.ind_red, &mut self.ind_green];
            random_blink(5, &mut leds).await;
        }
        let mut last_updated = Instant::now();
        loop {
            match select(
                self.eg_event_subscriber.next_message_pure(),
                self.reader_info_receiver.changed(),
            )
            .await
            {
                Either::First(event) => self.handle_eg_event(event).await,
                Either::Second(info) => self.handle_reader_info(info).await,
            };
            if last_updated.elapsed().as_millis() > 10 {
                self.update().await;
                last_updated = Instant::now();
            }
        }
    }

    async fn handle_eg_event(&mut self, event: EgEvent) {
        match event {
            EgEvent::EngineSwitched(engine_type) => self.switch_engine_type(engine_type).await,
            EgEvent::PolarityChanged((p1, p2)) => self.change_polarity(p1, p2).await,
        }
    }

    async fn handle_reader_info(&mut self, info: InputReaderInfo) {
        let value = info.pot_info.value;
        let mut updated = true;
        match info.pot_info.kind {
            PotKind::Attack => {
                self.attack = value;
            }
            PotKind::Decay => {
                self.decay = value;
            }
            PotKind::Sustain => {
                self.sustain = value;
            }
            PotKind::Release => {
                self.release = value;
            }
            PotKind::Extra1 => {
                self.extra_1 = value;
            }
            PotKind::Extra2 => {
                self.extra_2 = value;
            }
            _ => {
                updated = false;
            }
        }
        if updated
            && matches!(self.mode, ControlPanelMode::Normal)
            && matches!(self.page, OperationPage::Home)
        {
            self.display_request_sender
                .send(DisplayRequest::UpdatePot {
                    pot_info: info.pot_info,
                })
                .await;
        }
    }

    async fn update(&mut self) {
        let next_level = self.button.get_level();
        if next_level == Level::Low {
            // switched on
            match self.button_pressed_at {
                Some(button_pressed_at) => {
                    if button_pressed_at.elapsed().as_millis() > 2000 {
                        self.into_admin_menu_mode().await;
                    }
                }
                None => self.on_button_pressed(),
            }
        } else if self.button_pressed_at.is_some() {
            self.button_pressed_at = None;
            self.on_button_released().await;
        } else {
            // normal "button off" status, do regular task for the mode
            self.regular_update().await;
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
                    OperationPage::OutputPolarity => Some(Action::SetupPolarity),
                };
                self.mode = ControlPanelMode::ActionSelected;
            }
            ControlPanelMode::EngineTypeMenu => {
                self.mode = ControlPanelMode::EngineTypeSelected;
            }
            ControlPanelMode::AdminMenu => {
                self.mode = ControlPanelMode::AdminActionSelected;
            }
            ControlPanelMode::ChooseChangePolarityTarget => {
                self.mode = ControlPanelMode::ChangePolarityTargetSelected;
            }
            ControlPanelMode::ChangePolarity => {
                self.mode = ControlPanelMode::PolarityChanged;
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
            ControlPanelMode::ActionSelected => match &self.next_action {
                Some(action) => self.execute_action(action.clone()).await,
                None => error!("No action set up -- shouldn't happen"),
            },
            ControlPanelMode::EngineTypeSelected => {
                match &ENGINE_TYPE_MENU_ITEMS[self.menu_item_index].selection {
                    Some(engine_type) => {
                        self.request_switching_engine(&engine_type).await;
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
            ControlPanelMode::ChangePolarityTargetSelected => {
                self.into_change_polarity_mode().await
            }
            ControlPanelMode::PolarityChanged => self.conclude_polarity_change().await,
            _ => {
                self.ind_red.set_low();
                self.ind_green.set_low();
            }
        }
    }

    async fn regular_update(&mut self) {
        match self.mode {
            ControlPanelMode::Normal => self.update_normal().await,
            ControlPanelMode::ChooseChangePolarityTarget => {
                self.update_change_polarity_target().await
            }
            ControlPanelMode::ChangePolarity => self.update_change_polarity().await,
            ControlPanelMode::EngineTypeMenu => self.update_engine_type_menu().await,
            ControlPanelMode::AdminMenu => self.update_admin_menu().await,
            _ => {}
        }
    }

    async fn execute_action(&mut self, action: Action) {
        match action {
            Action::SelectEngineType => self.into_engine_type_menu_mode().await,
            Action::SetupPolarity => self.into_change_polarity_select_mode().await,
        }
    }

    // Normal mode //////////////////////////////////////////////////////////
    async fn update_normal(&mut self) {
        let engine_index = self.current_engine_type.index();
        if engine_index >= ALL_PAGES.len() {
            return; // shouldn't happen but being defensive here
        }
        let raw = self.encoder.count() as i16;
        let delta = (raw - self.encoder_last_raw) / 4;
        if delta == 0 {
            return;
        }
        self.encoder_last_raw = raw;

        // switch page
        let pages = ALL_PAGES[self.current_engine_type.index()];
        if pages.len() <= 1 {
            return; // switching page never happens
        }
        let mut index: i32 = (self.page_index as i32 + delta as i32) % pages.len() as i32;
        while index < 0 {
            index += pages.len() as i32;
        }
        self.page_index = index as usize;
        self.page = pages[self.page_index].clone();
        match self.page {
            OperationPage::Home => self.go_to_op_home().await,
            OperationPage::OutputPolarity => self.show_polarity().await,
        }
    }

    // Menu modes /////////////////////////////////////////////////////////

    /// Transit the mode to EngineTypeMenu.
    async fn into_engine_type_menu_mode(&mut self) {
        self.into_menu_mode(
            ControlPanelMode::EngineTypeMenu,
            self.engine_type_index,
            false,
            true,
            false,
        );
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

    // Change Polarity mode //////////////////////////////////////////////////

    async fn into_change_polarity_select_mode(&mut self) {
        debug!("into change_polarity_select_mode");
        self.into_menu_mode(
            ControlPanelMode::ChooseChangePolarityTarget,
            0,
            true,
            true,
            true,
        );
        let targets = POLARITY_CHANGE_TARGET_ITEMS[0].selection;
        self.display_request_sender
            .send(DisplayRequest::SetPolarityChangeTargets { targets })
            .await;
    }

    async fn update_change_polarity_target(&mut self) {
        self.update_menu_index(POLARITY_CHANGE_TARGET_ITEMS.len());
        if Instant::now().ge(&self.toggle_time) {
            if self.ind_red.is_set_low() {
                self.ind_red.set_high();
                self.ind_green.set_high();
                let targets = POLARITY_CHANGE_TARGET_ITEMS[self.menu_item_index].selection;
                self.display_request_sender
                    .send(DisplayRequest::SetPolarityChangeTargets { targets })
                    .await;
            } else {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.display_request_sender
                    .send(DisplayRequest::SetPolarityChangeTargets { targets: 0 })
                    .await;
            }
            self.toggle_time = self.toggle_time.saturating_add(Duration::from_millis(500));
        }
    }

    async fn into_change_polarity_mode(&mut self) {
        self.mode = ControlPanelMode::ChangePolarity;
        self.polarity_change_targets = POLARITY_CHANGE_TARGET_ITEMS[self.menu_item_index].selection;
        self.encoder_last_raw = self.encoder.count() as i16;
        self.display_request_sender
            .send(DisplayRequest::SetPolarityChangeTargets {
                targets: self.polarity_change_targets,
            })
            .await;
    }

    async fn update_change_polarity(&mut self) {
        if Instant::now().ge(&self.toggle_time) {
            let targets = self.polarity_change_targets;
            if self.ind_red.is_set_low() {
                self.ind_red.set_high();
                self.ind_green.set_high();

                let mut polarity_1 = self.polarity_1;
                let mut polarity_2 = self.polarity_2;
                let voice_1 = (targets & 0x1) != 0;
                let voice_2 = (targets & 0x2) != 0;
                if voice_1 {
                    polarity_1 = self.get_next_polarity(polarity_1);
                }
                if voice_2 {
                    polarity_2 = self.get_next_polarity(polarity_2);
                }

                let targets = POLARITY_CHANGE_TARGET_ITEMS[self.menu_item_index].selection;
                self.display_request_sender
                    .send(DisplayRequest::UpdatePolarities {
                        targets,
                        polarity_1,
                        polarity_2,
                        is_draw: true,
                    })
                    .await;
            } else {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.display_request_sender
                    .send(DisplayRequest::UpdatePolarities {
                        targets,
                        polarity_1: OutputPolarity::Positive,
                        polarity_2: OutputPolarity::Positive,
                        is_draw: false,
                    })
                    .await;
            }
            self.toggle_time = self.toggle_time.saturating_add(Duration::from_millis(500));
        }
    }

    async fn conclude_polarity_change(&mut self) {
        self.ind_red.set_low();
        self.ind_green.set_low();
        let targets = self.polarity_change_targets;
        let voice_1 = (targets & 0x1) != 0;
        let voice_2 = (targets & 0x2) != 0;
        if voice_1 {
            self.polarity_1 = self.get_next_polarity(self.polarity_1);
        }
        if voice_2 {
            self.polarity_2 = self.get_next_polarity(self.polarity_2);
        }
        self.eg_request_sender
            .send(EgRequest::ChangeOutputPolarities {
                polarity_1: self.polarity_1,
                polarity_2: self.polarity_2,
                send_notif: false,
            })
            .await;
        self.show_polarity().await;
        self.mode = ControlPanelMode::Normal;
    }

    fn get_next_polarity(&self, current_polarity: OutputPolarity) -> OutputPolarity {
        let raw = self.encoder.count() as i16;
        let delta = (raw - self.encoder_last_raw) / 4;
        let mut new_value: i8 = (current_polarity as u8 as i8 + (delta % 2) as i8) % 2;
        if new_value < 0 {
            new_value += 2;
        }
        OutputPolarity::try_from(new_value as u8).unwrap()
    }

    // Admin mode ////////////////////////////////////////////////////////////

    /// Transit the mode to AdminMenu.
    async fn into_admin_menu_mode(&mut self) {
        self.into_menu_mode(ControlPanelMode::AdminMenu, 0, true, false, true);
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
            AdminAction::ToggleDiagnoseMode => {
                self.request_toggle_eg_mode(EgOperationMode::Diagnose).await;
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
    fn into_menu_mode(
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

    /// Requests EnvelopeGenerator to switch engine type.
    async fn request_switching_engine(&mut self, engine_type: &EngineType) {
        self.eg_request_sender
            .send(EgRequest::SwitchEngine {
                engine_type: engine_type.clone(),
                send_notif: false,
            })
            .await;
    }

    /// Requests EnvelopeGenerator to switch operation mode.
    async fn request_toggle_eg_mode(&mut self, mode: EgOperationMode) {
        self.eg_request_sender
            .send(EgRequest::ToggleMode { mode })
            .await;
    }

    async fn switch_engine_type(&mut self, next_engine_type: EngineType) {
        self.engine_type_index = (next_engine_type.clone() as u8) as usize;
        self.current_engine_type = next_engine_type;
        self.page_index = 0;
        self.page = OperationPage::Home;
        self.encoder_last_raw = self.encoder.count() as i16;
        self.go_to_op_home().await;
    }

    async fn change_polarity(&mut self, polarity_1: OutputPolarity, polarity_2: OutputPolarity) {
        self.polarity_1 = polarity_1;
        self.polarity_2 = polarity_2;
        if matches!(self.mode, ControlPanelMode::Normal)
            && matches!(self.page, OperationPage::OutputPolarity)
        {
            self.show_polarity().await;
        }
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
                attack: self.attack,
                decay: self.decay,
                sustain: self.sustain,
                release: self.release,
                extra_1: self.extra_1,
                extra_2: self.extra_2,
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

    async fn show_polarity(&mut self) {
        self.display_request_sender
            .send(DisplayRequest::ShowPolarity {
                polarity_1: self.polarity_1,
                polarity_2: self.polarity_2,
            })
            .await;
    }
}

async fn random_blink(repeat: usize, leds: &mut [&mut Output<'static>]) {
    let rng = make_local_rng();
    for _ in 0..repeat {
        let random = rng.random_u64();
        Timer::after_millis(100 + random % 256).await;
        leds[random as usize % leds.len()].set_high();
        Timer::after_millis(30).await;
        leds[random as usize % leds.len()].set_low();
    }
}

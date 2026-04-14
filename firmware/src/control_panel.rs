mod calibrator;
mod cv_assigner;
mod diagnoser;
mod display;
mod menu;
mod note_scaler;
mod polarity_changer;

use analog3::rng::make_local_rng;
use defmt::{self, debug, error};
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
use heapless::{String, Vec};
use ssd1306_lite::{FontSize, TextBox};

use crate::{
    control_panel::menu::POLARITY_CHANGE_TARGET_ITEMS,
    definitions::{CvKind, PotKind},
    envelope_generator::{
        ConfigReader, EG_CHANNEL_SIZE, EG_PUBS, EG_SUBS, EgEvent, EgRequest, EngineType,
        Mode as EgOperationMode, OutputPolarity, get_eg_event_subscriber, get_eg_request_sender,
    },
    input_reader::PotInfo,
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

const PARA_DECAYS_PAGES: [OperationPage; 4] = [
    OperationPage::Home,
    OperationPage::OutputPolarity,
    OperationPage::CvAssignment,
    OperationPage::NoteScaling,
];
const ADDSR_PAGES: [OperationPage; 3] = [
    OperationPage::Home,
    OperationPage::OutputPolarity,
    OperationPage::CvAssignment,
];
const ADSR_PAGES: [OperationPage; 3] = [
    OperationPage::Home,
    OperationPage::OutputPolarity,
    OperationPage::CvAssignment,
];
const LINEAR_PAGES: [OperationPage; 3] = [
    OperationPage::Home,
    OperationPage::OutputPolarity,
    OperationPage::CvAssignment,
];

const ALL_PAGES: [&[OperationPage]; 4] =
    [&PARA_DECAYS_PAGES, &ADDSR_PAGES, &ADSR_PAGES, &LINEAR_PAGES];

const _: () = {
    assert!(ALL_PAGES.len() == EngineType::Linear as u8 as usize + 1);
};

const NOTE_SCALER_SAMPLE_PERIOD_MS: u64 = 10;
const NOTE_SCALER_VELOCITY_HISTORY_LEN: usize = 4;

#[derive(Clone, Copy)]
enum PolarityPhase {
    TargetSelect,
    ChangePolarity,
}

enum ControlPanelActionState {
    PolarityChanger {
        phase: PolarityPhase,
        current_index: usize,
        targets: u8,
        polarity_1: OutputPolarity,
        polarity_2: OutputPolarity,
        toggle_time: Instant,
        should_draw: bool,
        voice_1: bool,
        voice_2: bool,
        visible: bool,
    },
    CvAssigner {
        cv_kind: CvKind,
        candidates: Vec<PotKind, 5>,
        original_index: usize,
        current_index: usize,
        iteration_count: usize,
        turn_on: bool,
        blink_remaining: i32,
        ready_to_exit: bool,
    },
    NoteScaler {
        current_depth: u16,
        last_raw: i16,
        velocity_history: [i16; NOTE_SCALER_VELOCITY_HISTORY_LEN],
        velocity_history_index: usize,
        velocity_history_len: usize,
        charge: usize,
        blink_remaining: i32,
        ready_to_exit: bool,
    },
}

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

#[derive(Debug, defmt::Format)]
enum ControlPanelMode {
    Normal,
    ActionSelected,
    PolarityTargetSelect,
    PolarityChange,
    CvAssignment,
    NoteScalingAction,
    EngineTypeMenu,
    EngineTypeSelected,
    AdminMenu,
    AdminActionSelected,
}

#[derive(Clone, Copy)]
enum OperationPage {
    Home,
    OutputPolarity,
    CvAssignment,
    // VelocitySensitivity,
    NoteScaling,
}

#[derive(Clone, Copy)]
enum Action {
    SelectEngineType,
    SetupPolarity,
    AssignCv,
    SetNoteScaling,
}

struct ControlPanel {
    // display
    display_request_sender:
        channel::Sender<'static, ThreadModeRawMutex, DisplayRequest, DISPLAY_CHANNEL_LENGTH>,

    // EG
    eg_request_sender: channel::Sender<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE>,
    eg_event_subscriber:
        pubsub::Subscriber<'static, ThreadModeRawMutex, EgEvent, EG_CHANNEL_SIZE, EG_SUBS, EG_PUBS>,

    // EG state
    eg_config: ConfigReader,
    engine_type_index: usize,

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
    action_state: Option<ControlPanelActionState>,

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
            eg_config: ConfigReader::new(),
            engine_type_index: 0,
            encoder,
            button: encoder_button,
            ind_red: encoder_ind_red,
            ind_green: encoder_ind_green,
            button_pressed_at: Option::None,
            mode: ControlPanelMode::Normal,
            page: OperationPage::Home,
            page_index: 0,
            next_action: None,
            action_state: None,
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
                Timer::after_millis(10),
            )
            .await
            {
                Either::First(event) => self.handle_eg_event(event).await,
                Either::Second(()) => {}
            };
            if last_updated.elapsed().as_millis() > 10 {
                self.update().await;
                last_updated = Instant::now();
            }
        }
    }

    /// Put current counter to self.
    fn smash_counter(&mut self) {
        self.encoder_last_raw = self.encoder.count() as i16;
    }

    async fn handle_eg_event(&mut self, event: EgEvent) {
        match event {
            EgEvent::EngineSwitched(engine_type) => self.switch_engine_type(engine_type).await,
            EgEvent::PolarityChanged((p1, p2)) => self.change_polarity(p1, p2).await,
            EgEvent::PotMoved(pot_info) => self.handle_pot_moved(pot_info).await,
        }
    }

    async fn handle_pot_moved(&mut self, pot_info: PotInfo) {
        if matches!(self.mode, ControlPanelMode::Normal) && matches!(self.page, OperationPage::Home)
        {
            self.display_request_sender
                .send(DisplayRequest::UpdatePot { pot_info })
                .await;
        }
    }

    async fn update(&mut self) {
        let next_level = self.button.get_level();
        let was_pressed = self.button_pressed_at.is_some();
        if next_level == Level::Low {
            // button is pressed
            match self.button_pressed_at {
                Some(button_pressed_at) => {
                    if button_pressed_at.elapsed().as_millis() > 2000 {
                        self.into_admin_menu_mode().await;
                    }
                }
                None => self.on_button_pressed(),
            }
        }

        match self.mode {
            ControlPanelMode::Normal => {
                if next_level != Level::Low {
                    self.regular_update().await;
                }
            }
            ControlPanelMode::EngineTypeMenu => self.update_engine_type_menu().await,
            ControlPanelMode::AdminMenu => self.update_admin_menu().await,
            ControlPanelMode::PolarityTargetSelect
            | ControlPanelMode::PolarityChange
            | ControlPanelMode::CvAssignment
            | ControlPanelMode::NoteScalingAction => {
                self.update_action_state().await;
            }
            _ => {}
        }

        if next_level == Level::High && was_pressed {
            self.button_pressed_at = None;
            self.on_button_released().await;
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
                    OperationPage::CvAssignment => Some(Action::AssignCv),
                    OperationPage::NoteScaling => Some(Action::SetNoteScaling),
                };
                self.mode = ControlPanelMode::ActionSelected;
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
            ControlPanelMode::ActionSelected => match &self.next_action {
                Some(action) => self.execute_action(*action).await,
                None => error!("No action set up -- shouldn't happen"),
            },
            ControlPanelMode::EngineTypeSelected => {
                match &ENGINE_TYPE_MENU_ITEMS[self.menu_item_index].selection {
                    Some(engine_type) => {
                        self.request_switching_engine(*engine_type).await;
                        self.clear_screen(false, true).await;
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
            _ => {
                self.ind_red.set_low();
                self.ind_green.set_low();
            }
        }
    }

    async fn regular_update(&mut self) {
        match self.mode {
            ControlPanelMode::Normal => self.update_normal().await,
            ControlPanelMode::EngineTypeMenu => self.update_engine_type_menu().await,
            ControlPanelMode::AdminMenu => self.update_admin_menu().await,
            _ => {}
        }
    }

    async fn execute_action(&mut self, action: Action) {
        match action {
            Action::SelectEngineType => self.into_engine_type_menu_mode().await,
            Action::SetupPolarity => self.start_polarity_change().await,
            Action::AssignCv => self.start_cv_assigner(CvKind::A).await,
            Action::SetNoteScaling => self.start_note_scaling().await,
        }
    }

    async fn start_polarity_change(&mut self) {
        self.smash_counter();
        self.ind_red.set_low();
        self.ind_green.set_low();

        let current_index = 0;
        let targets = POLARITY_CHANGE_TARGET_ITEMS[current_index].selection;
        self.display_request_sender
            .send(DisplayRequest::SetPolarityChangeTargets { targets })
            .await;

        self.action_state = Some(ControlPanelActionState::PolarityChanger {
            phase: PolarityPhase::TargetSelect,
            current_index,
            targets,
            polarity_1: self.eg_config.out_polarity(0),
            polarity_2: self.eg_config.out_polarity(1),
            toggle_time: Instant::now().saturating_add(Duration::from_millis(500)),
            should_draw: false,
            voice_1: false,
            voice_2: false,
            visible: true,
        });
        self.mode = ControlPanelMode::PolarityTargetSelect;
    }

    async fn start_cv_assigner(&mut self, cv_kind: CvKind) {
        self.smash_counter();
        let current_destination = if cv_kind == CvKind::A {
            self.eg_config.cv_destination_a()
        } else {
            self.eg_config.cv_destination_b()
        };
        let other_destination = if cv_kind == CvKind::A {
            self.eg_config.cv_destination_b()
        } else {
            self.eg_config.cv_destination_a()
        };

        let mut candidates: Vec<PotKind, 5> = Vec::new();
        self.build_cv_candidates(other_destination, &mut candidates);
        let original_index = match candidates
            .iter()
            .position(|&dest| dest == current_destination)
        {
            Some(idx) => idx,
            None => 0,
        };

        self.action_state = Some(ControlPanelActionState::CvAssigner {
            cv_kind,
            candidates,
            original_index,
            current_index: original_index,
            iteration_count: 0,
            turn_on: false,
            blink_remaining: if cv_kind == CvKind::A { 0 } else { 14 },
            ready_to_exit: false,
        });
        self.ind_red.set_low();
        self.ind_green.set_low();
        self.mode = ControlPanelMode::CvAssignment;
    }

    async fn start_note_scaling(&mut self) {
        self.smash_counter();
        self.ind_red.set_high();
        self.ind_green.set_high();

        self.action_state = Some(ControlPanelActionState::NoteScaler {
            current_depth: self.eg_config.note_scaling_depth(0),
            last_raw: self.encoder_last_raw,
            velocity_history: [0; NOTE_SCALER_VELOCITY_HISTORY_LEN],
            velocity_history_index: 0,
            velocity_history_len: 0,
            charge: 0,
            blink_remaining: 0,
            ready_to_exit: false,
        });
        self.display_request_sender
            .send(DisplayRequest::UpdateNoteScaling {
                depth: self.eg_config.note_scaling_depth(0),
            })
            .await;
        self.mode = ControlPanelMode::NoteScalingAction;
    }

    async fn update_action_state(&mut self) {
        match self.mode {
            ControlPanelMode::PolarityTargetSelect => self.update_polarity_target_select().await,
            ControlPanelMode::PolarityChange => self.update_polarity_change().await,
            ControlPanelMode::CvAssignment => self.update_cv_assigner().await,
            ControlPanelMode::NoteScalingAction => self.update_note_scaling_action().await,
            _ => {}
        }
    }

    async fn update_polarity_target_select(&mut self) {
        let (current_index, targets, toggle_time, visible, voice_1, voice_2, phase, should_draw) =
            match self.action_state.as_mut() {
                Some(ControlPanelActionState::PolarityChanger {
                    phase,
                    current_index,
                    targets,
                    toggle_time,
                    visible,
                    voice_1,
                    voice_2,
                    should_draw,
                    ..
                }) if matches!(*phase, PolarityPhase::TargetSelect) => (
                    current_index,
                    targets,
                    toggle_time,
                    visible,
                    voice_1,
                    voice_2,
                    phase,
                    should_draw,
                ),
                _ => return,
            };

        let button_level = self.button.get_level();
        if button_level == Level::Low {
            if self.button_pressed_at.is_none() {
                self.button_pressed_at = Some(Instant::now());
            }
            return;
        }

        if self.button_pressed_at.is_some() {
            self.button_pressed_at = None;
            *targets = POLARITY_CHANGE_TARGET_ITEMS[*current_index].selection;
            self.display_request_sender
                .send(DisplayRequest::SetPolarityChangeTargets { targets: *targets })
                .await;
            self.ind_red.set_low();
            self.ind_green.set_low();

            *voice_1 = (*targets & 0x1) != 0;
            *voice_2 = (*targets & 0x2) != 0;
            *phase = PolarityPhase::ChangePolarity;
            *toggle_time = Instant::now().saturating_add(Duration::from_millis(500));
            *should_draw = true;
            *visible = true;

            self.mode = ControlPanelMode::PolarityChange;
            return;
        }

        let raw = self.encoder.count() as i16;
        let delta = (raw - self.encoder_last_raw) / 4;
        if delta != 0 {
            let len = POLARITY_CHANGE_TARGET_ITEMS.len() as i32;
            let mut next_index = *current_index as i32 + delta as i32;
            next_index %= len;
            if next_index < 0 {
                next_index += len;
            }
            *current_index = next_index as usize;
            *targets = POLARITY_CHANGE_TARGET_ITEMS[*current_index].selection;
            self.display_request_sender
                .send(DisplayRequest::SetPolarityChangeTargets { targets: *targets })
                .await;
            self.encoder_last_raw = raw;
        }

        if Instant::now().ge(toggle_time) {
            if *visible {
                self.ind_red.set_high();
                self.ind_green.set_high();
                self.display_request_sender
                    .send(DisplayRequest::SetPolarityChangeTargets { targets: *targets })
                    .await;
            } else {
                self.ind_red.set_low();
                self.ind_green.set_low();
                self.display_request_sender
                    .send(DisplayRequest::SetPolarityChangeTargets { targets: 0 })
                    .await;
            }
            *visible = !*visible;
            *toggle_time = toggle_time.saturating_add(Duration::from_millis(500));
        }
    }

    async fn update_polarity_change(&mut self) {
        if !matches!(
            self.action_state,
            Some(ControlPanelActionState::PolarityChanger {
                phase: PolarityPhase::ChangePolarity,
                ..
            })
        ) {
            return;
        }

        let button_level = self.button.get_level();
        if button_level == Level::Low {
            if self.button_pressed_at.is_none() {
                self.button_pressed_at = Some(Instant::now());
            }
            return;
        }

        if self.button_pressed_at.is_some() {
            self.button_pressed_at = None;
            self.ind_red.set_low();
            self.ind_green.set_low();

            let (polarity_1, polarity_2) = match self.action_state.as_mut() {
                Some(ControlPanelActionState::PolarityChanger {
                    polarity_1,
                    polarity_2,
                    ..
                }) => (*polarity_1, *polarity_2),
                _ => return,
            };

            self.eg_request_sender
                .send(EgRequest::ChangeOutputPolarities {
                    polarity_1,
                    polarity_2,
                    send_notif: false,
                })
                .await;
            self.show_polarity(polarity_1, polarity_2).await;
            self.action_state = None;
            self.mode = ControlPanelMode::Normal;
            return;
        }

        let raw = self.encoder.count() as i16;
        let delta = (raw - self.encoder_last_raw) / 4;
        if delta != 0 {
            let mut update = None;

            {
                if let Some(ControlPanelActionState::PolarityChanger {
                    polarity_1,
                    polarity_2,
                    voice_1,
                    voice_2,
                    should_draw,
                    ..
                }) = self.action_state.as_mut()
                {
                    if *voice_1 {
                        *polarity_1 = Self::next_polarity(*polarity_1, delta);
                    }
                    if *voice_2 {
                        *polarity_2 = Self::next_polarity(*polarity_2, delta);
                    }
                    let targets = if *voice_1 { 1 } else { 0 } | if *voice_2 { 2 } else { 0 };
                    update = Some((targets, *polarity_1, *polarity_2));
                    *should_draw = true;
                }
            }

            if let Some((targets, polarity_1, polarity_2)) = update {
                self.display_request_sender
                    .send(DisplayRequest::UpdatePolarities {
                        targets,
                        polarity_1,
                        polarity_2,
                        is_draw: true,
                    })
                    .await;
                self.encoder_last_raw = raw;
            }
        }

        if let Some(ControlPanelActionState::PolarityChanger {
            toggle_time,
            should_draw,
            voice_1,
            voice_2,
            polarity_1,
            polarity_2,
            ..
        }) = self.action_state.as_mut()
        {
            if Instant::now() >= *toggle_time {
                if *should_draw {
                    self.ind_red.set_high();
                    self.ind_green.set_high();
                    self.display_request_sender
                        .send(DisplayRequest::UpdatePolarities {
                            targets: if *voice_1 { 1 } else { 0 } | if *voice_2 { 2 } else { 0 },
                            polarity_1: *polarity_1,
                            polarity_2: *polarity_2,
                            is_draw: true,
                        })
                        .await;
                } else {
                    self.ind_red.set_low();
                    self.ind_green.set_low();
                    self.display_request_sender
                        .send(DisplayRequest::UpdatePolarities {
                            targets: if *voice_1 { 1 } else { 0 } | if *voice_2 { 2 } else { 0 },
                            polarity_1: OutputPolarity::Positive,
                            polarity_2: OutputPolarity::Positive,
                            is_draw: false,
                        })
                        .await;
                }
                *should_draw = !*should_draw;
                *toggle_time = toggle_time.saturating_add(Duration::from_millis(500));
            }
        }
    }

    fn next_polarity(current: OutputPolarity, delta: i16) -> OutputPolarity {
        let mut index = match current {
            OutputPolarity::Positive => 0,
            OutputPolarity::Negative => 1,
        };
        let count = 2;
        index = ((index as i32 + delta as i32) % count + count) % count;
        match index {
            0 => OutputPolarity::Positive,
            _ => OutputPolarity::Negative,
        }
    }

    async fn update_cv_assigner(&mut self) {
        let (
            cv_kind,
            candidates,
            original_index,
            current_index,
            blink_remaining,
            iteration_count,
            turn_on,
            ready_to_exit,
        ) = match self.action_state.as_mut() {
            Some(ControlPanelActionState::CvAssigner {
                cv_kind,
                candidates,
                original_index,
                current_index,
                iteration_count,
                turn_on,
                blink_remaining,
                ready_to_exit,
            }) => (
                *cv_kind,
                candidates,
                *original_index,
                current_index,
                blink_remaining,
                iteration_count,
                turn_on,
                ready_to_exit,
            ),
            _ => return,
        };

        *iteration_count += 1;
        if *blink_remaining >= 0 && *iteration_count % 6 == 0 {
            if *blink_remaining > 0 {
                self.ind_green.toggle();
            } else {
                self.ind_red.set_high();
                self.ind_green.set_high();
            }
            *blink_remaining -= 1;
        }

        let button_level = self.button.get_level();
        if button_level == Level::Low {
            if self.button_pressed_at.is_none() {
                self.button_pressed_at = Some(Instant::now());
            }
            return;
        }

        if self.button_pressed_at.is_some() {
            self.button_pressed_at = None;
            let new_destination = candidates[*current_index];
            self.eg_request_sender
                .send(EgRequest::ChangeCvDestination {
                    source: cv_kind,
                    destination: new_destination,
                })
                .await;
            self.display_request_sender
                .send(DisplayRequest::BlinkCvSource {
                    source: cv_kind,
                    turn_on: true,
                })
                .await;
            self.ind_red.set_low();
            self.ind_green.set_low();
            if cv_kind == CvKind::A {
                self.start_cv_assigner(CvKind::B).await;
                return;
            }
            *ready_to_exit = true;
            *blink_remaining = 14;
            *iteration_count = 0;
            return;
        }

        if *ready_to_exit && *blink_remaining == 0 {
            self.action_state = None;
            self.mode = ControlPanelMode::Normal;
            self.show_cv_assignment().await;
            self.smash_counter();
            return;
        }

        let raw = self.encoder.count() as i16;
        let delta = (raw - self.encoder_last_raw) / 4;
        let len = candidates.len() as i32;
        let mut next_index = original_index as i32 + delta as i32;
        next_index %= len;
        if next_index < 0 {
            next_index += len;
        }
        if next_index as usize != *current_index {
            *current_index = next_index as usize;
            let next_destination = candidates[*current_index];
            self.display_request_sender
                .send(DisplayRequest::UpdateCvAssignment {
                    source: cv_kind,
                    destination: next_destination,
                })
                .await;
        }

        if *iteration_count % 25 == 0 {
            self.display_request_sender
                .send(DisplayRequest::BlinkCvSource {
                    source: cv_kind,
                    turn_on: *turn_on,
                })
                .await;
            *turn_on = !*turn_on;
        }
    }

    async fn update_note_scaling_action(&mut self) {
        let state = match self.action_state.as_mut() {
            Some(ControlPanelActionState::NoteScaler { .. }) => self.action_state.as_mut().unwrap(),
            _ => return,
        };
        if let ControlPanelActionState::NoteScaler {
            current_depth,
            last_raw,
            velocity_history,
            velocity_history_index,
            velocity_history_len,
            charge,
            blink_remaining: blink_count,
            ready_to_exit,
        } = state
        {
            let button_level = self.button.get_level();
            if button_level == Level::Low {
                if self.button_pressed_at.is_none() {
                    self.button_pressed_at = Some(Instant::now());
                }
                if !*ready_to_exit {
                    *blink_count = 14;
                    self.ind_red.set_low();
                    self.ind_green.set_low();
                    *ready_to_exit = true;
                    self.eg_request_sender
                        .send(EgRequest::ChangeNoteScalingDepth {
                            depth: *current_depth,
                            save: true,
                        })
                        .await;
                    self.show_note_scaling().await;
                }
                return;
            }
            if *ready_to_exit {
                if *blink_count > 0 {
                    if *charge == 0 {
                        self.ind_green.toggle();
                        *blink_count -= 1;
                        *charge = 3;
                    } else {
                        *charge -= 1;
                    }
                } else {
                    self.button_pressed_at = None;
                    self.action_state = None;
                    self.mode = ControlPanelMode::Normal;
                    self.ind_red.set_low();
                    self.ind_green.set_low();
                }
                return;
            }

            let raw = self.encoder.count() as i16;
            let delta = (raw - *last_raw) / 4;

            velocity_history[*velocity_history_index] = delta;
            *velocity_history_index =
                (*velocity_history_index + 1) % NOTE_SCALER_VELOCITY_HISTORY_LEN;
            if *velocity_history_len < NOTE_SCALER_VELOCITY_HISTORY_LEN {
                *velocity_history_len += 1;
            }

            if *charge > 0 {
                *charge -= 1;
            }

            if delta == 0 || *charge > 0 {
                return;
            }

            *last_raw = raw;
            *charge = 4;

            let avg_velocity: i32 = velocity_history[..*velocity_history_len]
                .iter()
                .map(|&v| v as i32)
                .sum();

            let depth_step = 0x1 << ((avg_velocity.abs() / 2).min(8) + 4);

            let depth_delta = depth_step as i32 * avg_velocity.signum() as i32;
            let new_depth = (*current_depth as i32 + depth_delta).clamp(0, u16::MAX as i32) as u16;
            if new_depth != *current_depth {
                *current_depth = new_depth;

                self.eg_request_sender
                    .send(EgRequest::ChangeNoteScalingDepth {
                        depth: *current_depth,
                        save: false,
                    })
                    .await;

                self.display_request_sender
                    .send(DisplayRequest::UpdateNoteScaling {
                        depth: *current_depth,
                    })
                    .await;
            }
            self.encoder_last_raw = raw;
        }
    }

    fn build_cv_candidates(&self, skip: PotKind, candidates: &mut Vec<PotKind, 5>) {
        let possible_pots: &[PotKind] = match self.eg_config.engine_type(0) {
            EngineType::ParaDecays => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
                PotKind::Extra2,
                PotKind::Extra1,
            ],
            EngineType::TwoDecays => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
                PotKind::Extra2,
                PotKind::Extra1,
            ],
            EngineType::Adsr => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
            ],
            EngineType::Linear => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
            ],
        };
        for pot in possible_pots {
            if *pot != skip {
                candidates.push(*pot).unwrap();
            }
        }
    }

    // Normal mode //////////////////////////////////////////////////////////
    async fn update_normal(&mut self) {
        let engine_index = self.eg_config.engine_type(0).index();
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
        let pages = ALL_PAGES[self.eg_config.engine_type(0).index()];
        if pages.len() <= 1 {
            return; // switching page never happens
        }
        let mut index: i32 = (self.page_index as i32 - delta as i32) % pages.len() as i32;
        while index < 0 {
            index += pages.len() as i32;
        }
        self.page_index = index as usize;
        self.page = pages[self.page_index];
        match self.page {
            OperationPage::Home => self.go_to_op_home().await,
            OperationPage::OutputPolarity => {
                self.show_polarity(
                    self.eg_config.out_polarity(0),
                    self.eg_config.out_polarity(1),
                )
                .await
            }
            OperationPage::CvAssignment => self.show_cv_assignment().await,
            OperationPage::NoteScaling => self.show_note_scaling().await,
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
        self.smash_counter();
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
    async fn request_switching_engine(&mut self, engine_type: EngineType) {
        self.eg_request_sender
            .send(EgRequest::SwitchEngine {
                engine_type: engine_type,
                send_notif: true,
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
        self.engine_type_index = (next_engine_type as u8) as usize;
        self.page_index = 0;
        self.page = OperationPage::Home;
        self.smash_counter();
        self.go_to_op_home().await;
    }

    async fn change_polarity(&mut self, polarity_1: OutputPolarity, polarity_2: OutputPolarity) {
        if matches!(self.mode, ControlPanelMode::Normal)
            && matches!(self.page, OperationPage::OutputPolarity)
        {
            self.show_polarity(polarity_1, polarity_2).await;
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
            .send(DisplayRequest::GoToOpHome)
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

    async fn show_polarity(&mut self, polarity_1: OutputPolarity, polarity_2: OutputPolarity) {
        self.display_request_sender
            .send(DisplayRequest::ShowPolarity {
                polarity_1,
                polarity_2,
            })
            .await;
    }

    async fn show_cv_assignment(&mut self) {
        self.display_request_sender
            .send(DisplayRequest::ShowCvAssignment)
            .await;
    }

    async fn show_note_scaling(&mut self) {
        self.display_request_sender
            .send(DisplayRequest::ShowNoteScaling)
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

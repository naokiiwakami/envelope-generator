mod adsr_engine;
mod config;
mod definitions;
mod diag_engine;
mod linear_engine;
mod para_decays_engine;
mod two_phases_engine;
mod utils;

use analog3::{
    self,
    addresses_common::*,
    definitions::*,
    rng,
    storage::{self, load_string, load_u8, load_u16, load_u16_or_default, load_u32},
};
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{debug, warn};
use embassy_executor::Spawner;
use embassy_futures::{
    select::{Either5, select5},
    yield_now,
};
use embassy_stm32::{dac::Dac, flash::Error, gpio::Output, interrupt, mode::Blocking, pac};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{self, Channel},
    mutex::Mutex,
    pubsub::{self, PubSubChannel},
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use crate::{
    addresses::{
        ADDR_CV_DEST_A_ADSR, ADDR_CV_DEST_A_LINEAR, ADDR_CV_DEST_A_PARA_DECAYS,
        ADDR_CV_DEST_A_TWO_PHASES, ADDR_EG_TYPE_1, ADDR_NOTE_SCALING_DEPTH_1,
        ADDR_NOTE_SCALING_DEPTH_2, ADDR_OUT_ZERO_POINT_1, ADDR_OUT_ZERO_POINT_2,
        ADDR_OUTPUT_POLARITY_1, ADDR_VOICE_ID_1,
    },
    definitions::{CvKind, PotKind},
    envelope_generator::definitions::DEFAULT_NOTE_SCALING_DEPTH,
    input_reader::{InputReaderInfo, get_reader_info_receiver},
};

use self::{
    adsr_engine::AdsrEngine,
    config::EgConfig,
    definitions::{DEFAULT_ENGINE_TYPE, DEFAULT_VOICE_IDS, Engine, VoiceParams},
    diag_engine::DiagEngine,
    linear_engine::LinearEngine,
    para_decays_engine::ParaDecaysEngine,
    two_phases_engine::TwoPhasesEngine,
    utils::choose_output_converter,
};
pub use self::{
    config::ConfigReader,
    definitions::{
        DEFAULT_OUT_ZERO_POINT, EgEvent, EgRequest, EngineType, GateEventType, GateId, Mode,
        OutputPolarity,
    },
};

// UID and string
static UID: AtomicU32 = AtomicU32::new(u32::MAX);
static NAME: Mutex<ThreadModeRawMutex, String<A3_MAX_PROP_DATA_SIZE>> = Mutex::new(String::new());

// parameter tweaks
const POLLING_INTERVAL: Duration = Duration::from_micros(50); // 20 kHz
const BUF_SEGMENT_LENGTH: usize = 64;

// DAC buffers
const BUF_SIZE: usize = BUF_SEGMENT_LENGTH * 2;
static mut BUFFERS: [[u16; BUF_SIZE]; 2] = [[0; BUF_SIZE]; 2];
static mut HEADS: [usize; 2] = [0; 2];
static mut TAILS: [usize; 2] = [0; 2];

pub const EG_CHANNEL_SIZE: usize = 4;

// Request channel
static CHANNEL_REQUEST: Channel<ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE> = Channel::new();

pub fn get_eg_request_sender()
-> channel::Sender<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE> {
    CHANNEL_REQUEST.sender()
}

pub const EG_PUBS: usize = 1;
pub const EG_SUBS: usize = 2;

// Event channel
static CHANNEL_EVENT: PubSubChannel<
    ThreadModeRawMutex,
    EgEvent,
    EG_CHANNEL_SIZE,
    EG_SUBS,
    EG_PUBS,
> = PubSubChannel::new();

pub fn get_eg_event_subscriber()
-> pubsub::Subscriber<'static, ThreadModeRawMutex, EgEvent, EG_CHANNEL_SIZE, EG_SUBS, EG_PUBS> {
    CHANNEL_EVENT.subscriber().unwrap()
}

static SIGNAL_STORAGE: Signal<ThreadModeRawMutex, Result<Value, Error>> = Signal::new();

pub async fn start(
    spawner: Spawner,
    dac_channels: Dac<'static, Blocking>,
    ind_gate_1: Output<'static>,
    ind_gate_2: Output<'static>,
) {
    let mut eg_resources = EgResources::new(ind_gate_1, ind_gate_2);
    retrieve_stored_config(&mut eg_resources).await;
    spawner.spawn(run_envelope_generator(dac_channels, eg_resources).unwrap());
}

async fn retrieve_stored_config(eg_resources: &mut EgResources) {
    load_uid().await;
    load_name().await;
    for index in 0..2 {
        eg_resources
            .config
            .set_voice_id(index, load_voice_id(index).await);
        let engine_type = load_engine_type(index).await;
        eg_resources.config.set_engine_type(index, engine_type);
        eg_resources.voice_params[index].out_zero_point = load_out_zero_point(index).await;
        let polarity = load_out_polarity(index).await;
        eg_resources.voice_params[index].value_to_output = choose_output_converter(&polarity);
        eg_resources.config.set_out_polarity(index, polarity);
        if index == 0 {
            let (cv_destination_a, cv_destination_b) = load_cv_destinations(engine_type).await;
            eg_resources.config.set_cv_destination_a(cv_destination_a);
            eg_resources.config.set_cv_destination_b(cv_destination_b);
        }
        eg_resources
            .config
            .set_note_scaling_depth(index, load_note_scaling_depth(index).await);
    }
}

async fn load_voice_id(voice_index: usize) -> u16 {
    let address = ADDR_VOICE_ID_1 * voice_index as u16 * 2;
    let mut voice_id = load_u16(address, &SIGNAL_STORAGE).await;
    if voice_id == u16::MAX {
        voice_id = DEFAULT_VOICE_IDS[voice_index];
        storage::save(address, Value::U16(voice_id), &SIGNAL_STORAGE)
            .await
            .unwrap();
    }
    voice_id
}

async fn load_engine_type(voice_index: usize) -> EngineType {
    let address = ADDR_EG_TYPE_1 + voice_index as u16;
    let type_id = load_u8(address, &SIGNAL_STORAGE).await;
    match EngineType::try_from(type_id) {
        Ok(engine_type) => engine_type,
        Err(()) => {
            let engine_type = DEFAULT_ENGINE_TYPE;
            storage::save(address, Value::U8(engine_type as u8), &SIGNAL_STORAGE)
                .await
                .unwrap();
            engine_type
        }
    }
}

async fn save_engine_type(voice_index: usize, engine_type: &EngineType) {
    let address = ADDR_EG_TYPE_1 + voice_index as u16;
    storage::save(address, Value::U8(*engine_type as u8), &SIGNAL_STORAGE)
        .await
        .unwrap();
}

async fn load_out_polarity(voice_index: usize) -> OutputPolarity {
    let address = ADDR_OUTPUT_POLARITY_1 + voice_index as u16;
    let polarity_id = load_u8(address, &SIGNAL_STORAGE).await;
    match OutputPolarity::try_from(polarity_id) {
        Ok(polarity) => polarity,
        Err(()) => {
            let polarity = OutputPolarity::Positive;
            storage::save(address, Value::U8(polarity as u8), &SIGNAL_STORAGE)
                .await
                .unwrap();
            polarity
        }
    }
}

async fn save_out_polarity(voice_index: usize, polarity: OutputPolarity) {
    let address = ADDR_OUTPUT_POLARITY_1 + voice_index as u16;
    storage::save(address, Value::U8(polarity as u8), &SIGNAL_STORAGE)
        .await
        .unwrap();
}

async fn load_cv_destinations(engine_type: EngineType) -> (PotKind, PotKind) {
    let (address, default_a, default_b) = match engine_type {
        EngineType::Adsr => (ADDR_CV_DEST_A_ADSR, PotKind::Attack, PotKind::Decay),
        EngineType::TwoPhases => (ADDR_CV_DEST_A_TWO_PHASES, PotKind::Attack, PotKind::Decay),
        EngineType::ParaDecays => (ADDR_CV_DEST_A_PARA_DECAYS, PotKind::Attack, PotKind::Decay),
        EngineType::Linear => (ADDR_CV_DEST_A_LINEAR, PotKind::Attack, PotKind::Decay),
    };
    let pot_kind_id = load_u8(address, &SIGNAL_STORAGE).await;
    let destination_a = match PotKind::try_from(pot_kind_id) {
        Ok(pot_kind) => pot_kind,
        Err(()) => {
            storage::save(address, Value::U8(default_a as u8), &SIGNAL_STORAGE)
                .await
                .unwrap();
            default_a
        }
    };
    let pot_kind_id = load_u8(address + 1, &SIGNAL_STORAGE).await;
    let destination_b = match PotKind::try_from(pot_kind_id) {
        Ok(pot_kind) => pot_kind,
        Err(()) => {
            storage::save(address + 1, Value::U8(default_b as u8), &SIGNAL_STORAGE)
                .await
                .unwrap();
            default_b
        }
    };

    (destination_a, destination_b)
}

async fn save_cv_destination(engine_type: EngineType, cv_kind: CvKind, destination: PotKind) {
    let mut address = match engine_type {
        EngineType::Adsr => ADDR_CV_DEST_A_ADSR,
        EngineType::TwoPhases => ADDR_CV_DEST_A_TWO_PHASES,
        EngineType::ParaDecays => ADDR_CV_DEST_A_PARA_DECAYS,
        EngineType::Linear => ADDR_CV_DEST_A_LINEAR,
    };
    if matches!(cv_kind, CvKind::B) {
        address += 1;
    }
    storage::save(address, Value::U8(destination as u8), &SIGNAL_STORAGE)
        .await
        .unwrap();
}

async fn load_note_scaling_depth(voice_index: usize) -> u16 {
    let value = load_u16_or_default(
        ADDR_NOTE_SCALING_DEPTH_1 + 2 * voice_index as u16,
        &SIGNAL_STORAGE,
        DEFAULT_NOTE_SCALING_DEPTH,
    )
    .await;
    value
}

async fn save_note_scaling_depths(depth_1: u16, depth_2: u16) {
    storage::save(
        ADDR_NOTE_SCALING_DEPTH_1,
        Value::U16(depth_1),
        &SIGNAL_STORAGE,
    )
    .await
    .unwrap();
    storage::save(
        ADDR_NOTE_SCALING_DEPTH_2,
        Value::U16(depth_2),
        &SIGNAL_STORAGE,
    )
    .await
    .unwrap();
}

async fn load_out_zero_point(voice_index: usize) -> u16 {
    let value = load_u16_or_default(
        ADDR_OUT_ZERO_POINT_1 + 2 * voice_index as u16,
        &SIGNAL_STORAGE,
        DEFAULT_OUT_ZERO_POINT,
    )
    .await;
    debug!("loaded zero point (voice {}): {:#x}", voice_index, value);
    value
}

/// Loads UID of this module from the flash memory. If the UID is not determined yet,
/// the method creates one and save it.
async fn load_uid() {
    debug!("loading UID");
    let mut uid = load_u32(A3_ADDR_MODULE_UID, &SIGNAL_STORAGE).await;
    debug!("loaded UID: {=u32:#x}", uid);
    if uid == u32::MAX {
        uid = rng::RNG.random_u32() & 0x1fffffff; // a 29-bit random value
        debug!("generated UID={:#x}", uid);
        storage::save(A3_ADDR_MODULE_UID, Value::U32(uid), &SIGNAL_STORAGE)
            .await
            .unwrap();
    }
    UID.store(uid, Ordering::Relaxed);
}

pub fn get_uid() -> u32 {
    UID.load(Ordering::Relaxed)
}

/// Loads name of this module from the flash memory. If the name is not determined yet,
/// the method uses the default name and save it.
async fn load_name() {
    let mut loaded_name = load_string(A3_ADDR_MODULE_NAME, &SIGNAL_STORAGE).await;
    debug!("loaded name: {}", loaded_name.as_str());
    if loaded_name.len() == 0 {
        loaded_name = String::try_from("Humps").unwrap();
        storage::save(
            A3_ADDR_MODULE_NAME,
            Value::Text(loaded_name.clone()),
            &SIGNAL_STORAGE,
        )
        .await
        .unwrap();
    }
    let mut name = NAME.lock().await;
    name.clone_from(&loaded_name);
}

pub async fn get_name() -> String<A3_MAX_PROP_DATA_SIZE> {
    let s = NAME.lock().await;
    let mut name = String::<A3_MAX_PROP_DATA_SIZE>::new();
    name.clone_from(&s);
    name
}

#[embassy_executor::task]
async fn run_envelope_generator(
    dac_channels: Dac<'static, Blocking>,
    mut eg_resources: EgResources,
) {
    let (mut dac1, mut dac2) = dac_channels.split();
    dac1.enable();
    dac2.enable();

    loop {
        match eg_resources.voice_params[0].operation_mode {
            Mode::Normal => {
                let engine_type = eg_resources.config.engine_type(0);
                let (cv_destination_a, cv_destination_b) = load_cv_destinations(engine_type).await;
                eg_resources.config.set_cv_destination_a(cv_destination_a);
                eg_resources.config.set_cv_destination_b(cv_destination_b);
                let note_scaling_depth = load_note_scaling_depth(0).await;
                if eg_resources.config.note_scaling_depth(0) != note_scaling_depth {
                    save_note_scaling_depths(note_scaling_depth, note_scaling_depth).await;
                }
                match engine_type {
                    EngineType::ParaDecays => {
                        let mut eg = EnvelopeGenerator::<ParaDecaysEngine>::new(&mut eg_resources);
                        eg.run().await;
                    }
                    EngineType::TwoPhases => {
                        let mut eg = EnvelopeGenerator::<TwoPhasesEngine>::new(&mut eg_resources);
                        eg.run().await;
                    }
                    EngineType::Adsr => {
                        let mut eg = EnvelopeGenerator::<AdsrEngine>::new(&mut eg_resources);
                        eg.run().await;
                    }
                    EngineType::Linear => {
                        let mut eg = EnvelopeGenerator::<LinearEngine>::new(&mut eg_resources);
                        eg.run().await;
                    }
                }
            }
            Mode::Diagnose => {
                let mut eg = EnvelopeGenerator::<DiagEngine>::new(&mut eg_resources);
                eg.run().await;
            }
        }
    }
}

struct EgResources {
    config: EgConfig,
    request_receiver: channel::Receiver<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE>,
    event_publisher:
        pubsub::Publisher<'static, ThreadModeRawMutex, EgEvent, EG_CHANNEL_SIZE, EG_SUBS, EG_PUBS>,
    voice_params: [VoiceParams; 2],
    ind_gate_1: Output<'static>,
    ind_gate_2: Output<'static>,
}

impl EgResources {
    pub fn new(ind_gate_1: Output<'static>, ind_gate_2: Output<'static>) -> Self {
        Self {
            config: EgConfig::new(),
            request_receiver: CHANNEL_REQUEST.receiver(),
            event_publisher: CHANNEL_EVENT.publisher().unwrap(),
            voice_params: [VoiceParams::new(0), VoiceParams::new(1)],
            ind_gate_1,
            ind_gate_2,
        }
    }
}

struct EnvelopeGenerator<'a, EngineT: Engine> {
    config: &'a mut EgConfig,
    request_receiver:
        &'a mut channel::Receiver<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE>,
    event_publisher: &'a mut pubsub::Publisher<
        'static,
        ThreadModeRawMutex,
        EgEvent,
        EG_CHANNEL_SIZE,
        EG_SUBS,
        EG_PUBS,
    >,
    voice_1: EgVoice<'a, EngineT>,
    voice_2: EgVoice<'a, EngineT>,
}

impl<'a, EngineT: Engine> EnvelopeGenerator<'a, EngineT> {
    pub fn new(resources: &'a mut EgResources) -> Self {
        let (voice_params_0, voice_params_1) = resources.voice_params.split_at_mut(1);
        let voice_1 = EgVoice::new(
            0,
            &mut voice_params_0[0],
            &mut resources.ind_gate_1,
            &resources.config,
        );
        let voice_2 = EgVoice::new(
            1,
            &mut voice_params_1[0],
            &mut resources.ind_gate_2,
            &resources.config,
        );
        Self {
            config: &mut resources.config,
            request_receiver: &mut resources.request_receiver,
            event_publisher: &mut resources.event_publisher,
            voice_1,
            voice_2,
        }
    }

    pub async fn run(&mut self) {
        let rx_receiver = analog3::get_forwarder_receiver();
        let prop_request_receiver = analog3::get_prop_request_receiver();
        let mut input_reader_info_receiver = get_reader_info_receiver().await;
        debug!(
            "Notifying the initial engine type: {}",
            self.config.engine_type(0)
        );
        self.event_publisher
            .publish(EgEvent::EngineSwitched(self.config.engine_type(0)))
            .await;
        self.event_publisher
            .publish(EgEvent::PolarityChanged((
                self.config.out_polarity(0),
                self.config.out_polarity(1),
            )))
            .await;
        loop {
            match select5(
                rx_receiver.receive(),
                prop_request_receiver.receive(),
                Timer::after(POLLING_INTERVAL),
                self.request_receiver.receive(),
                input_reader_info_receiver.changed(),
            )
            .await
            {
                Either5::First(rx_frame) => self.handle_a3_message(&rx_frame).await,
                Either5::Second(prop_request) => self.config.handle_request(prop_request),
                Either5::Third(()) => {}
                Either5::Fourth(request) => {
                    if self.handle_request(request).await {
                        break;
                    }
                }
                Either5::Fifth(input) => self.consume_input(input).await,
            };
            self.regular_task();
        }
    }

    async fn handle_a3_message(&mut self, message: &A3Datagram) {
        if let A3DatagramId::Standard(id) = message.id {
            if id == self.config.voice_id(0) {
                self.voice_1.handle_a3_message(message).await;
            } else if id == self.config.voice_id(1) {
                self.voice_2.handle_a3_message(message).await;
            }
        }
    }

    fn regular_task(&mut self) {
        self.voice_1.update();
        self.voice_2.update();
    }

    /// Handles the incoming request.
    /// Returns true if the request causes exiting this engine type.
    async fn handle_request(&mut self, request: EgRequest) -> bool {
        match request {
            EgRequest::GateEvent { id, event } => {
                match id {
                    GateId::Gate1 => self.voice_1.handle_gate_event(event),
                    GateId::Gate2 => self.voice_2.handle_gate_event(event),
                };
                false
            }
            EgRequest::SwitchEngine {
                engine_type,
                send_notif,
            } => {
                debug!("switching engine to {}", engine_type.name());
                if self.config.engine_type(0) == engine_type {
                    debug!("No change to engine type");
                } else {
                    self.config.set_engine_type(0, engine_type);
                    self.config.set_engine_type(1, engine_type);
                    save_engine_type(0, &engine_type).await;
                    save_engine_type(1, &engine_type).await;
                }
                if send_notif {
                    self.event_publisher
                        .publish(EgEvent::EngineSwitched(engine_type))
                        .await;
                }
                true
            }
            EgRequest::ChangeOutputPolarities {
                polarity_1,
                polarity_2,
                send_notif,
            } => {
                debug!("switching polarities");
                if self.config.out_polarity(0) == polarity_1
                    && self.config.out_polarity(1) == polarity_2
                {
                    debug!("No change to polarities");
                } else {
                    self.config.set_out_polarity(0, polarity_1);
                    self.config.set_out_polarity(1, polarity_2);
                    save_out_polarity(0, polarity_1).await;
                    save_out_polarity(1, polarity_2).await;
                    self.voice_1.params.value_to_output = choose_output_converter(&polarity_1);
                    self.voice_2.params.value_to_output = choose_output_converter(&polarity_2);
                }
                if send_notif {
                    self.event_publisher
                        .publish(EgEvent::PolarityChanged((polarity_1, polarity_2)))
                        .await;
                }
                false
            }
            EgRequest::ChangeCvDestination {
                source,
                destination,
            } => {
                debug!("change cv destination");
                match source {
                    CvKind::A => {
                        if self.config.cv_destination_a() != destination {
                            self.config.set_cv_destination_a(destination);
                            save_cv_destination(self.config.engine_type(0), source, destination)
                                .await;
                        }
                    }
                    CvKind::B => {
                        if self.config.cv_destination_b() != destination {
                            self.config.set_cv_destination_b(destination);
                            save_cv_destination(self.config.engine_type(0), source, destination)
                                .await;
                        }
                    }
                }
                false
            }
            EgRequest::ChangeNoteScalingDepth { depth, save } => {
                self.config.set_note_scaling_depth(0, depth);
                self.config.set_note_scaling_depth(1, depth);
                if save {
                    save_note_scaling_depths(depth, depth).await;
                }
                false
            }
            EgRequest::ToggleMode { mode } => {
                debug!(
                    "toggle mode {}, current={}",
                    mode, self.voice_1.params.operation_mode
                );
                let next_mode = if self.voice_1.params.operation_mode == mode {
                    Mode::Normal
                } else {
                    mode
                };
                self.voice_1.params.operation_mode = next_mode;
                self.voice_2.params.operation_mode = next_mode;
                true
            }
            EgRequest::UpdateZeroPoints {
                value_1,
                value_2,
                save,
            } => {
                debug!("UpdateZeroPoints received: {:#x}, {:#x}", value_1, value_2);
                self.voice_1.params.out_zero_point = value_1;
                self.voice_2.params.out_zero_point = value_2;
                if save {
                    storage::save(ADDR_OUT_ZERO_POINT_1, Value::U16(value_1), &SIGNAL_STORAGE)
                        .await
                        .unwrap();
                    storage::save(ADDR_OUT_ZERO_POINT_2, Value::U16(value_2), &SIGNAL_STORAGE)
                        .await
                        .unwrap();
                }
                false
            }
        }
    }

    async fn consume_input(&mut self, input: InputReaderInfo) {
        self.config.translate(&input.pot_info);
        self.voice_1.update_params(&self.config, &input);
        self.voice_2.update_params(&self.config, &input);
        self.event_publisher
            .publish(EgEvent::PotMoved(input.pot_info.clone()))
            .await;
    }
}

struct EgVoice<'a, EngineT: Engine> {
    params: &'a mut VoiceParams,

    ind_gate: &'a mut Output<'static>,

    engine: EngineT,

    last_value: u16,
}

impl<'a, EngineT: Engine> EgVoice<'a, EngineT> {
    pub fn new(
        voice_index: usize,
        params: &'a mut VoiceParams,
        ind_gate: &'a mut Output<'static>,
        config: &EgConfig,
    ) -> Self {
        let mut engine = EngineT::new();
        engine.initialize(voice_index, config);
        Self {
            ind_gate,
            params,
            engine,
            last_value: 0,
        }
    }

    pub async fn handle_a3_message(&mut self, message: &A3Datagram) {
        if self.params.physical_gate_enabled {
            // do nothing when analog gate is enabled
            return;
        }
        let data = message.data;
        let mut index = 0usize;
        while index < message.size {
            let op = data[index];
            index += 1;
            match op {
                A3_VOICE_MSG_SET_NOTE => {
                    if index < message.size {
                        self.params.note = data[index];
                        index += 1;
                    } else {
                        warn!("could not fetch note");
                    }
                }
                A3_VOICE_MSG_GATE_ON => {
                    if index + 1 < message.size {
                        self.params.velocity = (data[index] as u16) << 8 + (data[index + 1] as u16);
                        index += 2;
                    } else {
                        warn!("could not fetch velocity");
                    }
                    self.gate_on();
                }
                A3_VOICE_MSG_GATE_OFF => self.gate_off(),
                _ => {} // do nothing
            }
            yield_now().await;
        }
    }

    pub fn handle_gate_event(&mut self, event: GateEventType) {
        match event {
            GateEventType::PhysicalGateEnabled => {
                self.params.physical_gate_enabled = true;
                self.ind_gate.set_high();
            }
            GateEventType::PhysicalGateDisabled => {
                self.params.physical_gate_enabled = false;
            }
            GateEventType::GateOn { velocity } => {
                self.params.velocity = velocity;
                self.gate_on()
            }
            GateEventType::GateOff => {
                self.gate_off();
            }
        }
    }

    fn gate_on(&mut self) {
        self.ind_gate.set_high();
        if self.queue_length() >= BUF_SEGMENT_LENGTH {
            // rewind
            unsafe {
                HEADS[self.params.voice_index] =
                    (TAILS[self.params.voice_index] + BUF_SEGMENT_LENGTH) % BUF_SIZE;
            }
        }
        self.engine.gate_on(&self.params);
    }

    fn gate_off(&mut self) {
        self.ind_gate.set_low();
        self.engine.gate_off();
    }

    pub fn update_params(&mut self, config: &EgConfig, input: &InputReaderInfo) {
        let index = self.params.voice_index;
        self.engine.update_params(index, config, input);
    }

    pub fn update(&mut self) {
        while self.queue_length() < BUF_SEGMENT_LENGTH {
            unsafe {
                let head = HEADS[self.params.voice_index];
                BUFFERS[self.params.voice_index][head] = self.last_value;
                HEADS[self.params.voice_index] = (head + 1) % BUF_SIZE;
            }
            self.last_value = self.engine.update(&self.params);
        }
    }

    pub fn queue_length(&self) -> usize {
        unsafe {
            let head = HEADS[self.params.voice_index];
            let tail = TAILS[self.params.voice_index];
            if head >= tail {
                head - tail
            } else {
                head + BUF_SIZE - tail
            }
        }
    }
}

#[interrupt]
unsafe fn TIM2() {
    unsafe {
        if HEADS[0] == TAILS[0] {
            // warn!("queue empty");
        } else {
            core::ptr::write_volatile(
                pac::DAC1.dhr12r(0).as_ptr() as *mut u32,
                BUFFERS[0][TAILS[0]] as u32,
            );
            TAILS[0] = (TAILS[0] + 1) % BUF_SIZE;
        }
        if HEADS[1] == TAILS[1] {
            // warn!("queue empty");
        } else {
            core::ptr::write_volatile(
                pac::DAC1.dhr12r(1).as_ptr() as *mut u32,
                BUFFERS[1][TAILS[1]] as u32,
            );
            TAILS[1] = (TAILS[1] + 1) % BUF_SIZE;
        }
    }
    pac::TIM2.sr().modify(|reg| reg.set_uif(false));
}

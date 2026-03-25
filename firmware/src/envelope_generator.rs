mod adsr_engine;
mod config;
mod definitions;
mod diag_engine;
mod linear_engine;
mod para_decays_engine;
mod two_decays_engine;

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
    pubsub::{self, PubSubChannel},
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use crate::{
    analog3::{self, addresses_common::*, definitions::*, storage},
    input_reader::{InputReaderInfo, get_reader_info_receiver},
};

pub use self::definitions::{EgEvent, EgRequest, EngineType, GateEventType, GateId};
use self::{
    adsr_engine::AdsrEngine, config::EgConfig, definitions::Engine, definitions::VoiceParams,
    diag_engine::DiagEngine, linear_engine::LinearEngine, para_decays_engine::ParaDecaysEngine,
    two_decays_engine::TwoDecaysEngine,
};

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

pub fn start(
    spawner: Spawner,
    dac_channels: Dac<'static, Blocking>,
    ind_gate_1: Output<'static>,
    ind_gate_2: Output<'static>,
) {
    let eg_resources = EgResources::new(ind_gate_1, ind_gate_2);
    spawner.spawn(run_envelope_generator(dac_channels, eg_resources).unwrap());
}

pub async fn get_uid() -> u32 {
    let Value::U32(mut uid) = storage::load(A3_ADDR_MODULE_UID, ValueType::U32, &SIGNAL_STORAGE)
        .await
        .unwrap()
    else {
        panic!("wrong type returned");
    };
    debug!("loaded UID: {=u32:#x}", uid);
    if uid == u32::MAX {
        uid = 0xe9de9d;
        storage::save(A3_ADDR_MODULE_UID, Value::U32(uid), &SIGNAL_STORAGE)
            .await
            .unwrap();
    }
    uid
}

pub async fn get_name() -> String<A3_MAX_PROP_DATA_SIZE> {
    let Value::Text(mut name) =
        storage::load(A3_ADDR_MODULE_NAME, ValueType::Text, &SIGNAL_STORAGE)
            .await
            .unwrap()
    else {
        panic!("wrong type returned");
    };
    debug!("loaded name: {}", name.as_str());
    if name.len() == 0 {
        name = String::try_from("Humps RS D").unwrap();
        storage::save(
            A3_ADDR_MODULE_NAME,
            Value::Text(name.clone()),
            &SIGNAL_STORAGE,
        )
        .await
        .unwrap();
    }
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
        match eg_resources.config.engine_type {
            EngineType::ParaDecays => {
                let mut eg = EnvelopeGenerator::<ParaDecaysEngine>::new(&mut eg_resources);
                eg.run().await;
            }
            EngineType::Addsr => {
                let mut eg = EnvelopeGenerator::<TwoDecaysEngine>::new(&mut eg_resources);
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
            EngineType::Diag => {
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
    ind_gate_1: Output<'static>,
    ind_gate_2: Output<'static>,
}

impl EgResources {
    pub fn new(ind_gate_1: Output<'static>, ind_gate_2: Output<'static>) -> Self {
        Self {
            config: EgConfig::new(0x101, 0x102, EngineType::ParaDecays),
            request_receiver: CHANNEL_REQUEST.receiver(),
            event_publisher: CHANNEL_EVENT.publisher().unwrap(),
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
        let voice_1 = EgVoice::new(0, &mut resources.ind_gate_1, &resources.config);
        let voice_2 = EgVoice::new(1, &mut resources.ind_gate_2, &resources.config);
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
        self.event_publisher
            .publish(EgEvent::EngineSwitched(self.config.engine_type.clone()))
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
                    if self.handle_request(request) {
                        break;
                    }
                }
                Either5::Fifth(input) => self.consume_input(input),
            };
            self.regular_task();
        }
    }

    async fn handle_a3_message(&mut self, message: &A3Datagram) {
        if let A3DatagramId::Standard(id) = message.id {
            if id == self.config.voice_id[0] {
                self.voice_1.handle_a3_message(message).await;
            } else if id == self.config.voice_id[1] {
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
    fn handle_request(&mut self, request: EgRequest) -> bool {
        match request {
            EgRequest::GateEvent { id, event } => {
                match id {
                    GateId::Gate1 => self.voice_1.handle_gate_event(event),
                    GateId::Gate2 => self.voice_2.handle_gate_event(event),
                };
                false
            }
            EgRequest::SwitchEngine(engine_type) => {
                debug!("switching engine to {}", engine_type.name());
                self.config.engine_type = engine_type;
                true
            }
        }
    }

    fn consume_input(&mut self, input: InputReaderInfo) {
        self.config.translate(&input.pot_info);
        self.voice_1.update_params(&self.config, &input);
        self.voice_2.update_params(&self.config, &input);
    }
}

struct EgVoice<'a, EngineT: Engine> {
    ind_gate: &'a mut Output<'static>,

    analog_gate_enabled: bool,

    params: VoiceParams,

    engine: EngineT,

    last_value: u16,
}

impl<'a, EngineT: Engine> EgVoice<'a, EngineT> {
    pub fn new(voice_index: usize, ind_gate: &'a mut Output<'static>, config: &EgConfig) -> Self {
        let mut engine = EngineT::new();
        engine.initialize(voice_index, config);
        Self {
            ind_gate,
            analog_gate_enabled: false,
            params: VoiceParams {
                voice_index,
                note: 60, // middle C
                velocity: 0,
            },
            engine,
            last_value: 0,
        }
    }

    pub async fn handle_a3_message(&mut self, message: &A3Datagram) {
        if self.analog_gate_enabled {
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
            GateEventType::AnalogGateEnabled => {
                self.analog_gate_enabled = true;
                self.ind_gate.set_high();
            }
            GateEventType::AnalogGateDisabled => {
                self.analog_gate_enabled = false;
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

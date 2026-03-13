mod config;
mod default_engine;
mod definitions;
mod diag_engine;

use defmt::{debug, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either5, select5};
use embassy_stm32::{
    dac::{self, Dac},
    gpio::Output,
    interrupt,
    mode::Blocking,
    pac,
    peripherals::DAC1,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel};
use embassy_time::{Duration, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use config::EgConfig;
use default_engine::DefaultEgEngine;
use definitions::{EngineType, VoiceParams};
use diag_engine::DiagEgEngine;

use crate::input_reader::{
    GateEvent, GateEventType, GateId, InputReaderInfo, get_gate_event_receiver,
};
use crate::{
    analog3::{
        self,
        addresses_common::*,
        definitions::*,
        property::{PropRequest, Property},
        storage,
    },
    input_reader::get_reader_info_receiver,
};

// parameter tweaks
const POLLING_INTERVAL: Duration = Duration::from_micros(100); // 20 kHz
const BUF_SEGMENT_LENGTH: usize = 128;

const BUF_SIZE: usize = BUF_SEGMENT_LENGTH * 2;
static mut BUFFERS: [[u16; BUF_SIZE]; 2] = [[0; BUF_SIZE]; 2];
static mut HEADS: [usize; 2] = [0; 2];
static mut TAILS: [usize; 2] = [0; 2];

pub fn start(
    spawner: Spawner,
    dac_channels: Dac<'static, DAC1, Blocking>,
    ind_gate_1: Output<'static>,
    ind_gate_2: Output<'static>,
) {
    let eg = EnvelopeGenerator::new(ind_gate_1, ind_gate_2);
    spawner.spawn(run_envelope_generator(dac_channels, eg).unwrap());
}

pub async fn get_uid() -> u32 {
    let Value::U32(mut uid) = storage::load(A3_ADDR_MODULE_UID, ValueType::U32)
        .await
        .unwrap()
    else {
        panic!("wrong type returned");
    };
    debug!("loaded UID: {=u32:#x}", uid);
    if uid == u32::MAX {
        uid = 0xe9de9d;
        storage::save(A3_ADDR_MODULE_UID, Value::U32(uid))
            .await
            .unwrap();
    }
    uid
}

pub async fn get_name() -> String<A3_MAX_PROP_DATA_SIZE> {
    let Value::Text(mut name) = storage::load(A3_ADDR_MODULE_NAME, ValueType::Text)
        .await
        .unwrap()
    else {
        panic!("wrong type returned");
    };
    debug!("loaded name: {}", name.as_str());
    if name.len() == 0 {
        name = String::try_from("Humps RS D").unwrap();
        storage::save(A3_ADDR_MODULE_NAME, Value::Text(name.clone()))
            .await
            .unwrap();
    }
    name
}

#[embassy_executor::task]
async fn run_envelope_generator(
    dac_channels: Dac<'static, DAC1, Blocking>,
    mut eg: EnvelopeGenerator,
) {
    let (mut dac1, mut dac2) = dac_channels.split();
    dac1.set_trigger(dac::TriggerSel::Tim2);
    dac1.set_triggering(true);
    dac1.enable();
    dac2.set_trigger(dac::TriggerSel::Tim2);
    dac2.set_triggering(true);
    dac2.enable();

    eg.run().await;
}

struct EnvelopeGenerator {
    config: EgConfig,
    gate_event_receiver: channel::Receiver<'static, ThreadModeRawMutex, GateEvent, 4>,
    voice_1: EgVoice,
    voice_2: EgVoice,
}

impl EnvelopeGenerator {
    pub fn new(ind_gate_1: Output<'static>, ind_gate_2: Output<'static>) -> Self {
        Self {
            config: EgConfig::new(0x101, 0x102),
            gate_event_receiver: get_gate_event_receiver(),
            voice_1: EgVoice::new(0, ind_gate_1),
            voice_2: EgVoice::new(1, ind_gate_2),
        }
    }

    pub async fn run(&mut self) {
        let rx_receiver = analog3::get_forwarder_receiver();
        let prop_request_receiver = analog3::get_prop_request_receiver();
        let mut input_reader_info_receiver = get_reader_info_receiver().await;
        loop {
            match select5(
                rx_receiver.receive(),
                prop_request_receiver.receive(),
                Timer::after(POLLING_INTERVAL),
                self.gate_event_receiver.receive(),
                input_reader_info_receiver.changed(),
            )
            .await
            {
                Either5::First(rx_frame) => self.handle_a3_message(&rx_frame).await,
                Either5::Second(prop_request) => match prop_request {
                    PropRequest::GetNumProperties { reply } => {
                        reply.signal(Some(Property::new(0, Value::U8(1))));
                    }
                    PropRequest::GetProperty { index, reply } => {
                        if index == 0 {
                            reply.signal(Some(Property::new(
                                self.config.prop_id,
                                Value::U32(self.config.value),
                            )));
                        } else {
                            reply.signal(None)
                        }
                    }
                    PropRequest::SetProperty {
                        prop_id,
                        length: _,
                        value,
                    } => {
                        if prop_id == self.config.prop_id {
                            let value = u32::from_be_bytes(value[..4].try_into().unwrap());
                            self.config.set_count(value);
                        }
                    }
                },
                Either5::Third(()) => {}
                Either5::Fourth(event) => self.handle_gate_event(event).await,
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

    async fn handle_gate_event(&mut self, event: GateEvent) {
        match event.id {
            GateId::Gate1 => self.voice_1.handle_gate_event(event.event).await,
            GateId::Gate2 => self.voice_2.handle_gate_event(event.event).await,
        };
    }

    fn consume_input(&mut self, input: InputReaderInfo) {
        self.config.translate(&input.pot_info);
        self.voice_1.update_params(&self.config, &input);
        self.voice_2.update_params(&self.config, &input);
    }
}

struct EgVoice {
    ind_gate: Output<'static>,

    analog_gate_enabled: bool,

    params: VoiceParams,

    // engines
    engine_type: EngineType,
    default_engine: DefaultEgEngine,
    diag_engine: DiagEgEngine,
}

impl EgVoice {
    pub fn new(voice_index: usize, ind_gate: Output<'static>) -> Self {
        Self {
            ind_gate,
            analog_gate_enabled: false,
            params: VoiceParams {
                voice_index,
                note: 60, // middle C
                velocity: 0,
            },
            engine_type: EngineType::Default,
            default_engine: DefaultEgEngine::new(),
            diag_engine: DiagEgEngine::new(),
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
                        debug!("note set to {}", self.params.note);
                        index += 1;
                    } else {
                        warn!("could not fetch note");
                    }
                }
                A3_VOICE_MSG_GATE_ON => {
                    if index + 1 < message.size {
                        self.params.velocity = (data[index] as u16) << 8 + (data[index + 1] as u16);
                        debug!("velocity set to {}", self.params.velocity);
                        index += 2;
                    } else {
                        warn!("could not fetch velocity");
                    }
                    self.gate_on();
                }
                A3_VOICE_MSG_GATE_OFF => self.gate_off(),
                _ => {} // do nothing
            }
        }
    }

    pub async fn handle_gate_event(&mut self, event: GateEventType) {
        match event {
            GateEventType::AnalogGateEnabled => {
                self.analog_gate_enabled = true;
                self.ind_gate.set_high();
            }
            GateEventType::AnalogGateDisabled => {
                self.analog_gate_enabled = false;
            }
            GateEventType::GateOn { level } => {
                self.params.velocity = level;
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
        match self.engine_type {
            EngineType::Default => self.default_engine.gate_on(&self.params),
            EngineType::Diag => self.diag_engine.gate_on(&self.params),
        }
    }

    fn gate_off(&mut self) {
        self.ind_gate.set_low();
        match self.engine_type {
            EngineType::Default => self.default_engine.gate_off(),
            EngineType::Diag => self.diag_engine.gate_off(),
        }
    }

    pub fn update_params(&mut self, config: &EgConfig, input: &InputReaderInfo) {
        let index = self.params.voice_index;
        match self.engine_type {
            EngineType::Default => self.default_engine.update_params(index, config, input),
            EngineType::Diag => self.diag_engine.update_params(index, config, input),
        }
    }

    pub fn update(&mut self) {
        let mut current_value: u16 = match self.engine_type {
            EngineType::Default => self.default_engine.out_buf,
            EngineType::Diag => self.diag_engine.out_buf,
        };
        while self.queue_length() < BUF_SEGMENT_LENGTH {
            unsafe {
                let head = HEADS[self.params.voice_index];
                BUFFERS[self.params.voice_index][head] = current_value;
                HEADS[self.params.voice_index] = (head + 1) % BUF_SIZE;
            }
            current_value = match self.engine_type {
                EngineType::Default => self.default_engine.update(&self.params),
                EngineType::Diag => self.diag_engine.update(&self.params),
            }
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
            pac::DAC1
                .dhr12r(0)
                .write(|reg| reg.set_dhr(BUFFERS[0][TAILS[0]]));
            TAILS[0] = (TAILS[0] + 1) % BUF_SIZE;
        }
        if HEADS[1] == TAILS[1] {
            // warn!("queue empty");
        } else {
            pac::DAC1
                .dhr12r(1)
                .write(|reg| reg.set_dhr(BUFFERS[1][TAILS[1]]));
            TAILS[1] = (TAILS[1] + 1) % BUF_SIZE;
        }
    }
    pac::TIM2.sr().modify(|reg| reg.set_uif(false));
}

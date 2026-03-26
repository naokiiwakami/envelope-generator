pub mod addresses_common;
pub mod can;
pub mod definitions;
mod indicator;
pub mod property;
pub mod storage;

use can::{CAN_REQUEST_CHANNEL_SIZE, CAN_RX_CHANNEL_SIZE, can_handler};
use core::cmp::min;
use definitions::*;
use defmt::debug;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    can::{Can, frame::FdFrame},
    flash::Error,
    gpio::Output,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{self, Channel},
    signal::Signal,
};
use embassy_time::Timer;
use embedded_can::Id;
use heapless::{String, Vec};
use indicator::{
    INDICATOR_CHANNEL_SIZE, IndicatorRequest, get_indicator_request_sender, run_indicator,
};

use crate::analog3::addresses_common::A3_ADDR_MODULE_NAME;
use crate::analog3::{
    can::{
        CanRequest, get_can_req_sender, get_can_rx_receiver, make_tx_ext_request,
        make_tx_std_request,
    },
    definitions::Value,
    property::{LV, PropRequest, Property},
};

// channels
pub const MSG_FWD_CHANNEL_SIZE: usize = 8;
static FWD_CHANNEL: Channel<ThreadModeRawMutex, A3Datagram, MSG_FWD_CHANNEL_SIZE> = Channel::new();

pub const CFG_CHANNEL_SIZE: usize = 2;
static CFG_CHANNEL: Channel<ThreadModeRawMutex, PropRequest<'static>, CFG_CHANNEL_SIZE> =
    Channel::new();

pub static SIGNAL_WIRE: Signal<ThreadModeRawMutex, LV> = Signal::new();

pub fn get_forwarder_receiver()
-> channel::Receiver<'static, ThreadModeRawMutex, A3Datagram, MSG_FWD_CHANNEL_SIZE> {
    FWD_CHANNEL.receiver()
}

pub fn get_prop_request_receiver()
-> channel::Receiver<'static, ThreadModeRawMutex, PropRequest<'static>, CFG_CHANNEL_SIZE> {
    CFG_CHANNEL.receiver()
}

static CHANNEL_A3_REQUEST: Channel<ThreadModeRawMutex, Analog3Request, 8> = Channel::new();

static SIGNAL_STORAGE: Signal<ThreadModeRawMutex, Result<Value, Error>> = Signal::new();

// config
#[non_exhaustive]
pub struct Analog3Config {
    module_type: u16,
    uid: u32,
    name: String<A3_MAX_PROP_DATA_SIZE>,
}

impl Analog3Config {
    pub fn new(module_type: u16, uid: u32, name: &str) -> Self {
        Self {
            module_type,
            uid,
            name: String::try_from(name).unwrap(),
        }
    }
}

// ok let's go
pub async fn start(
    a3_config: Analog3Config,
    can: Can<'static>,
    a3_red_led: Output<'static>,
    a3_blue_led: Output<'static>,
    spawner: Spawner,
) {
    spawner.spawn(run_indicator(a3_red_led, a3_blue_led).unwrap());
    let analog3 = Analog3::new(a3_config, spawner);
    spawner.spawn(run_analog3(analog3).unwrap());
    spawner.spawn(can_handler(can).unwrap());
}

pub async fn diagnose(done: &'static Signal<ThreadModeRawMutex, ()>) {
    let sender = CHANNEL_A3_REQUEST.sender();
    sender.send(Analog3Request::Diagnose { done }).await;
    done.wait().await;
}

#[embassy_executor::task]
async fn run_analog3(mut analog3: Analog3) {
    analog3.run().await;
}

enum Analog3Request {
    TerminateStream {
        wire_id: u16,
    },
    UpdateProperty {
        prop_id: u8,
        length: usize,
        value: [u8; A3_MAX_PROP_DATA_SIZE],
    },
    Diagnose {
        done: &'static Signal<ThreadModeRawMutex, ()>,
    },
}

pub struct Analog3 {
    pub uid: u32,
    pub id: u16,
    pub module_type: u16,
    pub name: String<A3_MAX_PROP_DATA_SIZE>,
    spawner: Spawner,
    wire_in_use: Option<u16>,
    rx_receiver: channel::Receiver<'static, ThreadModeRawMutex, FdFrame, CAN_RX_CHANNEL_SIZE>,
    rx_forwarder: channel::Sender<'static, ThreadModeRawMutex, A3Datagram, MSG_FWD_CHANNEL_SIZE>,
    can_req_sender:
        channel::Sender<'static, ThreadModeRawMutex, CanRequest, CAN_REQUEST_CHANNEL_SIZE>,
    indicator_req_sender:
        channel::Sender<'static, ThreadModeRawMutex, IndicatorRequest, INDICATOR_CHANNEL_SIZE>,
    config_request_sender:
        channel::Sender<'static, ThreadModeRawMutex, PropRequest<'static>, CFG_CHANNEL_SIZE>,
}

impl Analog3 {
    pub fn new(a3_config: Analog3Config, spawner: Spawner) -> Self {
        Self {
            uid: a3_config.uid,
            id: A3_ID_UNASSIGNED,
            module_type: a3_config.module_type,
            name: a3_config.name,
            spawner,
            wire_in_use: None,
            rx_receiver: get_can_rx_receiver(),
            rx_forwarder: FWD_CHANNEL.sender(),
            can_req_sender: get_can_req_sender(),
            indicator_req_sender: get_indicator_request_sender(),
            config_request_sender: CFG_CHANNEL.sender(),
        }
    }

    async fn run(&mut self) {
        let req_receiver = CHANNEL_A3_REQUEST.receiver();
        self.indicator_req_sender
            .send(IndicatorRequest::SetRedLed)
            .await;
        self.sign_in().await;
        loop {
            match select(self.rx_receiver.receive(), req_receiver.receive()).await {
                Either::First(frame) => {
                    self.handle_rx_message(frame).await;
                }
                Either::Second(request) => {
                    _ = self.handle_a3_command(request).await;
                }
            }
        }
    }

    async fn handle_a3_command(&mut self, request: Analog3Request) {
        match request {
            Analog3Request::TerminateStream { wire_id } => {
                self.terminate_stream(wire_id);
            }
            Analog3Request::UpdateProperty {
                prop_id,
                length,
                value,
            } => {
                let mut bytes = Vec::<u8, A3_MAX_PROP_DATA_SIZE>::new();
                if prop_id == A3_PROP_ID_NAME {
                    bytes.extend_from_slice(&value[..length]).unwrap();
                    self.name = String::from_utf8(bytes).unwrap();
                    storage::save(
                        A3_ADDR_MODULE_NAME,
                        Value::Text(self.name.clone()),
                        &SIGNAL_STORAGE,
                    )
                    .await
                    .unwrap();
                }
            }
            Analog3Request::Diagnose { done } => {
                self.diagnose().await;
                done.signal(());
            }
        };
    }

    async fn diagnose(&mut self) {
        self.indicator_req_sender
            .send(IndicatorRequest::ResetBlueLed)
            .await;
        self.indicator_req_sender
            .send(IndicatorRequest::ResetRedLed)
            .await;
        self.indicator_req_sender
            .send(IndicatorRequest::BlinkRedLed {
                blinks: 12,
                interval: 100,
            })
            .await;
        Timer::after_millis(3000).await;
        self.indicator_req_sender
            .send(IndicatorRequest::BlinkBlueLed {
                blinks: 12,
                interval: 100,
            })
            .await;
        Timer::after_millis(3000).await;
        let req = if self.id != A3_ID_UNASSIGNED {
            IndicatorRequest::SetBlueLed
        } else {
            IndicatorRequest::SetRedLed
        };
        self.indicator_req_sender.send(req).await;
    }

    /// Initiates the stream by registering the wire ID
    fn initiate_stream(&mut self, wire_id: u16) -> StreamStatus {
        // only one stream is allowed at a time
        match self.wire_in_use {
            Some(_) => StreamStatus::Busy,
            None => {
                self.wire_in_use = Some(wire_id);
                StreamStatus::Ready
            }
        }
    }

    // Terminates the stream by unregistering the wire ID
    fn terminate_stream(&mut self, wire_id: u16) -> StreamStatus {
        match self.wire_in_use {
            Some(in_use) => {
                if in_use != wire_id {
                    // something is wrong
                    StreamStatus::NoSuchStream
                } else {
                    self.wire_in_use = None;
                    StreamStatus::Ready
                }
            }
            None => {
                // ignore silently
                StreamStatus::Ready
            }
        }
    }

    async fn handle_rx_message(&mut self, rx_frame: FdFrame) {
        let is_admin_message = match rx_frame.id() {
            Id::Standard(id) => id.as_raw() >= A3_ID_ADMIN_WIRES_BASE,
            Id::Extended(_) => false, // TODO: Handle an extension frame
        };
        if is_admin_message {
            self.handle_admin_message(&rx_frame).await;
        }
        self.rx_forwarder
            .send(A3Datagram::from_fdcan(&rx_frame))
            .await;
    }

    async fn handle_admin_message(&mut self, rx_frame: &FdFrame) {
        let Id::Standard(id) = rx_frame.id() else {
            return;
        };
        let raw_id = id.as_raw();
        if raw_id == A3_ID_MISSION_CONTROL {
            self.handle_mission_control_message(rx_frame).await;
            return;
        }
        let Some(wire_id) = self.wire_in_use else {
            return;
        };
        if raw_id == wire_id {
            SIGNAL_WIRE.signal(LV::from_frame(rx_frame));
        }
    }

    async fn handle_mission_control_message(&mut self, rx_frame: &FdFrame) {
        let data = &rx_frame.data();
        let opcode = if rx_frame.header().len() > 0 {
            data[0]
        } else {
            A3_MC_NO_OPCODE
        };
        match opcode {
            A3_MC_SIGN_IN => {
                if self.id == A3_ID_UNASSIGNED {
                    self.sign_in().await;
                } else {
                    self.notify_id().await;
                }
            }
            A3_MC_ASSIGN_MODULE_ID => {
                let target_uid: u32 = (data[1] as u32) << 24
                    | (data[2] as u32) << 16
                    | (data[3] as u32) << 8
                    | (data[4] as u32);
                if target_uid == self.uid {
                    self.id = data[5] as u16 + A3_ID_IM_BASE;

                    self.id_assign_ack().await;

                    self.indicator_req_sender
                        .send(IndicatorRequest::SetBlueLed)
                        .await;
                    self.indicator_req_sender
                        .send(IndicatorRequest::ResetRedLed)
                        .await;
                }
            }
            _ => {
                let target_id = data[1] as u16 + A3_ID_IM_BASE;
                if target_id == self.id {
                    self.process_mission_control_command(opcode, data).await;
                }
            }
        }
    }

    async fn process_mission_control_command(&mut self, opcode: u8, data: &[u8]) {
        match opcode {
            A3_MC_PING => self.handle_ping(&data).await,
            A3_MC_REQUEST_NAME => self.handle_request_name(&data),
            A3_MC_REQUEST_CONFIG => self.handle_request_config(&data),
            A3_MC_MODIFY_CONFIG => self.handle_modify_config(&data),
            _ => {}
        }
    }

    async fn handle_ping(&self, data: &[u8]) {
        self.ping_reply().await;
        if data.len() >= 3 && data[2] > 0 {
            self.indicator_req_sender
                .send(IndicatorRequest::BlinkBlueLed {
                    blinks: 3,
                    interval: 70,
                })
                .await;
        }
    }

    fn handle_request_name(&mut self, data: &[u8]) {
        if data.len() < 3 {
            // it's a protocol error if the data  is missing the wire_num field.
            // ignore the message silengly in the case
            return;
        }
        let wire_num = data[2];
        let wire_id = wire_num as u16 + A3_ID_ADMIN_WIRES_BASE;
        let stream_status = self.initiate_stream(wire_id);
        self.spawner.spawn(
            reply_name(
                self.name.clone(),
                wire_num,
                CHANNEL_A3_REQUEST.sender(),
                stream_status,
            )
            .unwrap(),
        );
    }

    fn handle_request_config(&mut self, data: &[u8]) {
        if data.len() < 3 {
            // it's a protocol error if the wire number  is missing in the data.
            // ignore the message silengly in the case
            return;
        }
        let wire_num = data[2];
        let wire_id = wire_num as u16 + A3_ID_ADMIN_WIRES_BASE;
        let stream_status = self.initiate_stream(wire_id);
        self.spawner.spawn(
            reply_config(
                self.uid,
                self.module_type,
                self.name.clone(),
                wire_num,
                self.config_request_sender.clone(),
                CHANNEL_A3_REQUEST.sender(),
                stream_status,
            )
            .unwrap(),
        );
    }

    fn handle_modify_config(&mut self, data: &[u8]) {
        if data.len() < 3 {
            // it's a protocol error if the wire number  is missing in the data.
            // ignore the message silengly in the case
            return;
        }
        let wire_num = data[2];
        let wire_id = wire_num as u16 + A3_ID_ADMIN_WIRES_BASE;
        let stream_status = self.initiate_stream(wire_id);
        self.spawner.spawn(
            modify_config(
                wire_num,
                self.config_request_sender.clone(),
                CHANNEL_A3_REQUEST.sender(),
                stream_status,
            )
            .unwrap(),
        );
    }

    async fn sign_in(&self) {
        let request = make_tx_ext_request(self.uid, &[A3_ADMIN_SIGN_IN]);
        self.can_req_sender.send(request).await;
    }

    async fn notify_id(&self) {
        let request = make_tx_ext_request(
            self.uid,
            &[A3_ADMIN_NOTIFY_ID, (self.id - A3_ID_IM_BASE) as u8],
        );
        self.can_req_sender.send(request).await;
    }

    async fn id_assign_ack(&self) {
        let request = make_tx_std_request(self.id, &[A3_IM_ID_ASSIGN_ACK]);
        self.can_req_sender.send(request).await;
    }

    async fn ping_reply(&self) {
        let request = make_tx_std_request(self.id, &[A3_IM_PING_REPLY]);

        self.can_req_sender.send(request).await;
    }
}

#[non_exhaustive]
struct TxStream<'a> {
    pub payload: [u8; A3_STREAM_PAYLOAD_SIZE],
    pub index: usize,
    pub wire_id: u16,
    pub can_req_sender:
        &'a channel::Sender<'a, ThreadModeRawMutex, CanRequest, CAN_REQUEST_CHANNEL_SIZE>,
}

impl<'a> TxStream<'a> {
    pub fn new(
        wire_id: u16,
        can_req_sender: &'a channel::Sender<
            'a,
            ThreadModeRawMutex,
            CanRequest,
            CAN_REQUEST_CHANNEL_SIZE,
        >,
    ) -> Self {
        Self {
            payload: [0u8; 8],
            index: 0,
            wire_id,
            can_req_sender,
        }
    }

    #[inline]
    fn clear_payload(&mut self) {
        self.index = 0;
    }

    async fn put_data(&mut self, data: &[u8]) {
        let mut data_offset = 0;
        while data_offset < data.len() {
            let payload_space = A3_STREAM_PAYLOAD_SIZE - self.index;
            let bytes_to_put = min(payload_space, data.len() - data_offset);
            self.payload[self.index..self.index + bytes_to_put]
                .copy_from_slice(&data[data_offset..data_offset + bytes_to_put]);
            data_offset += bytes_to_put;
            self.index += bytes_to_put;
            if self.index == A3_STREAM_PAYLOAD_SIZE {
                self.flush_with_throttle().await;
                self.index = 0;
            }
        }
    }

    async fn flush_with_throttle(&mut self) {
        if self.index == 0 {
            return;
        }
        debug!("throttling");
        // TODO: put timeout
        // just wait without reading the content, the peer would send empty frames
        SIGNAL_WIRE.wait().await;
        self.flush().await;
    }

    async fn flush(&mut self) {
        if self.index == 0 {
            return;
        }
        let request = make_tx_std_request(self.wire_id, &self.payload[..self.index]);
        debug!("sending");
        self.can_req_sender.send(request).await;
        self.clear_payload();
    }

    async fn put_property(&mut self, property: Property) {
        let prop_id = property.prop_id;
        match property.value {
            Value::U8(value) => {
                self.put_data(&[prop_id, size_of_val(&value) as u8]).await;
                self.put_data(&value.to_be_bytes()).await;
            }
            Value::U16(value) => {
                self.put_data(&[prop_id, size_of_val(&value) as u8]).await;
                self.put_data(&value.to_be_bytes()).await;
            }
            Value::U32(value) => {
                self.put_data(&[prop_id, size_of_val(&value) as u8]).await;
                self.put_data(&value.to_be_bytes()).await;
            }
            Value::Text(text) => {
                self.put_data(&[prop_id, text.len() as u8]).await;
                self.put_data(&text.as_bytes()).await;
            }
            Value::Boolean(value) => {
                self.put_data(&[prop_id, size_of_val(&value) as u8]).await;
                self.put_data(&[if value { 1 } else { 0 }]).await;
            }
            Value::VectorU8(vec) => {
                self.put_data(&[prop_id, vec.len() as u8]).await;
                self.put_data(&vec.as_slice()).await;
            }
            Value::VectorU16(vec) => {
                self.put_data(&[prop_id, (vec.len() * 2) as u8]).await;
                for entry in vec {
                    self.put_data(&entry.to_be_bytes()).await;
                }
            }
        };
    }
}

#[embassy_executor::task]
async fn reply_name(
    name: String<A3_MAX_PROP_DATA_SIZE>,
    wire_num: u8,
    a3_request_sender: channel::Sender<'static, ThreadModeRawMutex, Analog3Request, 8>,
    stream_status: StreamStatus,
) {
    let can_req_sender = get_can_req_sender();
    let wire_id = wire_num as u16 + A3_ID_ADMIN_WIRES_BASE;

    let cont = match stream_status {
        StreamStatus::Ready => true,
        _ => false,
    };

    let mut stream = TxStream::new(wire_id, &can_req_sender);
    stream.put_data(&[stream_status as u8]).await;
    stream.flush().await;
    if !cont {
        return;
    }

    let data = name.as_bytes();

    stream.put_data(&[1, data.len() as u8]).await;
    stream.put_data(data).await;
    stream.flush_with_throttle().await;

    // release the stream
    a3_request_sender
        .send(Analog3Request::TerminateStream { wire_id })
        .await;
}

#[embassy_executor::task]
async fn reply_config(
    uid: u32,
    module_type: u16,
    name: String<A3_MAX_PROP_DATA_SIZE>,
    wire_num: u8,
    config_request_sender: channel::Sender<'static, ThreadModeRawMutex, PropRequest<'static>, 2>,
    a3_request_sender: channel::Sender<'static, ThreadModeRawMutex, Analog3Request, 8>,
    stream_status: StreamStatus,
) {
    let tx_sender = get_can_req_sender();
    let wire_id = wire_num as u16 + A3_ID_ADMIN_WIRES_BASE;

    let cont = match stream_status {
        StreamStatus::Ready => true,
        _ => false,
    };

    let mut stream = TxStream::new(wire_id, &tx_sender);
    stream.put_data(&[stream_status as u8]).await;
    stream.flush().await;
    if !cont {
        return;
    }

    static PROP_REPLY: Signal<ThreadModeRawMutex, Option<Property>> = Signal::new();
    config_request_sender
        .send(PropRequest::GetNumProperties { reply: &PROP_REPLY })
        .await;

    let reply = PROP_REPLY.wait().await.unwrap();
    let Value::U8(num_props) = reply.value else {
        a3_request_sender
            .send(Analog3Request::TerminateStream { wire_id })
            .await;
        return;
    };

    stream.put_data(&[num_props + 3]).await;

    // put common properties
    stream
        .put_property(Property::new(A3_PROP_ID_MODULE_UID, Value::U32(uid)))
        .await;
    stream
        .put_property(Property::new(
            A3_PROP_ID_MODULE_TYPE,
            Value::U16(module_type),
        ))
        .await;
    stream
        .put_property(Property::new(A3_PROP_ID_NAME, Value::Text(name)))
        .await;

    // put module properties
    for prop_index in 0..num_props {
        config_request_sender
            .send(PropRequest::GetProperty {
                index: prop_index,
                reply: &PROP_REPLY,
            })
            .await;
        if let Some(property) = PROP_REPLY.wait().await {
            stream.put_property(property).await;
        }
    }

    stream.flush_with_throttle().await;

    // release the stream
    a3_request_sender
        .send(Analog3Request::TerminateStream { wire_id })
        .await;
}

#[embassy_executor::task]
pub async fn modify_config(
    wire_num: u8,
    config_request_sender: channel::Sender<'static, ThreadModeRawMutex, PropRequest<'static>, 2>,
    a3_request_sender: channel::Sender<'static, ThreadModeRawMutex, Analog3Request, 8>,
    stream_status: StreamStatus,
) {
    let can_req_sender = get_can_req_sender();
    let wire_id = wire_num as u16 + A3_ID_ADMIN_WIRES_BASE;

    let mut stream = ModifyConfigStream::new(
        wire_id,
        &can_req_sender,
        &config_request_sender,
        &a3_request_sender,
    );
    stream.send_reply(stream_status.clone()).await;

    let StreamStatus::Ready = stream_status else {
        return;
    };

    loop {
        if stream.receive().await {
            break;
        }
        stream.send_continue().await;
    }

    debug!("DONE!");

    // release the stream
    a3_request_sender
        .send(Analog3Request::TerminateStream { wire_id })
        .await;
}

enum PropParserState {
    ReadingNumProps,
    ReadingPropId,
    ReadingPropLength,
    ReadingPropValue,
}

struct ModifyConfigStream<'a> {
    pub data: [u8; A3_MAX_PROP_DATA_SIZE],
    pub data_index: usize,
    pub wire_id: u16,
    pub can_req_sender:
        &'a channel::Sender<'a, ThreadModeRawMutex, CanRequest, CAN_REQUEST_CHANNEL_SIZE>,
    pub config_request_sender: &'a channel::Sender<'a, ThreadModeRawMutex, PropRequest<'static>, 2>,
    pub a3_request_sender: &'a channel::Sender<'a, ThreadModeRawMutex, Analog3Request, 8>,

    // TODO: we may want to split following to a parser object
    parser_state: PropParserState,
    num_properties: usize,
    prop_index: usize,
    prop_id: u8,
    prop_data_length: usize,
}

impl<'a> ModifyConfigStream<'a> {
    fn new(
        wire_id: u16,
        can_req_sender: &'a channel::Sender<
            'a,
            ThreadModeRawMutex,
            CanRequest,
            CAN_REQUEST_CHANNEL_SIZE,
        >,
        config_request_sender: &'a channel::Sender<'a, ThreadModeRawMutex, PropRequest<'static>, 2>,
        a3_request_sender: &'a channel::Sender<'a, ThreadModeRawMutex, Analog3Request, 8>,
    ) -> Self {
        Self {
            data: [0u8; A3_MAX_PROP_DATA_SIZE],
            data_index: 0,
            wire_id,
            can_req_sender,
            config_request_sender,
            a3_request_sender,
            parser_state: PropParserState::ReadingNumProps,
            num_properties: 0,
            prop_index: 0,
            prop_id: 0,
            prop_data_length: 0,
        }
    }

    /// receives data from the peer and handle it. Returns true when the stream is done
    async fn receive(&mut self) -> bool {
        let data = SIGNAL_WIRE.wait().await;
        let mut payload_index = 0;
        while payload_index < data.length {
            match self.parser_state {
                PropParserState::ReadingNumProps => {
                    self.num_properties = data.value[payload_index] as usize;
                    self.parser_state = PropParserState::ReadingPropId;
                    self.prop_index = 0;
                    payload_index += 1;
                }
                PropParserState::ReadingPropId => {
                    self.prop_id = data.value[payload_index];
                    self.parser_state = PropParserState::ReadingPropLength;
                    payload_index += 1;
                }
                PropParserState::ReadingPropLength => {
                    self.prop_data_length = data.value[payload_index] as usize;
                    self.prop_index = 0;
                    self.parser_state = PropParserState::ReadingPropValue;
                    payload_index += 1;
                }
                PropParserState::ReadingPropValue => {
                    let available = data.length - payload_index;
                    let remaining: usize = self.prop_data_length - self.data_index;
                    let to_fetch = min(available, remaining);
                    self.data[self.data_index..self.data_index + to_fetch]
                        .copy_from_slice(&data.value[payload_index..payload_index + to_fetch]);
                    payload_index += to_fetch;
                    self.data_index += to_fetch;
                    if self.data_index == self.prop_data_length {
                        // now the property data is ready. consume it
                        self.consume_property().await;
                        self.prop_index += 1;
                        self.data_index = 0;
                        self.parser_state = PropParserState::ReadingPropId;
                    }
                }
            }
        }
        if let PropParserState::ReadingPropId = self.parser_state {
            self.prop_index == self.num_properties
        } else {
            false
        }
    }

    async fn consume_property(&mut self) {
        if self.prop_id <= A3_PROP_ID_NAME {
            self.a3_request_sender
                .send(Analog3Request::UpdateProperty {
                    prop_id: self.prop_id,
                    length: self.prop_data_length,
                    value: self.data,
                })
                .await;
        } else {
            self.config_request_sender
                .send(PropRequest::SetProperty {
                    prop_id: self.prop_id,
                    length: self.prop_data_length,
                    value: self.data,
                })
                .await;
        }
        self.data = [0u8; A3_MAX_PROP_DATA_SIZE];
    }

    async fn send_reply(&self, status: StreamStatus) {
        let data = [status as u8];
        let request = make_tx_std_request(self.wire_id, &data);
        self.can_req_sender.send(request).await;
    }

    async fn send_continue(&self) {
        let data = [];
        let request = make_tx_std_request(self.wire_id, &data);
        debug!("CONTINUE");
        self.can_req_sender.send(request).await;
    }
}

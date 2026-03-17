use defmt::{self, debug};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, SampleTime},
    exti::ExtiInput,
    gpio::{Input, Level, Output},
    peripherals::*,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{self, Channel},
    signal::Signal,
    watch::{self, Watch},
};
use embassy_time::{Duration, Instant, Timer};

use crate::envelope_generator::{
    EVENT_CHANNEL_SIZE, EgEvent, GateEventType, GateId, get_event_sender,
};

// communications ///////////////////////////////
static WATCH_READER: Watch<ThreadModeRawMutex, InputReaderInfo, 2> = Watch::new();
static CHANNEL_ADC: Channel<ThreadModeRawMutex, InputReaderRequest, 2> = Channel::new();
static SIGNAL_GATE_1_READING: Signal<ThreadModeRawMutex, u16> = Signal::new();
static SIGNAL_GATE_2_READING: Signal<ThreadModeRawMutex, u16> = Signal::new();

/// ADC resources
pub struct AdcResources {
    pub adc: Adc<'static, ADC1>,
    pub dma: Peri<'static, DMA1_CH1>,

    pub mux_addr_0: Output<'static>,
    pub mux_addr_1: Output<'static>,
    pub mux_addr_2: Output<'static>,

    pub pots: AnyAdcChannel<'static, ADC1>,

    pub gate_1: AnyAdcChannel<'static, ADC1>,
    pub gate_2: AnyAdcChannel<'static, ADC1>,

    pub cv_1: AnyAdcChannel<'static, ADC1>,
    pub cv_2: AnyAdcChannel<'static, ADC1>,
}

#[derive(Clone, Debug, defmt::Format)]
#[repr(usize)]
pub enum PotKind {
    Attack = 0,
    Decay = 1,
    Sustain = 2,
    Release = 3,
    Extra1 = 4,
    Extra2 = 5,
    CvADepth = 6,
    CvBDepth = 7,
}

#[derive(Clone)]
pub struct PotInfo {
    pub kind: PotKind,
    pub value: u16,
}

#[derive(Clone)]
pub struct CvInfo {
    pub cv_1: u16,
    pub cv_2: u16,
}

#[derive(Clone)]
pub struct InputReaderInfo {
    pub pot_info: PotInfo,
    pub cv_info: CvInfo,
}

pub enum InputReaderRequest {
    ReadGate { gate_id: GateId },
}

pub fn start(
    spawner: Spawner,
    resources: AdcResources,
    gate_src_sw_1: Input<'static>,
    gate_src_sw_2: Input<'static>,
    ind_analog_gate_1: Output<'static>,
    ind_analog_gate_2: Output<'static>,
    gate_trigger_1: ExtiInput<'static>,
    gate_trigger_2: ExtiInput<'static>,
) {
    let input_reader = InputReader::new(resources);
    spawner.spawn(run_input_reader(input_reader).unwrap());
    spawner.spawn(
        run_analog_gate(
            gate_src_sw_1,
            ind_analog_gate_1,
            gate_trigger_1,
            GateId::Gate1,
        )
        .unwrap(),
    );
    spawner.spawn(
        run_analog_gate(
            gate_src_sw_2,
            ind_analog_gate_2,
            gate_trigger_2,
            GateId::Gate2,
        )
        .unwrap(),
    );
}

#[inline]
async fn get_change_receiver<T>(
    watch: &'static Watch<ThreadModeRawMutex, T, 2>,
) -> watch::Receiver<'static, ThreadModeRawMutex, T, 2>
where
    T: Clone,
{
    let mut sleep_millis = 1;
    loop {
        if let Some(receiver) = watch.receiver() {
            return receiver;
        }
        // exponential back off
        Timer::after_millis(sleep_millis).await;
        sleep_millis *= 2;
    }
}

pub async fn get_reader_info_receiver()
-> watch::Receiver<'static, ThreadModeRawMutex, InputReaderInfo, 2> {
    get_change_receiver(&WATCH_READER).await
}

static MUX_ADDRESSES: [(Level, Level, Level, PotKind); 8] = [
    (Level::Low, Level::Low, Level::Low, PotKind::Attack),
    (Level::Low, Level::Low, Level::High, PotKind::Decay),
    (Level::Low, Level::High, Level::Low, PotKind::Sustain),
    (Level::Low, Level::High, Level::High, PotKind::Release),
    (Level::High, Level::Low, Level::Low, PotKind::Extra1),
    (Level::High, Level::Low, Level::High, PotKind::Extra2),
    (Level::High, Level::High, Level::Low, PotKind::CvADepth),
    (Level::High, Level::High, Level::High, PotKind::CvBDepth),
];

#[embassy_executor::task]
async fn run_input_reader(mut input_reader: InputReader) {
    input_reader.run().await;
}

struct InputReader {
    // Peripherals
    resources: AdcResources,

    // pot reading
    cv1_depth: u16,
    cv2_depth: u16,
    pot_index: usize,
}

impl InputReader {
    pub fn new(resources: AdcResources) -> Self {
        Self {
            resources,
            cv1_depth: 0,
            cv2_depth: 0,
            pot_index: 0,
        }
    }

    pub async fn run(&mut self) {
        let request_receiver = CHANNEL_ADC.receiver();
        let mut reader_info_sender = WATCH_READER.sender();
        let mut wakeup_time = Instant::now();
        let mut now = wakeup_time;
        loop {
            let sleep_time = if now.gt(&wakeup_time) {
                Duration::from_millis(0)
            } else {
                wakeup_time.duration_since(now)
            };
            match select(Timer::after(sleep_time), request_receiver.receive()).await {
                Either::First(()) => {}
                Either::Second(request) => match request {
                    InputReaderRequest::ReadGate { gate_id } => self.read_gate_level(gate_id).await,
                },
            }
            now = Instant::now();
            if now.ge(&wakeup_time) {
                self.regular_reading(&mut reader_info_sender).await;
                wakeup_time = wakeup_time.saturating_add(Duration::from_millis(10));
            }
        }
    }

    async fn regular_reading(
        &mut self,
        reader_info_sender: &mut watch::Sender<'_, ThreadModeRawMutex, InputReaderInfo, 2>,
    ) {
        let (pot_info, cv_info) = self.run_adc(self.pot_index).await;
        self.pot_index = (self.pot_index + 1) % MUX_ADDRESSES.len();
        match pot_info.kind {
            PotKind::CvADepth => {
                self.cv1_depth = pot_info.value;
            }
            PotKind::CvBDepth => {
                self.cv2_depth = pot_info.value;
            }
            _ => {
                // debug!("sending the change");
                // pot_change_sender.send(info);
            }
        }
        reader_info_sender.send(InputReaderInfo { pot_info, cv_info });
    }

    async fn run_adc(&mut self, index: usize) -> (PotInfo, CvInfo) {
        // Set up the pot multiplexer
        let (addr2, addr1, addr0, kind) = &MUX_ADDRESSES[index];
        self.resources.mux_addr_0.set_level(*addr0);
        self.resources.mux_addr_1.set_level(*addr1);
        self.resources.mux_addr_2.set_level(*addr2);

        // run ADC
        let mut buffer = [0u16; 3];
        let sequence = [
            (&mut self.resources.pots, SampleTime::CYCLES160_5),
            (&mut self.resources.cv_1, SampleTime::CYCLES160_5),
            (&mut self.resources.cv_2, SampleTime::CYCLES160_5),
        ]
        .into_iter();
        self.resources
            .adc
            .read(self.resources.dma.reborrow(), sequence, &mut buffer)
            .await;

        (
            PotInfo {
                kind: kind.clone(),
                value: buffer[2],
            },
            CvInfo {
                cv_1: buffer[0],
                cv_2: buffer[1],
            },
        )
    }

    async fn read_gate_level(&mut self, gate_id: GateId) {
        let mut buffer = [0u16; 1];
        let channel = match gate_id {
            GateId::Gate1 => &mut self.resources.gate_1,
            GateId::Gate2 => &mut self.resources.gate_2,
        };
        let sequence = [(channel, SampleTime::CYCLES160_5)].into_iter();
        self.resources
            .adc
            .read(self.resources.dma.reborrow(), sequence, &mut buffer)
            .await;
        match gate_id {
            GateId::Gate1 => SIGNAL_GATE_1_READING.signal(buffer[0]),
            GateId::Gate2 => SIGNAL_GATE_2_READING.signal(buffer[0]),
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn run_analog_gate(
    src_sw: Input<'static>,
    ind_analog_gate: Output<'static>,
    trigger: ExtiInput<'static>,
    gate_id: GateId,
) {
    let mut analog_gate = AnalogGate::new(src_sw, ind_analog_gate, trigger, gate_id);
    analog_gate.run().await;
}
struct AnalogGate {
    src_sw: Input<'static>,
    ind_analog_gate: Output<'static>,
    trigger: ExtiInput<'static>,

    gate_id: GateId,
    state: AnalogGateState,

    event_sender: channel::Sender<'static, ThreadModeRawMutex, EgEvent, EVENT_CHANNEL_SIZE>,
}

impl AnalogGate {
    pub fn new(
        src_sw: Input<'static>,
        ind_analog_gate: Output<'static>,
        trigger: ExtiInput<'static>,
        gate_id: GateId,
    ) -> Self {
        Self {
            src_sw,
            ind_analog_gate,
            trigger,
            gate_id,
            state: AnalogGateState::Disabled,
            event_sender: get_event_sender(),
        }
    }

    pub async fn run(&mut self) {
        loop {
            match self.state {
                AnalogGateState::Disabled => {
                    Timer::after_millis(10).await;
                    if self.src_sw.is_high() {
                        debug!(
                            "Analog gate {:?} enabled, state={:?}",
                            self.gate_id, self.state
                        );
                        self.state = AnalogGateState::GateOff;
                        self.ind_analog_gate.set_high();
                        self.event_sender
                            .send(EgEvent::GateEvent {
                                id: self.gate_id.clone(),
                                event: GateEventType::AnalogGateEnabled,
                            })
                            .await;
                    }
                }
                _ => self.run_analog_gate().await,
            }
        }
    }

    async fn run_analog_gate(&mut self) {
        let mut request_sender = CHANNEL_ADC.sender();
        loop {
            match self.state {
                AnalogGateState::Disabled => {
                    break;
                }
                AnalogGateState::GateOff => match select(
                    self.trigger.wait_for_falling_edge(),
                    Timer::after_millis(10),
                )
                .await
                {
                    Either::First(()) => self.handle_gate_on(&mut request_sender).await,
                    Either::Second(()) => self.check_gate_switch().await,
                },
                AnalogGateState::GateOn => {
                    match select(self.trigger.wait_for_rising_edge(), Timer::after_millis(10)).await
                    {
                        Either::First(()) => self.handle_gate_off().await,
                        Either::Second(()) => self.check_gate_switch().await,
                    }
                }
            }
        }
    }

    async fn handle_gate_on(
        &mut self,
        sender: &mut channel::Sender<'static, ThreadModeRawMutex, InputReaderRequest, 2>,
    ) {
        self.state = AnalogGateState::GateOn;
        Timer::after_micros(500).await;
        sender
            .send(InputReaderRequest::ReadGate {
                gate_id: self.gate_id.clone(),
            })
            .await;
        let level: u16 = match self.gate_id {
            GateId::Gate1 => SIGNAL_GATE_1_READING.wait().await,
            GateId::Gate2 => SIGNAL_GATE_2_READING.wait().await,
        };
        // TODO: Calibrate and convert properly
        let velocity = 0xffff - (level << 4);
        self.event_sender
            .send(EgEvent::GateEvent {
                id: self.gate_id.clone(),
                event: GateEventType::GateOn { velocity },
            })
            .await;
    }

    async fn handle_gate_off(&mut self) {
        self.state = AnalogGateState::GateOff;
        self.event_sender
            .send(EgEvent::GateEvent {
                id: self.gate_id.clone(),
                event: GateEventType::GateOff,
            })
            .await;
    }

    async fn check_gate_switch(&mut self) {
        // TODO: assert analog gate enabled
        if self.src_sw.is_low() {
            debug!(
                "Analog gate {:?} disabled, state={:?}",
                self.gate_id, self.state
            );
            if !matches!(self.state, AnalogGateState::GateOff) {
                self.event_sender
                    .send(EgEvent::GateEvent {
                        id: self.gate_id.clone(),
                        event: GateEventType::GateOff,
                    })
                    .await;
            }
            self.event_sender
                .send(EgEvent::GateEvent {
                    id: self.gate_id.clone(),
                    event: GateEventType::AnalogGateDisabled,
                })
                .await;
            self.ind_analog_gate.set_low();
            self.state = AnalogGateState::Disabled
        }
    }
}

#[derive(Debug, defmt::Format)]
enum AnalogGateState {
    Disabled,
    GateOff,
    GateOn,
}

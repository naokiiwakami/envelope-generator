use defmt::{self, debug};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::bind_interrupts;
use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, SampleTime},
    dma,
    exti::ExtiInput,
    gpio::{Input, Level, Output},
    mode::Async,
    peripherals::*,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{self, Channel},
    signal::Signal,
    watch::{self, Watch},
};
use embassy_time::Timer;

use crate::envelope_generator::{
    EG_CHANNEL_SIZE, EgRequest, GateEventType, GateId, get_eg_request_sender,
};

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
});

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

    pub cv_a: AnyAdcChannel<'static, ADC1>,
    pub cv_b: AnyAdcChannel<'static, ADC1>,
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
    pub cv_a: i16,
    pub cv_b: i16,
}

#[derive(Clone)]
pub struct InputReaderInfo {
    pub pot_info: PotInfo,
    pub cv_info: CvInfo,
}

impl InputReaderInfo {
    /// create a blank info
    pub fn new(pot_kind: PotKind) -> Self {
        Self {
            pot_info: PotInfo {
                kind: pot_kind,
                value: 0,
            },
            cv_info: CvInfo { cv_a: 0, cv_b: 0 },
        }
    }
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
    gate_trigger_1: ExtiInput<'static, Async>,
    gate_trigger_2: ExtiInput<'static, Async>,
) {
    let input_reader = InputReader::new(resources);
    spawner.spawn(run_input_reader(input_reader).unwrap());
    let analog_gate_1 = AnalogGate::new(
        gate_src_sw_1,
        ind_analog_gate_1,
        gate_trigger_1,
        GateId::Gate1,
    );
    let analog_gate_2 = AnalogGate::new(
        gate_src_sw_2,
        ind_analog_gate_2,
        gate_trigger_2,
        GateId::Gate2,
    );
    spawner.spawn(run_analog_gate(analog_gate_1).unwrap());
    spawner.spawn(run_analog_gate(analog_gate_2).unwrap());
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
    pot_index: usize,
}

impl InputReader {
    pub fn new(resources: AdcResources) -> Self {
        Self {
            resources,
            pot_index: 0,
        }
    }

    pub async fn run(&mut self) {
        let request_receiver = CHANNEL_ADC.receiver();
        let mut reader_info_sender = WATCH_READER.sender();
        loop {
            match select(Timer::after_millis(10), request_receiver.receive()).await {
                Either::First(()) => {}
                Either::Second(request) => match request {
                    InputReaderRequest::ReadGate { gate_id } => self.read_gate_level(gate_id).await,
                },
            }
            self.regular_reading(&mut reader_info_sender).await;
        }
    }

    async fn regular_reading(
        &mut self,
        reader_info_sender: &mut watch::Sender<'_, ThreadModeRawMutex, InputReaderInfo, 2>,
    ) {
        let (pot_info, cv_info) = self.run_adc(self.pot_index).await;
        self.pot_index = (self.pot_index + 1) % MUX_ADDRESSES.len();
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
            (&mut self.resources.cv_a, SampleTime::CYCLES160_5),
            (&mut self.resources.cv_b, SampleTime::CYCLES160_5),
        ]
        .into_iter();
        self.resources
            .adc
            .read(self.resources.dma.reborrow(), Irqs, sequence, &mut buffer)
            .await;

        // Pots pick up noise so their values do not drop to zero at the bottoms.
        // It causes noticable slight level at the edge of the configuration.
        // We subtract 4 from the original value to mitigate this problem.
        let pot_value = if buffer[2] >= 4 {
            (buffer[2] - 4) << 4
        } else {
            0
        };

        // TODO: Calibrate the zero points
        let cv_a = 32767 - (buffer[0] << 4) as i16;
        let cv_b = 32767 - (buffer[1] << 4) as i16;

        (
            PotInfo {
                kind: kind.clone(),
                value: pot_value,
            },
            CvInfo { cv_a, cv_b },
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
            .read(self.resources.dma.reborrow(), Irqs, sequence, &mut buffer)
            .await;
        match gate_id {
            GateId::Gate1 => SIGNAL_GATE_1_READING.signal(buffer[0]),
            GateId::Gate2 => SIGNAL_GATE_2_READING.signal(buffer[0]),
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn run_analog_gate(mut analog_gate: AnalogGate) {
    analog_gate.run().await;
}
struct AnalogGate {
    src_sw: Input<'static>,
    ind_analog_gate: Output<'static>,
    trigger: ExtiInput<'static, Async>,

    gate_id: GateId,
    state: AnalogGateState,

    request_sender: channel::Sender<'static, ThreadModeRawMutex, EgRequest, EG_CHANNEL_SIZE>,
}

impl AnalogGate {
    pub fn new(
        src_sw: Input<'static>,
        ind_analog_gate: Output<'static>,
        trigger: ExtiInput<'static, Async>,
        gate_id: GateId,
    ) -> Self {
        Self {
            src_sw,
            ind_analog_gate,
            trigger,
            gate_id,
            state: AnalogGateState::Disabled,
            request_sender: get_eg_request_sender(),
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
                        self.request_sender
                            .send(EgRequest::GateEvent {
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
        self.request_sender
            .send(EgRequest::GateEvent {
                id: self.gate_id.clone(),
                event: GateEventType::GateOn { velocity },
            })
            .await;
    }

    async fn handle_gate_off(&mut self) {
        self.state = AnalogGateState::GateOff;
        self.request_sender
            .send(EgRequest::GateEvent {
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
                self.request_sender
                    .send(EgRequest::GateEvent {
                        id: self.gate_id.clone(),
                        event: GateEventType::GateOff,
                    })
                    .await;
            }
            self.request_sender
                .send(EgRequest::GateEvent {
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

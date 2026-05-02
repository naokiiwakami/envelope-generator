#![no_std]
#![no_main]

mod addresses;
mod control_panel;
mod definitions;
mod envelope_generator;
mod input_reader;
mod patch_controller;
mod utils;

use analog3::{Analog3Config, definitions::*, rng, storage};
use core::future::pending;
use defmt::debug;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::can::{self, Can};
use embassy_stm32::dac::Dac;
use embassy_stm32::dma;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::flash::{self, Flash};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c, Master};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::pac;
use embassy_stm32::peripherals::{self, ADC1, TIM3};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::qei::{self, Qei};
use embassy_stm32::{Peripherals, bind_interrupts, interrupt};
use {defmt_rtt as _, panic_probe as _};

use crate::envelope_generator::{get_name, get_uid};
use crate::input_reader::AdcResources;

bind_interrupts!(struct CanIrqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

bind_interrupts!(struct FlashIrqs {
    FLASH => flash::InterruptHandler;
});

bind_interrupts!(struct I2cIrqs {
    I2C2_3 => i2c::EventInterruptHandler<peripherals::I2C2>, i2c::ErrorInterruptHandler<peripherals::I2C2>;
    DMA1_CH4_7_DMA2_CH1_5_DMAMUX1_OVR => dma::InterruptHandler<peripherals::DMA1_CH4>, dma::InterruptHandler<peripherals::DMA1_CH5>;
});
bind_interrupts!(struct ExtiIrqsGate1 {
    EXTI2_3 => exti::InterruptHandler<interrupt::typelevel::EXTI2_3>;
});

bind_interrupts!(struct ExtiIrqsGate2 {
    EXTI4_15 => exti::InterruptHandler<interrupt::typelevel::EXTI4_15>;
});

/*
bind_interrupts!(struct Tim6Irqs {
    TIM6_DAC_LPTIM1 => exti::InterruptHandler<interrupt::typelevel::TIM6_DAC_LPTIM1>;
});
*/

async fn init() -> Peripherals {
    let mut config = embassy_stm32::Config::default();

    // configure the system clock
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(16),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div1, // M = 1
        mul: PllMul::Mul8,       // N = 8
        divp: None,
        divq: None,
        divr: Some(PllRDiv::Div2), // R = 2 → 64 MHz
    });
    config.rcc.sys = Sysclk::Pll1R;

    config.rcc.ahb_pre = AHBPrescaler::Div1; // 64 MHz
    config.rcc.apb1_pre = APBPrescaler::Div1; // G0 allows 64 MHz APB

    embassy_stm32::init(config)

    // peripherals
}

struct EgResources {
    can: Can<'static>,
    can_stb: Output<'static>,
    ind_a3_red: Output<'static>,
    ind_a3_blue: Output<'static>,
    ind_gate_1: Output<'static>,
    ind_gate_2: Output<'static>,
    gate_src_sw_1: Input<'static>,
    gate_src_sw_2: Input<'static>,
    ind_analog_gate_1: Output<'static>,
    ind_analog_gate_2: Output<'static>,
    gate_trigger_1: ExtiInput<'static, Async>,
    gate_trigger_2: ExtiInput<'static, Async>,
    patch_button: Input<'static>,
    patch_ind_red: Output<'static>,
    patch_ind_green: Output<'static>,
    flash: Flash<'static>,
    encoder: Qei<'static, TIM3>,
    encoder_button: Input<'static>,
    encoder_ind_red: Output<'static>,
    encoder_ind_green: Output<'static>,
    i2c: I2c<'static, Async, Master>,
    dac_channels: Dac<'static, Blocking>,
    adc_resources: AdcResources,
}

async fn setup_peripherals(p: Peripherals) -> EgResources {
    // can standby controller pin
    let mut can_stb = Output::new(p.PC13, Level::High, Speed::Low);

    // disable the CAN transceiver
    can_stb.set_high();

    // A3 indicators
    let ind_a3_red = Output::new(p.PC15, Level::Low, Speed::Low);
    let ind_a3_blue = Output::new(p.PC14, Level::Low, Speed::Low);

    // gate status indicator
    let ind_gate_1 = Output::new(p.PD2, Level::Low, Speed::Low);
    let ind_gate_2 = Output::new(p.PD3, Level::Low, Speed::Low);

    // input source
    let ind_analog_gate_1 = Output::new(p.PD0, Level::Low, Speed::Low);
    let ind_analog_gate_2 = Output::new(p.PD1, Level::Low, Speed::Low);
    let gate_src_sw_1 = Input::new(p.PA8, Pull::None);
    let gate_src_sw_2 = Input::new(p.PA9, Pull::None);

    // physical gate trigger
    let gate_trigger_1 = ExtiInput::new(p.PB2, p.EXTI2, Pull::Up, ExtiIrqsGate1);
    let gate_trigger_2 = ExtiInput::new(p.PB10, p.EXTI10, Pull::Up, ExtiIrqsGate2);

    // user switch
    let patch_button = Input::new(p.PA15, Pull::Up);

    // patch indicator
    let patch_ind_red = Output::new(p.PB12, Level::Low, Speed::Low);
    let patch_ind_green = Output::new(p.PB11, Level::Low, Speed::Low);

    let flash = Flash::new(p.FLASH, FlashIrqs);

    // start the CAN controller
    let can = {
        let mut can_config = can::CanConfigurator::new(p.FDCAN1, p.PB8, p.PB9, CanIrqs);
        let nominal_bitrate = 1_000_000;
        let data_bitrate = 4_000_000;
        debug!(
            "Starting CAN FD with nominal bitrate {} mbps, data bitrate {} mbps",
            nominal_bitrate / 1_000_000,
            data_bitrate / 1_000_000
        );
        can_config.set_bitrate(nominal_bitrate);
        can_config.set_fd_data_bitrate(data_bitrate, true);
        let nbrp = u16::from(can_config.config().nbtr.prescaler);
        let nseg1 = u8::from(can_config.config().nbtr.seg1);
        let nseg2 = u8::from(can_config.config().nbtr.seg2);
        let nsjw = u8::from(can_config.config().nbtr.sync_jump_width);
        debug!(
            "nbrp={}, nseg1={}, nseg2={}, nsjw={}",
            nbrp, nseg1, nseg2, nsjw
        );
        let dbrp = u16::from(can_config.config().dbtr.prescaler);
        let dseg1 = u8::from(can_config.config().dbtr.seg1);
        let dseg2 = u8::from(can_config.config().dbtr.seg2);
        let dsjw = u8::from(can_config.config().dbtr.sync_jump_width);
        debug!(
            "dbrp={}, dseg1={}, dseg2={}, dsjw={}",
            dbrp, dseg1, dseg2, dsjw
        );
        can_config.into_normal_mode()
    };

    // I2C
    let i2c = {
        let mut i2c_config = i2c::Config::default();
        i2c_config.frequency = Hertz(1_000_000);

        let i2c_peri = p.I2C2;
        let scl = p.PB13;
        let sda = p.PB14;
        let tx_dma = p.DMA1_CH4;
        let rx_dma = p.DMA1_CH5;
        I2c::new(i2c_peri, scl, sda, tx_dma, rx_dma, I2cIrqs, i2c_config)
    };

    // Configure TIM2 for 40 kHz (25 us) using direct register access
    // System clock is 64 MHz
    // PSC = 31 → 2 MHz clock, ARR = 49 → 40 kHz update rate

    // Enable TIM2 clock in RCC (APB1 timer)
    pac::RCC.apbenr1().modify(|w| w.set_tim2en(true));
    cortex_m::asm::dsb(); // Synchronization barrier

    pac::TIM2.cr1().write(|w| w.set_cen(false)); // Disable first
    pac::TIM2.psc().write_value(31); // Prescaler = 32-1
    pac::TIM2.arr().write_value(49); // Autoreload = 50-1
    pac::TIM2.cr1().modify(|w| w.set_arpe(true)); // Enable autoreload preload
    pac::TIM2
        .cr2()
        .modify(|w| w.set_mms(embassy_stm32::pac::timer::vals::Mms::Update)); // Output update event as trigger
    pac::TIM2.dier().modify(|w| w.set_uie(true)); // Enable update interrupt
    pac::TIM2.sr().write(|w| w.set_uif(false)); // Clear update interrupt flag

    // Force loading of PSC and ARR by triggering update event
    pac::TIM2.egr().write(|w| w.set_ug(true));
    pac::TIM2.sr().write(|w| w.set_uif(false)); // Clear flag again after UG event

    pac::TIM2.cr1().modify(|w| w.set_cen(true)); // Enable counter
    debug!("TIM2 configured for 25 us interrupts with DAC trigger output");

    unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::TIM2) };

    // DAC
    let dac_channels = Dac::new_blocking(p.DAC1, p.PA4, p.PA5);
    // enable triggers
    pac::DAC1.cr().modify(|w| {
        w.set_ten(0, true);
        w.set_tsel(0, 0b0010); // dac_chx_trg2 -> TIM2 TRGO
        w.set_ten(1, true);
        w.set_tsel(1, 0b0010); // dac_chx_trg2 -> TIM2 TRGO
    });

    // ADC
    let adc = Adc::new(p.ADC1);
    let mux_addr_0 = Output::new(p.PA7, Level::Low, Speed::Medium);
    let mux_addr_1 = Output::new(p.PB0, Level::Low, Speed::Medium);
    let mux_addr_2 = Output::new(p.PB1, Level::Low, Speed::Medium);
    let dma = p.DMA1_CH1;
    let adc_resources = input_reader::AdcResources {
        adc,
        dma,

        mux_addr_0,
        mux_addr_1,
        mux_addr_2,

        pots: p.PA6,

        gate_1: p.PA3,
        gate_2: p.PA2,

        cv_a: p.PA0,
        cv_b: p.PA1,
    };

    let mut qei_config = qei::Config::default();
    qei_config.ch1_pull = Pull::Up;
    qei_config.ch2_pull = Pull::Up;
    let encoder = Qei::new(p.TIM3, p.PC6, p.PC7, qei_config);
    let encoder_button = Input::new(p.PA10, Pull::Up);
    let encoder_ind_red = Output::new(p.PA12, Level::Low, Speed::Low);
    let encoder_ind_green = Output::new(p.PA11, Level::Low, Speed::Low);

    EgResources {
        can,
        can_stb,
        ind_a3_red,
        ind_a3_blue,
        ind_gate_1,
        ind_gate_2,
        gate_src_sw_1,
        gate_src_sw_2,
        ind_analog_gate_1,
        ind_analog_gate_2,
        gate_trigger_1,
        gate_trigger_2,
        patch_button,
        patch_ind_green,
        patch_ind_red,
        flash,
        encoder,
        encoder_button,
        encoder_ind_red,
        encoder_ind_green,
        i2c,
        dac_channels,
        adc_resources,
    }
}

fn generate_rng_initial_values(adc: &mut Adc<'static, ADC1>) -> (u16, u16, u16) {
    let mut temp_channel = adc.enable_temperature();
    let raw1 = adc.blocking_read(&mut temp_channel, SampleTime::Cycles1605);
    let raw2 = adc.blocking_read(&mut temp_channel, SampleTime::Cycles1605);
    let raw3 = adc.blocking_read(&mut temp_channel, SampleTime::Cycles1605);

    debug!(
        "RNG seeding from temperature sensor: {=u16} {=u16} {=u16}",
        raw1, raw2, raw3
    );

    (raw1, raw2, raw3)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init().await;
    let mut eg_resources = setup_peripherals(p).await;

    // seed the PRNG from the internal temperature sensor
    let initial_values = generate_rng_initial_values(&mut eg_resources.adc_resources.adc);
    rng::init(initial_values);

    // start the modules
    storage::start(spawner, eg_resources.flash).await;

    input_reader::start(
        spawner,
        eg_resources.adc_resources,
        eg_resources.gate_src_sw_1,
        eg_resources.gate_src_sw_2,
        eg_resources.ind_analog_gate_1,
        eg_resources.ind_analog_gate_2,
        eg_resources.gate_trigger_1,
        eg_resources.gate_trigger_2,
    )
    .await;

    patch_controller::start(
        spawner,
        eg_resources.patch_button,
        eg_resources.patch_ind_red,
        eg_resources.patch_ind_green,
    );

    control_panel::start(
        spawner,
        eg_resources.i2c,
        eg_resources.encoder,
        eg_resources.encoder_button,
        eg_resources.encoder_ind_red,
        eg_resources.encoder_ind_green,
    )
    .await;

    envelope_generator::start(
        spawner,
        eg_resources.dac_channels,
        eg_resources.ind_gate_1,
        eg_resources.ind_gate_2,
    )
    .await;

    let uid = get_uid();
    let name = get_name().await;

    let a3_config = Analog3Config::new(MODULE_TYPE_HUMPS, uid, name.as_str());
    analog3::start(
        a3_config,
        eg_resources.can,
        eg_resources.ind_a3_red,
        eg_resources.ind_a3_blue,
        spawner,
    )
    .await;

    // all modules have started, enable the CAN transceiver
    eg_resources.can_stb.set_low();

    pending::<()>().await;
}

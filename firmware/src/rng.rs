#![allow(dead_code)]

//! PRNG seeded from the STM32G0 internal temperature sensor.
//!
//! Supports both a global generator and a fast local generator using a shared
//! state API.

use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};
use defmt::debug;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals::ADC1;

static RNG_STATE: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

/// A generic RNG state container used by global and local generators.
pub trait RngState {
    fn get_state(&self) -> u64;
    fn set_state(&self, state: u64);
}

fn splitmix64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E3779B97F4A7C15);
    let mut z = x;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// Generic random number generator that wraps an RngState implementation.
pub struct Rng<S: RngState> {
    generator: S,
}

impl<S: RngState> Rng<S> {
    pub fn random_u64(&self) -> u64 {
        self.next_state()
    }

    pub fn random_u32(&self) -> u32 {
        self.next_state() as u32
    }

    pub fn random_u16(&self) -> u16 {
        (self.next_state() > 16) as u16
    }

    pub fn random_u8(&self) -> u8 {
        (self.next_state() >> 24) as u8
    }

    pub fn random_range(&self, max: u32) -> u32 {
        if max == 0 {
            return 0;
        }
        self.random_u32() % max
    }

    pub fn random_f32(&self) -> f32 {
        (self.random_u32() as f32) / (u32::MAX as f32)
    }

    fn next_state(&self) -> u64 {
        let current = self.generator.get_state();
        let next = splitmix64(current.wrapping_add(0x9E3779B97F4A7C15));
        self.generator.set_state(next);
        next
    }
}

/// Global generator that uses the shared static RNG state.
pub struct GlobalGenerator;

impl RngState for GlobalGenerator {
    fn get_state(&self) -> u64 {
        interrupt::free(|cs| *RNG_STATE.borrow(cs).borrow())
    }

    fn set_state(&self, state: u64) {
        interrupt::free(|cs| *RNG_STATE.borrow(cs).borrow_mut() = state);
    }
}

pub type GlobalRng = Rng<GlobalGenerator>;

/// Singleton global RNG instance.
pub static RNG: GlobalRng = Rng {
    generator: GlobalGenerator,
};

/// Initialize the global PRNG from the ADC temperature sensor.
pub fn init_from_temperature(adc: &mut Adc<'static, ADC1>) {
    let mut temp_channel = adc.enable_temperature();
    let raw1 = adc.blocking_read(&mut temp_channel, SampleTime::CYCLES39_5);
    let raw2 = adc.blocking_read(&mut temp_channel, SampleTime::CYCLES39_5);
    let raw3 = adc.blocking_read(&mut temp_channel, SampleTime::CYCLES39_5);

    let seed =
        splitmix64(((raw1 as u64) << 32) | (raw2 as u64)).wrapping_add(splitmix64(raw3 as u64));

    RNG.generator.set_state(seed);
    debug!(
        "RNG seeded from temperature sensor: {=u16} {=u16} {=u16}",
        raw1, raw2, raw3
    );
}

/// A fast local generator owned by the caller.
pub struct LocalGenerator {
    state: RefCell<u64>,
}

impl LocalGenerator {
    pub fn new() -> Self {
        Self {
            state: RefCell::new(RNG.random_u64()),
        }
    }

    pub fn with_seed(seed: u64) -> Self {
        Self {
            state: RefCell::new(seed),
        }
    }
}

impl RngState for LocalGenerator {
    fn get_state(&self) -> u64 {
        *self.state.borrow()
    }

    fn set_state(&self, state: u64) {
        *self.state.borrow_mut() = state;
    }
}

pub type LocalRng = Rng<LocalGenerator>;

/// Create a new local RNG seeded from the global RNG.
pub fn local_rng() -> LocalRng {
    Rng {
        generator: LocalGenerator::new(),
    }
}

use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{self, Channel};
use embassy_time::Timer;

pub const INDICATOR_CHANNEL_SIZE: usize = 4;
static INDICATOR_CHANNEL: Channel<ThreadModeRawMutex, IndicatorRequest, INDICATOR_CHANNEL_SIZE> =
    Channel::new();

pub fn get_indicator_request_sender()
-> channel::Sender<'static, ThreadModeRawMutex, IndicatorRequest, 4> {
    INDICATOR_CHANNEL.sender()
}

#[allow(unused)]
pub enum IndicatorRequest {
    SetRedLed,
    ResetRedLed,
    BlinkRedLed { blinks: u8, interval: u16 },
    SetBlueLed,
    ResetBlueLed,
    BlinkBlueLed { blinks: u8, interval: u16 },
}

#[embassy_executor::task]
pub async fn run_indicator(mut a3_red_led: Output<'static>, mut a3_blue_led: Output<'static>) {
    let request_receiver = INDICATOR_CHANNEL.receiver();
    loop {
        let request = request_receiver.receive().await;
        match request {
            IndicatorRequest::SetRedLed => a3_red_led.set_high(),
            IndicatorRequest::ResetRedLed => a3_red_led.set_low(),
            IndicatorRequest::SetBlueLed => a3_blue_led.set_high(),
            IndicatorRequest::ResetBlueLed => a3_blue_led.set_low(),
            IndicatorRequest::BlinkBlueLed { blinks, interval } => {
                blink_led(&mut a3_blue_led, blinks, interval).await
            }
            IndicatorRequest::BlinkRedLed { blinks, interval } => {
                blink_led(&mut a3_red_led, blinks, interval).await
            }
        }
    }
}

async fn blink_led(led: &mut Output<'static>, blinks: u8, interval: u16) {
    for _ in 0..(blinks * 2) {
        Timer::after_millis(interval as u64).await;
        led.toggle();
    }
}

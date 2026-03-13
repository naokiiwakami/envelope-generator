use embassy_executor::Spawner;
use embassy_stm32::{mode, usart::Uart};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use heapless::String;

static CHANNEL_SERIAL: Channel<ThreadModeRawMutex, String<128>, 2> = Channel::new();

pub fn start(spawner: Spawner, uart: Uart<'static, mode::Async>) {
    spawner.spawn(run_serial(uart).unwrap());
}

pub async fn print_text(message: String<128>) {
    let sender = CHANNEL_SERIAL.sender();
    sender.send(message).await;
}

#[embassy_executor::task]
async fn run_serial(mut uart: Uart<'static, mode::Async>) {
    uart.write(b"\r\n******************************\r\n")
        .await
        .unwrap();
    uart.write(b"  Analog3 Envelope Generator\r\n")
        .await
        .unwrap();
    uart.write(b"******************************\r\n\r\n")
        .await
        .unwrap();

    let receiver = CHANNEL_SERIAL.receiver();
    loop {
        let message = receiver.receive().await;
        uart.write(message.as_bytes()).await.unwrap();
    }
}

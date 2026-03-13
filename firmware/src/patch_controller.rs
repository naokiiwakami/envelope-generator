use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Output};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::Channel,
    mutex::Mutex,
    watch::{self, Watch},
};
use embassy_time::Timer;

static CHANNEL_PATCH_CONTROLLER: Channel<ThreadModeRawMutex, PatchControllerRequest, 2> =
    Channel::new();
static WATCH_PATCH_CONTROLLER: Watch<ThreadModeRawMutex, PatchControllerReply, 2> = Watch::new();
static NEXT_STREAM_ID: Mutex<ThreadModeRawMutex, u32> = Mutex::new(0);

pub fn start(
    spawner: Spawner,
    button: Input<'static>,
    ind_red: Output<'static>,
    ind_green: Output<'static>,
) {
    let patch_controller = PatchController::new(button, ind_red, ind_green);
    spawner.spawn(run_patch_controller(patch_controller).unwrap());
}

pub async fn diagnose_leds() {
    let stream_id = PatchController::get_next_stream_id().await;
    let request_sender = CHANNEL_PATCH_CONTROLLER.sender();
    request_sender
        .send(PatchControllerRequest::DiagnoseLeds { stream_id })
        .await;
    PatchController::receive_reply(stream_id).await;
}

pub async fn diagnose_button() {
    let stream_id = PatchController::get_next_stream_id().await;
    let request_sender = CHANNEL_PATCH_CONTROLLER.sender();
    request_sender
        .send(PatchControllerRequest::DiagnoseButton { stream_id })
        .await;
    PatchController::receive_reply(stream_id).await;
}

pub enum PatchControllerRequest {
    DiagnoseLeds { stream_id: u32 },
    DiagnoseButton { stream_id: u32 },
}

#[derive(Clone)]
struct PatchControllerReply {
    stream_id: u32,
    // result: Result<Value, Error>,
}

#[embassy_executor::task]
async fn run_patch_controller(mut patch_controller: PatchController) {
    patch_controller.run().await;
}

struct PatchController {
    button: Input<'static>,
    ind_red: Output<'static>,
    ind_green: Output<'static>,
}

impl PatchController {
    pub fn new(
        button: Input<'static>,
        ind_red: Output<'static>,
        ind_green: Output<'static>,
    ) -> Self {
        Self {
            button,
            ind_red,
            ind_green,
        }
    }

    pub async fn run(&mut self) {
        // self.blink_leds().await;
        let request_receiver = CHANNEL_PATCH_CONTROLLER.receiver();
        let reply_sender = WATCH_PATCH_CONTROLLER.sender();
        loop {
            let request = request_receiver.receive().await;
            match request {
                PatchControllerRequest::DiagnoseLeds { stream_id } => {
                    self.diagnose_leds().await;
                    reply_sender.send(PatchControllerReply { stream_id });
                }
                PatchControllerRequest::DiagnoseButton { stream_id } => {
                    self.diagnose_button().await;
                    reply_sender.send(PatchControllerReply { stream_id });
                }
            };
        }
    }

    async fn blink_leds(&mut self) {
        for _ in 0..24 {
            Timer::after_millis(100).await;
            self.ind_red.toggle();
        }
        Timer::after_millis(500).await;
        for _ in 0..24 {
            Timer::after_millis(100).await;
            self.ind_green.toggle();
        }
    }

    async fn diagnose_leds(&mut self) {
        self.blink_leds().await;
    }

    async fn diagnose_button(&mut self) {
        let mut prev_button_pressed = self.button.is_low();
        let mut count = 0u32;
        loop {
            if count % 50 == 0 {
                self.ind_red.toggle();
                self.ind_green.toggle();
            }
            count += 1;
            let button_pressed = self.button.is_low();
            if button_pressed && !prev_button_pressed {
                self.ind_red.set_low();
                self.ind_green.set_low();
                return;
            }
            prev_button_pressed = button_pressed;
            Timer::after_millis(10).await;
        }
    }

    async fn get_next_stream_id() -> u32 {
        let mut guard = NEXT_STREAM_ID.lock().await;
        let current = *guard;
        *guard += 1;
        current
    }

    async fn get_reply_receiver()
    -> watch::Receiver<'static, ThreadModeRawMutex, PatchControllerReply, 2> {
        let mut sleep_millis = 1;
        loop {
            if let Some(receiver) = WATCH_PATCH_CONTROLLER.receiver() {
                return receiver;
            }
            // exponential back off
            Timer::after_millis(sleep_millis).await;
            sleep_millis *= 2;
        }
    }

    pub async fn receive_reply(stream_id: u32) {
        let mut receiver = Self::get_reply_receiver().await;
        loop {
            let reply = receiver.changed().await;
            if reply.stream_id == stream_id {
                return;
            }
        }
    }
}

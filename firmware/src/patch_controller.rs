use analog3::rng::local_rng;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Output};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel, signal::Signal};
use embassy_time::Timer;

static CHANNEL_PATCH_CONTROLLER: Channel<ThreadModeRawMutex, PatchControllerRequest, 2> =
    Channel::new();

pub fn start(
    spawner: Spawner,
    button: Input<'static>,
    ind_red: Output<'static>,
    ind_green: Output<'static>,
) {
    let patch_controller = PatchController::new(button, ind_red, ind_green);
    spawner.spawn(run_patch_controller(patch_controller).unwrap());
}

pub async fn diagnose_leds(reply: &'static Signal<ThreadModeRawMutex, ()>) {
    let request_sender = CHANNEL_PATCH_CONTROLLER.sender();
    request_sender
        .send(PatchControllerRequest::DiagnoseLeds { reply })
        .await;
    reply.wait().await;
}

pub async fn diagnose_button(reply: &'static Signal<ThreadModeRawMutex, ()>) {
    let request_sender = CHANNEL_PATCH_CONTROLLER.sender();
    request_sender
        .send(PatchControllerRequest::DiagnoseButton { reply })
        .await;
    reply.wait().await;
}

pub enum PatchControllerRequest {
    DiagnoseLeds {
        reply: &'static Signal<ThreadModeRawMutex, ()>,
    },
    DiagnoseButton {
        reply: &'static Signal<ThreadModeRawMutex, ()>,
    },
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

async fn random_blink(repeat: usize, leds: &mut [&mut Output<'static>]) {
    let rng = local_rng();
    for _ in 0..repeat {
        let random = rng.random_u64();
        Timer::after_millis(100 + random % 256).await;
        leds[random as usize % leds.len()].set_high();
        Timer::after_millis(30).await;
        leds[random as usize % leds.len()].set_low();
    }
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
        let request_receiver = CHANNEL_PATCH_CONTROLLER.receiver();
        {
            let mut leds = [&mut self.ind_red, &mut self.ind_green];
            random_blink(5, &mut leds).await;
        }
        loop {
            let request = request_receiver.receive().await;
            match request {
                PatchControllerRequest::DiagnoseLeds { reply } => {
                    self.diagnose_leds().await;
                    reply.signal(());
                }
                PatchControllerRequest::DiagnoseButton { reply } => {
                    self.diagnose_button().await;
                    reply.signal(());
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
}

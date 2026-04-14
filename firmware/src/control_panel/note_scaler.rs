use defmt;
use embassy_stm32::gpio::Level;
use embassy_time::{Instant, Timer};
use heapless::Vec;

use crate::{
    definitions::{CvKind, PotKind},
    envelope_generator::{EgRequest, EngineType},
};

use super::{ControlPanel, DisplayRequest};

pub struct NoteScaler<'a> {
    control_panel: &'a mut ControlPanel,
    current_depth: u16,
}

impl<'a> NoteScaler<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        let current_depth = control_panel.eg_config.note_scaling_depth(0);
        Self {
            control_panel,
            current_depth,
        }
    }

    pub async fn execute(&mut self) {
        defmt::debug!("NoteScaler.execute()");
        self.control_panel.smash_counter();
        self.control_panel.ind_red.set_high();
        self.control_panel.ind_green.set_high();

        defmt::debug!("sending UpdateNoteScaling");
        self.control_panel
            .display_request_sender
            .send(DisplayRequest::UpdateNoteScaling {
                depth: self.current_depth,
            })
            .await;

        let mut last = self.control_panel.encoder_last_raw;
        loop {
            Timer::after_millis(10).await;
        }

        self.control_panel.ind_red.set_low();
        self.control_panel.ind_green.set_low();
    }
}

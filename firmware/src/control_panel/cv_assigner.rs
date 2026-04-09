use defmt;
use embassy_stm32::gpio::Level;
use embassy_time::{Instant, Timer};
use heapless::Vec;

use crate::{
    definitions::{CvKind, PotKind},
    envelope_generator::{EgRequest, EngineType},
};

use super::{ControlPanel, DisplayRequest};

pub struct CvAssigner<'a> {
    control_panel: &'a mut ControlPanel,
    current_cv_a_destination: PotKind,
    current_cv_b_destination: PotKind,
}

impl<'a> CvAssigner<'a> {
    pub fn new(control_panel: &'a mut ControlPanel) -> Self {
        let current_cv_a_destination = control_panel.eg_config.cv_a_destination();
        let current_cv_b_destination = control_panel.eg_config.cv_b_destination();
        Self {
            control_panel,
            current_cv_a_destination,
            current_cv_b_destination,
        }
    }

    pub async fn execute(&mut self) {
        defmt::debug!("CvAssigner.execute()");
        self.assign_cv(CvKind::A).await;
        self.assign_cv(CvKind::B).await;
        self.control_panel.ind_red.set_low();
        self.control_panel.ind_green.set_low();
    }

    async fn assign_cv(&mut self, cv_kind: CvKind) {
        self.control_panel.smash_counter();
        let (current_destination, skip) = match cv_kind {
            CvKind::A => (self.current_cv_a_destination, self.current_cv_b_destination),
            CvKind::B => (self.current_cv_b_destination, self.current_cv_a_destination),
        };

        let mut candidates: Vec<PotKind, 5> = Vec::new();
        self.build_candidates(skip, &mut candidates);
        let original_index = match candidates
            .iter()
            .position(|&dest| dest == current_destination)
        {
            Some(idx) => idx,
            None => 0,
        };
        defmt::debug!(
            "index={}, current={}, candidates size={}",
            original_index,
            current_destination,
            candidates.len()
        );

        let mut current_index = original_index;
        loop {
            Timer::after_millis(10).await;
            if self.control_panel.button.get_level() == Level::Low {
                if self.control_panel.button_pressed_at.is_none() {
                    self.control_panel.button_pressed_at = Some(Instant::now());
                    // handle button pressed
                }
            } else if self.control_panel.button_pressed_at.is_some() {
                self.control_panel.button_pressed_at = None;
                // handle button released
                let new_destination = candidates[current_index];
                self.control_panel
                    .eg_request_sender
                    .send(EgRequest::ChangeCvDestination {
                        source: cv_kind,
                        destination: new_destination,
                    })
                    .await;
                match cv_kind {
                    CvKind::A => self.current_cv_a_destination = new_destination,
                    CvKind::B => self.current_cv_b_destination = new_destination,
                };
                return;
            } else {
                let raw = self.control_panel.encoder.count() as i16;
                let delta = (raw - self.control_panel.encoder_last_raw) / 4;
                let mut next_index: i32 =
                    (original_index as i32 + delta as i32) % candidates.len() as i32;
                if next_index < 0 {
                    next_index += candidates.len() as i32;
                }
                if next_index as usize == current_index {
                    continue;
                }
                current_index = next_index as usize;
                let next_destination = candidates[current_index];
                defmt::debug!("next dest: {:?}, delta: {}", next_destination, delta);
                self.control_panel
                    .display_request_sender
                    .send(DisplayRequest::UpdateCvAssignment {
                        source: cv_kind,
                        destination: next_destination,
                    })
                    .await;
            }
        }
    }

    fn build_candidates(&self, skip: PotKind, candidates: &mut Vec<PotKind, 5>) {
        let possible_pots: &[PotKind] = match self.control_panel.eg_config.engine_type(0) {
            EngineType::ParaDecays => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
                PotKind::Extra2,
                PotKind::Extra1,
            ],
            EngineType::TwoDecays => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
                PotKind::Extra2,
                PotKind::Extra1,
            ],
            EngineType::Adsr => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
            ],
            EngineType::Linear => &[
                PotKind::Attack,
                PotKind::Decay,
                PotKind::Sustain,
                PotKind::Release,
            ],
        };
        for pot in possible_pots {
            if *pot != skip {
                candidates.push(*pot).unwrap();
            }
        }
    }
}

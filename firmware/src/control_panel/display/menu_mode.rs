use core::cmp::min;

use defmt::{debug, error};
use embedded_graphics::{pixelcolor::BinaryColor, primitives::PrimitiveStyleBuilder};
use ssd1306_lite::TextBox;

use crate::control_panel::menu::MenuItem;

use super::{Display, Request};

pub struct MenuMode<'a, ActionT> {
    display: &'a mut Display,

    title: &'a str,
    menu_items: &'a [MenuItem<ActionT>],
    top_line: usize,
    current_item: usize,
}

impl<'a, ActionT> MenuMode<'a, ActionT> {
    const NUM_LINES: usize = 3; // three lines fit in the screen
    const LINE_HEIGHT: i32 = 21;
    const MARGIN_TOP: i32 = 0;
    const INDENT: i32 = 4;

    pub fn new(
        display: &'a mut Display,
        title: &'a str,
        menu_items: &'static [MenuItem<ActionT>],
    ) -> Self {
        Self {
            display,
            title,
            menu_items,
            top_line: 0,
            current_item: usize::MAX,
        }
    }

    pub async fn run(&mut self) {
        debug!("Running menu mode, menu={}", self.title);
        loop {
            // while self.display.mode.is_menu_mode() {
            let request = self.display.fetch_request().await;
            debug!("[MenuMode]: request: {}", request.name());
            match request {
                Request::SwitchMode { .. } => {
                    self.display.handle_generic_request(request).await;
                    break;
                }
                Request::Clear { .. }
                | Request::Flush
                | Request::DrawLine { .. }
                | Request::DrawCircle { .. }
                | Request::DrawRectangle { .. }
                | Request::DrawTriangle { .. }
                | Request::DrawArc { .. }
                | Request::DisplayText { .. } => self.display.handle_generic_request(request).await,
                Request::DisplayEngineTypeMenuItem { index }
                | Request::DisplayAdminMenuItem { index } => {
                    if request.mode() != self.display.mode {
                        self.display
                            .switch_mode(request.mode(), Some(request))
                            .await;
                        break;
                    }
                    debug!("display current menu, index={}", index);
                    self.display_current_menu(index).await
                }
                _ => {
                    self.display
                        .switch_mode(request.mode(), Some(request))
                        .await;
                    break;
                }
            }
        }
    }

    async fn display_current_menu(&mut self, index: usize) {
        if index >= self.menu_items.len() {
            error!("display_current_menu: Index out of bounds; index={}", index);
            return;
        }
        debug!(
            "displaying {}, index: {}, current: {}",
            self.title, index, self.current_item
        );
        if index == self.current_item {
            // do nothing
            return;
        }
        if self.current_item < usize::MAX
            && index >= self.top_line
            && index < self.top_line + Self::NUM_LINES
        {
            // no need to scroll, just move the cursor
            let ypos = (self.current_item - self.top_line) as i32 * Self::LINE_HEIGHT;
            self.clear_line(false, ypos).await;
            self.print_menu_item(false, self.current_item, ypos).await;

            let ypos = (index - self.top_line) as i32 * Self::LINE_HEIGHT;
            self.clear_line(true, ypos).await;
            self.print_menu_item(true, index, ypos).await;
            self.current_item = index;
        } else {
            self.current_item = index;
            if index < self.top_line {
                // scroll up
                self.top_line = index;
            } else if index >= self.top_line + Self::NUM_LINES {
                // scroll down
                self.top_line = index + 1 - Self::NUM_LINES;
            }
            self.show_menu().await;
        }
        self.display.driver.flush().await;
    }

    /// Build entire menu page.
    /// This method does not check line boundaries assuming the caller takes care of it.
    async fn show_menu(&mut self) {
        self.display.clear(false, false).await;
        let tail = min(Self::NUM_LINES, self.menu_items.len() - self.top_line);

        for line in 0..tail {
            let ypos = line as i32 * Self::LINE_HEIGHT + Self::MARGIN_TOP;
            let index = line + self.top_line;
            let is_selected = index == self.current_item;
            if is_selected {
                self.clear_line(true, ypos).await;
            }
            self.print_menu_item(is_selected, index, ypos).await;
        }
    }

    async fn print_menu_item(&mut self, reverse: bool, index: usize, ypos: i32) {
        let color = if reverse {
            BinaryColor::Off
        } else {
            BinaryColor::On
        };
        self.display
            .display_text(
                self.menu_items[index].name,
                TextBox::simple(
                    Self::INDENT as usize,
                    (ypos + Self::MARGIN_TOP) as usize,
                    color,
                ),
                super::FontSize::Large,
                false,
            )
            .await;
    }

    async fn clear_line(&mut self, reverse: bool, ypos: i32) {
        self.display
            .driver
            .draw_rectangle(
                (0, ypos),
                128,
                Self::LINE_HEIGHT as u32,
                PrimitiveStyleBuilder::new()
                    .fill_color(if reverse {
                        BinaryColor::On
                    } else {
                        BinaryColor::Off
                    })
                    .build(),
            )
            .await;
    }
}

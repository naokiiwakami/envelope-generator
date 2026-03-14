use core::cmp::min;

use defmt::{debug, error};
use embassy_futures::yield_now;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};

use crate::control_panel::menu::MenuItem;

use super::{EgDisplay, Mode, Request};

pub struct MenuMode<'a, ActionT> {
    display: &'a mut EgDisplay,

    title: &'a str,
    menu_items: &'a [MenuItem<ActionT>],
    top_line: usize,
    current_item: usize,
}

impl<'a, ActionT> MenuMode<'a, ActionT> {
    const NUM_LINES: usize = 3; // three lines fit in the screen
    const LINE_HEIGHT: i32 = 16;
    const MARGIN_TOP: i32 = 1;
    const INDENT: i32 = 16;

    pub fn new(
        display: &'a mut EgDisplay,
        title: &'a str,
        menu_items: &'static [MenuItem<ActionT>],
    ) -> Self {
        Self {
            display,
            title,
            menu_items,
            top_line: 0,
            current_item: 0,
        }
    }

    pub async fn run(&mut self) {
        debug!("Running menu mode, menu={}", self.title);
        Rectangle::new(Point::zero(), Size::new(128, 16))
            .into_styled(
                PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .build(),
            )
            .draw(&mut self.display.display)
            .unwrap();
        yield_now().await;
        self.display
            .display_text(
                true,
                false,
                self.title,
                1,
                Point::new(Self::INDENT, Self::MARGIN_TOP),
            )
            .await;

        self.show_menu().await;
        self.display.display.flush().await.unwrap();
        while self.display.mode.is_menu_mode() {
            let request = self.display.fetch_request().await;
            match request {
                Request::Clear { .. } | Request::Flush => {
                    self.display.handle_generic_request(request).await
                }
                Request::DisplayOpMenuItem { index } | Request::DisplayAdminMenuItem { index } => {
                    debug!("display current menu, index={}", index);
                    self.display_current_menu(index).await
                }
                _ => self.display.switch_mode(request).await,
            }
        }
    }

    async fn display_current_menu(&mut self, index: usize) {
        if index >= self.menu_items.len() {
            error!("display_current_menu: Index out of bounds; index={}", index);
            return;
        }
        debug!("displaying {}, index: {}", self.title, index);
        if index == self.current_item {
            // do nothing
            return;
        }
        if index >= self.top_line && index < self.top_line + Self::NUM_LINES {
            // no need to scroll, just move the cursor
            self.cursor(self.current_item - self.top_line, true).await;
            self.cursor(index - self.top_line, false).await;
        } else {
            self.display.clear(false, false).await;
            if index < self.top_line {
                // scroll up
                self.top_line = index;
            } else {
                // scroll down
                self.top_line = index + 1 - Self::NUM_LINES;
            }
            self.show_menu().await;
        }
        self.current_item = index;
        self.display.display.flush().await.unwrap();
    }

    /// Build entire menu page.
    /// This method does not check line boundaries assuming the caller takes care of it.
    async fn show_menu(&mut self) {
        let tail = min(3, self.menu_items.len());

        for line in 0..tail {
            let ypos = (line as i32 + 1) * Self::LINE_HEIGHT + Self::MARGIN_TOP;
            let point = Point::new(20, ypos);
            let index = line + self.top_line;
            self.display
                .display_text(false, false, self.menu_items[index].name, 1, point)
                .await;
            if index == self.current_item {
                let point = Point::new(0, ypos);
                self.display.display_text(false, false, ">", 1, point).await;
            }
        }
    }

    /// Update cursor in the specified line.
    async fn cursor(&mut self, line: usize, clear: bool) {
        let ypos = (line as i32 + 1) * Self::LINE_HEIGHT + Self::MARGIN_TOP;
        let position = Point::new(0, ypos);
        if clear {
            Rectangle::new(position, Size::new(20, 16))
                .into_styled(
                    PrimitiveStyleBuilder::new()
                        .fill_color(BinaryColor::Off)
                        .build(),
                )
                .draw(&mut self.display.display)
                .unwrap();
        } else {
            self.display
                .display_text(false, false, ">", 1, position)
                .await;
        }
    }
}

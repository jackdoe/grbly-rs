use std::sync::Arc;

use parking_lot::Mutex;
use three_d::egui;

use crate::grbl::engine::Engine;

const DIM: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);
const GREEN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xff, 0x88);
const RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x44, 0x44);
const CMD: egui::Color32 = egui::Color32::from_rgb(0xff, 0xdd, 0xaa);
const RESP: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);

pub struct ConsoleState {
    pub input: String,
}

impl Default for ConsoleState {
    fn default() -> Self {
        Self { input: String::new() }
    }
}

pub struct LogBuffer {
    lines: Vec<String>,
    last_line: String,
    last_rep: usize,
}

impl LogBuffer {
    pub fn new() -> Self {
        Self { lines: Vec::new(), last_line: String::new(), last_rep: 0 }
    }

    pub fn add(&mut self, line: String) {
        if line == self.last_line && !self.lines.is_empty() {
            self.last_rep += 1;
            let n = self.lines.len();
            self.lines[n - 1] = format!("{} (x{})", line, self.last_rep);
            return;
        }
        self.last_line = line.clone();
        self.last_rep = 1;
        self.lines.push(line);
        if self.lines.len() > 1000 {
            let drain = self.lines.len() - 1000;
            self.lines.drain(..drain);
        }
    }

    pub fn lines(&self) -> &[String] {
        &self.lines
    }
}

pub fn draw(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    log: &Arc<Mutex<LogBuffer>>,
    state: &mut ConsoleState,
) {
    let log_lines: Vec<String> = log.lock().lines().to_vec();

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("CONSOLE").size(14.0).color(DIM).strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.add(egui::Button::new(egui::RichText::new("OPEN LOG").size(12.0)).min_size(egui::vec2(70.0, 24.0))).clicked() {
                let path = "/tmp/grbl.txt";
                let _ = std::process::Command::new("xdg-open").arg(path).spawn();
            }
        });
    });

    let available = ui.available_height() - 32.0;
    ui.allocate_ui(egui::vec2(ui.available_width(), available.max(40.0)), |ui| {
        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .stick_to_bottom(true)
            .show(ui, |ui| {
                if log_lines.is_empty() {
                    ui.label(egui::RichText::new("--- NO LOG ---").size(12.0).color(egui::Color32::from_rgb(0x33, 0x2a, 0x11)));
                }
                for line in &log_lines {
                    ui.label(egui::RichText::new(line).size(12.0).color(line_color(line)));
                }
            });
    });

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(">").size(14.0).color(DIM));
        let response = ui.add(
            egui::TextEdit::singleline(&mut state.input)
                .desired_width(ui.available_width())
                .font(egui::TextStyle::Monospace),
        );
        if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            let text = state.input.trim().to_string();
            if !text.is_empty() {
                engine.send(&text);
                state.input.clear();
            }
            response.request_focus();
        }
    });
}

fn line_color(line: &str) -> egui::Color32 {
    if line.starts_with("> ") {
        CMD
    } else if line.starts_with("ok") {
        GREEN
    } else if line.starts_with("error") || line.starts_with("ALARM") {
        RED
    } else {
        RESP
    }
}

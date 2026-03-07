use std::sync::Arc;
use three_d::egui;
use crate::grbl::engine::Engine;
use crate::grbl::serial;
use crate::grbl::state::*;

const AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0xaa, 0x00);
const DIM: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);
const GREEN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xff, 0x88);
const RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x44, 0x44);
const WHITE: egui::Color32 = egui::Color32::from_rgb(0xff, 0xdd, 0xaa);
const BTN_BG: egui::Color32 = egui::Color32::from_rgb(0x22, 0x22, 0x33);

pub struct ControlsState {
    pub port_list: Vec<String>,
    pub port_index: usize,
    pub jog_step: f32,
}

impl Default for ControlsState {
    fn default() -> Self {
        Self { port_list: Vec::new(), port_index: 0, jog_step: 1.0 }
    }
}

fn section(ui: &mut egui::Ui, label: &str) {
    ui.add_space(4.0);
    ui.label(egui::RichText::new(label.to_uppercase()).size(14.0).color(DIM).strong());
    ui.add_space(2.0);
}

fn wide_btn(text: &str) -> egui::Button<'_> {
    egui::Button::new(egui::RichText::new(text).size(14.0))
        .min_size(egui::vec2(0.0, 28.0))
}

fn wide_btn_colored(text: &str, fill: egui::Color32) -> egui::Button<'_> {
    wide_btn(text).fill(fill)
}

pub fn draw(
    ctx: &egui::Context,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    ui_state: &mut ControlsState,
) {
    egui::SidePanel::left("controls")
        .default_width(280.0)
        .resizable(false)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                connection_section(ui, engine, mstate, ui_state);
                ui.separator();
                status_section(ui, engine, mstate);
                ui.separator();
                jog_section(ui, engine, ui_state);
                ui.separator();
                overrides_section(ui, engine, mstate);
                ui.separator();
                actions_section(ui, engine);
                ui.separator();
                job_section(ui, engine, jstate);
            });
        });
}

fn connection_section(ui: &mut egui::Ui, engine: &Arc<Engine>, mstate: &MachineState, state: &mut ControlsState) {
    section(ui, "Connection");
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("PORT").size(12.0).color(DIM));
        let name = state.port_list.get(state.port_index).cloned().unwrap_or_else(|| "---".into());
        egui::ComboBox::from_id_salt("port_combo")
            .selected_text(egui::RichText::new(&name).size(13.0))
            .width(ui.available_width() - 4.0)
            .show_ui(ui, |ui| {
                for (i, p) in state.port_list.iter().enumerate() {
                    ui.selectable_value(&mut state.port_index, i, p);
                }
            });
    });
    ui.columns(2, |cols| {
        if cols[0].add_sized([cols[0].available_width(), 28.0], wide_btn("REFRESH")).clicked() {
            state.port_list = serial::list_ports();
            state.port_index = 0;
        }
        if mstate.connected {
            if cols[1].add_sized([cols[1].available_width(), 28.0], wide_btn_colored("DISCONNECT", RED)).clicked() {
                engine.disconnect();
            }
        } else if cols[1].add_sized([cols[1].available_width(), 28.0], wide_btn_colored("CONNECT", egui::Color32::from_rgb(0x00, 0x66, 0x33))).clicked() {
            if let Some(port) = state.port_list.get(state.port_index) {
                let _ = engine.connect(port, 115200);
            }
        }
    });
    let (color, text) = if mstate.connected {
        (GREEN, format!(">> {}", mstate.port))
    } else {
        (RED, ">> DISCONNECTED".into())
    };
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(8.0, 8.0), egui::Sense::hover());
        ui.painter().circle_filled(rect.center(), 4.0, color);
        ui.label(egui::RichText::new(text).color(color).size(12.0));
    });
}

fn status_section(ui: &mut egui::Ui, engine: &Arc<Engine>, mstate: &MachineState) {
    section(ui, "Status");
    let (color, text) = status_display(mstate.status);
    ui.label(egui::RichText::new(format!("[ {} ]", text)).size(20.0).color(color).strong());

    ui.add_space(4.0);
    ui.label(egui::RichText::new("WORK").size(11.0).color(DIM));
    position_row(ui, mstate.wpos, 18.0);

    ui.add_space(2.0);
    ui.label(egui::RichText::new("MACHINE").size(11.0).color(DIM));
    position_row(ui, mstate.mpos, 13.0);

    ui.add_space(4.0);
    let env = CUBIKO.envelope;
    ui.label(egui::RichText::new(format!("ENVELOPE  X:{:.0}  Y:{:.0}  Z:{:.0}", env.x, env.y, env.z)).size(11.0).color(DIM));
    if mstate.connected {
        ui.horizontal(|ui| {
            let (sl_color, sl_text) = if mstate.soft_limits {
                (GREEN, "SOFT LIMITS: ON")
            } else {
                (RED, "SOFT LIMITS: OFF")
            };
            ui.label(egui::RichText::new(sl_text).size(12.0).color(sl_color));
            let toggle_text = if mstate.soft_limits { "DISABLE" } else { "ENABLE" };
            let toggle_fill = if mstate.soft_limits {
                egui::Color32::from_rgb(0x33, 0x11, 0x11)
            } else {
                egui::Color32::from_rgb(0x11, 0x33, 0x11)
            };
            let btn = egui::Button::new(egui::RichText::new(toggle_text).size(11.0))
                .fill(toggle_fill)
                .min_size(egui::vec2(0.0, 20.0));
            if ui.add(btn).clicked() {
                // Unlock first in case GRBL is in alarm/locked state
                engine.send("$X");
                if mstate.soft_limits {
                    engine.send("$20=0");
                } else {
                    engine.send("$20=1");
                }
                engine.send("$$");
            }
        });
        let mt = mstate.max_travel;
        ui.label(egui::RichText::new(format!("FW TRAVEL  X:{:.0}  Y:{:.0}  Z:{:.0}", mt.x, mt.y, mt.z)).size(11.0).color(DIM));
    }
}

fn position_row(ui: &mut egui::Ui, pos: Vec3, size: f32) {
    let accent = egui::Color32::from_rgb(0x44, 0x88, 0xff);
    for (label, val) in [("X", pos.x), ("Y", pos.y), ("Z", pos.z)] {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new(format!("{}:", label)).size(size).color(accent));
            ui.label(egui::RichText::new(format!("{:>9.3}", val)).size(size).color(WHITE));
        });
    }
}

fn jog_section(ui: &mut egui::Ui, engine: &Arc<Engine>, state: &mut ControlsState) {
    section(ui, "Jog");
    ui.columns(3, |cols| {
        for (i, step) in [0.1f32, 1.0, 10.0].iter().enumerate() {
            let selected = (state.jog_step - step).abs() < 0.01;
            let text = if selected {
                egui::RichText::new(format!("{}", step)).size(14.0).color(egui::Color32::BLACK).strong()
            } else {
                egui::RichText::new(format!("{}", step)).size(14.0)
            };
            let btn = egui::Button::new(text)
                .min_size(egui::vec2(0.0, 26.0))
                .fill(if selected { AMBER } else { BTN_BG });
            if cols[i].add_sized([cols[i].available_width(), 26.0], btn).clicked() {
                state.jog_step = *step;
            }
        }
    });

    let step = state.jog_step;
    let jog_h = 32.0;

    ui.columns(3, |cols| {
        if cols[1].add_sized([cols[1].available_width(), jog_h], wide_btn("Y+")).clicked() {
            engine.send(&format!("$J=G91 G21 Y{:.3} F1000", step));
        }
    });
    ui.columns(3, |cols| {
        if cols[0].add_sized([cols[0].available_width(), jog_h], wide_btn("X-")).clicked() {
            engine.send(&format!("$J=G91 G21 X-{:.3} F1000", step));
        }
        if cols[2].add_sized([cols[2].available_width(), jog_h], wide_btn("X+")).clicked() {
            engine.send(&format!("$J=G91 G21 X{:.3} F1000", step));
        }
    });
    ui.columns(3, |cols| {
        if cols[1].add_sized([cols[1].available_width(), jog_h], wide_btn("Y-")).clicked() {
            engine.send(&format!("$J=G91 G21 Y-{:.3} F1000", step));
        }
    });
    ui.add_space(4.0);
    ui.columns(2, |cols| {
        if cols[0].add_sized([cols[0].available_width(), jog_h], wide_btn("Z-")).clicked() {
            engine.send(&format!("$J=G91 G21 Z-{:.3} F500", step));
        }
        if cols[1].add_sized([cols[1].available_width(), jog_h], wide_btn("Z+")).clicked() {
            engine.send(&format!("$J=G91 G21 Z{:.3} F500", step));
        }
    });
}

fn overrides_section(ui: &mut egui::Ui, engine: &Arc<Engine>, mstate: &MachineState) {
    section(ui, "Overrides");
    let feed_ovr = if mstate.feed_ovr == 0 { 100 } else { mstate.feed_ovr };
    let spindle_ovr = if mstate.spindle_ovr == 0 { 100 } else { mstate.spindle_ovr };
    override_row(ui, engine, "FEED", feed_ovr, 0x91, 0x92);
    override_row(ui, engine, "SPINDLE", spindle_ovr, 0x9A, 0x9B);
}

fn override_row(ui: &mut egui::Ui, engine: &Arc<Engine>, label: &str, pct: i32, inc: u8, dec: u8) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(label).size(13.0).color(DIM));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.add(egui::Button::new(egui::RichText::new("+").size(16.0)).min_size(egui::vec2(28.0, 28.0))).clicked() {
                engine.realtime(inc);
            }
            ui.label(egui::RichText::new(format!("{:>4}%", pct)).size(16.0).color(WHITE));
            if ui.add(egui::Button::new(egui::RichText::new("-").size(16.0)).min_size(egui::vec2(28.0, 28.0))).clicked() {
                engine.realtime(dec);
            }
        });
    });
}

fn actions_section(ui: &mut egui::Ui, engine: &Arc<Engine>) {
    section(ui, "Actions");
    ui.columns(2, |cols| {
        if cols[0].add_sized([cols[0].available_width(), 28.0], wide_btn("HOME")).clicked() { engine.send("$H"); }
        if cols[1].add_sized([cols[1].available_width(), 28.0], wide_btn("UNLOCK")).clicked() { engine.send("$X"); }
    });
    ui.columns(2, |cols| {
        if cols[0].add_sized([cols[0].available_width(), 28.0], wide_btn("ZERO XY")).clicked() { engine.send("G10 L20 P1 X0 Y0"); }
        if cols[1].add_sized([cols[1].available_width(), 28.0], wide_btn("ZERO Z")).clicked() { engine.send("G10 L20 P1 Z0"); }
    });
    ui.columns(2, |cols| {
        let on_btn = egui::Button::new(egui::RichText::new("SPINDLE ON").size(14.0).color(GREEN))
            .fill(egui::Color32::from_rgb(0x11, 0x33, 0x11))
            .min_size(egui::vec2(0.0, 28.0));
        if cols[0].add_sized([cols[0].available_width(), 28.0], on_btn).clicked() {
            engine.send("M3 S1000");
        }
        let off_btn = egui::Button::new(egui::RichText::new("SPINDLE OFF").size(14.0).color(RED))
            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
            .min_size(egui::vec2(0.0, 28.0));
        if cols[1].add_sized([cols[1].available_width(), 28.0], off_btn).clicked() {
            engine.send("M5");
        }
    });
}

fn job_section(ui: &mut egui::Ui, engine: &Arc<Engine>, jstate: &JobState) {
    section(ui, "Job");
    let is_running = jstate.status == JobStatus::Running;
    let is_paused = jstate.status == JobStatus::Paused;

    if is_running || is_paused {
        ui.columns(3, |cols| {
            let pause_label = if is_paused { "RESUME" } else { "PAUSE" };
            let pause = egui::Button::new(egui::RichText::new(pause_label).size(14.0).color(AMBER).strong())
                .fill(egui::Color32::from_rgb(0x33, 0x2a, 0x00)).min_size(egui::vec2(0.0, 32.0));
            if cols[0].add_sized([cols[0].available_width(), 32.0], pause).clicked() {
                if is_paused { engine.resume_job(); } else { engine.pause_job(); }
            }
            let stop = egui::Button::new(egui::RichText::new("STOP").size(14.0).color(AMBER).strong())
                .fill(egui::Color32::from_rgb(0x33, 0x22, 0x00)).min_size(egui::vec2(0.0, 32.0));
            if cols[1].add_sized([cols[1].available_width(), 32.0], stop).clicked() {
                engine.stop_job();
            }
            let estop = egui::Button::new(egui::RichText::new("E-STOP").size(14.0).color(RED).strong())
                .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11)).min_size(egui::vec2(0.0, 32.0));
            if cols[2].add_sized([cols[2].available_width(), 32.0], estop).clicked() {
                engine.stop_job();
                engine.soft_reset();
            }
        });
    } else {
        let estop = egui::Button::new(egui::RichText::new("E-STOP").size(14.0).color(RED).strong())
            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11)).min_size(egui::vec2(0.0, 32.0));
        if ui.add_sized([ui.available_width(), 32.0], estop).clicked() {
            engine.stop_job();
            engine.soft_reset();
        }
    }
}

fn status_display(s: Status) -> (egui::Color32, &'static str) {
    match s {
        Status::Idle => (GREEN, "IDLE"),
        Status::Run => (egui::Color32::from_rgb(0x44, 0x88, 0xff), "RUN"),
        Status::Hold => (AMBER, "HOLD"),
        Status::Alarm => (RED, "ALARM"),
        Status::Home => (AMBER, "HOME"),
        Status::Check => (egui::Color32::from_rgb(0x44, 0x88, 0xff), "CHECK"),
        Status::Jog => (egui::Color32::from_rgb(0x44, 0x88, 0xff), "JOG"),
        Status::Door => (RED, "DOOR"),
        Status::Sleep => (DIM, "SLEEP"),
        Status::Disconnected => (DIM, "---"),
    }
}

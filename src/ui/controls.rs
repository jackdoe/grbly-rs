use crate::grbl::engine::Engine;
use crate::grbl::serial;
use crate::grbl::state::*;
use crate::ui::probe::ProbeState;
use parking_lot::RwLock;
use std::sync::Arc;
use std::time::{Duration, Instant};
use three_d::egui;

const RETRACT_Z: f32 = 5.0;

const AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0xaa, 0x00);
const DIM: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);
const GREEN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xff, 0x88);
const RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x44, 0x44);
const WHITE: egui::Color32 = egui::Color32::from_rgb(0xff, 0xdd, 0xaa);
const BTN_BG: egui::Color32 = egui::Color32::from_rgb(0x22, 0x22, 0x33);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ControlsTab {
    #[default]
    Run,
    Jog,
    Setup,
    Probe,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConfirmAction {
    SoftLimitEnable,
    SoftLimitDisable,
    SpindleOn,
}

pub struct ControlsState {
    pub port_list: Vec<String>,
    pub port_index: usize,
    pub jog_step: f32,
    pub spindle_rpm: u32,
    travel: Vec3,
    last_max_travel: Vec3,
    tab: ControlsTab,
    confirm: Option<(ConfirmAction, Instant)>,
    notice: String,
}

impl Default for ControlsState {
    fn default() -> Self {
        Self {
            port_list: Vec::new(),
            port_index: 0,
            jog_step: 1.0,
            spindle_rpm: 10000,
            travel: Vec3::default(),
            last_max_travel: Vec3::default(),
            tab: ControlsTab::Run,
            confirm: None,
            notice: String::new(),
        }
    }
}

fn section(ui: &mut egui::Ui, label: &str) {
    ui.add_space(4.0);
    ui.label(
        egui::RichText::new(label.to_uppercase())
            .size(14.0)
            .color(DIM)
            .strong(),
    );
    ui.add_space(2.0);
}

fn wide_btn(text: &str) -> egui::Button<'_> {
    egui::Button::new(egui::RichText::new(text).size(14.0)).min_size(egui::vec2(0.0, 28.0))
}

fn wide_btn_colored(text: &str, fill: egui::Color32) -> egui::Button<'_> {
    wide_btn(text).fill(fill)
}

pub fn draw(
    ctx: &egui::Context,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    job_lock: &Arc<RwLock<JobState>>,
    ui_state: &mut ControlsState,
    probe_state: &mut ProbeState,
) {
    expire_confirm(ui_state);
    egui::SidePanel::left("controls")
        .default_width(292.0)
        .resizable(false)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.set_min_width(ui.available_width());
                connection_section(ui, engine, mstate, ui_state);
                ui.separator();
                machine_readout(ui, engine, mstate, jstate);
                draw_notice(ui, ui_state);
                ui.separator();
                tab_bar(ui, ui_state);
                ui.separator();
                match ui_state.tab {
                    ControlsTab::Run => run_section(ui, engine, mstate, jstate, ui_state),
                    ControlsTab::Jog => jog_section(ui, engine, mstate, ui_state),
                    ControlsTab::Setup => setup_section(ui, engine, mstate, ui_state),
                    ControlsTab::Probe => crate::ui::probe::draw(
                        ui, engine, mstate, jstate, job_lock, probe_state,
                    ),
                }
            });
        });
}

fn connection_section(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ControlsState,
) {
    section(ui, "Connection");
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("PORT").size(12.0).color(DIM));
        let name = state
            .port_list
            .get(state.port_index)
            .cloned()
            .unwrap_or_else(|| "---".into());
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
        if cols[0]
            .add_sized([cols[0].available_width(), 28.0], wide_btn("REFRESH"))
            .clicked()
        {
            state.port_list = serial::list_ports();
            state.port_index = 0;
        }
        if mstate.connected {
            if cols[1]
                .add_sized(
                    [cols[1].available_width(), 28.0],
                    wide_btn_colored("DISCONNECT", RED),
                )
                .clicked()
            {
                engine.disconnect();
                state.last_max_travel = Vec3::default();
                state.notice.clear();
            }
        } else if cols[1]
            .add_enabled(
                !state.port_list.is_empty(),
                wide_btn_colored("CONNECT", egui::Color32::from_rgb(0x00, 0x66, 0x33)),
            )
            .clicked()
        {
            if let Some(port) = state.port_list.get(state.port_index) {
                if let Err(err) = engine.connect(port, 115200) {
                    state.notice = format!("Connect failed: {err}");
                } else {
                    state.notice.clear();
                }
                state.last_max_travel = Vec3::default();
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

fn machine_readout(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
) {
    let (color, text) = status_display(mstate.status);
    ui.horizontal(|ui| {
        ui.label(
            egui::RichText::new(format!("[ {} ]", text))
                .size(20.0)
                .color(color)
                .strong(),
        );
        if matches!(mstate.status, Status::Hold | Status::Door) {
            let resume = egui::Button::new(
                egui::RichText::new("RESUME").size(13.0).color(GREEN).strong(),
            )
            .fill(egui::Color32::from_rgb(0x11, 0x33, 0x11))
            .min_size(egui::vec2(80.0, 28.0));
            if ui.add(resume).clicked() {
                engine.resume();
            }
        }
    });
    ui.add_space(4.0);
    ui.label(egui::RichText::new("WORK").size(11.0).color(DIM));
    position_row(ui, mstate.wpos, 18.0);
    ui.add_space(2.0);
    ui.label(egui::RichText::new("MACHINE").size(11.0).color(DIM));
    position_row(ui, mstate.mpos, 13.0);
    if mstate.max_travel.x > 0.0 {
        ui.label(
            egui::RichText::new(format!(
                "TRAVEL  X:{:.1}  Y:{:.1}  Z:{:.1}",
                mstate.max_travel.x, mstate.max_travel.y, mstate.max_travel.z
            ))
            .size(11.0)
            .color(DIM),
        );
    }
    match &jstate.heightmap {
        Some(map) => {
            let (lo, hi) = map.z_min_max();
            ui.label(
                egui::RichText::new(format!(
                    "MAP {}x{}  Δ {:.3}..{:.3} mm",
                    map.grid_x, map.grid_y, lo, hi
                ))
                .size(11.0)
                .color(GREEN),
            );
        }
        None => {
            ui.label(egui::RichText::new("MAP off").size(11.0).color(DIM));
        }
    }
    if !mstate.last_error.is_empty() {
        banner(ui, &mstate.last_error, RED);
    }
}

fn tab_bar(ui: &mut egui::Ui, state: &mut ControlsState) {
    ui.horizontal(|ui| {
        for (label, tab) in [
            ("RUN", ControlsTab::Run),
            ("JOG", ControlsTab::Jog),
            ("SETUP", ControlsTab::Setup),
            ("PROBE", ControlsTab::Probe),
        ] {
            if ui
                .selectable_label(state.tab == tab, egui::RichText::new(label).size(12.0))
                .clicked()
            {
                state.tab = tab;
            }
        }
    });
}

fn draw_notice(ui: &mut egui::Ui, state: &ControlsState) {
    if !state.notice.is_empty() {
        banner(ui, &state.notice, AMBER);
    }
    if let Some((action, _)) = state.confirm {
        let text = match action {
            ConfirmAction::SoftLimitEnable => "Confirm: click ENABLE again",
            ConfirmAction::SoftLimitDisable => "Confirm: click DISABLE again",
            ConfirmAction::SpindleOn => "Confirm: click SPINDLE ON again",
        };
        banner(ui, text, AMBER);
    }
}

fn banner(ui: &mut egui::Ui, text: &str, color: egui::Color32) {
    let frame = egui::Frame::default()
        .fill(egui::Color32::from_rgb(0x33, 0x22, 0x00))
        .inner_margin(egui::Margin::same(4.0));
    frame.show(ui, |ui| {
        ui.label(egui::RichText::new(text).size(11.0).color(color));
    });
}

fn run_section(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    state: &mut ControlsState,
) {
    section(ui, "Job");
    job_section(ui, engine, mstate, jstate);
    ui.separator();
    spindle_section(ui, engine, mstate, state);
    ui.separator();
    overrides_section(ui, engine, mstate);
}

fn setup_section(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ControlsState,
) {
    section(ui, "Machine");
    machine_actions(ui, engine, mstate);
    ui.separator();
    soft_limits(ui, engine, mstate, state);
    ui.separator();
    travel_editor(ui, engine, mstate, state);
}

fn position_row(ui: &mut egui::Ui, pos: Vec3, size: f32) {
    let accent = egui::Color32::from_rgb(0x44, 0x88, 0xff);
    for (label, val) in [("X", pos.x), ("Y", pos.y), ("Z", pos.z)] {
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(format!("{}:", label))
                    .size(size)
                    .color(accent),
            );
            ui.label(
                egui::RichText::new(format!("{:>9.3}", val))
                    .size(size)
                    .color(WHITE),
            );
        });
    }
}

fn jog_section(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ControlsState,
) {
    section(ui, "Jog");
    let can_jog = can_jog(mstate);
    ui.columns(3, |cols| {
        for (i, step) in [0.1f32, 1.0, 10.0].iter().enumerate() {
            let selected = (state.jog_step - step).abs() < 0.01;
            let text = if selected {
                egui::RichText::new(format!("{}", step))
                    .size(14.0)
                    .color(egui::Color32::BLACK)
                    .strong()
            } else {
                egui::RichText::new(format!("{}", step)).size(14.0)
            };
            let btn = egui::Button::new(text)
                .min_size(egui::vec2(0.0, 26.0))
                .fill(if selected { AMBER } else { BTN_BG });
            if cols[i]
                .add_sized([cols[i].available_width(), 26.0], btn)
                .clicked()
            {
                state.jog_step = *step;
            }
        }
    });

    let step = state.jog_step;
    let jog_h = 34.0;
    jog_button_row(
        ui,
        engine,
        can_jog,
        jog_h,
        &[("", ""), ("Y+", "Y"), ("", "")],
        step,
    );
    jog_button_row(
        ui,
        engine,
        can_jog,
        jog_h,
        &[("X-", "X-"), ("", ""), ("X+", "X")],
        step,
    );
    jog_button_row(
        ui,
        engine,
        can_jog,
        jog_h,
        &[("", ""), ("Y-", "Y-"), ("", "")],
        step,
    );
    ui.add_space(4.0);
    ui.columns(2, |cols| {
        if cols[0].add_enabled(can_jog, wide_btn("Z-")).clicked() {
            engine.send(&format!("$J=G91 G21 Z-{:.3} F500", step));
        }
        if cols[1].add_enabled(can_jog, wide_btn("Z+")).clicked() {
            engine.send(&format!("$J=G91 G21 Z{:.3} F500", step));
        }
    });
    if !can_jog {
        ui.label(
            egui::RichText::new("Jog is enabled only while connected and idle/jogging.")
                .size(11.0)
                .color(DIM),
        );
    }
}

fn jog_button_row(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    enabled: bool,
    height: f32,
    labels: &[(&str, &str); 3],
    step: f32,
) {
    ui.columns(3, |cols| {
        for (idx, (label, axis)) in labels.iter().enumerate() {
            if label.is_empty() {
                continue;
            }
            if cols[idx].add_enabled(enabled, wide_btn(label)).clicked() {
                let feed = if axis.starts_with('Z') { 500 } else { 1000 };
                engine.send(&format!("$J=G91 G21 {}{:.3} F{}", axis, step, feed));
            }
            cols[idx].allocate_space(egui::vec2(cols[idx].available_width(), height - 28.0));
        }
    });
}

fn overrides_section(ui: &mut egui::Ui, engine: &Arc<Engine>, mstate: &MachineState) {
    section(ui, "Overrides");
    let enabled = mstate.connected;
    let feed_ovr = if mstate.feed_ovr == 0 {
        100
    } else {
        mstate.feed_ovr
    };
    let spindle_ovr = if mstate.spindle_ovr == 0 {
        100
    } else {
        mstate.spindle_ovr
    };
    override_row(ui, engine, enabled, "FEED", feed_ovr, 0x91, 0x92);
    override_row(ui, engine, enabled, "SPINDLE", spindle_ovr, 0x9A, 0x9B);
}

fn override_row(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    enabled: bool,
    label: &str,
    pct: i32,
    inc: u8,
    dec: u8,
) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new(label).size(13.0).color(DIM));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui
                .add_enabled(
                    enabled,
                    egui::Button::new(egui::RichText::new("+").size(16.0))
                        .min_size(egui::vec2(28.0, 28.0)),
                )
                .clicked()
            {
                engine.realtime(inc);
            }
            ui.label(
                egui::RichText::new(format!("{:>4}%", pct))
                    .size(16.0)
                    .color(WHITE),
            );
            if ui
                .add_enabled(
                    enabled,
                    egui::Button::new(egui::RichText::new("-").size(16.0))
                        .min_size(egui::vec2(28.0, 28.0)),
                )
                .clicked()
            {
                engine.realtime(dec);
            }
        });
    });
}

fn retract_then(engine: &Arc<Engine>, mstate: &MachineState, follow: &[&str]) {
    if mstate.wpos.z < RETRACT_Z {
        engine.send(&format!("G90 G21 G0 Z{:.3}", RETRACT_Z));
    }
    for line in follow {
        engine.send(line);
    }
}

fn machine_actions(ui: &mut egui::Ui, engine: &Arc<Engine>, mstate: &MachineState) {
    let can_idle = mstate.connected && mstate.status == Status::Idle;
    let can_home =
        mstate.connected && !matches!(mstate.status, Status::Run | Status::Hold | Status::Jog);
    ui.columns(2, |cols| {
        if cols[0].add_enabled(can_home, wide_btn("HOME")).clicked() {
            engine.send("$H");
        }
        if cols[1]
            .add_enabled(mstate.connected, wide_btn("UNLOCK"))
            .clicked()
        {
            engine.send("$X");
        }
    });
    ui.columns(2, |cols| {
        if cols[0]
            .add_enabled(
                can_idle,
                egui::Button::new(egui::RichText::new("WORK HOME XY0").size(14.0))
                    .fill(BTN_BG)
                    .min_size(egui::vec2(cols[0].available_width(), 28.0)),
            )
            .clicked()
        {
            retract_then(engine, mstate, &["G90 G21 G0 X0 Y0"]);
        }
        if cols[1]
            .add_enabled(
                can_idle,
                egui::Button::new(egui::RichText::new("WORK HOME XYZ0").size(14.0))
                    .fill(BTN_BG)
                    .min_size(egui::vec2(cols[1].available_width(), 28.0)),
            )
            .clicked()
        {
            retract_then(engine, mstate, &["G90 G21 G0 X0 Y0", "G90 G21 G0 Z0"]);
        }
    });
    ui.columns(3, |cols| {
        if cols[0].add_enabled(can_idle, wide_btn("ZERO XY")).clicked() {
            engine.send("G10 L20 P1 X0 Y0");
        }
        if cols[1].add_enabled(can_idle, wide_btn("ZERO Z")).clicked() {
            engine.send("G10 L20 P1 Z0");
        }
        let xyz = egui::Button::new(egui::RichText::new("ZERO XYZ").size(14.0).color(GREEN))
            .fill(egui::Color32::from_rgb(0x11, 0x33, 0x11))
            .min_size(egui::vec2(0.0, 28.0));
        if cols[2].add_enabled(can_idle, xyz).clicked() {
            engine.send("G10 L20 P1 X0 Y0 Z0");
        }
    });
}

fn spindle_section(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ControlsState,
) {
    section(ui, "Spindle");
    let enabled = mstate.connected && matches!(mstate.status, Status::Idle | Status::Run);
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("RPM").size(12.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.spindle_rpm)
                .range(0..=30000)
                .speed(100.0),
        );
        if mstate.spindle > 0.0 {
            ui.label(
                egui::RichText::new(format!("live {:.0}", mstate.spindle))
                    .size(11.0)
                    .color(DIM),
            );
        }
    });
    ui.columns(2, |cols| {
        let on_label = if mstate.spindle > 0.0 {
            "SET RPM"
        } else {
            "SPINDLE ON"
        };
        let on_btn = egui::Button::new(egui::RichText::new(on_label).size(14.0).color(GREEN))
            .fill(egui::Color32::from_rgb(0x11, 0x33, 0x11))
            .min_size(egui::vec2(0.0, 28.0));
        if cols[0].add_enabled(enabled, on_btn).clicked() {
            let already_running = mstate.spindle > 0.0;
            if already_running || confirm(state, ConfirmAction::SpindleOn) {
                engine.send(&format!("M3 S{}", state.spindle_rpm));
                state.notice.clear();
            }
        }
        let off_btn = egui::Button::new(egui::RichText::new("SPINDLE OFF").size(14.0).color(RED))
            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
            .min_size(egui::vec2(0.0, 28.0));
        if cols[1].add_enabled(mstate.connected, off_btn).clicked() {
            state.confirm = None;
            engine.send("M5");
        }
    });
}

fn soft_limits(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ControlsState,
) {
    section(ui, "Soft Limits");
    let mt = mstate.max_travel;
    let (sl_color, sl_text) = if mstate.soft_limits {
        (GREEN, "SOFT LIMITS: ON")
    } else {
        (RED, "SOFT LIMITS: OFF")
    };
    ui.label(egui::RichText::new(sl_text).size(12.0).color(sl_color));
    let has_travel = mt.x > 0.0 && mt.y > 0.0 && mt.z > 0.0;
    let toggle_text = if mstate.soft_limits {
        "DISABLE"
    } else {
        "ENABLE"
    };
    let toggle_fill = if mstate.soft_limits {
        egui::Color32::from_rgb(0x33, 0x11, 0x11)
    } else {
        egui::Color32::from_rgb(0x11, 0x33, 0x11)
    };
    let action = if mstate.soft_limits {
        ConfirmAction::SoftLimitDisable
    } else {
        ConfirmAction::SoftLimitEnable
    };
    if ui
        .add_enabled(
            mstate.connected && (mstate.soft_limits || has_travel),
            egui::Button::new(egui::RichText::new(toggle_text).size(12.0))
                .fill(toggle_fill)
                .min_size(egui::vec2(ui.available_width(), 24.0)),
        )
        .clicked()
        && confirm(state, action)
    {
        if matches!(mstate.status, Status::Alarm | Status::Door) {
            engine.send("$X");
        }
        engine.send(if mstate.soft_limits { "$20=0" } else { "$20=1" });
        engine.send("$$");
        state.notice.clear();
    }
    if !mstate.soft_limits && !has_travel {
        state.notice = "Set non-zero X/Y/Z travel before enabling soft limits.".into();
    }
}

fn travel_editor(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ControlsState,
) {
    section(ui, "Travel");
    if mstate.max_travel != state.last_max_travel {
        state.last_max_travel = mstate.max_travel;
        state.travel = mstate.max_travel;
    }
    ui.horizontal(|ui| {
        drag_mm(ui, "X", &mut state.travel.x);
        drag_mm(ui, "Y", &mut state.travel.y);
        drag_mm(ui, "Z", &mut state.travel.z);
    });
    if ui
        .add_enabled(
            mstate.connected
                && state.travel.x > 0.0
                && state.travel.y > 0.0
                && state.travel.z > 0.0,
            egui::Button::new(egui::RichText::new("SET TRAVEL").size(12.0))
                .min_size(egui::vec2(ui.available_width(), 24.0)),
        )
        .clicked()
    {
        engine.send(&format!("$130={:.1}", state.travel.x));
        engine.send(&format!("$131={:.1}", state.travel.y));
        engine.send(&format!("$132={:.1}", state.travel.z));
        engine.send("$$");
        state.notice.clear();
    }
}

fn drag_mm(ui: &mut egui::Ui, label: &str, value: &mut f32) -> egui::Response {
    ui.label(egui::RichText::new(label).size(12.0).color(DIM));
    ui.add(
        egui::DragValue::new(value)
            .speed(0.1)
            .range(-10000.0..=10000.0)
            .suffix(" mm"),
    )
}

fn job_section(ui: &mut egui::Ui, engine: &Arc<Engine>, mstate: &MachineState, jstate: &JobState) {
    let is_running = jstate.status == JobStatus::Running;
    let is_paused = jstate.status == JobStatus::Paused;
    let connected = mstate.connected;

    if is_running || is_paused {
        ui.columns(3, |cols| {
            let pause_label = if is_paused { "RESUME" } else { "PAUSE" };
            let pause = egui::Button::new(
                egui::RichText::new(pause_label)
                    .size(14.0)
                    .color(AMBER)
                    .strong(),
            )
            .fill(egui::Color32::from_rgb(0x33, 0x2a, 0x00))
            .min_size(egui::vec2(0.0, 32.0));
            if cols[0].add_enabled(connected, pause).clicked() {
                if is_paused {
                    engine.resume_job();
                } else {
                    engine.pause_job();
                }
            }
            let stop =
                egui::Button::new(egui::RichText::new("STOP").size(14.0).color(AMBER).strong())
                    .fill(egui::Color32::from_rgb(0x33, 0x22, 0x00))
                    .min_size(egui::vec2(0.0, 32.0));
            if cols[1].add_enabled(connected, stop).clicked() {
                engine.stop_job();
            }
            let estop =
                egui::Button::new(egui::RichText::new("E-STOP").size(14.0).color(RED).strong())
                    .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
                    .min_size(egui::vec2(0.0, 32.0));
            if cols[2].add_enabled(connected, estop).clicked() {
                engine.stop_job();
                engine.soft_reset();
            }
        });
    } else {
        let estop = egui::Button::new(egui::RichText::new("E-STOP").size(14.0).color(RED).strong())
            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
            .min_size(egui::vec2(0.0, 32.0));
        if ui.add_enabled(connected, estop).clicked() {
            engine.stop_job();
            engine.soft_reset();
        }
    }
}

fn confirm(state: &mut ControlsState, action: ConfirmAction) -> bool {
    let confirmed = state
        .confirm
        .map(|(pending, at)| pending == action && at.elapsed() < Duration::from_secs(3))
        .unwrap_or(false);
    if confirmed {
        state.confirm = None;
        true
    } else {
        state.confirm = Some((action, Instant::now()));
        false
    }
}

fn expire_confirm(state: &mut ControlsState) {
    if state
        .confirm
        .map(|(_, at)| at.elapsed() >= Duration::from_secs(3))
        .unwrap_or(false)
    {
        state.confirm = None;
    }
}

fn can_jog(mstate: &MachineState) -> bool {
    mstate.connected && matches!(mstate.status, Status::Idle | Status::Jog)
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

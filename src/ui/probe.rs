use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;

use parking_lot::{Mutex, RwLock};
use three_d::egui;

use crate::grbl::engine::Engine;
use crate::grbl::heightmap::{self, HeightMap};
use crate::grbl::state::{JobState, JobStatus, MachineState, Status, Vec3};

const AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0xaa, 0x00);
const DIM: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);
const GREEN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xff, 0x88);
const RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x44, 0x44);
const WHITE: egui::Color32 = egui::Color32::from_rgb(0xff, 0xdd, 0xaa);
const CYAN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xcc, 0xff);
const BTN_BG: egui::Color32 = egui::Color32::from_rgb(0x22, 0x22, 0x33);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ProbeMode {
    #[default]
    Auto,
    Manual,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ProbePhase {
    #[default]
    Idle,
    Single,
    Auto,
    Manual,
    Done,
}

#[derive(Default)]
struct ProbeShared {
    samples: Vec<Option<f32>>,
    current_index: usize,
    error: String,
    finished: bool,
    single_z: Option<f32>,
    single_msg: String,
}

pub struct ProbeState {
    pub grid_x: u32,
    pub grid_y: u32,
    pub safe_z: f32,
    pub max_depth: f32,
    pub probe_feed: f32,
    pub mode: ProbeMode,
    pub phase: ProbePhase,
    pub bbox_min: (f32, f32),
    pub bbox_max: (f32, f32),
    pub manual_jog_step: f32,
    cancel: Arc<AtomicBool>,
    shared: Arc<Mutex<ProbeShared>>,
}

impl ProbeState {
    pub fn samples_snapshot(&self) -> (Vec<Option<f32>>, Option<usize>) {
        let sh = self.shared.lock();
        let in_progress = matches!(self.phase, ProbePhase::Auto | ProbePhase::Manual);
        let cur = if in_progress { Some(sh.current_index) } else { None };
        (sh.samples.clone(), cur)
    }
}

impl Default for ProbeState {
    fn default() -> Self {
        Self {
            grid_x: 5,
            grid_y: 5,
            safe_z: 1.0,
            max_depth: 0.3,
            probe_feed: 50.0,
            mode: ProbeMode::Auto,
            phase: ProbePhase::Idle,
            bbox_min: (0.0, 0.0),
            bbox_max: (0.0, 0.0),
            manual_jog_step: 0.1,
            cancel: Arc::new(AtomicBool::new(false)),
            shared: Arc::new(Mutex::new(ProbeShared::default())),
        }
    }
}

fn grid_total(s: &ProbeState) -> usize {
    s.grid_x as usize * s.grid_y as usize
}

fn grid_xy(bbox_min: (f32, f32), bbox_max: (f32, f32), gx: u32, gy: u32, idx: usize) -> (f32, f32) {
    let i = (idx % gx as usize) as u32;
    let j = (idx / gx as usize) as u32;
    let fx = i as f32 / (gx - 1).max(1) as f32;
    let fy = j as f32 / (gy - 1).max(1) as f32;
    (
        bbox_min.0 + fx * (bbox_max.0 - bbox_min.0),
        bbox_min.1 + fy * (bbox_max.1 - bbox_min.1),
    )
}

pub fn draw(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    job_lock: &Arc<RwLock<JobState>>,
    state: &mut ProbeState,
) {
    poll_completion(state, job_lock);

    pin_indicator(ui, mstate);

    ui.separator();
    section(ui, "Parameters");
    parameters(ui, state);

    ui.separator();
    section(ui, "Z Probe (single)");
    single_probe(ui, engine, mstate, state);

    ui.separator();
    section(ui, "Heightmap");
    grid_section(ui, engine, mstate, jstate, state);

    ui.separator();
    map_section(ui, jstate, job_lock, state);
}

fn section(ui: &mut egui::Ui, label: &str) {
    ui.add_space(2.0);
    ui.label(
        egui::RichText::new(label.to_uppercase())
            .size(13.0)
            .color(DIM)
            .strong(),
    );
    ui.add_space(2.0);
}

fn pin_indicator(ui: &mut egui::Ui, mstate: &MachineState) {
    let (color, text) = if !mstate.connected {
        (DIM, "PROBE PIN: ---")
    } else if mstate.probe_active {
        (GREEN, "PROBE PIN: CONNECTED")
    } else {
        (AMBER, "PROBE PIN: OPEN")
    };
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
        ui.painter().circle_filled(rect.center(), 6.0, color);
        ui.label(egui::RichText::new(text).size(13.0).color(color).strong());
    });
    ui.label(
        egui::RichText::new("touch the bit to copper to test wiring")
            .size(10.0)
            .color(DIM),
    );
}

fn parameters(ui: &mut egui::Ui, state: &mut ProbeState) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("safe_z").size(11.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.safe_z)
                .speed(0.05)
                .range(0.1..=20.0)
                .suffix(" mm"),
        );
    });
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("max_depth").size(11.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.max_depth)
                .speed(0.05)
                .range(0.05..=5.0)
                .suffix(" mm"),
        );
    });
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("feed").size(11.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.probe_feed)
                .speed(5.0)
                .range(5.0..=500.0)
                .suffix(" mm/min"),
        );
    });
}

fn single_probe(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ProbeState,
) {
    let busy = state.phase != ProbePhase::Idle && state.phase != ProbePhase::Done;
    let can_probe =
        mstate.connected && matches!(mstate.status, Status::Idle | Status::Jog) && !busy;

    ui.columns(2, |cols| {
        let probe_btn = egui::Button::new(egui::RichText::new("PROBE HERE").size(12.0).color(CYAN))
            .fill(egui::Color32::from_rgb(0x00, 0x22, 0x33))
            .min_size(egui::vec2(0.0, 26.0));
        if cols[0].add_enabled(can_probe, probe_btn).clicked() {
            spawn_single_probe(engine.clone(), mstate.wpos, state, false);
        }
        let zero_btn =
            egui::Button::new(egui::RichText::new("PROBE → ZERO Z").size(12.0).color(GREEN))
                .fill(egui::Color32::from_rgb(0x00, 0x33, 0x11))
                .min_size(egui::vec2(0.0, 26.0));
        if cols[1].add_enabled(can_probe, zero_btn).clicked() {
            spawn_single_probe(engine.clone(), mstate.wpos, state, true);
        }
    });

    let sh = state.shared.lock();
    if let Some(z) = sh.single_z {
        ui.label(
            egui::RichText::new(format!("last probe: Z = {:.3} mm", z))
                .size(11.0)
                .color(WHITE),
        );
    }
    if !sh.single_msg.is_empty() {
        let color = if sh.single_z.is_some() { GREEN } else { AMBER };
        ui.label(egui::RichText::new(&sh.single_msg).size(11.0).color(color));
    }
}

fn spawn_single_probe(
    engine: Arc<Engine>,
    wpos: Vec3,
    state: &mut ProbeState,
    zero_after: bool,
) {
    state.phase = ProbePhase::Single;
    {
        let mut sh = state.shared.lock();
        sh.single_z = None;
        sh.single_msg = "probing...".into();
        sh.finished = false;
    }
    let shared = state.shared.clone();
    let safe_z = state.safe_z;
    let max_depth = state.max_depth;
    let feed = state.probe_feed;
    thread::spawn(move || {
        let result = engine.probe_at(wpos.x, wpos.y, safe_z, max_depth, feed);
        match result {
            Ok(z) => {
                if zero_after {
                    engine.send(&format!("G10 L20 P1 Z{:.4}", safe_z - z));
                    let mut sh = shared.lock();
                    sh.single_z = Some(z);
                    sh.single_msg = format!("zeroed Z at probed surface (was {:.3})", z);
                    sh.finished = true;
                } else {
                    let mut sh = shared.lock();
                    sh.single_z = Some(z);
                    sh.single_msg = format!("probed Z = {:.3} mm", z);
                    sh.finished = true;
                }
            }
            Err(e) => {
                let mut sh = shared.lock();
                sh.single_z = None;
                sh.single_msg = format!("probe failed: {e}");
                sh.finished = true;
            }
        }
    });
}

fn grid_section(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    state: &mut ProbeState,
) {
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("grid").size(11.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.grid_x)
                .range(2..=10)
                .speed(1.0),
        );
        ui.label(egui::RichText::new("×").size(11.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.grid_y)
                .range(2..=10)
                .speed(1.0),
        );
        ui.label(
            egui::RichText::new(format!("= {} pts", state.grid_x * state.grid_y))
                .size(11.0)
                .color(DIM),
        );
    });

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("mode").size(11.0).color(DIM));
        if ui
            .selectable_label(state.mode == ProbeMode::Auto, "AUTO")
            .clicked()
        {
            state.mode = ProbeMode::Auto;
        }
        if ui
            .selectable_label(state.mode == ProbeMode::Manual, "MANUAL")
            .clicked()
        {
            state.mode = ProbeMode::Manual;
        }
    });

    let bbox_valid = jstate.bounds_max.x > jstate.bounds_min.x
        && jstate.bounds_max.y > jstate.bounds_min.y;
    if bbox_valid {
        ui.label(
            egui::RichText::new(format!(
                "bbox  X: {:.1}..{:.1}   Y: {:.1}..{:.1}",
                jstate.bounds_min.x, jstate.bounds_max.x, jstate.bounds_min.y, jstate.bounds_max.y
            ))
            .size(11.0)
            .color(DIM),
        );
    } else {
        ui.label(
            egui::RichText::new("load gcode to define probe bbox")
                .size(11.0)
                .color(AMBER),
        );
    }

    let busy = state.phase == ProbePhase::Auto
        || state.phase == ProbePhase::Manual
        || state.phase == ProbePhase::Single;
    let can_start = mstate.connected
        && mstate.status == Status::Idle
        && jstate.status != JobStatus::Running
        && jstate.status != JobStatus::Paused
        && bbox_valid
        && !busy;

    match state.phase {
        ProbePhase::Idle | ProbePhase::Done | ProbePhase::Single => {
            let label = match state.mode {
                ProbeMode::Auto => "START PROBE (AUTO)",
                ProbeMode::Manual => "START PROBE (MANUAL)",
            };
            let btn = egui::Button::new(egui::RichText::new(label).size(12.0).color(GREEN))
                .fill(egui::Color32::from_rgb(0x11, 0x33, 0x11))
                .min_size(egui::vec2(ui.available_width(), 28.0));
            if ui.add_enabled(can_start, btn).clicked() {
                start_grid(state, engine, jstate);
            }
            if mstate.connected
                && (mstate.wpos.x.abs() > 0.1
                    || mstate.wpos.y.abs() > 0.1
                    || mstate.wpos.z.abs() > 0.1)
            {
                ui.label(
                    egui::RichText::new(
                        "warning: WPos isn't (0,0,0). ZERO XYZ at the corner first.",
                    )
                    .size(10.0)
                    .color(AMBER),
                );
            }
        }
        ProbePhase::Auto => {
            grid_progress(ui, state);
            let abort = egui::Button::new(egui::RichText::new("ABORT").size(12.0).color(RED))
                .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
                .min_size(egui::vec2(ui.available_width(), 24.0));
            if ui.add(abort).clicked() {
                state.cancel.store(true, Ordering::Relaxed);
            }
        }
        ProbePhase::Manual => {
            grid_progress(ui, state);
            manual_controls(ui, engine, mstate, state);
        }
    }
}

fn grid_progress(ui: &mut egui::Ui, state: &ProbeState) {
    let sh = state.shared.lock();
    let total = grid_total(state);
    let i = sh.current_index.min(total.saturating_sub(1));
    let (x, y) = grid_xy(state.bbox_min, state.bbox_max, state.grid_x, state.grid_y, i);
    ui.label(
        egui::RichText::new(format!(
            "probing {}/{} at ({:.2}, {:.2})",
            i + 1,
            total,
            x,
            y
        ))
        .size(11.0)
        .color(WHITE),
    );
    if !sh.error.is_empty() {
        ui.label(egui::RichText::new(&sh.error).size(11.0).color(RED));
    }
}

fn manual_controls(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    state: &mut ProbeState,
) {
    let can_jog = mstate.connected && matches!(mstate.status, Status::Idle | Status::Jog);

    ui.horizontal(|ui| {
        for &step in &[1.0_f32, 0.1, 0.01] {
            let selected = (state.manual_jog_step - step).abs() < 1e-4;
            let label = format!("{}", step);
            let btn = egui::Button::new(egui::RichText::new(&label).size(11.0))
                .fill(if selected { AMBER } else { BTN_BG })
                .min_size(egui::vec2(0.0, 22.0));
            if ui.add_sized([50.0, 22.0], btn).clicked() {
                state.manual_jog_step = step;
            }
        }
    });

    ui.columns(2, |cols| {
        let zp = egui::Button::new(egui::RichText::new("Z+").size(12.0))
            .min_size(egui::vec2(0.0, 24.0));
        if cols[0].add_enabled(can_jog, zp).clicked() {
            engine.send(&format!("$J=G91 G21 Z{:.4} F500", state.manual_jog_step));
        }
        let zm = egui::Button::new(egui::RichText::new("Z-").size(12.0))
            .min_size(egui::vec2(0.0, 24.0));
        if cols[1].add_enabled(can_jog, zm).clicked() {
            engine.send(&format!("$J=G91 G21 Z-{:.4} F500", state.manual_jog_step));
        }
    });

    ui.columns(2, |cols| {
        let done = egui::Button::new(egui::RichText::new("DONE").size(12.0).color(GREEN))
            .fill(egui::Color32::from_rgb(0x11, 0x33, 0x11))
            .min_size(egui::vec2(0.0, 24.0));
        if cols[0].add_enabled(can_jog, done).clicked() {
            advance_manual(state, engine, mstate);
        }
        let abort = egui::Button::new(egui::RichText::new("ABORT").size(12.0).color(RED))
            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
            .min_size(egui::vec2(0.0, 24.0));
        if cols[1].add(abort).clicked() {
            abort_manual(state, engine);
        }
    });
}

fn map_section(
    ui: &mut egui::Ui,
    jstate: &JobState,
    job_lock: &Arc<RwLock<JobState>>,
    state: &mut ProbeState,
) {
    section(ui, "Map");
    if let Some(map) = &jstate.heightmap {
        let (lo, hi) = map.z_min_max();
        ui.label(
            egui::RichText::new(format!(
                "{}×{}  Δ {:.3}..{:.3} mm",
                map.grid_x, map.grid_y, lo, hi
            ))
            .size(11.0)
            .color(WHITE),
        );
        let clear = egui::Button::new(egui::RichText::new("CLEAR MAP").size(11.0).color(RED))
            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
            .min_size(egui::vec2(ui.available_width(), 22.0));
        if ui.add(clear).clicked() {
            let mut j = job_lock.write();
            j.heightmap = None;
            j.version = j.version.wrapping_add(1);
            heightmap::clear_cached();
            state.phase = ProbePhase::Idle;
        }
    } else {
        ui.label(egui::RichText::new("no map").size(11.0).color(DIM));
    }
}

fn start_grid(state: &mut ProbeState, engine: &Arc<Engine>, jstate: &JobState) {
    state.bbox_min = (jstate.bounds_min.x, jstate.bounds_min.y);
    state.bbox_max = (jstate.bounds_max.x, jstate.bounds_max.y);
    let total = grid_total(state);
    state.cancel.store(false, Ordering::Relaxed);
    {
        let mut sh = state.shared.lock();
        sh.samples = vec![None; total];
        sh.current_index = 0;
        sh.error.clear();
        sh.finished = false;
    }

    match state.mode {
        ProbeMode::Auto => {
            state.phase = ProbePhase::Auto;
            spawn_auto(state, engine.clone());
        }
        ProbeMode::Manual => {
            state.phase = ProbePhase::Manual;
            let (x, y) = grid_xy(state.bbox_min, state.bbox_max, state.grid_x, state.grid_y, 0);
            engine.send(&format!(
                "G90 G21 G0 X{:.3} Y{:.3} Z{:.3}",
                x, y, state.safe_z
            ));
        }
    }
}

fn spawn_auto(state: &ProbeState, engine: Arc<Engine>) {
    let total = grid_total(state);
    let cancel = state.cancel.clone();
    let shared = state.shared.clone();
    let bbox_min = state.bbox_min;
    let bbox_max = state.bbox_max;
    let grid_x = state.grid_x;
    let grid_y = state.grid_y;
    let safe_z = state.safe_z;
    let max_depth = state.max_depth;
    let feed = state.probe_feed;

    thread::spawn(move || {
        for idx in 0..total {
            if cancel.load(Ordering::Relaxed) {
                let mut sh = shared.lock();
                sh.error = "cancelled".into();
                sh.finished = true;
                return;
            }
            shared.lock().current_index = idx;

            let (x, y) = grid_xy(bbox_min, bbox_max, grid_x, grid_y, idx);

            match engine.probe_at(x, y, safe_z, max_depth, feed) {
                Ok(z) => {
                    shared.lock().samples[idx] = Some(z);
                }
                Err(e) => {
                    let mut sh = shared.lock();
                    sh.error = format!("probe failed at {}/{}: {}", idx + 1, total, e);
                    sh.finished = true;
                    return;
                }
            }
        }
        shared.lock().finished = true;
    });
}

fn advance_manual(state: &mut ProbeState, engine: &Arc<Engine>, mstate: &MachineState) {
    let total = grid_total(state);
    let idx = state.shared.lock().current_index;
    state.shared.lock().samples[idx] = Some(mstate.wpos.z);
    let next = idx + 1;
    if next >= total {
        engine.send(&format!("G90 G21 G0 Z{:.3}", state.safe_z));
        state.shared.lock().finished = true;
        return;
    }
    state.shared.lock().current_index = next;
    let (x, y) = grid_xy(state.bbox_min, state.bbox_max, state.grid_x, state.grid_y, next);
    engine.send(&format!("G90 G21 G0 Z{:.3}", state.safe_z));
    engine.send(&format!("G90 G21 G0 X{:.3} Y{:.3}", x, y));
}

fn abort_manual(state: &mut ProbeState, engine: &Arc<Engine>) {
    engine.send(&format!("G90 G21 G0 Z{:.3}", state.safe_z));
    state.cancel.store(true, Ordering::Relaxed);
    let mut sh = state.shared.lock();
    sh.error = "aborted".into();
    sh.finished = true;
}

fn poll_completion(state: &mut ProbeState, job_lock: &Arc<RwLock<JobState>>) {
    let finished = state.shared.lock().finished;
    if !finished {
        return;
    }

    if state.phase == ProbePhase::Single {
        state.phase = ProbePhase::Idle;
        state.shared.lock().finished = false;
        return;
    }

    if state.phase == ProbePhase::Auto || state.phase == ProbePhase::Manual {
        let (cancelled, samples) = {
            let sh = state.shared.lock();
            let cancelled = sh.error == "cancelled" || sh.error == "aborted";
            let samples = sh.samples.clone();
            (cancelled, samples)
        };
        let all_set = !samples.is_empty() && samples.iter().all(|s| s.is_some());
        if all_set && !cancelled {
            let z: Vec<f32> = samples.iter().map(|s| s.unwrap()).collect();
            if let Ok(map) = HeightMap::new(
                state.bbox_min,
                state.bbox_max,
                state.grid_x,
                state.grid_y,
                z,
            ) {
                let arc = Arc::new(map);
                let _ = heightmap::save_cached(&arc);
                let mut j = job_lock.write();
                j.heightmap = Some(arc);
                j.version = j.version.wrapping_add(1);
            }
            state.phase = ProbePhase::Done;
        } else {
            state.phase = ProbePhase::Idle;
        }
        let mut sh = state.shared.lock();
        sh.finished = false;
    }
}

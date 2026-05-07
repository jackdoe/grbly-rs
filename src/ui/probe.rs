use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;

use parking_lot::{Mutex, RwLock};
use three_d::egui;

use crate::grbl::engine::{Engine, SleepInhibitor};
use crate::grbl::heightmap::{self, grid_point, HeightMap};
use crate::grbl::state::{compute_line_map_modified, JobState, JobStatus, MachineState, Status};

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
    Grid,
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
    pub skipped: HashSet<usize>,
    safe_z: f32,
    max_depth: f32,
    probe_feed: f32,
    mode: ProbeMode,
    phase: ProbePhase,
    bbox_min: (f32, f32),
    bbox_max: (f32, f32),
    manual_jog_step: f32,
    cancel: Arc<AtomicBool>,
    shared: Arc<Mutex<ProbeShared>>,
}

impl ProbeState {
    pub fn samples_snapshot(&self) -> (Vec<Option<f32>>, Option<usize>, Vec<bool>) {
        let sh = self.shared.lock();
        let cur = if self.phase == ProbePhase::Grid {
            Some(sh.current_index)
        } else {
            None
        };
        let total = (self.grid_x * self.grid_y) as usize;
        let skipped: Vec<bool> = (0..total).map(|i| self.skipped.contains(&i)).collect();
        (sh.samples.clone(), cur, skipped)
    }
}

impl Default for ProbeState {
    fn default() -> Self {
        Self {
            grid_x: 5,
            grid_y: 5,
            skipped: HashSet::new(),
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

pub fn draw(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    job_lock: &Arc<RwLock<JobState>>,
    state: &mut ProbeState,
) {
    poll_completion(engine, state, job_lock);

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
    map_section(ui, engine, jstate, job_lock, state);
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
    let busy = state.phase != ProbePhase::Idle;
    let can_probe =
        mstate.connected && matches!(mstate.status, Status::Idle | Status::Jog) && !busy;

    ui.columns(2, |cols| {
        let probe_btn = egui::Button::new(egui::RichText::new("PROBE HERE").size(12.0).color(CYAN))
            .fill(egui::Color32::from_rgb(0x00, 0x22, 0x33))
            .min_size(egui::vec2(0.0, 26.0));
        if cols[0].add_enabled(can_probe, probe_btn).clicked() {
            spawn_single_probe(engine.clone(), state, false);
        }
        let zero_btn =
            egui::Button::new(egui::RichText::new("PROBE → ZERO Z").size(12.0).color(GREEN))
                .fill(egui::Color32::from_rgb(0x00, 0x33, 0x11))
                .min_size(egui::vec2(0.0, 26.0));
        if cols[1].add_enabled(can_probe, zero_btn).clicked() {
            spawn_single_probe(engine.clone(), state, true);
        }
    });

    let (last_z, last_msg) = {
        let sh = state.shared.lock();
        (sh.single_z, sh.single_msg.clone())
    };
    if let Some(z) = last_z {
        ui.label(
            egui::RichText::new(format!("last probe: Z = {:.3} mm", z))
                .size(11.0)
                .color(WHITE),
        );
        let set_btn =
            egui::Button::new(egui::RichText::new("SET Z0 HERE").size(12.0).color(GREEN))
                .fill(egui::Color32::from_rgb(0x00, 0x33, 0x11))
                .min_size(egui::vec2(ui.available_width(), 24.0));
        if ui.add_enabled(can_probe, set_btn).clicked() {
            engine.send(&format!("G10 L20 P1 Z{:.4}", mstate.wpos.z - z));
            let mut sh = state.shared.lock();
            sh.single_z = None;
            sh.single_msg = format!("Z0 set at probed surface (was Z={:.3})", z);
        }
    }
    if !last_msg.is_empty() {
        let color = if last_z.is_some() { GREEN } else { AMBER };
        ui.label(egui::RichText::new(&last_msg).size(11.0).color(color));
    }
}

fn spawn_single_probe(engine: Arc<Engine>, state: &mut ProbeState, zero_after: bool) {
    state.phase = ProbePhase::Single;
    {
        let mut sh = state.shared.lock();
        sh.single_z = None;
        sh.single_msg = "probing...".into();
        sh.finished = false;
    }
    let shared = state.shared.clone();
    let max_depth = state.max_depth;
    let feed = state.probe_feed;
    thread::spawn(move || {
        let result = engine.probe_here(max_depth, feed);
        let mut sh = shared.lock();
        match result {
            Ok(z) => {
                if zero_after {
                    drop(sh);
                    let current_z = engine.state.read().wpos.z;
                    engine.send(&format!("G10 L20 P1 Z{:.4}", current_z - z));
                    let mut sh = shared.lock();
                    sh.single_z = Some(z);
                    sh.single_msg = format!("zeroed Z at probed surface (was {:.3})", z);
                    sh.finished = true;
                } else {
                    sh.single_z = Some(z);
                    sh.single_msg = format!("probed Z = {:.3} mm", z);
                    sh.finished = true;
                }
            }
            Err(e) => {
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
    let mut grid_changed = false;
    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("grid").size(11.0).color(DIM));
        if ui
            .add(
                egui::DragValue::new(&mut state.grid_x)
                    .range(2..=10)
                    .speed(1.0),
            )
            .changed()
        {
            grid_changed = true;
        }
        ui.label(egui::RichText::new("×").size(11.0).color(DIM));
        if ui
            .add(
                egui::DragValue::new(&mut state.grid_y)
                    .range(2..=10)
                    .speed(1.0),
            )
            .changed()
        {
            grid_changed = true;
        }
        let active = (state.grid_x * state.grid_y) as usize - state.skipped.len();
        ui.label(
            egui::RichText::new(format!(
                "= {} pts ({} active)",
                state.grid_x * state.grid_y,
                active
            ))
            .size(11.0)
            .color(DIM),
        );
    });
    if grid_changed {
        state.skipped.clear();
    }

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
        grid_widget(ui, state);
    } else {
        ui.label(
            egui::RichText::new("load gcode to define probe bbox")
                .size(11.0)
                .color(AMBER),
        );
    }

    let can_start = mstate.connected
        && mstate.status == Status::Idle
        && jstate.status != JobStatus::Running
        && jstate.status != JobStatus::Paused
        && bbox_valid
        && state.phase == ProbePhase::Idle;

    let err = state.shared.lock().error.clone();
    if !err.is_empty() {
        ui.label(egui::RichText::new(&err).size(11.0).color(RED));
    }

    match state.phase {
        ProbePhase::Idle | ProbePhase::Single => {
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
        ProbePhase::Grid => {
            grid_progress(ui, state);
            match state.mode {
                ProbeMode::Auto => {
                    let abort =
                        egui::Button::new(egui::RichText::new("ABORT").size(12.0).color(RED))
                            .fill(egui::Color32::from_rgb(0x33, 0x11, 0x11))
                            .min_size(egui::vec2(ui.available_width(), 24.0));
                    if ui.add(abort).clicked() {
                        state.cancel.store(true, Ordering::Relaxed);
                    }
                }
                ProbeMode::Manual => manual_controls(ui, engine, mstate, state),
            }
        }
    }
}

fn grid_progress(ui: &mut egui::Ui, state: &ProbeState) {
    let sh = state.shared.lock();
    let total = grid_total(state);
    let i = sh.current_index.min(total.saturating_sub(1));
    let (x, y) = grid_point(state.bbox_min, state.bbox_max, state.grid_x, state.grid_y, i);
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
    engine: &Arc<Engine>,
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
            let line_count = j.lines.len();
            j.heightmap = None;
            j.line_map_modified = Arc::new(vec![false; line_count]);
            j.transform_cache = None;
            j.version = j.version.wrapping_add(1);
            drop(j);
            heightmap::clear_cached();
            state.phase = ProbePhase::Idle;
            engine.log("heightmap cleared".into());
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

    state.phase = ProbePhase::Grid;
    match state.mode {
        ProbeMode::Auto => spawn_auto(state, engine.clone()),
        ProbeMode::Manual => match next_unskipped(state, 0) {
            Some(first) => {
                state.shared.lock().current_index = first;
                let (x, y) =
                    grid_point(state.bbox_min, state.bbox_max, state.grid_x, state.grid_y, first);
                engine.send(&format!(
                    "G90 G21 G0 X{:.3} Y{:.3} Z{:.3}",
                    x, y, state.safe_z
                ));
            }
            None => {
                let mut sh = state.shared.lock();
                sh.error = "all points skipped".into();
                sh.finished = true;
            }
        },
    }
}

fn next_unskipped(state: &ProbeState, from: usize) -> Option<usize> {
    let total = grid_total(state);
    (from..total).find(|i| !state.skipped.contains(i))
}

fn fill_skipped(samples: &[Option<f32>], skipped: &HashSet<usize>) -> Option<Vec<f32>> {
    let probed: Vec<f32> = samples
        .iter()
        .enumerate()
        .filter_map(|(i, s)| if skipped.contains(&i) { None } else { *s })
        .collect();
    if probed.is_empty() {
        return None;
    }
    let avg = probed.iter().sum::<f32>() / probed.len() as f32;
    let needs_all = samples.iter().enumerate().all(|(i, s)| skipped.contains(&i) || s.is_some());
    if !needs_all {
        return None;
    }
    Some(samples.iter().map(|s| s.unwrap_or(avg)).collect())
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
    let skipped = state.skipped.clone();

    thread::spawn(move || {
        let _inhibitor = SleepInhibitor::new("CNC autoprobe running");
        for idx in 0..total {
            if cancel.load(Ordering::Relaxed) {
                let mut sh = shared.lock();
                sh.error = "cancelled".into();
                sh.finished = true;
                return;
            }
            if skipped.contains(&idx) {
                continue;
            }
            shared.lock().current_index = idx;

            let (x, y) = grid_point(bbox_min, bbox_max, grid_x, grid_y, idx);

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

fn grid_widget(ui: &mut egui::Ui, state: &mut ProbeState) {
    let (samples, current_idx) = {
        let sh = state.shared.lock();
        let cur = if state.phase == ProbePhase::Grid {
            Some(sh.current_index)
        } else {
            None
        };
        (sh.samples.clone(), cur)
    };
    let allow_toggle = state.phase == ProbePhase::Idle;
    let cell_size = 22.0_f32;
    ui.label(
        egui::RichText::new("click to skip / unskip (clamp here)")
            .size(10.0)
            .color(DIM),
    );
    for j in (0..state.grid_y).rev() {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            for i in 0..state.grid_x {
                let idx = (j * state.grid_x + i) as usize;
                let is_skipped = state.skipped.contains(&idx);
                let probed_z = samples.get(idx).copied().flatten();
                let is_active = current_idx == Some(idx);
                let (fill, glyph) = if is_skipped {
                    (egui::Color32::from_rgb(0x66, 0x11, 0x11), "X")
                } else if is_active {
                    (egui::Color32::from_rgb(0x00, 0x33, 0x55), "·")
                } else if probed_z.is_some() {
                    (egui::Color32::from_rgb(0x11, 0x44, 0x22), "·")
                } else {
                    (egui::Color32::from_rgb(0x33, 0x2a, 0x00), "·")
                };
                let text_color = if is_skipped { RED } else { WHITE };
                let btn =
                    egui::Button::new(egui::RichText::new(glyph).size(11.0).color(text_color))
                        .fill(fill)
                        .min_size(egui::vec2(cell_size, cell_size));
                if ui.add_enabled(allow_toggle, btn).clicked() {
                    if is_skipped {
                        state.skipped.remove(&idx);
                    } else {
                        state.skipped.insert(idx);
                    }
                }
            }
        });
    }
}

fn retract(engine: &Arc<Engine>, safe_z: f32) {
    engine.send(&format!("G90 G21 G0 Z{:.3}", safe_z));
}

fn advance_manual(state: &mut ProbeState, engine: &Arc<Engine>, mstate: &MachineState) {
    let idx = state.shared.lock().current_index;
    state.shared.lock().samples[idx] = Some(mstate.wpos.z);
    retract(engine, state.safe_z);
    match next_unskipped(state, idx + 1) {
        Some(next) => {
            state.shared.lock().current_index = next;
            let (x, y) = grid_point(state.bbox_min, state.bbox_max, state.grid_x, state.grid_y, next);
            engine.send(&format!("G90 G21 G0 X{:.3} Y{:.3}", x, y));
        }
        None => {
            state.shared.lock().finished = true;
        }
    }
}

fn abort_manual(state: &mut ProbeState, engine: &Arc<Engine>) {
    retract(engine, state.safe_z);
    state.cancel.store(true, Ordering::Relaxed);
    state.shared.lock().finished = true;
}

fn poll_completion(
    engine: &Arc<Engine>,
    state: &mut ProbeState,
    job_lock: &Arc<RwLock<JobState>>,
) {
    let finished = state.shared.lock().finished;
    if !finished {
        return;
    }

    if state.phase == ProbePhase::Grid {
        let samples = state.shared.lock().samples.clone();
        if let Some(z) = fill_skipped(&samples, &state.skipped) {
            match HeightMap::new(
                state.bbox_min,
                state.bbox_max,
                state.grid_x,
                state.grid_y,
                z,
            ) {
                Ok(map) => {
                    let arc = Arc::new(map);
                    match heightmap::save_cached(&arc) {
                        Ok(()) => engine.log(format!(
                            "heightmap saved ({}x{})",
                            arc.grid_x, arc.grid_y
                        )),
                        Err(e) => engine.log(format!(
                            "!! heightmap save failed (kept in memory): {e}"
                        )),
                    }
                    let mut j = job_lock.write();
                    let line_map_modified =
                        compute_line_map_modified(&j.segments, j.lines.len(), Some(&arc));
                    j.line_map_modified = Arc::new(line_map_modified);
                    j.heightmap = Some(arc);
                    j.transform_cache = None;
                    j.version = j.version.wrapping_add(1);
                }
                Err(e) => engine.log(format!("!! heightmap build failed: {e}")),
            }
        }
    }

    state.phase = ProbePhase::Idle;
    state.shared.lock().finished = false;
}

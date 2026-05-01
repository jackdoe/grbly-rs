use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use parking_lot::RwLock;
use three_d::egui;

use crate::gcode;
use crate::gcode::words::has_word;
use crate::grbl::engine::Engine;
use crate::grbl::state::*;
use crate::ui::scene::MaterialState;

pub struct LoadReport {
    pub line_count: usize,
    pub segment_count: usize,
}

struct PassTaskResult {
    segments: Arc<Vec<Segment>>,
    line_count: usize,
    tolerance_mm: f32,
    seg_pass_counts: Vec<u16>,
    line_pass_counts: Vec<u16>,
}

enum LoadTaskResult {
    Loaded(LoadReport),
    Cancelled,
    Failed(String),
}

fn load_file(path: &Path, job_lock: &RwLock<JobState>) -> Result<LoadReport, String> {
    let content =
        std::fs::read_to_string(path).map_err(|err| format!("Failed to read file: {err}"))?;
    let lines: Vec<String> = content.lines().map(String::from).collect();
    let (segs, bmin, bmax) = gcode::parser::parse_with_bounds(&lines);
    let total_dist: f32 = segs.iter().map(|s| s.start.dist(s.end)).sum();
    let report = LoadReport {
        line_count: lines.len(),
        segment_count: segs.len(),
    };

    let mut j = job_lock.write();
    j.seg_violations = Arc::new(vec![false; segs.len()]);
    j.violated_lines = Arc::new(vec![false; lines.len()]);
    j.seg_pass_counts = Arc::new(vec![1; segs.len()]);
    j.line_pass_counts = Arc::new(vec![1; lines.len()]);
    j.pass_tolerance_mm = 0.0;
    j.lines = Arc::new(lines);
    j.segments = Arc::new(segs);
    j.bounds_min = bmin;
    j.bounds_max = bmax;
    j.total_dist = total_dist;
    j.version = j.version.wrapping_add(1);
    j.status = JobStatus::Idle;
    j.current_line = 0;

    Ok(report)
}

const AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0xaa, 0x00);
const DIM: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);
const GREEN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xff, 0x88);
const RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x44, 0x44);
const CYAN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xcc, 0xff);
const LINE_NUM: egui::Color32 = egui::Color32::from_rgb(0x55, 0x44, 0x22);
const CODE_TEXT: egui::Color32 = egui::Color32::from_rgb(0xcc, 0xaa, 0x66);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EditorFilter {
    #[default]
    All,
    Z,
    Limits,
    Heat,
    Rapids,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SoftLimitValidationKey {
    job_version: usize,
    connected: bool,
    soft_limits: bool,
    wco: Vec3,
    max_travel: Vec3,
}

pub struct EditorState {
    pub simulating: bool,
    pub sim_playing: bool,
    pub sim_seg: usize,
    pub sim_frac: f32,
    pub sim_last_tick: Instant,
    pub warning: String,
    pub sim_pos: Vec3,
    pub sim_feed: f32,
    pub z_locked: bool,
    pub filter: EditorFilter,
    pub sim_line: usize,
    pub jump_line: usize,
    pub pass_tolerance_mm: f32,
    pub show_heatmap: bool,
    pub manual_focus_line: Option<usize>,
    load_receiver: Option<Receiver<LoadTaskResult>>,
    pass_receiver: Option<Receiver<PassTaskResult>>,
    live_start_confirm: Option<Instant>,
    live_started_at: Option<Instant>,
    last_live_status: JobStatus,
    last_soft_limit_validation: SoftLimitValidationKey,
}

impl Default for EditorState {
    fn default() -> Self {
        Self {
            simulating: false,
            sim_playing: false,
            sim_seg: 0,
            sim_frac: 0.0,
            sim_last_tick: Instant::now(),
            warning: String::new(),
            sim_pos: Vec3::default(),
            sim_feed: 20.0,
            z_locked: false,
            filter: EditorFilter::All,
            sim_line: 0,
            jump_line: 1,
            pass_tolerance_mm: DEFAULT_PASS_TOLERANCE_MM,
            show_heatmap: true,
            manual_focus_line: None,
            load_receiver: None,
            pass_receiver: None,
            live_start_confirm: None,
            live_started_at: None,
            last_live_status: JobStatus::Idle,
            last_soft_limit_validation: SoftLimitValidationKey::default(),
        }
    }
}

fn btn(text: &str) -> egui::Button<'_> {
    egui::Button::new(egui::RichText::new(text).size(11.0)).min_size(egui::vec2(0.0, 20.0))
}

fn btn_col(text: &str, text_col: egui::Color32, fill: egui::Color32) -> egui::Button<'_> {
    egui::Button::new(egui::RichText::new(text).size(11.0).color(text_col))
        .fill(fill)
        .min_size(egui::vec2(0.0, 20.0))
}

pub struct DrawArgs<'a> {
    pub engine: &'a Arc<Engine>,
    pub mstate: &'a MachineState,
    pub jstate: &'a JobState,
    pub job_lock: &'a Arc<RwLock<JobState>>,
    pub state: &'a mut EditorState,
    pub material: &'a MaterialState,
}

pub fn draw(ui: &mut egui::Ui, args: DrawArgs<'_>) {
    let DrawArgs {
        engine,
        mstate,
        jstate,
        job_lock,
        state,
        material,
    } = args;

    poll_load_task(ui, state);
    poll_pass_task(ui, state, job_lock);
    sync_validation(state, job_lock, jstate, mstate);
    request_pass_recompute(ui, state, job_lock);
    update_live_timer(state, jstate.status);

    let has_lines = !jstate.lines.is_empty();
    draw_toolbar(
        ui,
        ToolbarArgs {
            engine,
            mstate,
            jstate,
            job_lock,
            state,
            material,
            has_lines,
        },
    );
    advance_simulation(state, jstate, job_lock);
    draw_notices(ui, state);

    if has_lines {
        draw_progress(ui, state, jstate);
        draw_navigation(ui, state, jstate);
    }

    if jstate.lines.is_empty() {
        ui.centered_and_justified(|ui| {
            ui.label(egui::RichText::new("NO FILE LOADED").size(14.0).color(DIM));
        });
        return;
    }

    draw_gcode_lines(ui, state, jstate);
}

pub fn start_load_file(path: PathBuf, job_lock: &Arc<RwLock<JobState>>, state: &mut EditorState) {
    if state.load_receiver.is_some() {
        state.warning = "A file is already loading".into();
        return;
    }

    let (tx, rx) = mpsc::channel();
    let job_lock = job_lock.clone();
    thread::spawn(move || {
        let result = match load_file(&path, &job_lock) {
            Ok(report) => LoadTaskResult::Loaded(report),
            Err(err) => LoadTaskResult::Failed(err),
        };
        let _ = tx.send(result);
    });

    state.load_receiver = Some(rx);
    state.warning = "Loading file...".into();
}

fn start_open_dialog(job_lock: &Arc<RwLock<JobState>>, state: &mut EditorState) {
    if state.load_receiver.is_some() {
        state.warning = "A file is already loading".into();
        return;
    }

    let dialog_task = rfd::AsyncFileDialog::new()
        .add_filter("G-code", &["nc", "gcode", "ngc", "tap", "gc"])
        .pick_file();
    let (tx, rx) = mpsc::channel();
    let job_lock = job_lock.clone();

    thread::spawn(move || {
        let result = match pollster::block_on(dialog_task) {
            Some(file) => match load_file(file.path(), &job_lock) {
                Ok(report) => LoadTaskResult::Loaded(report),
                Err(err) => LoadTaskResult::Failed(err),
            },
            None => LoadTaskResult::Cancelled,
        };
        let _ = tx.send(result);
    });

    state.load_receiver = Some(rx);
    state.warning = "Opening file...".into();
}

fn poll_load_task(ui: &mut egui::Ui, state: &mut EditorState) {
    let Some(rx) = state.load_receiver.take() else {
        return;
    };

    match rx.try_recv() {
        Ok(LoadTaskResult::Loaded(report)) => {
            reset_simulation(state);
            state.manual_focus_line = None;
            state.jump_line = 1;
            state.warning = format!(
                "Loaded {} lines, {} preview segments",
                report.line_count, report.segment_count
            );
        }
        Ok(LoadTaskResult::Cancelled) => {
            state.warning.clear();
        }
        Ok(LoadTaskResult::Failed(err)) => {
            state.warning = err;
        }
        Err(TryRecvError::Empty) => {
            state.load_receiver = Some(rx);
            ui.ctx().request_repaint_after(Duration::from_millis(50));
        }
        Err(TryRecvError::Disconnected) => {
            state.warning = "File load task stopped unexpectedly".into();
        }
    }
}

fn request_pass_recompute(
    ui: &mut egui::Ui,
    state: &mut EditorState,
    job_lock: &Arc<RwLock<JobState>>,
) {
    state.pass_tolerance_mm = normalize_pass_tolerance(state.pass_tolerance_mm);
    if !state.show_heatmap || state.load_receiver.is_some() || state.pass_receiver.is_some() {
        return;
    }

    let job = job_lock.read();
    if job.lines.is_empty() || pass_counts_are_current(&job, state.pass_tolerance_mm) {
        return;
    }

    let segments = job.segments.clone();
    let line_count = job.lines.len();
    let tolerance_mm = state.pass_tolerance_mm;
    drop(job);

    let (tx, rx) = mpsc::channel();
    let task_segments = segments.clone();
    thread::spawn(move || {
        let (seg_pass_counts, line_pass_counts) =
            compute_pass_counts(&task_segments, line_count, tolerance_mm);
        let _ = tx.send(PassTaskResult {
            segments: task_segments,
            line_count,
            tolerance_mm,
            seg_pass_counts,
            line_pass_counts,
        });
    });

    state.pass_receiver = Some(rx);
    state.warning = format!("Updating heatmap tolerance {:.3} mm...", tolerance_mm);
    ui.ctx().request_repaint_after(Duration::from_millis(50));
}

fn poll_pass_task(ui: &mut egui::Ui, state: &mut EditorState, job_lock: &Arc<RwLock<JobState>>) {
    let Some(rx) = state.pass_receiver.take() else {
        return;
    };

    match rx.try_recv() {
        Ok(result) => {
            let desired = normalize_pass_tolerance(state.pass_tolerance_mm);
            let mut job = job_lock.write();
            if Arc::ptr_eq(&job.segments, &result.segments)
                && job.lines.len() == result.line_count
                && (result.tolerance_mm - desired).abs() <= f32::EPSILON
            {
                let changed = job.pass_tolerance_mm != result.tolerance_mm
                    || job.seg_pass_counts.as_ref().as_slice() != result.seg_pass_counts.as_slice()
                    || job.line_pass_counts.as_ref().as_slice()
                        != result.line_pass_counts.as_slice();

                job.seg_pass_counts = Arc::new(result.seg_pass_counts);
                job.line_pass_counts = Arc::new(result.line_pass_counts);
                job.pass_tolerance_mm = result.tolerance_mm;
                if changed {
                    job.version = job.version.wrapping_add(1);
                }
                state.warning = format!("Heatmap updated at {:.3} mm", result.tolerance_mm);
                ui.ctx().request_repaint();
            }
        }
        Err(TryRecvError::Empty) => {
            state.pass_receiver = Some(rx);
            ui.ctx().request_repaint_after(Duration::from_millis(50));
        }
        Err(TryRecvError::Disconnected) => {
            state.warning = "Heatmap task stopped unexpectedly".into();
        }
    }
}

fn sync_validation(
    state: &mut EditorState,
    job_lock: &Arc<RwLock<JobState>>,
    jstate: &JobState,
    mstate: &MachineState,
) {
    state.pass_tolerance_mm = normalize_pass_tolerance(state.pass_tolerance_mm);

    let key = SoftLimitValidationKey {
        job_version: jstate.version,
        connected: mstate.connected,
        soft_limits: mstate.soft_limits,
        wco: mstate.wco,
        max_travel: mstate.max_travel,
    };
    if key != state.last_soft_limit_validation {
        let mut job = job_lock.write();
        recompute_soft_limit_violations(&mut job, mstate);
        state.last_soft_limit_validation = SoftLimitValidationKey {
            job_version: job.version,
            ..key
        };
    }
}

fn update_live_timer(state: &mut EditorState, status: JobStatus) {
    if status == JobStatus::Running && state.last_live_status != JobStatus::Running {
        state.live_started_at = Some(Instant::now());
    }
    if matches!(status, JobStatus::Idle) {
        state.live_started_at = None;
    }
    state.last_live_status = status;
}

struct ToolbarArgs<'a> {
    engine: &'a Arc<Engine>,
    mstate: &'a MachineState,
    jstate: &'a JobState,
    job_lock: &'a Arc<RwLock<JobState>>,
    state: &'a mut EditorState,
    material: &'a MaterialState,
    has_lines: bool,
}

fn draw_toolbar(ui: &mut egui::Ui, args: ToolbarArgs<'_>) {
    let ToolbarArgs {
        engine,
        mstate,
        jstate,
        job_lock,
        state,
        material,
        has_lines,
    } = args;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 3.0;
        ui.label(egui::RichText::new("G-CODE").size(11.0).color(DIM).strong());
        let loading = state.load_receiver.is_some();
        if ui.add_enabled(!loading, btn("OPEN")).clicked() {
            start_open_dialog(job_lock, state);
        }
        if loading {
            ui.label(egui::RichText::new("LOADING").size(10.0).color(DIM));
        }

        ui.separator();
        ui.label(egui::RichText::new("SIM").size(10.0).color(CYAN));
        let play_label = if state.simulating && !state.sim_playing {
            "RESUME"
        } else {
            "PLAY"
        };
        if state.sim_playing {
            if ui
                .add(btn_col(
                    "PAUSE",
                    CYAN,
                    egui::Color32::from_rgb(0x00, 0x22, 0x33),
                ))
                .clicked()
            {
                state.sim_playing = false;
            }
        } else if ui
            .add_enabled(
                has_lines,
                btn_col(play_label, CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33)),
            )
            .clicked()
        {
            if !state.simulating {
                reset_simulation(state);
                state.simulating = true;
            }
            state.sim_playing = true;
            state.sim_last_tick = Instant::now();
        }
        if ui
            .add_enabled(
                has_lines,
                btn_col("STEP", CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33)),
            )
            .clicked()
        {
            if !state.simulating {
                reset_simulation(state);
                state.simulating = true;
            }
            state.sim_playing = false;
            step_simulation_line(state, jstate);
        }
        if ui
            .add_enabled(
                state.simulating,
                btn_col("RESET", CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33)),
            )
            .clicked()
        {
            reset_simulation(state);
        }

        ui.separator();
        let zl_text = if state.z_locked { "ZL ON" } else { "ZL" };
        let zl_fill = if state.z_locked {
            egui::Color32::from_rgb(0x88, 0x00, 0x00)
        } else {
            egui::Color32::from_rgb(0x22, 0x11, 0x11)
        };
        if ui.add(btn_col(zl_text, RED, zl_fill)).clicked() {
            state.z_locked = !state.z_locked;
            let mut j = job_lock.write();
            j.z_locked = state.z_locked;
            j.version = j.version.wrapping_add(1);
        }

        ui.separator();
        ui.label(egui::RichText::new("LIVE").size(10.0).color(RED));
        if ui
            .add_enabled(
                has_lines && jstate.status != JobStatus::Running,
                btn_col("RESET", AMBER, egui::Color32::from_rgb(0x22, 0x11, 0x00)),
            )
            .clicked()
        {
            engine.reset_job();
            state.live_start_confirm = None;
        }
        if ui
            .add_enabled(
                can_live_step(mstate, jstate, has_lines),
                btn_col("STEP", AMBER, egui::Color32::from_rgb(0x22, 0x11, 0x00)),
            )
            .clicked()
        {
            engine.step_line();
        }

        let armed = state
            .live_start_confirm
            .map(|t| t.elapsed() < Duration::from_secs(3))
            .unwrap_or(false);
        let start_label = if armed { "START ARMED" } else { "START" };
        if ui
            .add_enabled(
                can_live_start(mstate, jstate, has_lines),
                btn_col(
                    start_label,
                    GREEN,
                    egui::Color32::from_rgb(0x11, 0x22, 0x00),
                ),
            )
            .clicked()
        {
            if armed {
                state.live_start_confirm = None;
                state.warning.clear();
                engine.reset_job();
                engine.start_job();
            } else {
                state.live_start_confirm = Some(Instant::now());
                state.warning = if mstate.spindle == 0.0 {
                    "START ARMED - spindle speed is 0; click START ARMED to run anyway".into()
                } else {
                    "START ARMED - click START ARMED to stream to machine".into()
                };
            }
        }

        ui.separator();
        ui.label(
            egui::RichText::new(format!(
                "MAT {:.1}x{:.1} T{:.1} O{:.1},{:.1}",
                material.width,
                material.height,
                material.thickness,
                material.offset_x,
                material.offset_y
            ))
            .size(10.0)
            .color(DIM),
        );
    });
}

fn draw_notices(ui: &mut egui::Ui, state: &EditorState) {
    if !state.warning.is_empty() {
        let frame = egui::Frame::default()
            .fill(egui::Color32::from_rgb(0x55, 0x22, 0x00))
            .inner_margin(egui::Margin::same(4.0));
        frame.show(ui, |ui: &mut egui::Ui| {
            ui.label(egui::RichText::new(&state.warning).size(12.0).color(AMBER));
        });
    }
}

fn draw_progress(ui: &mut egui::Ui, state: &mut EditorState, jstate: &JobState) {
    let total = jstate.lines.len().max(1);
    let live_line = jstate.current_line.min(total);
    let live_pct = live_line as f32 / total as f32;

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("LIVE").size(11.0).color(AMBER));
        ui.add(
            egui::ProgressBar::new(live_pct)
                .desired_width(180.0)
                .text(format!("{live_line}/{total}")),
        );
        ui.label(
            egui::RichText::new(format!(
                "{:.0}%  {:.1} mm",
                live_pct * 100.0,
                jstate.total_dist
            ))
            .size(11.0)
            .color(DIM),
        );
        if let Some(started) = state.live_started_at {
            ui.label(
                egui::RichText::new(format!("elapsed {}", fmt_secs(started.elapsed().as_secs())))
                    .size(11.0)
                    .color(DIM),
            );
        }
    });

    if state.simulating {
        let sim_line = state.sim_line.min(total);
        let sim_pct = sim_line as f32 / total as f32;
        let eta_secs = (jstate.total_dist / state.sim_feed).max(0.0) as u64;
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("SIM").size(11.0).color(CYAN));
            ui.add(
                egui::ProgressBar::new(sim_pct)
                    .desired_width(180.0)
                    .text(format!("{sim_line}/{total}")),
            );
            ui.add(
                egui::Slider::new(&mut state.sim_feed, 1.0..=500.0)
                    .suffix(" mm/s")
                    .logarithmic(true),
            );
            ui.label(
                egui::RichText::new(format!("est {}", fmt_secs(eta_secs)))
                    .size(11.0)
                    .color(DIM),
            );
        });
    }
}

fn draw_navigation(ui: &mut egui::Ui, state: &mut EditorState, jstate: &JobState) {
    let n = jstate.lines.len();
    if state.jump_line == 0 {
        state.jump_line = 1;
    }
    if state.jump_line > n {
        state.jump_line = n;
    }

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("VIEW").size(10.0).color(DIM));
        for (label, filter) in [
            ("ALL", EditorFilter::All),
            ("Z", EditorFilter::Z),
            ("LIMIT", EditorFilter::Limits),
            ("HEAT", EditorFilter::Heat),
            ("RAPID", EditorFilter::Rapids),
        ] {
            if ui
                .selectable_label(
                    state.filter == filter,
                    egui::RichText::new(label).size(11.0),
                )
                .clicked()
            {
                state.filter = filter;
            }
        }
        ui.separator();
        ui.label(egui::RichText::new("LINE").size(10.0).color(DIM));
        ui.add(
            egui::DragValue::new(&mut state.jump_line)
                .range(1..=n)
                .speed(1.0),
        );
        if ui.add(btn("GO")).clicked() {
            state.manual_focus_line = Some(state.jump_line.saturating_sub(1).min(n - 1));
        }
        if ui.add(btn("FOLLOW")).clicked() {
            state.manual_focus_line = None;
        }
        ui.separator();
        let tol_fill = if state.show_heatmap {
            egui::Color32::from_rgb(0x33, 0x2a, 0x00)
        } else {
            egui::Color32::from_rgb(0x22, 0x22, 0x33)
        };
        if ui.add(btn_col("TOL", AMBER, tol_fill)).clicked() {
            state.show_heatmap = !state.show_heatmap;
        }
        if state.show_heatmap {
            ui.add(
                egui::DragValue::new(&mut state.pass_tolerance_mm)
                    .range(0.001..=2.0)
                    .speed(0.01)
                    .suffix(" mm"),
            );
        }
    });
}

fn advance_simulation(
    state: &mut EditorState,
    jstate: &JobState,
    _job_lock: &Arc<RwLock<JobState>>,
) {
    if !state.simulating || !state.sim_playing {
        return;
    }

    let dt = state.sim_last_tick.elapsed().as_secs_f32();
    state.sim_last_tick = Instant::now();
    let segments = &jstate.segments;
    let mut remaining = state.sim_feed * dt;

    while remaining > 0.0 && state.sim_seg < segments.len() {
        if jstate
            .seg_violations
            .get(state.sim_seg)
            .copied()
            .unwrap_or(false)
        {
            state.sim_playing = false;
            state.warning = format!(
                "SIM SOFT LIMIT at line {}",
                segments[state.sim_seg].line + 1
            );
            break;
        }
        let seg = &segments[state.sim_seg];
        let seg_len = seg.start.dist(seg.end);
        let left_in_seg = (1.0 - state.sim_frac) * seg_len;

        if seg_len < 0.001 || remaining >= left_in_seg {
            remaining -= left_in_seg;
            state.sim_seg += 1;
            state.sim_frac = 0.0;
        } else {
            state.sim_frac += remaining / seg_len;
            remaining = 0.0;
        }
    }

    if state.sim_seg >= segments.len() {
        state.sim_playing = false;
        state.sim_pos = segments.last().map(|s| s.end).unwrap_or_default();
        state.sim_line = jstate.lines.len();
    } else {
        let seg = &segments[state.sim_seg];
        state.sim_pos = seg.start.lerp(seg.end, state.sim_frac);
        state.sim_line = seg_to_line(segments, state.sim_seg);
    }
    if state.z_locked {
        state.sim_pos.z = 0.0;
    }
}

fn reset_simulation(state: &mut EditorState) {
    state.simulating = false;
    state.sim_playing = false;
    state.sim_seg = 0;
    state.sim_frac = 0.0;
    state.sim_line = 0;
    state.sim_pos = Vec3::default();
    state.sim_last_tick = Instant::now();
}

fn step_simulation_line(state: &mut EditorState, jstate: &JobState) {
    if state.sim_seg < jstate.segments.len() {
        let start_line = jstate.segments[state.sim_seg].line;
        while state.sim_seg < jstate.segments.len()
            && jstate.segments[state.sim_seg].line == start_line
        {
            state.sim_pos = jstate.segments[state.sim_seg].end;
            state.sim_seg += 1;
        }
        state.sim_frac = 0.0;
        state.sim_line = seg_to_line(&jstate.segments, state.sim_seg);
        if state.z_locked {
            state.sim_pos.z = 0.0;
        }
    } else {
        state.sim_line = jstate.lines.len();
    }
}

fn draw_gcode_lines(ui: &mut egui::Ui, state: &mut EditorState, jstate: &JobState) {
    let active_line = if state.simulating {
        state.sim_line
    } else {
        jstate.current_line
    };
    let running =
        state.simulating || jstate.status == JobStatus::Running || jstate.current_line > 0;
    let center_line = state.manual_focus_line.unwrap_or(active_line);

    let visible: Vec<usize> = match state.filter {
        EditorFilter::All => Vec::new(),
        EditorFilter::Z => (0..jstate.lines.len())
            .filter(|&i| has_word(&jstate.lines[i], b'Z'))
            .collect(),
        EditorFilter::Limits => (0..jstate.lines.len())
            .filter(|&i| jstate.violated_lines.get(i).copied().unwrap_or(false))
            .collect(),
        EditorFilter::Heat => (0..jstate.lines.len())
            .filter(|&i| jstate.line_pass_counts.get(i).copied().unwrap_or(1) > 1)
            .collect(),
        EditorFilter::Rapids => (0..jstate.lines.len())
            .filter(|&i| line_has_rapid(jstate, i))
            .collect(),
    };

    let header = match state.filter {
        EditorFilter::All => format!("{} lines  [{}]", jstate.lines.len(), active_line),
        EditorFilter::Z => format!("{} Z-lines  [{}]", visible.len(), active_line),
        EditorFilter::Limits => format!("{} limit lines  [{}]", visible.len(), active_line),
        EditorFilter::Heat => format!("{} repeated lines  [{}]", visible.len(), active_line),
        EditorFilter::Rapids => format!("{} rapid lines  [{}]", visible.len(), active_line),
    };
    ui.label(egui::RichText::new(header).size(11.0).color(DIM));

    let row_h = 17.0;
    if state.filter == EditorFilter::All {
        let n = jstate.lines.len();
        let window_radius = 90;
        let win_start = center_line.saturating_sub(window_radius);
        let win_end = (center_line + window_radius).min(n);

        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .show(ui, |ui| {
                if win_start > 0 {
                    ui.allocate_space(egui::vec2(ui.available_width(), win_start as f32 * row_h));
                }
                for i in win_start..win_end {
                    draw_line_row(ui, state, jstate, i, running, active_line);
                }
                if win_end < n {
                    ui.allocate_space(egui::vec2(
                        ui.available_width(),
                        (n - win_end) as f32 * row_h,
                    ));
                }
            });
    } else {
        egui::ScrollArea::vertical()
            .auto_shrink([false; 2])
            .show(ui, |ui| {
                for i in visible {
                    draw_line_row(ui, state, jstate, i, running, active_line);
                }
            });
    }
}

fn draw_line_row(
    ui: &mut egui::Ui,
    state: &mut EditorState,
    jstate: &JobState,
    i: usize,
    running: bool,
    active_line: usize,
) {
    let is_current = running && i == active_line;
    let is_focused = state.manual_focus_line == Some(i);
    let r = ui.horizontal(|ui| {
        if is_current || is_focused {
            let color = if is_current {
                egui::Color32::from_rgba_unmultiplied(0xff, 0xaa, 0x00, 0x22)
            } else {
                egui::Color32::from_rgba_unmultiplied(0x00, 0xcc, 0xff, 0x18)
            };
            ui.painter()
                .rect_filled(ui.available_rect_before_wrap(), 0.0, color);
        }
        ui.label(
            egui::RichText::new(format!("{:>5}", i + 1))
                .size(12.0)
                .color(LINE_NUM),
        );
        if has_word(&jstate.lines[i], b'Z') {
            ui.label(egui::RichText::new("Z").size(12.0).color(CYAN));
        }
        if line_has_rapid(jstate, i) {
            ui.label(egui::RichText::new("R").size(12.0).color(AMBER));
        }
        let pass_count = jstate.line_pass_counts.get(i).copied().unwrap_or(1);
        if state.show_heatmap && pass_count > 1 {
            let text = egui::RichText::new(format!("x{pass_count}"))
                .size(11.0)
                .color(egui::Color32::BLACK)
                .strong();
            ui.label(text.background_color(heat_badge_color(pass_count)));
        }
        let is_violated = jstate.violated_lines.get(i).copied().unwrap_or(false);
        let line_col = if is_current {
            AMBER
        } else if is_violated {
            RED
        } else {
            CODE_TEXT
        };
        ui.label(
            egui::RichText::new(&jstate.lines[i])
                .size(12.0)
                .color(line_col),
        );
    });
    if r.response.clicked() {
        state.manual_focus_line = Some(i);
        state.jump_line = i + 1;
    }
    if is_current && state.manual_focus_line.is_none() {
        r.response.scroll_to_me(Some(egui::Align::Center));
    }
}

fn can_live_start(mstate: &MachineState, jstate: &JobState, has_lines: bool) -> bool {
    has_lines
        && mstate.connected
        && mstate.status == Status::Idle
        && !matches!(jstate.status, JobStatus::Running | JobStatus::Paused)
}

fn can_live_step(mstate: &MachineState, jstate: &JobState, has_lines: bool) -> bool {
    has_lines
        && mstate.connected
        && mstate.status == Status::Idle
        && !matches!(jstate.status, JobStatus::Running | JobStatus::Paused)
}

fn line_has_rapid(jstate: &JobState, line: usize) -> bool {
    jstate.segments.iter().any(|s| s.line == line && s.rapid)
}

fn heat_badge_color(pass_count: u16) -> egui::Color32 {
    match pass_count {
        0 | 1 => DIM,
        2 => egui::Color32::from_rgb(0xff, 0xdd, 0x33),
        3 => egui::Color32::from_rgb(0xff, 0x99, 0x22),
        4 => egui::Color32::from_rgb(0xff, 0x55, 0x22),
        _ => egui::Color32::from_rgb(0xff, 0x22, 0x66),
    }
}

fn seg_to_line(segments: &[Segment], seg_idx: usize) -> usize {
    if seg_idx < segments.len() {
        segments[seg_idx].line
    } else {
        segments.last().map(|s| s.line + 1).unwrap_or(0)
    }
}

fn fmt_secs(secs: u64) -> String {
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    if h > 0 {
        format!("{h}h {m:02}m {s:02}s")
    } else {
        format!("{m}m {s:02}s")
    }
}

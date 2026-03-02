use std::sync::Arc;
use std::time::Instant;

use parking_lot::RwLock;
use three_d::egui;

use crate::gcode;
use crate::grbl::engine::Engine;
use crate::grbl::state::*;

const AMBER: egui::Color32 = egui::Color32::from_rgb(0xff, 0xaa, 0x00);
const DIM: egui::Color32 = egui::Color32::from_rgb(0x88, 0x77, 0x44);
const GREEN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xff, 0x88);
const RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x44, 0x44);
const CYAN: egui::Color32 = egui::Color32::from_rgb(0x00, 0xcc, 0xff);
const LINE_NUM: egui::Color32 = egui::Color32::from_rgb(0x55, 0x44, 0x22);
const CODE_TEXT: egui::Color32 = egui::Color32::from_rgb(0xcc, 0xaa, 0x66);

pub struct EditorState {
    pub simulating: bool,
    pub sim_playing: bool,
    pub sim_line: usize,
    pub sim_last_advance: Instant,
    pub warning: String,
    pub sim_pos: Vec3,
    pub sim_speed: f32,
    spindle_warn: Option<Instant>,
}

impl Default for EditorState {
    fn default() -> Self {
        Self {
            simulating: false,
            sim_playing: false,
            sim_line: 0,
            sim_last_advance: Instant::now(),
            warning: String::new(),
            sim_pos: Vec3::default(),
            sim_speed: 20.0,
            spindle_warn: None,
        }
    }
}

fn btn(text: &str) -> egui::Button<'_> {
    egui::Button::new(egui::RichText::new(text).size(12.0))
        .min_size(egui::vec2(44.0, 22.0))
}

fn btn_col(text: &str, text_col: egui::Color32, fill: egui::Color32) -> egui::Button<'_> {
    egui::Button::new(egui::RichText::new(text).size(12.0).color(text_col))
        .fill(fill)
        .min_size(egui::vec2(44.0, 22.0))
}

pub fn draw(
    ui: &mut egui::Ui,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    job_lock: &Arc<RwLock<JobState>>,
    state: &mut EditorState,
    profile: &MachineProfile,
) {
    let spindle_warning_active = state.spindle_warn
        .map(|t| t.elapsed().as_secs() < 3)
        .unwrap_or(false);
    if !spindle_warning_active {
        state.spindle_warn = None;
    }

    let has_lines = !jstate.lines.is_empty();

    ui.horizontal(|ui| {
        ui.label(egui::RichText::new("G-CODE").size(14.0).color(DIM).strong());
        ui.add_space(4.0);
        if ui.add(btn("OPEN")).clicked() {
            let job_lock = job_lock.clone();
            std::thread::spawn(move || {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("G-code", &["nc", "gcode", "ngc", "tap", "gc"])
                    .pick_file()
                {
                    if let Ok(content) = std::fs::read_to_string(&path) {
                        let lines: Vec<String> = content.lines().map(String::from).collect();
                        let (segs, bmin, bmax) = gcode::parser::parse_with_bounds(&lines);
                        let mut j = job_lock.write();
                        j.lines = lines;
                        j.segments = segs;
                        j.bounds_min = bmin;
                        j.bounds_max = bmax;
                        j.status = JobStatus::Idle;
                        j.current_line = 0;
                    }
                }
            });
        }
        ui.add_space(6.0);
        ui.label(egui::RichText::new("SIM:").size(11.0).color(CYAN));
        if state.simulating && state.sim_playing {
            if ui.add(btn_col("PAUSE", CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33))).clicked() {
                state.sim_playing = false;
            }
        } else if ui.add(btn_col("PLAY", CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33))).clicked() && has_lines {
            if !state.simulating {
                state.sim_line = 0;
                state.sim_pos = Vec3::default();
                let mut j = job_lock.write();
                j.current_line = 0;
                j.status = JobStatus::Running;
            }
            state.simulating = true;
            state.sim_playing = true;
            state.sim_last_advance = Instant::now();
        }
        if ui.add(btn_col("STEP", CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33))).clicked() && has_lines {
            if !state.simulating {
                state.simulating = true;
                state.sim_line = 0;
                state.sim_pos = Vec3::default();
                let mut j = job_lock.write();
                j.current_line = 0;
                j.status = JobStatus::Running;
            }
            state.sim_playing = false;
            let mut j = job_lock.write();
            if state.sim_line < j.lines.len() {
                state.sim_line += 1;
                j.current_line = state.sim_line;
                state.sim_pos = sim_position_at_line(&j.segments, state.sim_line);
            }
        }
        if ui.add(btn_col("RESET", CYAN, egui::Color32::from_rgb(0x00, 0x22, 0x33))).clicked() && state.simulating {
            state.sim_line = 0;
            state.sim_playing = false;
            state.sim_pos = Vec3::default();
            let mut j = job_lock.write();
            j.current_line = 0;
        }
        if state.simulating {
            if ui.add(btn_col("EXIT", egui::Color32::from_rgb(0x88, 0x88, 0x88), egui::Color32::from_rgb(0x11, 0x11, 0x18))).clicked() {
                state.simulating = false;
                state.sim_playing = false;
                let mut j = job_lock.write();
                j.status = JobStatus::Idle;
                j.current_line = 0;
            }
        }

        ui.add_space(12.0);
        ui.label(egui::RichText::new("LIVE:").size(11.0).color(RED));
        if ui.add(btn_col("RESET", AMBER, egui::Color32::from_rgb(0x22, 0x11, 0x00))).clicked() {
            engine.reset_job();
        }
        if ui.add(btn_col("STEP", AMBER, egui::Color32::from_rgb(0x22, 0x11, 0x00))).clicked() && has_lines {
            engine.step_line();
        }
        if ui.add(btn_col("START", GREEN, egui::Color32::from_rgb(0x11, 0x22, 0x00))).clicked() && has_lines {
            if mstate.spindle == 0.0 && !spindle_warning_active {
                state.spindle_warn = Some(Instant::now());
            } else {
                state.spindle_warn = None;
                engine.start_job();
            }
        }
    });

    if spindle_warning_active {
        let frame = egui::Frame::default()
            .fill(egui::Color32::from_rgb(0x55, 0x11, 0x00))
            .inner_margin(egui::Margin::same(4.0));
        frame.show(ui, |ui: &mut egui::Ui| {
            ui.label(egui::RichText::new("!! SPINDLE NOT RUNNING - CLICK START AGAIN TO OVERRIDE").size(12.0).color(RED));
        });
    }

    if !state.warning.is_empty() {
        let frame = egui::Frame::default()
            .fill(egui::Color32::from_rgb(0x55, 0x22, 0x00))
            .inner_margin(egui::Margin::same(4.0));
        frame.show(ui, |ui: &mut egui::Ui| {
            ui.label(egui::RichText::new(&state.warning).size(12.0).color(AMBER));
        });
    }

    if state.simulating {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("SPEED").size(11.0).color(DIM));
            ui.add(egui::Slider::new(&mut state.sim_speed, 1.0..=200.0).suffix(" ln/s").logarithmic(true));
        });
        if state.sim_playing {
            let interval_ms = (1000.0 / state.sim_speed) as u128;
            if state.sim_last_advance.elapsed().as_millis() >= interval_ms {
                state.sim_last_advance = Instant::now();
                let mut j = job_lock.write();
                if state.sim_line >= j.lines.len() {
                    j.status = JobStatus::Complete;
                    state.sim_playing = false;
                } else {
                    state.sim_line += 1;
                    j.current_line = state.sim_line;
                    state.sim_pos = sim_position_at_line(&j.segments, state.sim_line);
                }
            }
        }
    }

    state.warning = check_bounds(jstate, profile);

    let current = if state.simulating {
        job_lock.read().current_line
    } else {
        jstate.current_line
    };
    let running = state.simulating || jstate.status == JobStatus::Running;

    if jstate.lines.is_empty() {
        ui.centered_and_justified(|ui| {
            ui.label(egui::RichText::new("NO FILE LOADED").size(14.0).color(DIM));
        });
        return;
    }

    ui.label(egui::RichText::new(format!("{} lines  [{}]", jstate.lines.len(), current)).size(11.0).color(DIM));

    let row_h = 17.0;
    let n = jstate.lines.len();
    let window_radius = 80;
    let win_start = current.saturating_sub(window_radius);
    let win_end = (current + window_radius).min(n);

    egui::ScrollArea::vertical()
        .auto_shrink([false; 2])
        .show(ui, |ui| {
            if win_start > 0 {
                ui.allocate_space(egui::vec2(ui.available_width(), win_start as f32 * row_h));
            }

            for i in win_start..win_end {
                let is_current = running && i == current;
                let r = ui.horizontal(|ui| {
                    if is_current {
                        ui.painter().rect_filled(
                            ui.available_rect_before_wrap(),
                            0.0,
                            egui::Color32::from_rgba_unmultiplied(0xff, 0xaa, 0x00, 0x22),
                        );
                    }
                    ui.label(egui::RichText::new(format!("{:>5}", i + 1)).size(12.0).color(LINE_NUM));
                    let line_col = if is_current { AMBER } else { CODE_TEXT };
                    ui.label(egui::RichText::new(&jstate.lines[i]).size(12.0).color(line_col));
                });
                if is_current {
                    r.response.scroll_to_me(Some(egui::Align::Center));
                }
            }

            if win_end < n {
                ui.allocate_space(egui::vec2(ui.available_width(), (n - win_end) as f32 * row_h));
            }
        });
}

fn sim_position_at_line(segments: &[Segment], line: usize) -> Vec3 {
    let mut pos = Vec3::default();
    for seg in segments {
        if seg.line > line { break; }
        pos = seg.end;
    }
    pos
}

fn check_bounds(jstate: &JobState, profile: &MachineProfile) -> String {
    if jstate.segments.is_empty() {
        return String::new();
    }
    let env = profile.envelope;
    let bmin = jstate.bounds_min;
    let bmax = jstate.bounds_max;
    let mut issues = Vec::new();
    if bmax.x > env.x { issues.push(format!("X MAX {:.1} > {:.1}", bmax.x, env.x)); }
    if bmax.y > env.y { issues.push(format!("Y MAX {:.1} > {:.1}", bmax.y, env.y)); }
    if bmin.z < -env.z { issues.push(format!("Z MIN {:.1} < {:.1}", bmin.z, -env.z)); }
    if bmin.x < 0.0 { issues.push(format!("X MIN {:.1} < 0", bmin.x)); }
    if bmin.y < 0.0 { issues.push(format!("Y MIN {:.1} < 0", bmin.y)); }
    if bmax.z > env.z { issues.push(format!("Z MAX {:.1} > {:.1}", bmax.z, env.z)); }
    if issues.is_empty() {
        String::new()
    } else {
        format!("!! OUT OF BOUNDS: {}", issues.join(" | "))
    }
}

mod gcode;
mod grbl;
mod material;
mod ui;

use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::{Mutex, RwLock};
use three_d::*;
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::WindowBuilder;

use grbl::engine::Engine;
use grbl::heightmap;
use grbl::state::*;
use ui::console::LogBuffer;
use ui::probe::ProbeState;
use ui::scene::{ProbePreview, Scene};

fn setup_theme(ctx: &egui::Context) {
    let mut style = (*ctx.style()).clone();
    style.override_text_style = Some(egui::TextStyle::Monospace);
    style.spacing.button_padding = egui::vec2(12.0, 6.0);
    style.spacing.item_spacing = egui::vec2(6.0, 4.0);

    let mut visuals = egui::Visuals::dark();
    let bg = egui::Color32::from_rgb(0x0a, 0x0a, 0x14);
    let panel = egui::Color32::from_rgb(0x10, 0x10, 0x1c);
    let widget_bg = egui::Color32::from_rgb(0x1a, 0x1a, 0x2a);
    let border = egui::Color32::from_rgb(0x33, 0x33, 0x44);
    let amber = egui::Color32::from_rgb(0xff, 0xaa, 0x00);

    visuals.panel_fill = panel;
    visuals.window_fill = panel;
    visuals.extreme_bg_color = bg;
    visuals.faint_bg_color = widget_bg;

    visuals.widgets.noninteractive.bg_fill = widget_bg;
    visuals.widgets.noninteractive.fg_stroke =
        egui::Stroke::new(1.0, egui::Color32::from_rgb(0x99, 0x88, 0x55));
    visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, border);

    visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(0x22, 0x22, 0x33);
    visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, amber);
    visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, border);

    visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(0x33, 0x2a, 0x11);
    visuals.widgets.hovered.fg_stroke =
        egui::Stroke::new(1.0, egui::Color32::from_rgb(0xff, 0xcc, 0x44));
    visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.0, amber);

    visuals.widgets.active.bg_fill = egui::Color32::from_rgb(0x44, 0x33, 0x00);
    visuals.widgets.active.fg_stroke = egui::Stroke::new(1.0, egui::Color32::WHITE);
    visuals.widgets.active.bg_stroke = egui::Stroke::new(1.0, amber);

    visuals.selection.bg_fill = egui::Color32::from_rgb(0x33, 0x2a, 0x00);
    visuals.selection.stroke = egui::Stroke::new(1.0, amber);

    visuals.override_text_color = Some(egui::Color32::from_rgb(0xff, 0xaa, 0x00));

    style.visuals = visuals;
    ctx.set_style(style);
}

fn main() {
    let state = Arc::new(RwLock::new(MachineState::default()));
    let job = Arc::new(RwLock::new(JobState::default()));
    let engine = Arc::new(Engine::new(state.clone(), job.clone()));
    let log = Arc::new(Mutex::new(LogBuffer::new()));

    {
        let log_clone = log.clone();
        engine.set_on_log(move |line| {
            log_clone.lock().add(line);
        });
    }

    match heightmap::load_cached() {
        Ok(Some(map)) => {
            let grid_x = map.grid_x;
            let grid_y = map.grid_y;
            let arc = Arc::new(map);
            let mut j = job.write();
            let line_map_modified =
                compute_line_map_modified(&j.segments, j.lines.len(), Some(&arc));
            j.line_map_modified = Arc::new(line_map_modified);
            j.heightmap = Some(arc);
            j.transform_cache = None;
            j.version = j.version.wrapping_add(1);
            drop(j);
            engine.log(format!("heightmap loaded from cache ({grid_x}x{grid_y})"));
        }
        Ok(None) => {}
        Err(e) => {
            engine.log(format!("!! heightmap cache load failed: {e}"));
        }
    }

    let event_loop = EventLoop::new();
    let winit_window = WindowBuilder::new()
        .with_title("Grbly")
        .with_inner_size(winit::dpi::LogicalSize::new(1920.0, 1080.0))
        .with_max_inner_size(winit::dpi::LogicalSize::new(1920.0, 1080.0))
        .with_min_inner_size(winit::dpi::LogicalSize::new(2.0, 2.0))
        .build(&event_loop)
        .unwrap();
    winit_window.focus_window();

    let gl = WindowedContext::from_winit_window(&winit_window, SurfaceSettings::default()).unwrap();
    let context: Context = (*gl).clone();

    let viewport = {
        let (w, h): (u32, u32) = winit_window.inner_size().into();
        Viewport::new_at_origo(w, h)
    };

    let mut camera = Camera::new_perspective(
        viewport,
        vec3(200.0, -150.0, 150.0),
        vec3(75.0, 55.0, 20.0),
        vec3(0.0, 0.0, 1.0),
        degrees(45.0),
        0.1,
        10000.0,
    );

    let mut gui = three_d::GUI::new(&context);
    let mut scene = Scene::new(&context);

    let mut ui_state = ui::app::UiState::default();

    let mut camera_controller = ui::camera::CameraController::default();

    let mut frame_input_generator = FrameInputGenerator::from_winit_window(&winit_window);

    event_loop.run(move |event, _, control_flow| match event {
        winit::event::Event::MainEventsCleared => {
            winit_window.request_redraw();
        }
        winit::event::Event::RedrawRequested(_) => {
            let mut frame_input = frame_input_generator.generate(&gl);

            camera_controller.handle_events(&mut frame_input.events, &mut camera);

            let mstate = state.read().clone();
            let jstate = job.read().clone();

            gui.update(
                &mut frame_input.events,
                frame_input.accumulated_time,
                frame_input.viewport,
                frame_input.device_pixel_ratio,
                |ctx| {
                    if !ui_state.theme_set {
                        setup_theme(ctx);
                        ui_state.theme_set = true;
                    }

                    ui::controls::draw(
                        ctx,
                        &engine,
                        &mstate,
                        &jstate,
                        &job,
                        &mut ui_state.controls,
                        &mut ui_state.probe,
                    );

                    egui::TopBottomPanel::bottom("bottom_panels")
                        .resizable(true)
                        .default_height(250.0)
                        .show(ctx, |ui| {
                            ui.columns(2, |cols| {
                                ui::editor::draw(
                                    &mut cols[0],
                                    ui::editor::DrawArgs {
                                        engine: &engine,
                                        mstate: &mstate,
                                        jstate: &jstate,
                                        job_lock: &job,
                                        state: &mut ui_state.editor,
                                    },
                                );
                                ui::console::draw(
                                    &mut cols[1],
                                    &engine,
                                    &log,
                                    &mut ui_state.console,
                                );
                            });
                        });

                    handle_keyboard(ctx, &engine, &mstate, &jstate, ui_state.controls.jog_step);
                },
            );

            camera_controller.handle_wheel(&mut frame_input.events, &mut camera);

            camera.set_viewport(frame_input.viewport);

            let tool_pos = if ui_state.editor.simulating {
                ui_state.editor.sim_pos
            } else {
                mstate.wpos
            };
            let material = if ui_state.editor.simulating {
                ui_state.editor.material.as_ref()
            } else {
                None
            };
            scene.update(ui::scene::SceneUpdate {
                context: &context,
                tool_pos,
                mstate: &mstate,
                jstate: &jstate,
                show_heatmap: ui_state.editor.show_heatmap,
                probe_preview: build_probe_preview(&jstate, &ui_state.probe),
                material,
                visibility: ui_state.editor.visibility,
            });

            let objects = scene.collect();
            frame_input
                .screen()
                .clear(ClearState::color_and_depth(0.03, 0.03, 0.06, 1.0, 1.0))
                .render(&camera, objects, &[]);

            let _ = gui.render();
            gl.swap_buffers().unwrap();

            *control_flow = ControlFlow::WaitUntil(Instant::now() + Duration::from_millis(16));
        }
        winit::event::Event::WindowEvent { ref event, .. } => {
            frame_input_generator.handle_winit_window_event(event);
            match event {
                winit::event::WindowEvent::Resized(physical_size) => {
                    gl.resize(*physical_size);
                }
                winit::event::WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                    gl.resize(**new_inner_size);
                }
                winit::event::WindowEvent::CloseRequested => {
                    *control_flow = ControlFlow::Exit;
                }
                winit::event::WindowEvent::DroppedFile(path) => {
                    ui::editor::start_load_file(path.clone(), &job, &mut ui_state.editor);
                    winit_window.request_redraw();
                }
                _ => {}
            }
        }
        _ => {}
    });
}

fn build_probe_preview(jstate: &JobState, probe: &ProbeState) -> Option<ProbePreview> {
    let bbox_valid = jstate.bounds_max.x > jstate.bounds_min.x
        && jstate.bounds_max.y > jstate.bounds_min.y;
    if !bbox_valid {
        return None;
    }
    let (bbox_min, bbox_max, gx, gy, samples, current_index, skipped) =
        if let Some(map) = &jstate.heightmap {
            let total = (map.grid_x * map.grid_y) as usize;
            (
                map.bbox_min,
                map.bbox_max,
                map.grid_x,
                map.grid_y,
                Some(map.z.iter().map(|z| Some(*z)).collect()),
                None,
                vec![false; total],
            )
        } else {
            let (s, ci, skip) = probe.samples_snapshot();
            (
                (jstate.bounds_min.x, jstate.bounds_min.y),
                (jstate.bounds_max.x, jstate.bounds_max.y),
                probe.grid_x,
                probe.grid_y,
                if s.is_empty() { None } else { Some(s) },
                ci,
                skip,
            )
        };
    Some(ProbePreview {
        bbox_min,
        bbox_max,
        grid_x: gx,
        grid_y: gy,
        samples,
        current_index,
        skipped,
    })
}

fn handle_keyboard(
    ctx: &egui::Context,
    engine: &Arc<Engine>,
    mstate: &MachineState,
    jstate: &JobState,
    jog_step: f32,
) {
    if ctx.wants_keyboard_input() {
        return;
    }
    let can_jog = mstate.connected && matches!(mstate.status, Status::Idle | Status::Jog);
    ctx.input(|input| {
        if can_jog && input.key_pressed(egui::Key::ArrowLeft) {
            engine.send(&format!("$J=G91 G21 X-{:.1} F1000", jog_step));
        }
        if can_jog && input.key_pressed(egui::Key::ArrowRight) {
            engine.send(&format!("$J=G91 G21 X{:.1} F1000", jog_step));
        }
        if can_jog && input.key_pressed(egui::Key::ArrowUp) {
            engine.send(&format!("$J=G91 G21 Y{:.1} F1000", jog_step));
        }
        if can_jog && input.key_pressed(egui::Key::ArrowDown) {
            engine.send(&format!("$J=G91 G21 Y-{:.1} F1000", jog_step));
        }
        if can_jog && input.key_pressed(egui::Key::PageUp) {
            engine.send(&format!("$J=G91 G21 Z{:.1} F500", jog_step));
        }
        if can_jog && input.key_pressed(egui::Key::PageDown) {
            engine.send(&format!("$J=G91 G21 Z-{:.1} F500", jog_step));
        }
        if input.key_pressed(egui::Key::Space) {
            match jstate.status {
                JobStatus::Running => engine.pause_job(),
                JobStatus::Paused => engine.resume_job(),
                _ => {}
            }
        }
    });
}

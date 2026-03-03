mod grbl;
mod gcode;
mod ui;

use std::sync::Arc;

use parking_lot::{Mutex, RwLock};
use three_d::*;
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::WindowBuilder;

use grbl::engine::Engine;
use grbl::state::*;
use ui::console::LogBuffer;
use ui::scene::Scene;

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
    visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(0x99, 0x88, 0x55));
    visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, border);

    visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(0x22, 0x22, 0x33);
    visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0, amber);
    visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, border);

    visuals.widgets.hovered.bg_fill = egui::Color32::from_rgb(0x33, 0x2a, 0x11);
    visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(0xff, 0xcc, 0x44));
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
    let mut scene = Scene::new(&context, &CUBIKO);

    let mut controls_state = ui::controls::ControlsState::default();
    let mut editor_state = ui::editor::EditorState::default();
    let mut console_state = ui::console::ConsoleState::default();
    let mut theme_set = false;

    #[derive(Clone, Copy, PartialEq)]
    enum CamDrag { None, Orbit, Zoom, Pan }
    let mut cam_drag = CamDrag::None;

    let mut frame_input_generator = FrameInputGenerator::from_winit_window(&winit_window);

    event_loop.run(move |event, _, control_flow| match event {
        winit::event::Event::MainEventsCleared => {
            winit_window.request_redraw();
        }
        winit::event::Event::RedrawRequested(_) => {
            let mut frame_input = frame_input_generator.generate(&gl);

            let active_drag = cam_drag;
            for event in &mut frame_input.events {
                match event {
                    three_d::Event::MousePress { button, modifiers, handled, .. }
                        if *button == three_d::MouseButton::Left && (modifiers.ctrl || modifiers.command) =>
                    {
                        cam_drag = CamDrag::Pan;
                        *handled = true;
                    }
                    three_d::Event::MousePress { button, modifiers, handled, .. }
                        if *button == three_d::MouseButton::Right && (modifiers.ctrl || modifiers.command) =>
                    {
                        cam_drag = CamDrag::Zoom;
                        *handled = true;
                    }
                    three_d::Event::MousePress { button, handled, .. }
                        if *button == three_d::MouseButton::Right =>
                    {
                        cam_drag = CamDrag::Orbit;
                        *handled = true;
                    }
                    three_d::Event::MouseRelease { button, handled, .. }
                        if (*button == three_d::MouseButton::Left && active_drag == CamDrag::Pan)
                        || (*button == three_d::MouseButton::Right && active_drag != CamDrag::Pan) =>
                    {
                        cam_drag = CamDrag::None;
                        *handled = true;
                    }
                    three_d::Event::MouseMotion { delta, handled, .. } if active_drag == CamDrag::Pan => {
                        let pos = camera.position();
                        let tgt = camera.target();
                        let up = camera.up();
                        let fwd = (tgt - pos).normalize();
                        let speed = pos.distance(tgt) * 0.002;
                        let right = fwd.cross(up).normalize();
                        let cam_up = right.cross(fwd);
                        let offset = right * (-delta.0 as f32 * speed) + cam_up * (delta.1 as f32 * speed);
                        camera.set_view(pos + offset, tgt + offset, up);
                        *handled = true;
                    }
                    three_d::Event::MouseMotion { delta, handled, .. } if active_drag == CamDrag::Orbit => {
                        let pos = camera.position();
                        let tgt = camera.target();
                        let off = pos - tgt;
                        let dist = off.magnitude();
                        let theta = off.y.atan2(off.x) - delta.0 as f32 * 0.005;
                        let phi = (off.z / dist).acos() - delta.1 as f32 * 0.005;
                        let phi = phi.clamp(0.05, std::f32::consts::PI - 0.05);
                        let new_off = vec3(
                            dist * phi.sin() * theta.cos(),
                            dist * phi.sin() * theta.sin(),
                            dist * phi.cos(),
                        );
                        camera.set_view(tgt + new_off, tgt, vec3(0.0, 0.0, 1.0));
                        *handled = true;
                    }
                    three_d::Event::MouseMotion { delta, handled, .. } if active_drag == CamDrag::Zoom => {
                        let pos = camera.position();
                        let tgt = camera.target();
                        let up = camera.up();
                        let dist = pos.distance(tgt);
                        let factor = 1.0 - delta.1 as f32 * 0.005;
                        let new_dist = (dist * factor).clamp(1.0, 10000.0);
                        let fwd = (tgt - pos).normalize();
                        camera.set_view(tgt - fwd * new_dist, tgt, up);
                        *handled = true;
                    }
                    _ => {}
                }
            }

            let mstate = state.read().clone();
            let jstate = job.read().clone();

            gui.update(
                &mut frame_input.events,
                frame_input.accumulated_time,
                frame_input.viewport,
                frame_input.device_pixel_ratio,
                |ctx| {
                    if !theme_set {
                        setup_theme(ctx);
                        theme_set = true;
                    }

                    ui::controls::draw(ctx, &engine, &mstate, &jstate, &mut controls_state);

                    egui::TopBottomPanel::bottom("bottom_panels")
                        .resizable(true)
                        .default_height(250.0)
                        .show(ctx, |ui| {
                            ui.columns(2, |cols| {
                                ui::editor::draw(&mut cols[0], &engine, &mstate, &jstate, &job, &mut editor_state, &CUBIKO);
                                ui::console::draw(&mut cols[1], &engine, &log, &mut console_state);
                            });
                        });

                    handle_keyboard(ctx, &engine, &jstate, controls_state.jog_step);
                },
            );

            for event in &mut frame_input.events {
                if let three_d::Event::MouseWheel { delta, handled, .. } = event {
                    if !*handled {
                        let pos = camera.position();
                        let tgt = camera.target();
                        let up = camera.up();
                        let dist = pos.distance(tgt);
                        let factor = 1.0 - delta.1 as f32 * 0.001;
                        let new_dist = (dist * factor).clamp(1.0, 10000.0);
                        let fwd = (tgt - pos).normalize();
                        camera.set_view(tgt - fwd * new_dist, tgt, up);
                        *handled = true;
                    }
                }
            }

            camera.set_viewport(frame_input.viewport);

            let tool_pos = if editor_state.simulating {
                editor_state.sim_pos
            } else {
                mstate.wpos
            };
            scene.update(&context, tool_pos, &jstate, &CUBIKO);

            let objects = scene.collect();
            frame_input
                .screen()
                .clear(ClearState::color_and_depth(0.03, 0.03, 0.06, 1.0, 1.0))
                .render(&camera, objects, &[]);

            let _ = gui.render();
            gl.swap_buffers().unwrap();

            *control_flow = ControlFlow::Poll;
            winit_window.request_redraw();
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
                    ui::editor::load_file(path, &job);
                    let size = winit_window.inner_size();
                    #[allow(deprecated)]
                    let fake = winit::event::WindowEvent::CursorMoved {
                        device_id: unsafe { std::mem::zeroed() },
                        position: winit::dpi::PhysicalPosition::new(
                            size.width as f64 / 2.0,
                            size.height as f64 * 0.75,
                        ),
                        modifiers: winit::event::ModifiersState::default(),
                    };
                    frame_input_generator.handle_winit_window_event(&fake);
                }
                _ => {}
            }
        }
        _ => {}
    });
}

fn handle_keyboard(
    ctx: &egui::Context,
    engine: &Arc<Engine>,
    jstate: &JobState,
    jog_step: f32,
) {
    if ctx.wants_keyboard_input() {
        return;
    }
    ctx.input(|input| {
        if input.key_pressed(egui::Key::ArrowLeft) {
            engine.send(&format!("$J=G91 G21 X-{:.1} F1000", jog_step));
        }
        if input.key_pressed(egui::Key::ArrowRight) {
            engine.send(&format!("$J=G91 G21 X{:.1} F1000", jog_step));
        }
        if input.key_pressed(egui::Key::ArrowUp) {
            engine.send(&format!("$J=G91 G21 Y{:.1} F1000", jog_step));
        }
        if input.key_pressed(egui::Key::ArrowDown) {
            engine.send(&format!("$J=G91 G21 Y-{:.1} F1000", jog_step));
        }
        if input.key_pressed(egui::Key::PageUp) {
            engine.send(&format!("$J=G91 G21 Z{:.1} F500", jog_step));
        }
        if input.key_pressed(egui::Key::PageDown) {
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

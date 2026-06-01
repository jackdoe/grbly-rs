use three_d::*;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum DragMode {
    #[default]
    None,
    Orbit,
    Pan,
}

#[derive(Default)]
pub struct CameraController {
    drag: DragMode,
}

impl CameraController {
    pub fn handle_events(&mut self, events: &mut [Event], camera: &mut Camera) {
        let active_drag = self.drag;
        for event in events {
            match event {
                Event::MousePress {
                    button,
                    modifiers,
                    handled,
                    ..
                } if !*handled => {
                    match button {
                        MouseButton::Left if modifiers.shift => self.drag = DragMode::Pan,
                        MouseButton::Left => self.drag = DragMode::Orbit,
                        MouseButton::Right => self.drag = DragMode::Pan,
                        _ => {}
                    }
                    if self.drag != DragMode::None {
                        *handled = true;
                    }
                }
                Event::MouseRelease {
                    button, handled, ..
                } if active_drag != DragMode::None
                    && matches!(button, MouseButton::Left | MouseButton::Right) =>
                {
                    self.drag = DragMode::None;
                    *handled = true;
                }
                Event::MouseMotion { delta, handled, .. } if active_drag == DragMode::Orbit => {
                    orbit(camera, *delta);
                    *handled = true;
                }
                Event::MouseMotion { delta, handled, .. } if active_drag == DragMode::Pan => {
                    pan(camera, *delta);
                    *handled = true;
                }
                _ => {}
            }
        }
    }

    pub fn handle_wheel(&mut self, events: &mut [Event], camera: &mut Camera) {
        for event in events {
            if let Event::MouseWheel { delta, handled, .. } = event {
                if !*handled {
                    zoom(camera, delta.1, 0.001);
                    *handled = true;
                }
            }
        }
    }
}

fn pan(camera: &mut Camera, delta: (f32, f32)) {
    let pos = camera.position();
    let tgt = camera.target();
    let up = camera.up();
    let fwd = (tgt - pos).normalize();
    let speed = pos.distance(tgt) * 0.002;
    let right = fwd.cross(up).normalize();
    let cam_up = right.cross(fwd);
    let offset = right * (-delta.0 * speed) + cam_up * (delta.1 * speed);
    camera.set_view(pos + offset, tgt + offset, up);
}

fn orbit(camera: &mut Camera, delta: (f32, f32)) {
    let pos = camera.position();
    let tgt = camera.target();
    let off = pos - tgt;
    let dist = off.magnitude();
    let theta = off.y.atan2(off.x) - delta.0 * 0.005;
    let phi = (off.z / dist).acos() - delta.1 * 0.005;
    let phi = phi.clamp(0.05, std::f32::consts::PI - 0.05);
    let new_off = vec3(
        dist * phi.sin() * theta.cos(),
        dist * phi.sin() * theta.sin(),
        dist * phi.cos(),
    );
    camera.set_view(tgt + new_off, tgt, vec3(0.0, 0.0, 1.0));
}

fn zoom(camera: &mut Camera, delta_y: f32, sensitivity: f32) {
    let pos = camera.position();
    let tgt = camera.target();
    let up = camera.up();
    let dist = pos.distance(tgt);
    let factor = 1.0 - delta_y * sensitivity;
    let new_dist = (dist * factor).clamp(1.0, 10000.0);
    let fwd = (tgt - pos).normalize();
    camera.set_view(tgt - fwd * new_dist, tgt, up);
}

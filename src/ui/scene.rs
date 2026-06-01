use crate::grbl::heightmap::grid_point;
use crate::grbl::state::{self, JobState, MachineState, Segment};
use crate::material::MaterialField;
use three_d::renderer::*;

type V3 = state::Vec3;

pub struct ProbePreview {
    pub bbox_min: (f32, f32),
    pub bbox_max: (f32, f32),
    pub grid_x: u32,
    pub grid_y: u32,
    pub samples: Option<Vec<Option<f32>>>,
    pub current_index: Option<usize>,
    pub skipped: Vec<bool>,
}

#[derive(Clone, Copy, Debug)]
pub struct SceneVisibility {
    pub grid: bool,
    pub triad: bool,
    pub machine_box: bool,
    pub toolpath: bool,
    pub bounds_box: bool,
    pub probe_points: bool,
    pub trail: bool,
    pub gantry: bool,
    pub material: bool,
}

impl Default for SceneVisibility {
    fn default() -> Self {
        Self {
            grid: true,
            triad: true,
            machine_box: true,
            toolpath: true,
            bounds_box: true,
            probe_points: true,
            trail: true,
            gantry: true,
            material: true,
        }
    }
}

pub struct Scene {
    grid: Gm<Mesh, ColorMaterial>,
    triad: Gm<Mesh, ColorMaterial>,
    machine_box: Option<Gm<Mesh, ColorMaterial>>,
    toolpath: Option<Gm<Mesh, ColorMaterial>>,
    gantry: Option<Gm<Mesh, ColorMaterial>>,
    trail: Option<Gm<Mesh, ColorMaterial>>,
    bounds_box: Option<Gm<Mesh, ColorMaterial>>,
    probe_points: Option<Gm<Mesh, ColorMaterial>>,
    material: Option<Gm<Mesh, ColorMaterial>>,
    visibility: SceneVisibility,
    trail_points: Vec<V3>,
    trail_dirty: bool,
    last_pos: V3,
    last_z_clearance: f32,
    last_version: usize,
    last_connected: bool,
    last_wco: V3,
    last_max_travel: V3,
    last_show_heatmap: bool,
    last_probe_signature: ProbeSignature,
    last_material_version: Option<u32>,
    last_trail_reset: u64,
}

#[derive(Clone, Default, PartialEq)]
struct ProbeSignature {
    bbox_min: (f32, f32),
    bbox_max: (f32, f32),
    grid_x: u32,
    grid_y: u32,
    samples: Vec<Option<f32>>,
    current_index: Option<usize>,
    skipped: Vec<bool>,
}

pub struct SceneUpdate<'a> {
    pub context: &'a Context,
    pub tool_pos: V3,
    pub mstate: &'a MachineState,
    pub jstate: &'a JobState,
    pub show_heatmap: bool,
    pub probe_preview: Option<ProbePreview>,
    pub material: Option<&'a MaterialField>,
    pub visibility: SceneVisibility,
    pub trail_reset: u64,
}

const LINE_W: f32 = 0.3;
const GRID_W: f32 = 0.15;
const THICK_W: f32 = 0.5;

const CUBIKO_TRAVEL_X: f32 = 150.0;
const CUBIKO_TRAVEL_Y: f32 = 110.0;
const CUBIKO_TRAVEL_Z: f32 = 40.0;

fn v3(v: V3) -> Vector3<f32> {
    vec3(v.x, v.y, v.z)
}

impl Scene {
    pub fn new(context: &Context) -> Self {
        Self {
            grid: build_grid(context, 200.0, 200.0),
            triad: build_axes(context, 200.0),
            machine_box: None,
            toolpath: None,
            gantry: None,
            trail: None,
            bounds_box: None,
            probe_points: None,
            material: None,
            visibility: SceneVisibility::default(),
            trail_points: Vec::new(),
            trail_dirty: false,
            last_pos: V3::default(),
            last_z_clearance: 0.0,
            last_version: 0,
            last_connected: false,
            last_wco: V3::default(),
            last_max_travel: V3::default(),
            last_show_heatmap: true,
            last_probe_signature: ProbeSignature::default(),
            last_material_version: None,
            last_trail_reset: 0,
        }
    }

    pub fn update(&mut self, args: SceneUpdate<'_>) {
        let SceneUpdate {
            context,
            tool_pos,
            mstate,
            jstate,
            show_heatmap,
            probe_preview,
            material,
            visibility,
            trail_reset,
        } = args;
        self.visibility = visibility;

        let wpos = tool_pos;
        if trail_reset != self.last_trail_reset {
            self.last_trail_reset = trail_reset;
            self.trail_points.clear();
            self.trail_points.push(wpos);
            self.last_pos = wpos;
            self.trail = None;
            self.trail_dirty = false;
        }
        let wpos_changed = wpos != self.last_pos;
        if wpos_changed {
            self.trail_points.push(wpos);
            if self.trail_points.len() > 5000 {
                let drain = self.trail_points.len() - 5000;
                self.trail_points.drain(..drain);
            }
            self.last_pos = wpos;
            self.trail_dirty = true;
        }

        let wco = mstate.wco;
        let z_clearance = if wco.z.abs() > 0.01 { -wco.z } else { 30.0 };
        let z_clearance_changed = (z_clearance - self.last_z_clearance).abs() > f32::EPSILON;
        if self.gantry.is_none() || wpos_changed || z_clearance_changed {
            self.gantry = Some(build_gantry(context, wpos, z_clearance));
            self.last_z_clearance = z_clearance;
        }

        if self.trail_dirty && self.trail_points.len() >= 2 {
            self.trail = Some(build_trail(context, &self.trail_points));
            self.trail_dirty = false;
        }

        if jstate.version != self.last_version || show_heatmap != self.last_show_heatmap {
            self.last_version = jstate.version;
            self.last_show_heatmap = show_heatmap;
            if !jstate.segments.is_empty() {
                self.toolpath = Some(build_toolpath(
                    context,
                    &jstate.segments,
                    &jstate.seg_violations,
                    &jstate.seg_pass_counts,
                    jstate.bounds_min,
                    jstate.bounds_max,
                    show_heatmap,
                ));
                self.bounds_box = Some(build_wire_box(
                    context,
                    jstate.bounds_min,
                    jstate.bounds_max,
                    Srgba::new(0x55, 0x55, 0x77, 0x66),
                    GRID_W,
                ));
            } else {
                self.toolpath = None;
                self.bounds_box = None;
            }
        }

        let mt = mstate.max_travel;
        let connected_changed = mstate.connected != self.last_connected;
        if connected_changed || wco != self.last_wco || mt != self.last_max_travel {
            self.last_connected = mstate.connected;
            self.last_wco = wco;
            self.last_max_travel = mt;
            if mstate.connected {
                let tx = if mt.x > 0.0 { mt.x } else { CUBIKO_TRAVEL_X };
                let ty = if mt.y > 0.0 { mt.y } else { CUBIKO_TRAVEL_Y };
                let tz = if mt.z > 0.0 { mt.z } else { CUBIKO_TRAVEL_Z };
                let home_w = V3 {
                    x: -wco.x,
                    y: -wco.y,
                    z: -wco.z,
                };
                let far_w = V3 {
                    x: tx - wco.x,
                    y: ty - wco.y,
                    z: -tz - wco.z,
                };
                let bmin = V3 {
                    x: home_w.x.min(far_w.x),
                    y: home_w.y.min(far_w.y),
                    z: home_w.z.min(far_w.z),
                };
                let bmax = V3 {
                    x: home_w.x.max(far_w.x),
                    y: home_w.y.max(far_w.y),
                    z: home_w.z.max(far_w.z),
                };
                self.machine_box = Some(build_wire_box(
                    context,
                    bmin,
                    bmax,
                    Srgba::new(0x44, 0x77, 0xbb, 0x99),
                    LINE_W,
                ));
            } else {
                self.machine_box = None;
            }
        }

        let new_sig = match &probe_preview {
            None => ProbeSignature::default(),
            Some(p) => ProbeSignature {
                bbox_min: p.bbox_min,
                bbox_max: p.bbox_max,
                grid_x: p.grid_x,
                grid_y: p.grid_y,
                samples: p.samples.clone().unwrap_or_default(),
                current_index: p.current_index,
                skipped: p.skipped.clone(),
            },
        };
        if new_sig != self.last_probe_signature {
            self.last_probe_signature = new_sig;
            self.probe_points = probe_preview.as_ref().and_then(|p| build_probe_points(context, p));
        }

        let mat_version = material.map(|m| m.version);
        if mat_version != self.last_material_version {
            self.last_material_version = mat_version;
            self.material = material.map(|m| build_material(context, m));
        }
    }

    pub fn collect(&self) -> Vec<&Gm<Mesh, ColorMaterial>> {
        let v = &self.visibility;
        let mut out: Vec<&Gm<Mesh, ColorMaterial>> = Vec::new();
        if v.grid {
            out.push(&self.grid);
        }
        if v.triad {
            out.push(&self.triad);
        }
        if v.material {
            if let Some(ref m) = self.material {
                out.push(m);
            }
        }
        if v.machine_box {
            if let Some(ref m) = self.machine_box {
                out.push(m);
            }
        }
        if v.toolpath {
            if let Some(ref m) = self.toolpath {
                out.push(m);
            }
        }
        if v.bounds_box {
            if let Some(ref m) = self.bounds_box {
                out.push(m);
            }
        }
        if v.probe_points {
            if let Some(ref m) = self.probe_points {
                out.push(m);
            }
        }
        if v.trail {
            if let Some(ref m) = self.trail {
                out.push(m);
            }
        }
        if v.gantry {
            if let Some(ref m) = self.gantry {
                out.push(m);
            }
        }
        out
    }
}

struct LineBuilder {
    positions: Vec<Vector3<f32>>,
    colors: Vec<Srgba>,
    indices: Vec<u32>,
}

impl LineBuilder {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
            colors: Vec::new(),
            indices: Vec::new(),
        }
    }

    fn add(&mut self, start: V3, end: V3, color: Srgba, width: f32) {
        let s = v3(start);
        let e = v3(end);
        let dir = e - s;
        let up = vec3(0.0, 0.0, 1.0);
        let mut perp = dir.cross(up);
        if perp.magnitude() < 0.0001 {
            perp = dir.cross(vec3(1.0, 0.0, 0.0));
        }
        if perp.magnitude() < 0.0001 {
            return;
        }
        perp = perp.normalize() * width * 0.5;

        let base = self.positions.len() as u32;
        self.positions.push(s + perp);
        self.positions.push(s - perp);
        self.positions.push(e + perp);
        self.positions.push(e - perp);
        self.colors.extend([color; 4]);
        self.indices
            .extend([base, base + 1, base + 2, base + 1, base + 3, base + 2]);
    }

    fn build(self, context: &Context) -> Gm<Mesh, ColorMaterial> {
        let cpu_mesh = CpuMesh {
            positions: Positions::F32(self.positions),
            indices: Indices::U32(self.indices),
            colors: Some(self.colors),
            ..Default::default()
        };
        let mesh = Mesh::new(context, &cpu_mesh);
        let material = ColorMaterial {
            color: Srgba::WHITE,
            is_transparent: true,
            render_states: RenderStates {
                blend: Blend::TRANSPARENCY,
                ..Default::default()
            },
            ..Default::default()
        };
        Gm::new(mesh, material)
    }
}

fn build_wire_box(
    context: &Context,
    bmin: V3,
    bmax: V3,
    color: Srgba,
    width: f32,
) -> Gm<Mesh, ColorMaterial> {
    let c = [
        V3 {
            x: bmin.x,
            y: bmin.y,
            z: bmin.z,
        },
        V3 {
            x: bmax.x,
            y: bmin.y,
            z: bmin.z,
        },
        V3 {
            x: bmax.x,
            y: bmax.y,
            z: bmin.z,
        },
        V3 {
            x: bmin.x,
            y: bmax.y,
            z: bmin.z,
        },
        V3 {
            x: bmin.x,
            y: bmin.y,
            z: bmax.z,
        },
        V3 {
            x: bmax.x,
            y: bmin.y,
            z: bmax.z,
        },
        V3 {
            x: bmax.x,
            y: bmax.y,
            z: bmax.z,
        },
        V3 {
            x: bmin.x,
            y: bmax.y,
            z: bmax.z,
        },
    ];
    let edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    let mut lb = LineBuilder::new();
    for (a, b) in edges {
        lb.add(c[a], c[b], color, width);
    }
    lb.build(context)
}

fn build_grid(context: &Context, x_span: f32, y_span: f32) -> Gm<Mesh, ColorMaterial> {
    let mm1 = Srgba::new(0x10, 0x10, 0x18, 0x66);
    let mm10 = Srgba::new(0x20, 0x20, 0x30, 0xaa);
    let mm50 = Srgba::new(0x33, 0x33, 0x55, 0xcc);
    let origin_col = Srgba::new(0x44, 0x44, 0x66, 0xff);
    let mut lb = LineBuilder::new();

    let x_max = x_span;
    let y_max = y_span;
    let mut x = 0.0f32;
    while x <= x_max {
        let ix = x.round() as i32;
        let (col, w) = if ix == 0 {
            (origin_col, LINE_W)
        } else if ix % 50 == 0 {
            (mm50, LINE_W * 0.8)
        } else if ix % 10 == 0 {
            (mm10, GRID_W)
        } else {
            (mm1, GRID_W * 0.5)
        };
        lb.add(
            V3 { x, y: 0.0, z: 0.0 },
            V3 {
                x,
                y: y_max,
                z: 0.0,
            },
            col,
            w,
        );
        x += 1.0;
    }
    let mut y = 0.0f32;
    while y <= y_max {
        let iy = y.round() as i32;
        let (col, w) = if iy == 0 {
            (origin_col, LINE_W)
        } else if iy % 50 == 0 {
            (mm50, LINE_W * 0.8)
        } else if iy % 10 == 0 {
            (mm10, GRID_W)
        } else {
            (mm1, GRID_W * 0.5)
        };
        lb.add(
            V3 { x: 0.0, y, z: 0.0 },
            V3 {
                x: x_max,
                y,
                z: 0.0,
            },
            col,
            w,
        );
        y += 1.0;
    }
    lb.build(context)
}

fn build_axes(context: &Context, grid_span: f32) -> Gm<Mesh, ColorMaterial> {
    let red = Srgba::new(0xff, 0x55, 0x55, 0xff);
    let green = Srgba::new(0x55, 0xff, 0x55, 0xff);
    let blue = Srgba::new(0x55, 0x88, 0xff, 0xff);
    let red_dim = Srgba::new(0xff, 0x55, 0x55, 0x88);
    let green_dim = Srgba::new(0x55, 0xff, 0x55, 0x88);

    let arrow_len = 30.0_f32;
    let head = 4.0_f32;
    let z_lift = 0.05_f32;

    let mut lb = LineBuilder::new();

    let x0 = V3 { x: 0.0, y: 0.0, z: z_lift };
    let x_tip = V3 { x: arrow_len, y: 0.0, z: z_lift };
    lb.add(x0, x_tip, red, THICK_W * 1.4);
    lb.add(
        x_tip,
        V3 { x: arrow_len - head, y: head * 0.6, z: z_lift },
        red,
        THICK_W,
    );
    lb.add(
        x_tip,
        V3 { x: arrow_len - head, y: -head * 0.6, z: z_lift },
        red,
        THICK_W,
    );
    draw_letter_xy(
        &mut lb,
        'X',
        V3 { x: arrow_len + 4.0, y: -3.0, z: z_lift },
        6.0,
        red,
    );

    let y_tip = V3 { x: 0.0, y: arrow_len, z: z_lift };
    lb.add(x0, y_tip, green, THICK_W * 1.4);
    lb.add(
        y_tip,
        V3 { x: head * 0.6, y: arrow_len - head, z: z_lift },
        green,
        THICK_W,
    );
    lb.add(
        y_tip,
        V3 { x: -head * 0.6, y: arrow_len - head, z: z_lift },
        green,
        THICK_W,
    );
    draw_letter_xy(
        &mut lb,
        'Y',
        V3 { x: -3.0, y: arrow_len + 4.0, z: z_lift },
        6.0,
        green,
    );

    let z_tip = V3 { x: 0.0, y: 0.0, z: 20.0 };
    lb.add(V3::default(), z_tip, blue, THICK_W);
    lb.add(z_tip, V3 { x: 1.5, y: 0.0, z: 18.0 }, blue, THICK_W);
    lb.add(z_tip, V3 { x: -1.5, y: 0.0, z: 18.0 }, blue, THICK_W);
    draw_letter_xz(
        &mut lb,
        'Z',
        V3 { x: -3.0, y: 0.0, z: 22.0 },
        5.0,
        blue,
    );

    let far = (grid_span - 15.0).max(arrow_len + 20.0);
    let big = 12.0_f32;
    draw_letter_xy(
        &mut lb,
        'X',
        V3 { x: far, y: -big - 3.0, z: z_lift },
        big,
        red_dim,
    );
    draw_letter_xy(
        &mut lb,
        'Y',
        V3 { x: -big - 3.0, y: far, z: z_lift },
        big,
        green_dim,
    );

    lb.build(context)
}

fn letter_strokes(ch: char) -> &'static [(f32, f32, f32, f32)] {
    match ch {
        'X' => &[(0.0, 0.0, 1.0, 1.0), (0.0, 1.0, 1.0, 0.0)],
        'Y' => &[
            (0.0, 1.0, 0.5, 0.5),
            (1.0, 1.0, 0.5, 0.5),
            (0.5, 0.5, 0.5, 0.0),
        ],
        'Z' => &[
            (0.0, 1.0, 1.0, 1.0),
            (1.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
        ],
        _ => &[],
    }
}

fn draw_letter_xy(lb: &mut LineBuilder, ch: char, pos: V3, size: f32, color: Srgba) {
    for &(x1, y1, x2, y2) in letter_strokes(ch) {
        let p1 = V3 {
            x: pos.x + x1 * size,
            y: pos.y + y1 * size,
            z: pos.z,
        };
        let p2 = V3 {
            x: pos.x + x2 * size,
            y: pos.y + y2 * size,
            z: pos.z,
        };
        lb.add(p1, p2, color, LINE_W * 1.5);
    }
}

fn draw_letter_xz(lb: &mut LineBuilder, ch: char, pos: V3, size: f32, color: Srgba) {
    for &(x1, y1, x2, y2) in letter_strokes(ch) {
        let p1 = V3 {
            x: pos.x + x1 * size,
            y: pos.y,
            z: pos.z + y1 * size,
        };
        let p2 = V3 {
            x: pos.x + x2 * size,
            y: pos.y,
            z: pos.z + y2 * size,
        };
        lb.add(p1, p2, color, LINE_W * 1.5);
    }
}

fn build_probe_points(context: &Context, p: &ProbePreview) -> Option<Gm<Mesh, ColorMaterial>> {
    if p.grid_x < 2 || p.grid_y < 2 {
        return None;
    }
    if p.bbox_max.0 <= p.bbox_min.0 || p.bbox_max.1 <= p.bbox_min.1 {
        return None;
    }
    let mut lb = LineBuilder::new();
    let total = p.grid_x as usize * p.grid_y as usize;
    let pending = Srgba::new(0xff, 0xaa, 0x00, 0xcc);
    let probed = Srgba::new(0x00, 0xff, 0x88, 0xff);
    let active = Srgba::new(0x00, 0xcc, 0xff, 0xff);
    let skipped = Srgba::new(0xff, 0x33, 0x33, 0xff);
    for idx in 0..total {
        let (x, y) = grid_point(p.bbox_min, p.bbox_max, p.grid_x, p.grid_y, idx);
        let is_skipped = p.skipped.get(idx).copied().unwrap_or(false);
        let sample_z = p
            .samples
            .as_ref()
            .and_then(|s| s.get(idx).copied().flatten());
        let z = sample_z.unwrap_or(0.0);
        let r = if Some(idx) == p.current_index { 2.5 } else { 1.5 };
        if is_skipped {
            lb.add(V3 { x: x - r, y: y - r, z }, V3 { x: x + r, y: y + r, z }, skipped, LINE_W);
            lb.add(V3 { x: x - r, y: y + r, z }, V3 { x: x + r, y: y - r, z }, skipped, LINE_W);
            continue;
        }
        let color = if Some(idx) == p.current_index {
            active
        } else if sample_z.is_some() {
            probed
        } else {
            pending
        };
        lb.add(V3 { x: x - r, y, z }, V3 { x: x + r, y, z }, color, LINE_W);
        lb.add(V3 { x, y: y - r, z }, V3 { x, y: y + r, z }, color, LINE_W);
        lb.add(V3 { x, y, z: z - 0.5 }, V3 { x, y, z: z + 1.5 }, color, GRID_W);
    }
    Some(lb.build(context))
}


fn build_toolpath(
    context: &Context,
    segments: &[Segment],
    seg_violations: &[bool],
    seg_pass_counts: &[u16],
    bmin: V3,
    bmax: V3,
    show_heatmap: bool,
) -> Gm<Mesh, ColorMaterial> {
    let mut lb = LineBuilder::new();
    for (i, seg) in segments.iter().enumerate() {
        let violated = seg_violations.get(i).copied().unwrap_or(false);
        let pass_count = seg_pass_counts.get(i).copied().unwrap_or(1);
        let color = if violated {
            Srgba::new(0xff, 0x22, 0x22, 0xff)
        } else if show_heatmap && pass_count > 1 {
            heatmap_color(pass_count)
        } else if seg.rapid {
            Srgba::new(0xff, 0x88, 0x00, 0xff)
        } else {
            depth_color(seg.end.z, bmin.z, bmax.z)
        };
        let w = if violated {
            THICK_W
        } else if show_heatmap && pass_count > 1 {
            (LINE_W + (pass_count as f32 - 1.0) * 0.08).min(THICK_W * 1.8)
        } else if seg.rapid {
            GRID_W
        } else {
            LINE_W
        };
        lb.add(seg.start, seg.end, color, w);
    }
    lb.build(context)
}

fn heatmap_color(pass_count: u16) -> Srgba {
    match pass_count {
        0 | 1 => Srgba::new(0x00, 0xff, 0x88, 0xff),
        2 => Srgba::new(0xff, 0xdd, 0x33, 0xff),
        3 => Srgba::new(0xff, 0x99, 0x22, 0xff),
        4 => Srgba::new(0xff, 0x55, 0x22, 0xff),
        _ => Srgba::new(0xff, 0x22, 0x66, 0xff),
    }
}

fn build_gantry(context: &Context, wpos: V3, z_top: f32) -> Gm<Mesh, ColorMaterial> {
    let _rail = Srgba::new(0x44, 0x44, 0x66, 0x55);
    let spin = Srgba::new(0x88, 0x88, 0xaa, 0xbb);
    let tip = Srgba::new(0xdd, 0xdd, 0xee, 0xff);
    let drop_c = Srgba::new(0x00, 0xff, 0xff, 0x44);
    let cross = Srgba::new(0x00, 0xff, 0xff, 0xff);

    let mut lb = LineBuilder::new();
    lb.add(
        V3 {
            x: wpos.x,
            y: wpos.y,
            z: z_top,
        },
        V3 {
            x: wpos.x,
            y: wpos.y,
            z: wpos.z + 8.0,
        },
        spin,
        LINE_W,
    );

    let tw = 3.0f32;
    let tl = 8.0f32;
    lb.add(
        V3 {
            x: wpos.x - tw,
            y: wpos.y,
            z: wpos.z + tl,
        },
        wpos,
        tip,
        GRID_W,
    );
    lb.add(
        V3 {
            x: wpos.x + tw,
            y: wpos.y,
            z: wpos.z + tl,
        },
        wpos,
        tip,
        GRID_W,
    );
    lb.add(
        V3 {
            x: wpos.x,
            y: wpos.y - tw,
            z: wpos.z + tl,
        },
        wpos,
        tip,
        GRID_W,
    );
    lb.add(
        V3 {
            x: wpos.x,
            y: wpos.y + tw,
            z: wpos.z + tl,
        },
        wpos,
        tip,
        GRID_W,
    );

    if wpos.z.abs() > 0.5 {
        lb.add(
            wpos,
            V3 {
                x: wpos.x,
                y: wpos.y,
                z: 0.0,
            },
            drop_c,
            GRID_W,
        );
    }
    let cd = 4.0f32;
    lb.add(
        V3 {
            x: wpos.x - cd,
            y: wpos.y,
            z: wpos.z,
        },
        V3 {
            x: wpos.x + cd,
            y: wpos.y,
            z: wpos.z,
        },
        cross,
        LINE_W,
    );
    lb.add(
        V3 {
            x: wpos.x,
            y: wpos.y - cd,
            z: wpos.z,
        },
        V3 {
            x: wpos.x,
            y: wpos.y + cd,
            z: wpos.z,
        },
        cross,
        LINE_W,
    );
    let shadow = Srgba::new(0x00, 0xff, 0xff, 0x33);
    let sd = 3.0f32;
    lb.add(
        V3 {
            x: wpos.x - sd,
            y: wpos.y,
            z: 0.0,
        },
        V3 {
            x: wpos.x + sd,
            y: wpos.y,
            z: 0.0,
        },
        shadow,
        LINE_W,
    );
    lb.add(
        V3 {
            x: wpos.x,
            y: wpos.y - sd,
            z: 0.0,
        },
        V3 {
            x: wpos.x,
            y: wpos.y + sd,
            z: 0.0,
        },
        shadow,
        LINE_W,
    );

    lb.build(context)
}

fn build_trail(context: &Context, points: &[V3]) -> Gm<Mesh, ColorMaterial> {
    let n = points.len();
    let mut lb = LineBuilder::new();
    for i in 1..n {
        let t = i as f32 / n as f32;
        let a = (32.0 + t * 223.0) as u8;
        let color = Srgba::new(0xff, 0xaa, 0x00, a);
        lb.add(points[i - 1], points[i], color, LINE_W);
    }
    lb.build(context)
}

fn depth_color(z: f32, zmin: f32, zmax: f32) -> Srgba {
    if zmin >= zmax {
        return Srgba::new(0x00, 0xff, 0x88, 0xff);
    }
    let t = (z - zmin) / (zmax - zmin);
    Srgba::new(
        0x00,
        (68.0 + t * 187.0) as u8,
        (255.0 - t * 119.0) as u8,
        0xff,
    )
}

fn build_material(context: &Context, field: &MaterialField) -> Gm<Mesh, ColorMaterial> {
    let nx = field.nx;
    let ny = field.ny;
    let mut positions = Vec::with_capacity(nx * ny);
    let mut colors = Vec::with_capacity(nx * ny);
    for iy in 0..ny {
        let y = field.bbox_min_y + (iy as f32 + 0.5) * field.res;
        for ix in 0..nx {
            let x = field.bbox_min_x + (ix as f32 + 0.5) * field.res;
            let z = field.heights[iy * nx + ix];
            positions.push(vec3(x, y, z));
            colors.push(material_color(z, field.z_top, field.z_floor));
        }
    }
    let mut indices = Vec::with_capacity((nx - 1) * (ny - 1) * 6);
    for iy in 0..ny - 1 {
        for ix in 0..nx - 1 {
            let a = (iy * nx + ix) as u32;
            let b = a + 1;
            let c = a + nx as u32;
            let d = c + 1;
            indices.extend([a, c, d, a, d, b]);
        }
    }
    let cpu_mesh = CpuMesh {
        positions: Positions::F32(positions),
        indices: Indices::U32(indices),
        colors: Some(colors),
        ..Default::default()
    };
    let mesh = Mesh::new(context, &cpu_mesh);
    let material = ColorMaterial {
        color: Srgba::WHITE,
        is_transparent: true,
        render_states: RenderStates {
            blend: Blend::TRANSPARENCY,
            ..Default::default()
        },
        ..Default::default()
    };
    Gm::new(mesh, material)
}

fn material_color(z: f32, z_top: f32, z_floor: f32) -> Srgba {
    if z >= z_top - 1e-4 {
        return Srgba::new(0xc8, 0x96, 0x3e, 0xc0);
    }
    let span = (z_top - z_floor).max(0.05);
    let t = ((z_top - z) / span).clamp(0.0, 1.0);
    let r = (0xaa as f32 - t * 0x55 as f32) as u8;
    let g = (0x55 as f32 - t * 0x33 as f32) as u8;
    let b = (0x22 as f32 + t * 0x66 as f32) as u8;
    Srgba::new(r, g, b, 0xee)
}

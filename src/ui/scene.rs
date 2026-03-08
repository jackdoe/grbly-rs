use three_d::renderer::*;
use crate::grbl::state::{self, MachineState, JobState, Segment};

type V3 = state::Vec3;

/// User-editable material/workpiece parameters
#[derive(Clone, Debug)]
pub struct MaterialState {
    pub width: f32,
    pub height: f32,
    pub thickness: f32,
    pub offset_x: f32,
    pub offset_y: f32,
    // String buffers for text fields
    pub width_s: String,
    pub height_s: String,
    pub thickness_s: String,
    pub offset_x_s: String,
    pub offset_y_s: String,
}

impl Default for MaterialState {
    fn default() -> Self {
        Self {
            width: 50.8,
            height: 76.2,
            thickness: 1.6,
            offset_x: 0.0,
            offset_y: 0.0,
            width_s: "50.8".into(),
            height_s: "76.2".into(),
            thickness_s: "1.6".into(),
            offset_x_s: "0".into(),
            offset_y_s: "0".into(),
        }
    }
}

impl MaterialState {
}

pub struct Scene {
    pub grid: Gm<Mesh, ColorMaterial>,
    pub triad: Gm<Mesh, ColorMaterial>,
    pub machine_box: Option<Gm<Mesh, ColorMaterial>>,
    pub material_slab: Option<Gm<Mesh, ColorMaterial>>,
    pub toolpath: Option<Gm<Mesh, ColorMaterial>>,
    pub gantry: Option<Gm<Mesh, ColorMaterial>>,
    pub trail: Option<Gm<Mesh, ColorMaterial>>,
    pub bounds_box: Option<Gm<Mesh, ColorMaterial>>,
    trail_points: Vec<V3>,
    trail_dirty: bool,
    last_pos: V3,
    last_version: usize,
    last_connected: bool,
    last_wco: V3,
    last_max_travel: V3,
    last_material_version: u32,
}

const LINE_W: f32 = 0.3;
const GRID_W: f32 = 0.15;
const THICK_W: f32 = 0.5;

/// Cubiko default travel extents (mm) used when $130/$131/$132 are zero
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
            triad: build_triad(context),
            machine_box: None,
            material_slab: None,
            toolpath: None,
            gantry: None,
            trail: None,
            bounds_box: None,
            trail_points: Vec::new(),
            trail_dirty: false,
            last_pos: V3::default(),
            last_version: 0,
            last_connected: false,
            last_wco: V3::default(),
            last_max_travel: V3::default(),
            last_material_version: u32::MAX,
        }
    }

    pub fn update(
        &mut self,
        context: &Context,
        tool_pos: V3,
        mstate: &MachineState,
        jstate: &JobState,
        material: &MaterialState,
        material_version: u32,
    ) {
        let wpos = tool_pos;
        if wpos != self.last_pos {
            self.trail_points.push(wpos);
            if self.trail_points.len() > 5000 {
                let drain = self.trail_points.len() - 5000;
                self.trail_points.drain(..drain);
            }
            self.last_pos = wpos;
            self.trail_dirty = true;
        }

        // Gantry always updates (position changes)
        let wco = mstate.wco;
        let z_clearance = if wco.z.abs() > 0.01 { -wco.z } else { 30.0 };
        self.gantry = Some(build_gantry(context, wpos, z_clearance));

        if self.trail_dirty && self.trail_points.len() >= 2 {
            self.trail = Some(build_trail(context, &self.trail_points));
            self.trail_dirty = false;
        }

        if jstate.version != self.last_version {
            self.last_version = jstate.version;
            if !jstate.segments.is_empty() {
                self.toolpath = Some(build_toolpath(context, &jstate.segments, &jstate.seg_violations, jstate.bounds_min, jstate.bounds_max));
                self.bounds_box = Some(build_wire_box(context, jstate.bounds_min, jstate.bounds_max, Srgba::new(0x55, 0x55, 0x77, 0x66), GRID_W));
            } else {
                self.toolpath = None;
                self.bounds_box = None;
            }
        }

        // Update machine travel box when connection state, WCO, or travel settings change
        let mt = mstate.max_travel;
        let connected_changed = mstate.connected != self.last_connected;
        if connected_changed || wco != self.last_wco || mt != self.last_max_travel {
            self.last_connected = mstate.connected;
            self.last_wco = wco;
            self.last_max_travel = mt;
            if mstate.connected {
                // Use Cubiko defaults as fallback when max_travel axes are zero
                let tx = if mt.x > 0.0 { mt.x } else { CUBIKO_TRAVEL_X };
                let ty = if mt.y > 0.0 { mt.y } else { CUBIKO_TRAVEL_Y };
                let tz = if mt.z > 0.0 { mt.z } else { CUBIKO_TRAVEL_Z };
                // Machine travel in work coordinates:
                // Cubiko: XY home at MPos(0,0), travel goes positive to MPos(+tx,+ty)
                //         Z  home at MPos(0),   travel goes negative to MPos(-tz)
                // WPos = MPos - WCO
                let home_w = V3 { x: -wco.x, y: -wco.y, z: -wco.z };
                let far_w = V3 { x: tx - wco.x, y: ty - wco.y, z: -tz - wco.z };
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
                self.machine_box = Some(build_wire_box(context, bmin, bmax, Srgba::new(0x44, 0x77, 0xbb, 0x99), LINE_W));
            } else {
                self.machine_box = None;
            }
        }

        // Update material slab
        if material_version != self.last_material_version {
            self.last_material_version = material_version;
            if material.width > 0.0 && material.height > 0.0 && material.thickness > 0.0 {
                self.material_slab = Some(build_material(context, material));
            } else {
                self.material_slab = None;
            }
        }
    }

    pub fn collect(&self) -> Vec<&Gm<Mesh, ColorMaterial>> {
        let mut out: Vec<&Gm<Mesh, ColorMaterial>> = vec![&self.grid, &self.triad];
        if let Some(ref v) = self.machine_box { out.push(v); }
        if let Some(ref v) = self.material_slab { out.push(v); }
        if let Some(ref v) = self.toolpath { out.push(v); }
        if let Some(ref v) = self.bounds_box { out.push(v); }
        if let Some(ref v) = self.trail { out.push(v); }
        if let Some(ref v) = self.gantry { out.push(v); }
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
        Self { positions: Vec::new(), colors: Vec::new(), indices: Vec::new() }
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
        self.indices.extend([base, base + 1, base + 2, base + 1, base + 3, base + 2]);
    }

    fn add_quad(&mut self, corners: [V3; 4], color: Srgba) {
        let base = self.positions.len() as u32;
        for c in &corners {
            self.positions.push(v3(*c));
            self.colors.push(color);
        }
        self.indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
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

fn build_wire_box(context: &Context, bmin: V3, bmax: V3, color: Srgba, width: f32) -> Gm<Mesh, ColorMaterial> {
    let c = [
        V3 { x: bmin.x, y: bmin.y, z: bmin.z },
        V3 { x: bmax.x, y: bmin.y, z: bmin.z },
        V3 { x: bmax.x, y: bmax.y, z: bmin.z },
        V3 { x: bmin.x, y: bmax.y, z: bmin.z },
        V3 { x: bmin.x, y: bmin.y, z: bmax.z },
        V3 { x: bmax.x, y: bmin.y, z: bmax.z },
        V3 { x: bmax.x, y: bmax.y, z: bmax.z },
        V3 { x: bmin.x, y: bmax.y, z: bmax.z },
    ];
    let edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)];
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
        lb.add(V3 { x, y: 0.0, z: 0.0 }, V3 { x, y: y_max, z: 0.0 }, col, w);
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
        lb.add(V3 { x: 0.0, y, z: 0.0 }, V3 { x: x_max, y, z: 0.0 }, col, w);
        y += 1.0;
    }
    lb.build(context)
}

fn build_triad(context: &Context) -> Gm<Mesh, ColorMaterial> {
    let len = 15.0f32;
    let mut lb = LineBuilder::new();
    lb.add(V3::default(), V3 { x: len, y: 0.0, z: 0.0 }, Srgba::new(0xff, 0x55, 0x55, 0xff), THICK_W);
    lb.add(V3::default(), V3 { x: 0.0, y: len, z: 0.0 }, Srgba::new(0x55, 0xff, 0x55, 0xff), THICK_W);
    lb.add(V3::default(), V3 { x: 0.0, y: 0.0, z: len }, Srgba::new(0x55, 0x88, 0xff, 0xff), THICK_W);
    lb.build(context)
}

fn build_material(context: &Context, mat: &MaterialState) -> Gm<Mesh, ColorMaterial> {
    let x0 = mat.offset_x;
    let y0 = mat.offset_y;
    let x1 = x0 + mat.width;
    let y1 = y0 + mat.height;
    let z_top = 0.0f32;
    let z_bot = -mat.thickness;

    let mut lb = LineBuilder::new();

    // Top face (semi-transparent green)
    let top_col = Srgba::new(0x22, 0x88, 0x44, 0x33);
    lb.add_quad([
        V3 { x: x0, y: y0, z: z_top },
        V3 { x: x1, y: y0, z: z_top },
        V3 { x: x1, y: y1, z: z_top },
        V3 { x: x0, y: y1, z: z_top },
    ], top_col);

    // Bottom face
    let bot_col = Srgba::new(0x22, 0x88, 0x44, 0x22);
    lb.add_quad([
        V3 { x: x0, y: y0, z: z_bot },
        V3 { x: x1, y: y0, z: z_bot },
        V3 { x: x1, y: y1, z: z_bot },
        V3 { x: x0, y: y1, z: z_bot },
    ], bot_col);

    // Wireframe edges
    let edge_col = Srgba::new(0x44, 0xff, 0x88, 0xaa);
    // Top edges
    lb.add(V3 { x: x0, y: y0, z: z_top }, V3 { x: x1, y: y0, z: z_top }, edge_col, GRID_W);
    lb.add(V3 { x: x1, y: y0, z: z_top }, V3 { x: x1, y: y1, z: z_top }, edge_col, GRID_W);
    lb.add(V3 { x: x1, y: y1, z: z_top }, V3 { x: x0, y: y1, z: z_top }, edge_col, GRID_W);
    lb.add(V3 { x: x0, y: y1, z: z_top }, V3 { x: x0, y: y0, z: z_top }, edge_col, GRID_W);
    // Bottom edges
    lb.add(V3 { x: x0, y: y0, z: z_bot }, V3 { x: x1, y: y0, z: z_bot }, edge_col, GRID_W);
    lb.add(V3 { x: x1, y: y0, z: z_bot }, V3 { x: x1, y: y1, z: z_bot }, edge_col, GRID_W);
    lb.add(V3 { x: x1, y: y1, z: z_bot }, V3 { x: x0, y: y1, z: z_bot }, edge_col, GRID_W);
    lb.add(V3 { x: x0, y: y1, z: z_bot }, V3 { x: x0, y: y0, z: z_bot }, edge_col, GRID_W);
    // Vertical edges
    lb.add(V3 { x: x0, y: y0, z: z_top }, V3 { x: x0, y: y0, z: z_bot }, edge_col, GRID_W);
    lb.add(V3 { x: x1, y: y0, z: z_top }, V3 { x: x1, y: y0, z: z_bot }, edge_col, GRID_W);
    lb.add(V3 { x: x1, y: y1, z: z_top }, V3 { x: x1, y: y1, z: z_bot }, edge_col, GRID_W);
    lb.add(V3 { x: x0, y: y1, z: z_top }, V3 { x: x0, y: y1, z: z_bot }, edge_col, GRID_W);

    lb.build(context)
}

fn build_toolpath(context: &Context, segments: &[Segment], seg_violations: &[bool], bmin: V3, bmax: V3) -> Gm<Mesh, ColorMaterial> {
    let mut lb = LineBuilder::new();
    for (i, seg) in segments.iter().enumerate() {
        let violated = seg_violations.get(i).copied().unwrap_or(false);
        let color = if violated {
            Srgba::new(0xff, 0x22, 0x22, 0xff)
        } else if seg.rapid {
            Srgba::new(0xff, 0x88, 0x00, 0xff)
        } else {
            depth_color(seg.end.z, bmin.z, bmax.z)
        };
        let w = if violated { THICK_W } else if seg.rapid { GRID_W } else { LINE_W };
        lb.add(seg.start, seg.end, color, w);
    }
    lb.build(context)
}

fn build_gantry(context: &Context, wpos: V3, z_top: f32) -> Gm<Mesh, ColorMaterial> {
    let _rail = Srgba::new(0x44, 0x44, 0x66, 0x55);
    let spin = Srgba::new(0x88, 0x88, 0xaa, 0xbb);
    let tip = Srgba::new(0xdd, 0xdd, 0xee, 0xff);
    let drop_c = Srgba::new(0x00, 0xff, 0xff, 0x44);
    let cross = Srgba::new(0x00, 0xff, 0xff, 0xff);

    let mut lb = LineBuilder::new();
    // Spindle column from top down to tool tip
    lb.add(V3 { x: wpos.x, y: wpos.y, z: z_top }, V3 { x: wpos.x, y: wpos.y, z: wpos.z + 8.0 }, spin, LINE_W);

    let tw = 3.0f32;
    let tl = 8.0f32;
    lb.add(V3 { x: wpos.x - tw, y: wpos.y, z: wpos.z + tl }, wpos, tip, GRID_W);
    lb.add(V3 { x: wpos.x + tw, y: wpos.y, z: wpos.z + tl }, wpos, tip, GRID_W);
    lb.add(V3 { x: wpos.x, y: wpos.y - tw, z: wpos.z + tl }, wpos, tip, GRID_W);
    lb.add(V3 { x: wpos.x, y: wpos.y + tw, z: wpos.z + tl }, wpos, tip, GRID_W);

    // Drop line to Z=0
    if wpos.z.abs() > 0.5 {
        lb.add(wpos, V3 { x: wpos.x, y: wpos.y, z: 0.0 }, drop_c, GRID_W);
    }
    // Crosshair at tool tip
    let cd = 4.0f32;
    lb.add(V3 { x: wpos.x - cd, y: wpos.y, z: wpos.z }, V3 { x: wpos.x + cd, y: wpos.y, z: wpos.z }, cross, LINE_W);
    lb.add(V3 { x: wpos.x, y: wpos.y - cd, z: wpos.z }, V3 { x: wpos.x, y: wpos.y + cd, z: wpos.z }, cross, LINE_W);
    // Shadow on Z=0
    let shadow = Srgba::new(0x00, 0xff, 0xff, 0x33);
    let sd = 3.0f32;
    lb.add(V3 { x: wpos.x - sd, y: wpos.y, z: 0.0 }, V3 { x: wpos.x + sd, y: wpos.y, z: 0.0 }, shadow, LINE_W);
    lb.add(V3 { x: wpos.x, y: wpos.y - sd, z: 0.0 }, V3 { x: wpos.x, y: wpos.y + sd, z: 0.0 }, shadow, LINE_W);

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

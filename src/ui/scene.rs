use three_d::renderer::*;
use crate::grbl::state::{self, MachineProfile, JobState, Segment};

type V3 = state::Vec3;

pub struct Scene {
    pub envelope: Gm<Mesh, ColorMaterial>,
    pub grid: Gm<Mesh, ColorMaterial>,
    pub triad: Gm<Mesh, ColorMaterial>,
    pub toolpath: Option<Gm<Mesh, ColorMaterial>>,
    pub gantry: Option<Gm<Mesh, ColorMaterial>>,
    pub trail: Option<Gm<Mesh, ColorMaterial>>,
    pub bounds_box: Option<Gm<Mesh, ColorMaterial>>,
    trail_points: Vec<V3>,
    trail_dirty: bool,
    last_pos: V3,
    last_version: usize,
}

const LINE_W: f32 = 0.3;
const GRID_W: f32 = 0.15;
const THICK_W: f32 = 0.5;

fn v3(v: V3) -> Vector3<f32> {
    vec3(v.x, v.y, v.z)
}

impl Scene {
    pub fn new(context: &Context, profile: &MachineProfile) -> Self {
        let env = profile.envelope;
        Self {
            envelope: build_wire_box(context, V3::default(), env, Srgba::new(0x44, 0x77, 0xbb, 0xaa), LINE_W),
            grid: build_grid(context, env),
            triad: build_triad(context),
            toolpath: None,
            gantry: None,
            trail: None,
            bounds_box: None,
            trail_points: Vec::new(),
            trail_dirty: false,
            last_pos: V3::default(),
            last_version: 0,
        }
    }

    pub fn update(&mut self, context: &Context, tool_pos: V3, jstate: &JobState, profile: &MachineProfile) {
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

        self.gantry = Some(build_gantry(context, wpos, profile));

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
    }

    pub fn collect(&self) -> Vec<&Gm<Mesh, ColorMaterial>> {
        let mut out: Vec<&Gm<Mesh, ColorMaterial>> = vec![&self.envelope, &self.grid, &self.triad];
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

fn build_grid(context: &Context, env: V3) -> Gm<Mesh, ColorMaterial> {
    let minor = Srgba::new(0x18, 0x18, 0x28, 0xaa);
    let major = Srgba::new(0x30, 0x30, 0x50, 0xcc);
    let origin = Srgba::new(0x44, 0x44, 0x66, 0xff);
    let mut lb = LineBuilder::new();

    let mut x = 0.0f32;
    while x <= env.x {
        let ix = x.round() as i32;
        let (col, w) = if ix == 0 { (origin, LINE_W) } else if ix % 50 == 0 { (major, LINE_W * 0.8) } else { (minor, GRID_W) };
        lb.add(V3 { x, y: 0.0, z: 0.0 }, V3 { x, y: env.y, z: 0.0 }, col, w);
        x += 10.0;
    }
    let mut y = 0.0f32;
    while y <= env.y {
        let iy = y.round() as i32;
        let (col, w) = if iy == 0 { (origin, LINE_W) } else if iy % 50 == 0 { (major, LINE_W * 0.8) } else { (minor, GRID_W) };
        lb.add(V3 { x: 0.0, y, z: 0.0 }, V3 { x: env.x, y, z: 0.0 }, col, w);
        y += 10.0;
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

fn build_gantry(context: &Context, wpos: V3, profile: &MachineProfile) -> Gm<Mesh, ColorMaterial> {
    let env = profile.envelope;
    let rail = Srgba::new(0x44, 0x44, 0x66, 0x55);
    let spin = Srgba::new(0x88, 0x88, 0xaa, 0xbb);
    let tip = Srgba::new(0xdd, 0xdd, 0xee, 0xff);
    let drop_c = Srgba::new(0x00, 0xff, 0xff, 0x44);
    let cross = Srgba::new(0x00, 0xff, 0xff, 0xff);

    let mut lb = LineBuilder::new();
    lb.add(V3 { x: 0.0, y: wpos.y, z: env.z }, V3 { x: env.x, y: wpos.y, z: env.z }, rail, GRID_W);
    lb.add(V3 { x: wpos.x, y: 0.0, z: env.z }, V3 { x: wpos.x, y: env.y, z: env.z }, rail, GRID_W);
    lb.add(V3 { x: wpos.x, y: wpos.y, z: env.z }, V3 { x: wpos.x, y: wpos.y, z: wpos.z + 8.0 }, spin, LINE_W);

    let tw = 3.0f32;
    let tl = 8.0f32;
    lb.add(V3 { x: wpos.x - tw, y: wpos.y, z: wpos.z + tl }, wpos, tip, GRID_W);
    lb.add(V3 { x: wpos.x + tw, y: wpos.y, z: wpos.z + tl }, wpos, tip, GRID_W);
    lb.add(V3 { x: wpos.x, y: wpos.y - tw, z: wpos.z + tl }, wpos, tip, GRID_W);
    lb.add(V3 { x: wpos.x, y: wpos.y + tw, z: wpos.z + tl }, wpos, tip, GRID_W);

    if wpos.z.abs() > 1.0 {
        lb.add(wpos, V3 { x: wpos.x, y: wpos.y, z: 0.0 }, drop_c, GRID_W);
        let sd = 3.0f32;
        lb.add(V3 { x: wpos.x - sd, y: wpos.y, z: 0.0 }, V3 { x: wpos.x + sd, y: wpos.y, z: 0.0 }, cross, LINE_W);
        lb.add(V3 { x: wpos.x, y: wpos.y - sd, z: 0.0 }, V3 { x: wpos.x, y: wpos.y + sd, z: 0.0 }, cross, LINE_W);
    }
    let cd = 4.0f32;
    lb.add(V3 { x: wpos.x - cd, y: wpos.y, z: wpos.z }, V3 { x: wpos.x + cd, y: wpos.y, z: wpos.z }, cross, LINE_W);
    lb.add(V3 { x: wpos.x, y: wpos.y - cd, z: wpos.z }, V3 { x: wpos.x, y: wpos.y + cd, z: wpos.z }, cross, LINE_W);

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

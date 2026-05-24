use crate::grbl::state::{Segment, Vec3};

pub struct MaterialField {
    pub bbox_min_x: f32,
    pub bbox_min_y: f32,
    pub res: f32,
    pub nx: usize,
    pub ny: usize,
    pub z_top: f32,
    pub z_floor: f32,
    pub heights: Vec<f32>,
    pub carved_up_to: usize,
    pub endmill_diameter: f32,
    pub version: u32,
}

impl MaterialField {
    pub fn new(bmin: Vec3, bmax: Vec3, endmill_diameter: f32) -> Self {
        let res = (endmill_diameter * 0.35).clamp(0.08, 0.6);
        let pad = endmill_diameter.max(1.0);
        let bbox_min_x = bmin.x - pad;
        let bbox_min_y = bmin.y - pad;
        let bbox_max_x = bmax.x + pad;
        let bbox_max_y = bmax.y + pad;
        let nx = (((bbox_max_x - bbox_min_x) / res).ceil() as usize).max(2);
        let ny = (((bbox_max_y - bbox_min_y) / res).ceil() as usize).max(2);
        let z_top = 0.02;
        let z_floor = bmin.z.min(-0.1);
        Self {
            bbox_min_x,
            bbox_min_y,
            res,
            nx,
            ny,
            z_top,
            z_floor,
            heights: vec![z_top; nx * ny],
            carved_up_to: 0,
            endmill_diameter,
            version: 1,
        }
    }

    pub fn reset(&mut self) {
        for h in self.heights.iter_mut() {
            *h = self.z_top;
        }
        self.carved_up_to = 0;
        self.version = self.version.wrapping_add(1);
    }

    pub fn carve_up_to(&mut self, segments: &[Segment], target: usize) {
        let target = target.min(segments.len());
        if target < self.carved_up_to {
            self.reset();
        }
        if target == self.carved_up_to {
            return;
        }
        let radius = self.endmill_diameter * 0.5;
        for seg in &segments[self.carved_up_to..target] {
            self.carve(seg, radius);
        }
        self.carved_up_to = target;
        self.version = self.version.wrapping_add(1);
    }

    fn carve(&mut self, seg: &Segment, radius: f32) {
        if seg.rapid {
            return;
        }
        let (sx, sy, sz) = (seg.start.x, seg.start.y, seg.start.z);
        let (ex, ey, ez) = (seg.end.x, seg.end.y, seg.end.z);
        if sz >= self.z_top && ez >= self.z_top {
            return;
        }
        let dx = ex - sx;
        let dy = ey - sy;
        let len2 = dx * dx + dy * dy;
        let r2 = radius * radius;

        let min_x = sx.min(ex) - radius;
        let max_x = sx.max(ex) + radius;
        let min_y = sy.min(ey) - radius;
        let max_y = sy.max(ey) + radius;

        let ix0 = (((min_x - self.bbox_min_x) / self.res).floor().max(0.0) as usize).min(self.nx);
        let iy0 = (((min_y - self.bbox_min_y) / self.res).floor().max(0.0) as usize).min(self.ny);
        let ix1 = (((max_x - self.bbox_min_x) / self.res).ceil() as usize).min(self.nx);
        let iy1 = (((max_y - self.bbox_min_y) / self.res).ceil() as usize).min(self.ny);

        for iy in iy0..iy1 {
            let cy = self.bbox_min_y + (iy as f32 + 0.5) * self.res;
            let row = iy * self.nx;
            for ix in ix0..ix1 {
                let cx = self.bbox_min_x + (ix as f32 + 0.5) * self.res;
                let t = if len2 < 1e-8 {
                    0.0
                } else {
                    (((cx - sx) * dx + (cy - sy) * dy) / len2).clamp(0.0, 1.0)
                };
                let px = sx + t * dx;
                let py = sy + t * dy;
                let ddx = cx - px;
                let ddy = cy - py;
                if ddx * ddx + ddy * ddy > r2 {
                    continue;
                }
                let z = sz + t * (ez - sz);
                let h = &mut self.heights[row + ix];
                if *h > z {
                    *h = z;
                }
            }
        }
    }
}

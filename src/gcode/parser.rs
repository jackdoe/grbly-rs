use crate::gcode::words::parse_words;
use crate::grbl::state::{Segment, Vec3};

struct Parser {
    pos: Vec3,
    absolute: bool,
    metric: bool,
    motion: i32,
}

pub fn parse_with_bounds(lines: &[String]) -> (Vec<Segment>, Vec3, Vec3) {
    let mut p = Parser {
        pos: Vec3::default(),
        absolute: true,
        metric: true,
        motion: 0,
    };
    let mut bmin = Vec3 {
        x: f32::MAX,
        y: f32::MAX,
        z: f32::MAX,
    };
    let mut bmax = Vec3 {
        x: f32::MIN,
        y: f32::MIN,
        z: f32::MIN,
    };

    let update_bounds = |v: Vec3, bmin: &mut Vec3, bmax: &mut Vec3| {
        if v.x < bmin.x {
            bmin.x = v.x;
        }
        if v.y < bmin.y {
            bmin.y = v.y;
        }
        if v.z < bmin.z {
            bmin.z = v.z;
        }
        if v.x > bmax.x {
            bmax.x = v.x;
        }
        if v.y > bmax.y {
            bmax.y = v.y;
        }
        if v.z > bmax.z {
            bmax.z = v.z;
        }
    };

    update_bounds(p.pos, &mut bmin, &mut bmax);

    let mut segs = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        let before = segs.len();
        p.parse_line(line, i, &mut segs);
        for s in &segs[before..] {
            update_bounds(s.start, &mut bmin, &mut bmax);
            update_bounds(s.end, &mut bmin, &mut bmax);
        }
    }
    (segs, bmin, bmax)
}

impl Parser {
    fn parse_line(&mut self, raw: &str, line_num: usize, out: &mut Vec<Segment>) {
        let words = parse_words(raw);
        if words.is_empty() {
            return;
        }

        let mut has_motion = false;
        let mut x = 0.0f64;
        let mut y = 0.0f64;
        let mut z = 0.0f64;
        let mut ii = 0.0f64;
        let mut jj = 0.0f64;
        let mut rr = 0.0f64;
        let (mut got_x, mut got_y, mut got_z) = (false, false, false);
        let (mut got_i, mut got_j) = (false, false);
        let mut got_r = false;

        for w in &words {
            match w.letter {
                b'G' => match w.value as i32 {
                    0 => {
                        self.motion = 0;
                        has_motion = true;
                    }
                    1 => {
                        self.motion = 1;
                        has_motion = true;
                    }
                    2 => {
                        self.motion = 2;
                        has_motion = true;
                    }
                    3 => {
                        self.motion = 3;
                        has_motion = true;
                    }
                    90 => {
                        self.absolute = true;
                    }
                    91 => {
                        self.absolute = false;
                    }
                    20 => {
                        self.metric = false;
                    }
                    21 => {
                        self.metric = true;
                    }
                    _ => {}
                },
                b'X' => {
                    x = w.value;
                    got_x = true;
                    has_motion = true;
                }
                b'Y' => {
                    y = w.value;
                    got_y = true;
                    has_motion = true;
                }
                b'Z' => {
                    z = w.value;
                    got_z = true;
                    has_motion = true;
                }
                b'I' => {
                    ii = w.value;
                    got_i = true;
                }
                b'J' => {
                    jj = w.value;
                    got_j = true;
                }
                b'R' => {
                    rr = w.value;
                    got_r = true;
                }
                b'F' => {}
                _ => {}
            }
        }

        if !has_motion {
            return;
        }

        let mut target = self.pos;
        let unit = if self.metric { 1.0 } else { 25.4 };
        if self.absolute {
            if got_x {
                target.x = (x * unit) as f32;
            }
            if got_y {
                target.y = (y * unit) as f32;
            }
            if got_z {
                target.z = (z * unit) as f32;
            }
        } else {
            if got_x {
                target.x += (x * unit) as f32;
            }
            if got_y {
                target.y += (y * unit) as f32;
            }
            if got_z {
                target.z += (z * unit) as f32;
            }
        }

        match self.motion {
            0 | 1 => {
                out.push(Segment {
                    start: self.pos,
                    end: target,
                    rapid: self.motion == 0,
                    line: line_num,
                });
                self.pos = target;
            }
            2 | 3 => {
                let mut center = self.pos;
                let clockwise = self.motion == 2;
                if got_r {
                    center =
                        arc_center_from_radius(self.pos, target, (rr * unit) as f32, clockwise);
                } else {
                    if got_i {
                        center.x += (ii * unit) as f32;
                    }
                    if got_j {
                        center.y += (jj * unit) as f32;
                    }
                }
                tessellate_arc(self.pos, target, center, clockwise, line_num, out);
                self.pos = target;
            }
            _ => {}
        }
    }
}

pub(crate) fn arc_center_from_radius(start: Vec3, end: Vec3, radius: f32, clockwise: bool) -> Vec3 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let chord = (dx * dx + dy * dy).sqrt();
    let r = radius.abs();
    if chord < 0.0001 || r < chord * 0.5 {
        return start;
    }

    let mx = (start.x + end.x) * 0.5;
    let my = (start.y + end.y) * 0.5;
    let h = (r * r - (chord * 0.5).powi(2)).sqrt();
    let px = -dy / chord;
    let py = dx / chord;
    let candidates = [
        Vec3 {
            x: mx + px * h,
            y: my + py * h,
            z: start.z,
        },
        Vec3 {
            x: mx - px * h,
            y: my - py * h,
            z: start.z,
        },
    ];
    let wants_major = radius < 0.0;
    candidates
        .into_iter()
        .min_by(|a, b| {
            let da = arc_sweep(start, end, *a, clockwise).abs();
            let db = arc_sweep(start, end, *b, clockwise).abs();
            let sa = if wants_major { -da } else { da };
            let sb = if wants_major { -db } else { db };
            sa.partial_cmp(&sb).unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or(start)
}

pub(crate) fn tessellate_arc_points(
    start: Vec3,
    end: Vec3,
    center: Vec3,
    clockwise: bool,
) -> Vec<Vec3> {
    let total_angle = arc_sweep(start, end, center, clockwise);
    let start_angle = ((start.y - center.y) as f64).atan2((start.x - center.x) as f64);
    let step_size = 2.0 * std::f64::consts::PI / 36.0;
    let steps = ((total_angle.abs() / step_size).max(1.0)) as usize;
    let radius =
        (((start.x - center.x) as f64).powi(2) + ((start.y - center.y) as f64).powi(2)).sqrt();
    let mut pts = Vec::with_capacity(steps);
    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let angle = start_angle + t * total_angle;
        pts.push(Vec3 {
            x: center.x + (radius * angle.cos()) as f32,
            y: center.y + (radius * angle.sin()) as f32,
            z: start.z + (t as f32) * (end.z - start.z),
        });
    }
    pts
}

fn tessellate_arc(
    start: Vec3,
    end: Vec3,
    center: Vec3,
    clockwise: bool,
    line: usize,
    out: &mut Vec<Segment>,
) {
    let pts = tessellate_arc_points(start, end, center, clockwise);
    let mut prev = start;
    for pt in pts {
        out.push(Segment {
            start: prev,
            end: pt,
            rapid: false,
            line,
        });
        prev = pt;
    }
}

fn arc_sweep(start: Vec3, end: Vec3, center: Vec3, clockwise: bool) -> f64 {
    let start_angle = ((start.y - center.y) as f64).atan2((start.x - center.x) as f64);
    let mut end_angle = ((end.y - center.y) as f64).atan2((end.x - center.x) as f64);
    if clockwise {
        if end_angle >= start_angle {
            end_angle -= 2.0 * std::f64::consts::PI;
        }
    } else if end_angle <= start_angle {
        end_angle += 2.0 * std::f64::consts::PI;
    }

    end_angle - start_angle
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lines(strs: &[&str]) -> Vec<String> {
        strs.iter().map(|s| s.to_string()).collect()
    }

    fn parse(lines: &[String]) -> Vec<Segment> {
        let (segs, _, _) = parse_with_bounds(lines);
        segs
    }

    #[test]
    fn linear_moves() {
        let segs = parse(&lines(&["G90 G21", "G0 X10 Y10 Z5", "G1 X20 Y20 Z-1 F500"]));
        assert_eq!(segs.len(), 2);
        assert!(segs[0].rapid);
        assert_eq!(
            segs[0].end,
            Vec3 {
                x: 10.0,
                y: 10.0,
                z: 5.0
            }
        );
        assert!(!segs[1].rapid);
        assert_eq!(segs[1].end.x, 20.0);
        assert_eq!(segs[1].end.y, 20.0);
    }

    #[test]
    fn incremental_mode() {
        let segs = parse(&lines(&["G91", "G0 X5 Y5", "G0 X5 Y5"]));
        assert_eq!(segs.len(), 2);
        assert_eq!(segs[1].end.x, 10.0);
        assert_eq!(segs[1].end.y, 10.0);
    }

    #[test]
    fn arc_cw() {
        let segs = parse(&lines(&["G90 G21", "G0 X10 Y0", "G2 X0 Y10 I-10 J0"]));
        let arc_segs: Vec<_> = segs.iter().filter(|s| !s.rapid).collect();
        assert!(arc_segs.len() >= 8);
        let last = &segs[segs.len() - 1];
        assert!((last.end.x).abs() < 0.1);
        assert!((last.end.y - 10.0).abs() < 0.1);
    }

    #[test]
    fn comment_stripping() {
        let segs = parse(&lines(&[
            "G0 X10 (this is a comment)",
            "; full line comment",
            "G0 X20",
        ]));
        assert_eq!(segs.len(), 2);
    }

    #[test]
    fn bounds() {
        let (_, bmin, bmax) = parse_with_bounds(&lines(&["G0 X-5 Y-10", "G0 X50 Y30 Z-3"]));
        assert_eq!(bmin.x, -5.0);
        assert_eq!(bmin.y, -10.0);
        assert_eq!(bmin.z, -3.0);
        assert_eq!(bmax.x, 50.0);
        assert_eq!(bmax.y, 30.0);
        assert_eq!(bmax.z, 0.0);
    }

    #[test]
    fn inch_mode_scales_to_mm() {
        let segs = parse(&lines(&["G20", "G0 X1 Y0.5"]));
        assert_eq!(segs[0].end.x, 25.4);
        assert_eq!(segs[0].end.y, 12.7);
    }

    #[test]
    fn arc_with_radius_word() {
        let segs = parse(&lines(&["G90 G21", "G0 X10 Y0", "G3 X0 Y10 R10"]));
        assert!(segs.len() > 2);
        let last = segs.last().unwrap();
        assert!((last.end.x).abs() < 0.1);
        assert!((last.end.y - 10.0).abs() < 0.1);
    }
}

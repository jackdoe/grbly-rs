use crate::grbl::state::{Segment, Vec3};

struct Word {
    letter: u8,
    value: f64,
}

struct Parser {
    pos: Vec3,
    absolute: bool,
    metric: bool,
    motion: i32,
}

pub fn parse_with_bounds(lines: &[String]) -> (Vec<Segment>, Vec3, Vec3) {
    let mut p = Parser { pos: Vec3::default(), absolute: true, metric: true, motion: 0 };
    let mut bmin = Vec3 { x: f32::MAX, y: f32::MAX, z: f32::MAX };
    let mut bmax = Vec3 { x: f32::MIN, y: f32::MIN, z: f32::MIN };

    let update_bounds = |v: Vec3, bmin: &mut Vec3, bmax: &mut Vec3| {
        if v.x < bmin.x { bmin.x = v.x; }
        if v.y < bmin.y { bmin.y = v.y; }
        if v.z < bmin.z { bmin.z = v.z; }
        if v.x > bmax.x { bmax.x = v.x; }
        if v.y > bmax.y { bmax.y = v.y; }
        if v.z > bmax.z { bmax.z = v.z; }
    };

    update_bounds(p.pos, &mut bmin, &mut bmax);

    let mut segs = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        for s in p.parse_line(line, i) {
            update_bounds(s.start, &mut bmin, &mut bmax);
            update_bounds(s.end, &mut bmin, &mut bmax);
            segs.push(s);
        }
    }
    (segs, bmin, bmax)
}

fn strip_comments(line: &str) -> String {
    let line = if let Some(idx) = line.find(';') { &line[..idx] } else { line };
    let mut out = String::new();
    let mut depth = 0i32;
    for c in line.chars() {
        match c {
            '(' => depth += 1,
            ')' => { depth -= 1; }
            _ if depth == 0 => out.push(c),
            _ => {}
        }
    }
    out.trim().to_string()
}

fn parse_words(s: &str) -> Vec<Word> {
    let s = s.trim();
    let bytes = s.as_bytes();
    let mut words = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b' ' || bytes[i] == b'\t' {
            i += 1;
            continue;
        }
        let b = bytes[i];
        let is_alpha = (b >= b'A' && b <= b'Z') || (b >= b'a' && b <= b'z');
        if !is_alpha {
            i += 1;
            continue;
        }
        let letter = b & 0xDF;
        i += 1;
        let j = i;
        while i < bytes.len() && (bytes[i] == b'-' || bytes[i] == b'.' || (bytes[i] >= b'0' && bytes[i] <= b'9')) {
            i += 1;
        }
        let val: f64 = s[j..i].parse().unwrap_or(0.0);
        words.push(Word { letter, value: val });
    }
    words
}

impl Parser {
    fn parse_line(&mut self, raw: &str, line_num: usize) -> Vec<Segment> {
        let clean = strip_comments(raw);
        if clean.is_empty() {
            return Vec::new();
        }

        let words = parse_words(&clean);

        let mut has_motion = false;
        let mut x = 0.0f64;
        let mut y = 0.0f64;
        let mut z = 0.0f64;
        let mut ii = 0.0f64;
        let mut jj = 0.0f64;
        let (mut got_x, mut got_y, mut got_z) = (false, false, false);
        let (mut got_i, mut got_j) = (false, false);

        for w in &words {
            match w.letter {
                b'G' => {
                    match w.value as i32 {
                        0 => { self.motion = 0; has_motion = true; }
                        1 => { self.motion = 1; has_motion = true; }
                        2 => { self.motion = 2; has_motion = true; }
                        3 => { self.motion = 3; has_motion = true; }
                        90 => { self.absolute = true; }
                        91 => { self.absolute = false; }
                        20 => { self.metric = false; }
                        21 => { self.metric = true; }
                        _ => {}
                    }
                }
                b'X' => { x = w.value; got_x = true; has_motion = true; }
                b'Y' => { y = w.value; got_y = true; has_motion = true; }
                b'Z' => { z = w.value; got_z = true; has_motion = true; }
                b'I' => { ii = w.value; got_i = true; }
                b'J' => { jj = w.value; got_j = true; }
                b'F' => {}
                _ => {}
            }
        }

        if !has_motion {
            return Vec::new();
        }

        let mut target = self.pos;
        if self.absolute {
            if got_x { target.x = x as f32; }
            if got_y { target.y = y as f32; }
            if got_z { target.z = z as f32; }
        } else {
            if got_x { target.x += x as f32; }
            if got_y { target.y += y as f32; }
            if got_z { target.z += z as f32; }
        }

        match self.motion {
            0 | 1 => {
                let seg = Segment {
                    start: self.pos,
                    end: target,
                    rapid: self.motion == 0,
                    line: line_num,
                };
                self.pos = target;
                vec![seg]
            }
            2 | 3 => {
                let mut center = self.pos;
                if got_i { center.x += ii as f32; }
                if got_j { center.y += jj as f32; }
                let segs = tessellate_arc(self.pos, target, center, self.motion == 2, line_num);
                self.pos = target;
                segs
            }
            _ => Vec::new(),
        }
    }
}

fn tessellate_arc(start: Vec3, end: Vec3, center: Vec3, clockwise: bool, line: usize) -> Vec<Segment> {
    let start_angle = ((start.y - center.y) as f64).atan2((start.x - center.x) as f64);
    let mut end_angle = ((end.y - center.y) as f64).atan2((end.x - center.x) as f64);

    if clockwise {
        if end_angle >= start_angle {
            end_angle -= 2.0 * std::f64::consts::PI;
        }
    } else if end_angle <= start_angle {
        end_angle += 2.0 * std::f64::consts::PI;
    }

    let total_angle = end_angle - start_angle;
    let step_size = 2.0 * std::f64::consts::PI / 36.0;
    let steps = ((total_angle.abs() / step_size).max(1.0)) as usize;
    let radius = (((start.x - center.x) as f64).powi(2) + ((start.y - center.y) as f64).powi(2)).sqrt();

    let mut segs = Vec::with_capacity(steps);
    let mut prev = start;
    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let angle = start_angle + t * total_angle;
        let pt = Vec3 {
            x: center.x + (radius * angle.cos()) as f32,
            y: center.y + (radius * angle.sin()) as f32,
            z: start.z + (t as f32) * (end.z - start.z),
        };
        segs.push(Segment { start: prev, end: pt, rapid: false, line });
        prev = pt;
    }
    segs
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
        assert_eq!(segs[0].end, Vec3 { x: 10.0, y: 10.0, z: 5.0 });
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
        let segs = parse(&lines(&["G0 X10 (this is a comment)", "; full line comment", "G0 X20"]));
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
}

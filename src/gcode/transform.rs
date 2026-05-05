use crate::gcode::parser::{arc_center_from_radius, tessellate_arc_points};
use crate::gcode::words::{parse_words, strip_comments, Word};
use crate::grbl::heightmap::HeightMap;
use crate::grbl::state::Vec3;

const CHUNK_MM: f32 = 1.0;

struct ModalState {
    pos: Vec3,
    absolute: bool,
    metric: bool,
    motion: i32,
    feed: f32,
}

impl Default for ModalState {
    fn default() -> Self {
        Self {
            pos: Vec3::default(),
            absolute: true,
            metric: true,
            motion: 0,
            feed: 0.0,
        }
    }
}

pub fn transform_for_stream(
    lines: &[String],
    heightmap: Option<&HeightMap>,
) -> (Vec<String>, Vec<usize>) {
    let Some(hmap) = heightmap else {
        let src: Vec<usize> = (0..lines.len()).collect();
        return (lines.to_vec(), src);
    };

    let mut out_lines: Vec<String> = Vec::new();
    let mut out_src: Vec<usize> = Vec::new();
    let mut state = ModalState::default();

    for (i, raw) in lines.iter().enumerate() {
        let stripped = strip_comments(raw);
        if stripped.is_empty() {
            continue;
        }
        process_line(&stripped, i, &mut state, hmap, &mut out_lines, &mut out_src);
    }

    (out_lines, out_src)
}

fn process_line(
    line: &str,
    src: usize,
    state: &mut ModalState,
    hmap: &HeightMap,
    out_lines: &mut Vec<String>,
    out_src: &mut Vec<usize>,
) {
    let words = parse_words(line);

    let mut has_motion_word = false;
    let (mut x, mut y, mut z) = (0.0f64, 0.0f64, 0.0f64);
    let (mut ii, mut jj, mut rr) = (0.0f64, 0.0f64, 0.0f64);
    let (mut got_x, mut got_y, mut got_z) = (false, false, false);
    let (mut got_i, mut got_j) = (false, false);
    let mut got_r = false;
    let mut motion_override: Option<i32> = None;
    let mut feed_in_line: Option<f32> = None;
    let mut other_words: Vec<String> = Vec::new();

    for w in &words {
        match w.letter {
            b'G' => {
                let n = w.value as i32;
                match n {
                    0 | 1 | 2 | 3 => {
                        motion_override = Some(n);
                        has_motion_word = true;
                    }
                    20 => state.metric = false,
                    21 => state.metric = true,
                    90 => state.absolute = true,
                    91 => state.absolute = false,
                    _ => other_words.push(format_word(w)),
                }
            }
            b'X' => {
                x = w.value;
                got_x = true;
                has_motion_word = true;
            }
            b'Y' => {
                y = w.value;
                got_y = true;
                has_motion_word = true;
            }
            b'Z' => {
                z = w.value;
                got_z = true;
                has_motion_word = true;
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
            b'F' => {
                feed_in_line = Some(w.value as f32);
            }
            _ => other_words.push(format_word(w)),
        }
    }

    if let Some(m) = motion_override {
        state.motion = m;
    }
    if let Some(f) = feed_in_line {
        state.feed = f;
    }

    if !has_motion_word {
        out_lines.push(line.to_string());
        out_src.push(src);
        return;
    }

    let unit: f32 = if state.metric { 1.0 } else { 25.4 };
    let mut target = state.pos;
    if state.absolute {
        if got_x {
            target.x = (x as f32) * unit;
        }
        if got_y {
            target.y = (y as f32) * unit;
        }
        if got_z {
            target.z = (z as f32) * unit;
        }
    } else {
        if got_x {
            target.x += (x as f32) * unit;
        }
        if got_y {
            target.y += (y as f32) * unit;
        }
        if got_z {
            target.z += (z as f32) * unit;
        }
    }

    if !other_words.is_empty() {
        out_lines.push(other_words.join(" "));
        out_src.push(src);
    }

    let rapid = state.motion == 0;
    let arc = state.motion == 2 || state.motion == 3;
    let clockwise = state.motion == 2;

    if !arc {
        emit_subdiv(state.pos, target, rapid, state.feed, hmap, src, out_lines, out_src);
    } else {
        let mut center = state.pos;
        if got_r {
            center =
                arc_center_from_radius(state.pos, target, (rr as f32) * unit, clockwise);
        } else {
            if got_i {
                center.x += (ii as f32) * unit;
            }
            if got_j {
                center.y += (jj as f32) * unit;
            }
        }
        let pts = tessellate_arc_points(state.pos, target, center, clockwise);
        let mut prev = state.pos;
        for pt in pts {
            emit_subdiv(prev, pt, false, state.feed, hmap, src, out_lines, out_src);
            prev = pt;
        }
    }

    state.pos = target;
}

fn emit_subdiv(
    start: Vec3,
    end: Vec3,
    rapid: bool,
    feed: f32,
    hmap: &HeightMap,
    src: usize,
    out_lines: &mut Vec<String>,
    out_src: &mut Vec<usize>,
) {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let xy_len = (dx * dx + dy * dy).sqrt();
    let n = if xy_len < 0.001 {
        1
    } else {
        (xy_len / CHUNK_MM).ceil() as usize
    };

    for k in 1..=n {
        let t = k as f32 / n as f32;
        let sub = start.lerp(end, t);
        let dz = hmap.dz(sub.x, sub.y);
        let z_total = sub.z + dz;
        let line = if rapid {
            format!("G90 G21 G0 X{:.3} Y{:.3} Z{:.4}", sub.x, sub.y, z_total)
        } else if feed > 0.0 {
            format!(
                "G90 G21 G1 X{:.3} Y{:.3} Z{:.4} F{:.1}",
                sub.x, sub.y, z_total, feed
            )
        } else {
            format!("G90 G21 G1 X{:.3} Y{:.3} Z{:.4}", sub.x, sub.y, z_total)
        };
        out_lines.push(line);
        out_src.push(src);
    }
}

fn format_word(w: &Word) -> String {
    if w.value == w.value.trunc() && w.value.abs() < 1e9 {
        format!("{}{}", w.letter as char, w.value as i64)
    } else {
        format!("{}{}", w.letter as char, w.value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_map() -> HeightMap {
        HeightMap::new((0.0, 0.0), (50.0, 50.0), 2, 2, vec![0.0; 4]).unwrap()
    }

    fn tilt_map() -> HeightMap {
        HeightMap::new(
            (0.0, 0.0),
            (50.0, 50.0),
            2,
            2,
            vec![0.0, 0.5, 0.0, 0.5],
        )
        .unwrap()
    }

    fn lines(strs: &[&str]) -> Vec<String> {
        strs.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn no_heightmap_is_identity() {
        let input = lines(&["G90 G21", "G0 X10", "G1 X20 F100", "M3 S1000"]);
        let (out, src) = transform_for_stream(&input, None);
        assert_eq!(out, input);
        assert_eq!(src, vec![0, 1, 2, 3]);
    }

    #[test]
    fn flat_map_preserves_z_at_end_of_move() {
        let m = flat_map();
        let input = lines(&["G90 G21", "G1 X10 Y0 Z-0.1 F200"]);
        let (out, _src) = transform_for_stream(&input, Some(&m));
        let last = out.iter().rev().find(|l| l.contains("G1")).unwrap();
        assert!(last.contains("X10.000"), "{last}");
        assert!(last.contains("Z-0.1000"), "{last}");
    }

    #[test]
    fn tilt_map_offsets_z_along_x() {
        let m = tilt_map();
        let input = lines(&["G90 G21", "G1 X50 Y0 Z0 F200"]);
        let (out, _) = transform_for_stream(&input, Some(&m));
        let last_motion = out
            .iter()
            .rev()
            .find(|l| l.contains("G1") && l.contains("X"))
            .unwrap();
        assert!(last_motion.contains("X50.000"));
        assert!(last_motion.contains("Z0.5000"));
        let first_motion = out.iter().find(|l| l.contains("G1") && l.contains("X1.")).unwrap();
        assert!(first_motion.contains("Z0.0100"));
    }

    #[test]
    fn subdivision_count_matches_chunk_size() {
        let m = flat_map();
        let input = lines(&["G90 G21", "G1 X10 Y0 F100"]);
        let (out, _) = transform_for_stream(&input, Some(&m));
        assert_eq!(out.iter().filter(|l| l.contains("G1")).count(), 10);
    }

    #[test]
    fn z_only_move_emits_single_line() {
        let m = tilt_map();
        let input = lines(&["G90 G21", "G0 X25 Y25", "G1 Z-0.1 F100"]);
        let (out, _) = transform_for_stream(&input, Some(&m));
        let z_lines: Vec<_> = out
            .iter()
            .filter(|l| l.contains("G1") && !l.contains("F100"))
            .collect();
        let _ = z_lines;
        let g1_count = out.iter().filter(|l| l.contains("G1")).count();
        assert_eq!(g1_count, 1);
        let line = out.iter().find(|l| l.contains("G1")).unwrap();
        assert!(line.contains("X25.000"));
        assert!(line.contains("Y25.000"));
    }

    #[test]
    fn passes_through_non_motion_lines() {
        let m = flat_map();
        let input = lines(&["G90 G21", "M3 S1000", "G1 X1 F100", "M5"]);
        let (out, _) = transform_for_stream(&input, Some(&m));
        assert!(out.iter().any(|l| l == "M3 S1000"));
        assert!(out.iter().any(|l| l == "M5"));
    }

    #[test]
    fn incremental_mode_resolved_to_absolute() {
        let m = flat_map();
        let input = lines(&["G91", "G1 X5 Y0 F100", "G1 X5 Y0"]);
        let (out, _) = transform_for_stream(&input, Some(&m));
        let last = out.iter().rev().find(|l| l.contains("G1")).unwrap();
        assert!(last.contains("X10.000"));
    }

    #[test]
    fn inch_mode_scales_to_mm() {
        let m = flat_map();
        let input = lines(&["G20", "G1 X1 Y0 F100"]);
        let (out, _) = transform_for_stream(&input, Some(&m));
        let last = out.iter().rev().find(|l| l.contains("G1")).unwrap();
        assert!(last.contains("X25.400"));
    }

    #[test]
    fn source_line_indices_track_original() {
        let m = flat_map();
        let input = lines(&["G90 G21", "G1 X10 Y0 F100", "M5"]);
        let (out, src) = transform_for_stream(&input, Some(&m));
        assert_eq!(src.len(), out.len());
        let m5_idx = out.iter().position(|l| l == "M5").unwrap();
        assert_eq!(src[m5_idx], 2);
    }
}

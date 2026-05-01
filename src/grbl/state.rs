#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub enum Status {
    #[default]
    Disconnected,
    Idle,
    Run,
    Hold,
    Alarm,
    Home,
    Check,
    Jog,
    Door,
    Sleep,
}

#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn dist(self, other: Vec3) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn lerp(self, other: Vec3, t: f32) -> Vec3 {
        Vec3 {
            x: self.x + (other.x - self.x) * t,
            y: self.y + (other.y - self.y) * t,
            z: self.z + (other.z - self.z) * t,
        }
    }
}

#[derive(Clone, Default, Debug)]
pub struct MachineState {
    pub port: String,
    pub baud: u32,
    pub connected: bool,
    pub status: Status,
    pub mpos: Vec3,
    pub wpos: Vec3,
    pub wco: Vec3,
    pub feed: f32,
    pub feed_ovr: i32,
    pub spindle: f32,
    pub spindle_ovr: i32,
    pub alarm_code: i32,
    pub soft_limits: bool,
    pub max_travel: Vec3,
    pub last_error: String,
}

#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub enum JobStatus {
    #[default]
    Idle,
    Running,
    Paused,
    Complete,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub start: Vec3,
    pub end: Vec3,
    pub rapid: bool,
    pub line: usize,
}

use std::collections::HashMap;
use std::sync::Arc;

#[derive(Clone, Default, Debug)]
pub struct JobState {
    pub lines: Arc<Vec<String>>,
    pub current_line: usize,
    pub status: JobStatus,
    pub segments: Arc<Vec<Segment>>,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
    pub z_locked: bool,
    pub total_dist: f32,
    pub seg_violations: Arc<Vec<bool>>,
    pub violated_lines: Arc<Vec<bool>>,
    pub seg_pass_counts: Arc<Vec<u16>>,
    pub line_pass_counts: Arc<Vec<u16>>,
    pub pass_tolerance_mm: f32,
    pub version: usize,
}

pub const DEFAULT_PASS_TOLERANCE_MM: f32 = 0.05;

pub fn normalize_pass_tolerance(tolerance_mm: f32) -> f32 {
    tolerance_mm.max(0.001)
}

pub fn compute_pass_counts(
    segments: &[Segment],
    line_count: usize,
    tolerance_mm: f32,
) -> (Vec<u16>, Vec<u16>) {
    let tolerance_mm = normalize_pass_tolerance(tolerance_mm);
    let angle_step = angle_bucket_size(tolerance_mm);
    let mut line_groups: Vec<PathLineGroup> = Vec::new();
    let mut group_buckets: HashMap<PathLineKey, Vec<usize>> = HashMap::new();

    for (idx, segment) in segments.iter().enumerate() {
        let Some(projection) = PathProjection::new(segment, tolerance_mm, angle_step) else {
            continue;
        };

        let group_id = if let Some(group_id) =
            find_matching_line_group(&projection, &line_groups, &group_buckets, tolerance_mm)
        {
            group_id
        } else {
            let group_id = line_groups.len();
            line_groups.push(PathLineGroup::new(projection));
            group_buckets
                .entry(projection.key)
                .or_default()
                .push(group_id);
            group_id
        };

        let group = &line_groups[group_id];
        if let Some(interval) = PathInterval::new(idx, segment, group.dir_x, group.dir_y) {
            line_groups[group_id].intervals.push(interval);
        }
    }

    let mut seg_counts = vec![1; segments.len()];
    for group in &line_groups {
        apply_path_coverage_counts(&group.intervals, &mut seg_counts);
    }

    let mut line_counts = vec![1; line_count];
    for (idx, segment) in segments.iter().enumerate() {
        if segment.line < line_counts.len() {
            line_counts[segment.line] = line_counts[segment.line].max(seg_counts[idx]);
        }
    }

    (seg_counts, line_counts)
}

pub fn pass_counts_are_current(job: &JobState, tolerance_mm: f32) -> bool {
    (job.pass_tolerance_mm - tolerance_mm).abs() <= f32::EPSILON
        && job.seg_pass_counts.len() == job.segments.len()
        && job.line_pass_counts.len() == job.lines.len()
}

const MIN_PATH_LEN_MM: f32 = 0.001;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PathLineKey {
    angle: i32,
    offset: i32,
    rapid: bool,
}

#[derive(Clone, Copy, Debug)]
struct PathProjection {
    key: PathLineKey,
    theta: f32,
    dir_x: f32,
    dir_y: f32,
    offset: f32,
    start: Vec3,
    end: Vec3,
}

impl PathProjection {
    fn new(segment: &Segment, tolerance_mm: f32, angle_step: f32) -> Option<Self> {
        let dx = segment.end.x - segment.start.x;
        let dy = segment.end.y - segment.start.y;
        let xy_len = (dx * dx + dy * dy).sqrt();
        if xy_len < MIN_PATH_LEN_MM || !xy_len.is_finite() {
            return None;
        }

        let mut theta = dy.atan2(dx);
        if theta < 0.0 {
            theta += std::f32::consts::PI;
        }
        if theta >= std::f32::consts::PI {
            theta -= std::f32::consts::PI;
        }

        let angle = normalize_angle_bucket((theta / angle_step).round() as i32, angle_step);

        let theta_q = angle as f32 * angle_step;
        let dir_x = theta_q.cos();
        let dir_y = theta_q.sin();
        let normal_x = -dir_y;
        let normal_y = dir_x;
        let offset = segment.start.x * normal_x + segment.start.y * normal_y;
        let offset_bucket = quantize(offset, tolerance_mm);

        Some(Self {
            key: PathLineKey {
                angle,
                offset: offset_bucket,
                rapid: segment.rapid,
            },
            theta: theta_q,
            dir_x,
            dir_y,
            offset,
            start: segment.start,
            end: segment.end,
        })
    }
}

#[derive(Clone, Copy, Debug)]
struct PathInterval {
    segment_index: usize,
    start: f32,
    end: f32,
}

impl PathInterval {
    fn new(segment_index: usize, segment: &Segment, dir_x: f32, dir_y: f32) -> Option<Self> {
        let a = segment.start.x * dir_x + segment.start.y * dir_y;
        let b = segment.end.x * dir_x + segment.end.y * dir_y;
        let start = a.min(b);
        let end = a.max(b);
        if end - start < MIN_PATH_LEN_MM {
            return None;
        }
        Some(Self {
            segment_index,
            start,
            end,
        })
    }
}

#[derive(Clone, Debug)]
struct PathLineGroup {
    key: PathLineKey,
    theta: f32,
    dir_x: f32,
    dir_y: f32,
    normal_x: f32,
    normal_y: f32,
    offset: f32,
    intervals: Vec<PathInterval>,
}

impl PathLineGroup {
    fn new(projection: PathProjection) -> Self {
        Self {
            key: projection.key,
            theta: projection.theta,
            dir_x: projection.dir_x,
            dir_y: projection.dir_y,
            normal_x: -projection.dir_y,
            normal_y: projection.dir_x,
            offset: projection.offset,
            intervals: Vec::new(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct PathCoverageEvent {
    position: f32,
    delta: i32,
}

#[derive(Clone, Copy, Debug)]
struct PathCoverageSpan {
    start: f32,
    end: f32,
    count: u16,
}

fn find_matching_line_group(
    projection: &PathProjection,
    groups: &[PathLineGroup],
    buckets: &HashMap<PathLineKey, Vec<usize>>,
    tolerance_mm: f32,
) -> Option<usize> {
    let angle_step = angle_bucket_size(tolerance_mm);
    for angle in projection.key.angle - 1..=projection.key.angle + 1 {
        for offset in projection.key.offset - 1..=projection.key.offset + 1 {
            let key = PathLineKey {
                angle: normalize_angle_bucket(angle, angle_step),
                offset,
                rapid: projection.key.rapid,
            };
            let Some(group_ids) = buckets.get(&key) else {
                continue;
            };
            for &group_id in group_ids {
                let group = &groups[group_id];
                if projection_fits_group(projection, group, tolerance_mm, angle_step) {
                    return Some(group_id);
                }
            }
        }
    }
    None
}

fn projection_fits_group(
    projection: &PathProjection,
    group: &PathLineGroup,
    tolerance_mm: f32,
    angle_step: f32,
) -> bool {
    projection.key.rapid == group.key.rapid
        && directionless_angle_diff(projection.theta, group.theta) <= angle_step
        && point_line_distance(projection.start, group).abs() <= tolerance_mm
        && point_line_distance(projection.end, group).abs() <= tolerance_mm
}

fn point_line_distance(point: Vec3, group: &PathLineGroup) -> f32 {
    point.x * group.normal_x + point.y * group.normal_y - group.offset
}

fn apply_path_coverage_counts(intervals: &[PathInterval], seg_counts: &mut [u16]) {
    if intervals.len() < 2 {
        return;
    }

    let mut events = Vec::with_capacity(intervals.len() * 2);
    for interval in intervals {
        events.push(PathCoverageEvent {
            position: interval.start,
            delta: 1,
        });
        events.push(PathCoverageEvent {
            position: interval.end,
            delta: -1,
        });
    }
    events.sort_by(|a, b| a.position.total_cmp(&b.position));

    let mut spans = Vec::with_capacity(events.len().saturating_sub(1));
    let mut coverage = 0i32;
    let mut previous = None;
    let mut i = 0;
    while i < events.len() {
        let position = events[i].position;
        if let Some(start) = previous {
            if position - start >= MIN_PATH_LEN_MM && coverage > 0 {
                spans.push(PathCoverageSpan {
                    start,
                    end: position,
                    count: coverage.min(u16::MAX as i32) as u16,
                });
            }
        }

        while i < events.len() && events[i].position == position {
            coverage += events[i].delta;
            i += 1;
        }
        previous = Some(position);
    }

    if spans.is_empty() {
        return;
    }

    let coverage_tree = RangeMax::new(&spans.iter().map(|span| span.count).collect::<Vec<_>>());
    for interval in intervals {
        let first = spans.partition_point(|span| span.end <= interval.start + MIN_PATH_LEN_MM);
        let last = spans.partition_point(|span| span.start < interval.end - MIN_PATH_LEN_MM);
        if first < last {
            let count = coverage_tree.query(first, last).max(1);
            seg_counts[interval.segment_index] = seg_counts[interval.segment_index].max(count);
        }
    }
}

struct RangeMax {
    base: usize,
    nodes: Vec<u16>,
}

impl RangeMax {
    fn new(values: &[u16]) -> Self {
        let base = values.len().next_power_of_two();
        let mut nodes = vec![0; base * 2];
        nodes[base..base + values.len()].copy_from_slice(values);
        for idx in (1..base).rev() {
            nodes[idx] = nodes[idx * 2].max(nodes[idx * 2 + 1]);
        }
        Self { base, nodes }
    }

    fn query(&self, mut start: usize, mut end: usize) -> u16 {
        start += self.base;
        end += self.base;
        let mut out = 0;
        while start < end {
            if start % 2 == 1 {
                out = out.max(self.nodes[start]);
                start += 1;
            }
            if end % 2 == 1 {
                end -= 1;
                out = out.max(self.nodes[end]);
            }
            start /= 2;
            end /= 2;
        }
        out
    }
}

fn quantize(v: f32, tolerance_mm: f32) -> i32 {
    (v / tolerance_mm).round() as i32
}

fn angle_bucket_size(tolerance_mm: f32) -> f32 {
    (tolerance_mm / 10.0).clamp(0.001, 0.05)
}

fn normalize_angle_bucket(bucket: i32, angle_step: f32) -> i32 {
    let bins = (std::f32::consts::PI / angle_step).round() as i32;
    bucket.rem_euclid(bins)
}

fn directionless_angle_diff(a: f32, b: f32) -> f32 {
    let diff = (a - b).abs();
    diff.min(std::f32::consts::PI - diff)
}

pub fn recompute_soft_limit_violations(job: &mut JobState, machine: &MachineState) -> bool {
    let mut seg_violations = vec![false; job.segments.len()];
    let mut violated_lines = vec![false; job.lines.len()];

    if let Some((bmin, bmax)) = soft_limit_bounds(machine) {
        for (idx, segment) in job.segments.iter().enumerate() {
            let violated = !inside_bounds(segment.start, bmin, bmax)
                || !inside_bounds(segment.end, bmin, bmax);
            seg_violations[idx] = violated;
            if violated && segment.line < violated_lines.len() {
                violated_lines[segment.line] = true;
            }
        }
    }

    if seg_violations.as_slice() == job.seg_violations.as_ref().as_slice()
        && violated_lines.as_slice() == job.violated_lines.as_ref().as_slice()
    {
        return false;
    }

    job.seg_violations = Arc::new(seg_violations);
    job.violated_lines = Arc::new(violated_lines);
    job.version = job.version.wrapping_add(1);
    true
}

pub fn soft_limit_bounds(machine: &MachineState) -> Option<(Vec3, Vec3)> {
    let mt = machine.max_travel;
    if !machine.connected || !machine.soft_limits || mt.x <= 0.0 || mt.y <= 0.0 || mt.z <= 0.0 {
        return None;
    }

    let home_w = Vec3 {
        x: -machine.wco.x,
        y: -machine.wco.y,
        z: -machine.wco.z,
    };
    let far_w = Vec3 {
        x: mt.x - machine.wco.x,
        y: mt.y - machine.wco.y,
        z: -mt.z - machine.wco.z,
    };
    Some((
        Vec3 {
            x: home_w.x.min(far_w.x),
            y: home_w.y.min(far_w.y),
            z: home_w.z.min(far_w.z),
        },
        Vec3 {
            x: home_w.x.max(far_w.x),
            y: home_w.y.max(far_w.y),
            z: home_w.z.max(far_w.z),
        },
    ))
}

fn inside_bounds(v: Vec3, bmin: Vec3, bmax: Vec3) -> bool {
    v.x >= bmin.x
        && v.x <= bmax.x
        && v.y >= bmin.y
        && v.y <= bmax.y
        && v.z >= bmin.z
        && v.z <= bmax.z
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recomputes_soft_limit_violations() {
        let mut job = JobState {
            lines: Arc::new(vec!["G0 X10".into(), "G0 X200".into()]),
            segments: Arc::new(vec![
                Segment {
                    start: Vec3::default(),
                    end: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    rapid: true,
                    line: 0,
                },
                Segment {
                    start: Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    end: Vec3 {
                        x: 200.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    rapid: true,
                    line: 1,
                },
            ]),
            ..Default::default()
        };
        let machine = MachineState {
            connected: true,
            soft_limits: true,
            max_travel: Vec3 {
                x: 100.0,
                y: 100.0,
                z: 20.0,
            },
            ..Default::default()
        };

        assert!(recompute_soft_limit_violations(&mut job, &machine));
        assert_eq!(job.seg_violations.as_ref().as_slice(), &[false, true]);
        assert_eq!(job.violated_lines.as_ref().as_slice(), &[false, true]);
    }

    #[test]
    fn computes_repeated_segment_pass_counts() {
        let segments = vec![
            Segment {
                start: Vec3::default(),
                end: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                rapid: false,
                line: 0,
            },
            Segment {
                start: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                end: Vec3::default(),
                rapid: false,
                line: 1,
            },
            Segment {
                start: Vec3::default(),
                end: Vec3 {
                    x: 0.0,
                    y: 10.0,
                    z: 0.0,
                },
                rapid: false,
                line: 2,
            },
        ];

        let (seg_counts, line_counts) =
            compute_pass_counts(&segments, 3, DEFAULT_PASS_TOLERANCE_MM);

        assert_eq!(seg_counts, vec![2, 2, 1]);
        assert_eq!(line_counts, vec![2, 2, 1]);
    }

    #[test]
    fn rapid_and_cut_pass_counts_are_separate() {
        let segments = vec![
            Segment {
                start: Vec3::default(),
                end: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                rapid: false,
                line: 0,
            },
            Segment {
                start: Vec3::default(),
                end: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                rapid: true,
                line: 1,
            },
        ];

        let (seg_counts, line_counts) =
            compute_pass_counts(&segments, 2, DEFAULT_PASS_TOLERANCE_MM);

        assert_eq!(seg_counts, vec![1, 1]);
        assert_eq!(line_counts, vec![1, 1]);
    }

    #[test]
    fn tolerance_controls_repeated_segment_matching() {
        let segments = vec![
            Segment {
                start: Vec3::default(),
                end: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                rapid: false,
                line: 0,
            },
            Segment {
                start: Vec3 {
                    x: 0.02,
                    y: -0.01,
                    z: 0.0,
                },
                end: Vec3 {
                    x: 10.03,
                    y: 0.01,
                    z: 0.0,
                },
                rapid: false,
                line: 1,
            },
        ];

        let (loose_seg_counts, loose_line_counts) = compute_pass_counts(&segments, 2, 0.05);
        let (tight_seg_counts, tight_line_counts) = compute_pass_counts(&segments, 2, 0.005);

        assert_eq!(loose_seg_counts, vec![2, 2]);
        assert_eq!(loose_line_counts, vec![2, 2]);
        assert_eq!(tight_seg_counts, vec![1, 1]);
        assert_eq!(tight_line_counts, vec![1, 1]);
    }

    #[test]
    fn repeated_paths_match_across_z_heights() {
        let segments = vec![
            Segment {
                start: Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 5.0,
                },
                end: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 5.0,
                },
                rapid: true,
                line: 0,
            },
            Segment {
                start: Vec3 {
                    x: 0.01,
                    y: 0.0,
                    z: 12.0,
                },
                end: Vec3 {
                    x: 10.01,
                    y: 0.0,
                    z: 12.0,
                },
                rapid: true,
                line: 1,
            },
            Segment {
                start: Vec3 {
                    x: 3.0,
                    y: 3.0,
                    z: 0.0,
                },
                end: Vec3 {
                    x: 3.0,
                    y: 3.0,
                    z: 8.0,
                },
                rapid: true,
                line: 2,
            },
        ];

        let (seg_counts, line_counts) = compute_pass_counts(&segments, 3, 0.05);

        assert_eq!(seg_counts, vec![2, 2, 1]);
        assert_eq!(line_counts, vec![2, 2, 1]);
    }

    #[test]
    fn overlapping_collinear_paths_count_as_repeated() {
        let segments = vec![
            Segment {
                start: Vec3::default(),
                end: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 5.0,
                },
                rapid: true,
                line: 0,
            },
            Segment {
                start: Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 8.0,
                },
                end: Vec3 {
                    x: 8.0,
                    y: 0.0,
                    z: 8.0,
                },
                rapid: true,
                line: 1,
            },
            Segment {
                start: Vec3 {
                    x: 4.0,
                    y: 0.0,
                    z: 12.0,
                },
                end: Vec3 {
                    x: 6.0,
                    y: 0.0,
                    z: 12.0,
                },
                rapid: true,
                line: 2,
            },
            Segment {
                start: Vec3 {
                    x: 10.0,
                    y: 0.0,
                    z: 2.0,
                },
                end: Vec3::default(),
                rapid: true,
                line: 3,
            },
        ];

        let (seg_counts, line_counts) = compute_pass_counts(&segments, 4, 0.05);

        assert_eq!(seg_counts, vec![4, 4, 4, 4]);
        assert_eq!(line_counts, vec![4, 4, 4, 4]);
    }
}

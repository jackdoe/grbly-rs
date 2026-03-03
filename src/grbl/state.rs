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
}


#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
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

#[derive(Clone, Default, Debug)]
pub struct JobState {
    pub lines: Vec<String>,
    pub current_line: usize,
    pub status: JobStatus,
    pub segments: Vec<Segment>,
    pub bounds_min: Vec3,
    pub bounds_max: Vec3,
    pub z_locked: bool,
}

pub struct MachineProfile {
    pub envelope: Vec3,
}

pub const CUBIKO: MachineProfile = MachineProfile {
    envelope: Vec3 { x: 150.0, y: 110.0, z: 40.0 },
};

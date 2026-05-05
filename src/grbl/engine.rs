use std::collections::VecDeque;
use std::fs::OpenOptions;
use std::io::{BufRead, BufReader, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;
use std::time::Duration;

use parking_lot::{Condvar, Mutex, RwLock};

use crate::gcode::transform::transform_for_stream;
use crate::gcode::words::strip_comments;

use super::parser::{parse_response, Response, ResponseType};
use super::serial::Serial;
use super::state::*;

#[derive(Clone, Copy, Debug)]
pub struct ProbeReply {
    pub prb: Vec3,
    pub ok: bool,
}

#[derive(Debug)]
pub enum ProbeError {
    Busy,
    Timeout,
    NoContact,
    NotConnected,
}

impl std::fmt::Display for ProbeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProbeError::Busy => write!(f, "probe already in flight"),
            ProbeError::Timeout => write!(f, "probe timed out"),
            ProbeError::NoContact => write!(f, "probe never triggered (wire? depth?)"),
            ProbeError::NotConnected => write!(f, "machine not connected"),
        }
    }
}

struct SendQueue {
    capacity: usize,
    pending: VecDeque<String>,
    in_flight: VecDeque<usize>,
    used: usize,
}

impl SendQueue {
    fn new(capacity: usize) -> Self {
        Self {
            capacity,
            pending: VecDeque::new(),
            in_flight: VecDeque::new(),
            used: 0,
        }
    }

    fn enqueue(&mut self, line: String) {
        self.pending.push_back(line);
    }

    fn flush(&mut self) -> Vec<String> {
        let mut out = Vec::new();
        while let Some(front) = self.pending.front() {
            let size = front.len() + 1;
            if self.used + size > self.capacity && !self.in_flight.is_empty() {
                break;
            }
            let line = self.pending.pop_front().unwrap();
            self.in_flight.push_back(size);
            self.used += size;
            out.push(line);
        }
        out
    }

    fn ack(&mut self) {
        if let Some(size) = self.in_flight.pop_front() {
            self.used -= size;
        }
    }

    fn has_space_for(&self, len: usize) -> bool {
        let size = len + 1;
        self.used + size <= self.capacity || self.in_flight.is_empty()
    }

    fn is_idle(&self) -> bool {
        self.pending.is_empty() && self.in_flight.is_empty()
    }

    fn clear(&mut self) {
        self.pending.clear();
        self.in_flight.clear();
        self.used = 0;
    }

    #[cfg(test)]
    fn in_flight_bytes(&self) -> usize {
        self.used
    }
}

type OnLog = Arc<Mutex<Option<Arc<dyn Fn(String) + Send + Sync>>>>;

struct SendPipe {
    queue: Mutex<SendQueue>,
    buf_ready: Condvar,
    write_port: Mutex<Option<Box<dyn serialport::SerialPort + Send>>>,
    state: Arc<RwLock<MachineState>>,
    on_log: OnLog,
    file_log: Mutex<std::fs::File>,
    probe_result_tx: Mutex<Option<Sender<ProbeReply>>>,
}

impl SendPipe {
    fn new(state: Arc<RwLock<MachineState>>) -> Self {
        let file = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("/tmp/grbl.txt")
            .expect("failed to open /tmp/grbl.txt");
        Self {
            queue: Mutex::new(SendQueue::new(128)),
            buf_ready: Condvar::new(),
            write_port: Mutex::new(None),
            state,
            on_log: Arc::new(Mutex::new(None)),
            file_log: Mutex::new(file),
            probe_result_tx: Mutex::new(None),
        }
    }

    fn send(&self, line: &str) {
        let line = strip_comments(line);
        if line.is_empty() {
            return;
        }
        let to_send = {
            let mut q = self.queue.lock();
            q.enqueue(line);
            q.flush()
        };
        self.write_to_serial(&to_send);
    }

    fn ack(&self) {
        let to_send = {
            let mut q = self.queue.lock();
            q.ack();
            let flushed = q.flush();
            self.buf_ready.notify_all();
            flushed
        };
        self.write_to_serial(&to_send);
    }

    fn send_job_line(
        &self,
        line: &str,
        should_stop: &dyn Fn() -> bool,
        is_paused: &dyn Fn() -> bool,
    ) -> bool {
        let line = strip_comments(line);
        if line.is_empty() {
            return true;
        }

        loop {
            if should_stop() {
                return false;
            }
            if is_paused() {
                std::thread::sleep(Duration::from_millis(50));
                continue;
            }

            let to_send = {
                let mut q = self.queue.lock();
                while !q.has_space_for(line.len()) {
                    if should_stop() {
                        return false;
                    }
                    if is_paused() {
                        break;
                    }
                    self.buf_ready.wait_for(&mut q, Duration::from_millis(50));
                }
                if !q.has_space_for(line.len()) {
                    continue;
                }
                q.enqueue(line);
                q.flush()
            };
            self.write_to_serial(&to_send);
            return true;
        }
    }

    fn wait_job_idle(&self, should_stop: &dyn Fn() -> bool, is_paused: &dyn Fn() -> bool) -> bool {
        let mut q = self.queue.lock();
        while !q.is_idle() {
            if should_stop() {
                return false;
            }
            if is_paused() {
                drop(q);
                std::thread::sleep(Duration::from_millis(50));
                q = self.queue.lock();
                continue;
            }
            self.buf_ready.wait_for(&mut q, Duration::from_millis(50));
        }
        true
    }

    fn realtime(&self, b: u8) {
        let err = {
            let mut wp = self.write_port.lock();
            if let Some(ref mut port) = *wp {
                port.write_all(&[b]).err()
            } else {
                None
            }
        };
        if let Some(err) = err {
            self.serial_error(format!("serial realtime write failed: {err}"));
        }
    }

    fn clear(&self) {
        let mut q = self.queue.lock();
        q.clear();
        self.buf_ready.notify_all();
    }

    fn serial_error(&self, msg: String) {
        self.log(format!("!! {msg}"));
        *self.write_port.lock() = None;
        self.clear();
        let mut s = self.state.write();
        s.connected = false;
        s.status = Status::Disconnected;
        s.last_error = msg;
    }

    fn log(&self, msg: String) {
        {
            let mut f = self.file_log.lock();
            let _ = writeln!(f, "{}", msg);
        }
        if let Some(ref cb) = *self.on_log.lock() {
            cb(msg);
        }
    }

    fn write_to_serial(&self, lines: &[String]) {
        if lines.is_empty() {
            return;
        }

        let mut err = None;
        {
            let mut wp = self.write_port.lock();
            if let Some(ref mut port) = *wp {
                for line in lines {
                    if let Err(e) = port.write_all(line.as_bytes()) {
                        err = Some(format!("serial write failed for `{line}`: {e}"));
                        break;
                    }
                    if let Err(e) = port.write_all(b"\n") {
                        err = Some(format!("serial newline write failed for `{line}`: {e}"));
                        break;
                    }
                }
            }
        }

        {
            let mut f = self.file_log.lock();
            for line in lines {
                let _ = writeln!(f, "> {}", line);
            }
        }
        if let Some(ref cb) = *self.on_log.lock() {
            for line in lines {
                cb(format!("> {}", line));
            }
        }

        if let Some(err) = err {
            self.serial_error(err);
        }
    }
}

pub struct Engine {
    pub state: Arc<RwLock<MachineState>>,
    pub job: Arc<RwLock<JobState>>,
    pipe: Arc<SendPipe>,
    stop_flag: Mutex<Option<Arc<AtomicBool>>>,
    probe_in_flight: AtomicBool,
}

impl Engine {
    pub fn new(state: Arc<RwLock<MachineState>>, job: Arc<RwLock<JobState>>) -> Self {
        Self {
            state: state.clone(),
            job,
            pipe: Arc::new(SendPipe::new(state)),
            stop_flag: Mutex::new(None),
            probe_in_flight: AtomicBool::new(false),
        }
    }

    pub fn set_on_log(&self, f: impl Fn(String) + Send + Sync + 'static) {
        *self.pipe.on_log.lock() = Some(Arc::new(f));
    }

    pub fn connect(&self, port: &str, baud: u32) -> std::io::Result<()> {
        if let Some(stop) = self.stop_flag.lock().take() {
            stop.store(true, Ordering::Relaxed);
        }

        let serial = match Serial::open(port, baud) {
            Ok(serial) => serial,
            Err(err) => {
                self.state.write().last_error = format!("failed to connect to {port}: {err}");
                return Err(err);
            }
        };
        let (write_port, reader) = serial.into_parts();

        *self.pipe.write_port.lock() = Some(write_port);
        self.pipe.clear();

        {
            let mut s = self.state.write();
            s.port = port.to_string();
            s.baud = baud;
            s.connected = true;
            s.last_error.clear();
        }

        let stop = Arc::new(AtomicBool::new(false));
        *self.stop_flag.lock() = Some(stop.clone());

        {
            let state = self.state.clone();
            let job = self.job.clone();
            let pipe = self.pipe.clone();
            let stop = stop.clone();
            std::thread::spawn(move || read_loop(reader, stop, state, job, pipe));
        }

        {
            let pipe = self.pipe.clone();
            let stop = stop.clone();
            std::thread::spawn(move || poll_loop(stop, pipe));
        }

        self.send("$$");

        Ok(())
    }

    pub fn disconnect(&self) {
        if let Some(stop) = self.stop_flag.lock().take() {
            stop.store(true, Ordering::Relaxed);
        }
        *self.pipe.write_port.lock() = None;
        self.pipe.clear();
        let mut s = self.state.write();
        s.connected = false;
        s.status = Status::Disconnected;
    }

    pub fn send(&self, line: &str) {
        self.pipe.send(line);
    }

    pub fn realtime(&self, b: u8) {
        self.pipe.realtime(b);
    }
    pub fn feed_hold(&self) {
        self.realtime(b'!');
    }
    pub fn resume(&self) {
        self.realtime(b'~');
    }

    pub fn soft_reset(&self) {
        self.realtime(0x18);
        self.pipe.clear();
    }

    pub fn start_job(self: &Arc<Self>) {
        {
            let machine = self.state.read().clone();
            let mut j = self.job.write();
            if matches!(j.status, JobStatus::Running | JobStatus::Paused) {
                return;
            }
            recompute_soft_limit_violations(&mut j, &machine);
            j.status = JobStatus::Running;
            j.current_line = 0;
        }
        let engine = self.clone();
        std::thread::spawn(move || engine.stream_job());
    }

    pub fn pause_job(&self) {
        let mut j = self.job.write();
        if j.status == JobStatus::Running {
            j.status = JobStatus::Paused;
            drop(j);
            self.feed_hold();
        }
    }

    pub fn resume_job(&self) {
        let mut j = self.job.write();
        if j.status == JobStatus::Paused {
            j.status = JobStatus::Running;
            drop(j);
            self.resume();
        }
    }

    pub fn stop_job(&self) {
        self.job.write().status = JobStatus::Idle;
        self.soft_reset();
    }

    pub fn step_line(&self) {
        if self.job.read().status == JobStatus::Running {
            return;
        }
        let (transformed, src, lines_len, violated_lines) = {
            let j = self.job.read();
            let lines = j.lines.clone();
            let hmap = j.heightmap.clone();
            let lines_len = j.lines.len();
            let violated_lines = j.violated_lines.clone();
            drop(j);
            let (t, s) = transform_for_stream(&lines, hmap.as_deref());
            (t, s, lines_len, violated_lines)
        };

        loop {
            let mut j = self.job.write();
            if j.current_line >= lines_len {
                return;
            }
            let source = j.current_line;
            if source < violated_lines.len() && violated_lines[source] {
                let msg = format!("SOFT LIMIT at line {}: blocked", source + 1);
                drop(j);
                self.pipe.log(msg);
                return;
            }
            j.current_line += 1;
            drop(j);

            let start_idx = src.partition_point(|&s| s < source);
            let end_idx = src.partition_point(|&s| s <= source);
            let mut sent_any = false;
            for line in &transformed[start_idx..end_idx] {
                let stripped = strip_comments(line).trim().to_string();
                if !stripped.is_empty() {
                    self.pipe.send(&stripped);
                    sent_any = true;
                }
            }
            if sent_any {
                return;
            }
        }
    }

    pub fn reset_job(&self) {
        let mut j = self.job.write();
        j.current_line = 0;
        j.status = JobStatus::Idle;
    }

    pub fn probe_at(
        &self,
        x: f32,
        y: f32,
        safe_z: f32,
        max_depth: f32,
        feed: f32,
    ) -> Result<f32, ProbeError> {
        if !self.state.read().connected {
            return Err(ProbeError::NotConnected);
        }
        if self.probe_in_flight.swap(true, Ordering::Acquire) {
            return Err(ProbeError::Busy);
        }

        let (tx, rx) = mpsc::channel::<ProbeReply>();
        *self.pipe.probe_result_tx.lock() = Some(tx);

        self.pipe
            .send(&format!("G90 G21 G0 X{:.3} Y{:.3} Z{:.3}", x, y, safe_z));
        self.pipe
            .send(&format!("G38.3 Z-{:.3} F{:.1}", max_depth, feed));

        let result = rx.recv_timeout(Duration::from_secs(60));
        *self.pipe.probe_result_tx.lock() = None;

        self.pipe.send(&format!("G90 G21 G0 Z{:.3}", safe_z));
        self.probe_in_flight.store(false, Ordering::Release);

        let reply = result.map_err(|_| ProbeError::Timeout)?;
        if !reply.ok {
            return Err(ProbeError::NoContact);
        }
        let wco = self.state.read().wco;
        Ok(reply.prb.z - wco.z)
    }

    fn stream_job(&self) {
        let (lines, violated_lines, transformed, src) = {
            let j = self.job.read();
            let lines = j.lines.clone();
            let violated_lines = j.violated_lines.clone();
            let hmap = j.heightmap.clone();
            drop(j);
            let (t, s) = transform_for_stream(&lines, hmap.as_deref());
            (lines, violated_lines, t, s)
        };

        let should_stop = || {
            matches!(
                self.job.read().status,
                JobStatus::Idle | JobStatus::Complete
            )
        };
        let is_paused = || self.job.read().status == JobStatus::Paused;

        for (i, line) in transformed.iter().enumerate() {
            if should_stop() {
                return;
            }
            while is_paused() {
                std::thread::sleep(Duration::from_millis(50));
                if should_stop() {
                    return;
                }
            }
            let source_line = src[i];
            if source_line < violated_lines.len() && violated_lines[source_line] {
                self.pipe.log(format!(
                    "SOFT LIMIT at line {}: {}",
                    source_line + 1,
                    lines[source_line].trim()
                ));
                self.job.write().status = JobStatus::Idle;
                return;
            }
            let stripped = strip_comments(line).trim().to_string();
            if stripped.is_empty() {
                self.job.write().current_line = source_line + 1;
                continue;
            }
            if !self.pipe.send_job_line(&stripped, &should_stop, &is_paused) {
                return;
            }
            self.job.write().current_line = source_line + 1;
        }
        if self.pipe.wait_job_idle(&should_stop, &is_paused) && !should_stop() {
            self.job.write().status = JobStatus::Complete;
        }
    }
}

fn read_loop(
    mut reader: BufReader<Box<dyn serialport::SerialPort>>,
    stop: Arc<AtomicBool>,
    state: Arc<RwLock<MachineState>>,
    job: Arc<RwLock<JobState>>,
    pipe: Arc<SendPipe>,
) {
    let mut buf = String::new();
    loop {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        buf.clear();
        match reader.read_line(&mut buf) {
            Ok(0) => {
                pipe.serial_error("serial reader closed".into());
                return;
            }
            Ok(_) => {
                let line = buf.trim().to_string();
                if line.is_empty() {
                    continue;
                }
                let r = parse_response(&line);
                apply_response(&r, &state, &job, &pipe);
                pipe.log(line);
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(err) => {
                pipe.serial_error(format!("serial read failed: {err}"));
                return;
            }
        }
    }
}

fn poll_loop(stop: Arc<AtomicBool>, pipe: Arc<SendPipe>) {
    loop {
        std::thread::sleep(Duration::from_millis(200));
        if stop.load(Ordering::Relaxed) {
            return;
        }
        pipe.realtime(b'?');
    }
}

fn apply_response(
    r: &Response,
    state: &Arc<RwLock<MachineState>>,
    job: &Arc<RwLock<JobState>>,
    pipe: &SendPipe,
) {
    match r.resp_type {
        ResponseType::Ok | ResponseType::Error => {
            pipe.ack();
        }
        ResponseType::Status => {
            let mut s = state.write();
            s.status = r.status;
            if r.has_wco {
                s.wco = r.wco;
            }
            if r.has_mpos {
                s.mpos = r.mpos;
                s.wpos = Vec3 {
                    x: r.mpos.x - s.wco.x,
                    y: r.mpos.y - s.wco.y,
                    z: r.mpos.z - s.wco.z,
                };
            }
            if r.has_wpos {
                s.wpos = r.wpos;
                s.mpos = Vec3 {
                    x: r.wpos.x + s.wco.x,
                    y: r.wpos.y + s.wco.y,
                    z: r.wpos.z + s.wco.z,
                };
            }
            s.feed = r.feed;
            s.spindle = r.spindle;
            if r.feed_ovr != 0 {
                s.feed_ovr = r.feed_ovr;
            }
            if r.spindle_ovr != 0 {
                s.spindle_ovr = r.spindle_ovr;
            }
            s.probe_active = r.has_pins && r.pins.contains('P');
        }
        ResponseType::Alarm => {
            state.write().status = Status::Alarm;
            state.write().alarm_code = r.alarm_code;
            job.write().status = JobStatus::Idle;
            pipe.clear();
        }
        ResponseType::Setting => {
            let mut s = state.write();
            match r.setting_num {
                20 => s.soft_limits = r.setting_val != 0.0,
                130 => s.max_travel.x = r.setting_val,
                131 => s.max_travel.y = r.setting_val,
                132 => s.max_travel.z = r.setting_val,
                _ => {}
            }
        }
        ResponseType::Welcome => {
            let mut s = state.write();
            s.status = Status::Idle;
            s.alarm_code = 0;
        }
        ResponseType::Probe => {
            if let Some(tx) = pipe.probe_result_tx.lock().as_ref() {
                let _ = tx.send(ProbeReply {
                    prb: r.prb,
                    ok: r.probe_ok,
                });
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn character_counting() {
        let mut q = SendQueue::new(128);
        q.enqueue("G0 X10".into());
        q.enqueue("G0 X20".into());
        q.enqueue("G0 X30".into());
        let sent = q.flush();
        assert_eq!(sent.len(), 3);

        q.enqueue("G0 X40 Y40 Z40 F1000 S5000 M3 G90 G21 (this is a really long line that should fill the buffer significantly)".into());
        q.enqueue("G0 X50".into());
        let _ = q.flush();
        assert!(q.in_flight_bytes() <= 128);
    }

    #[test]
    fn ack_releases_buffer() {
        let mut q = SendQueue::new(128);
        q.enqueue("G0 X10".into());
        let _ = q.flush();
        let before = q.in_flight_bytes();
        q.ack();
        let after = q.in_flight_bytes();
        assert!(after < before);
    }

    #[test]
    fn strip_comments_dollar() {
        assert_eq!(strip_comments("$$"), "$$");
        assert_eq!(strip_comments("$20=0"), "$20=0");
    }
}

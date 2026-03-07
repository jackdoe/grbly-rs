use std::io::{BufRead, BufReader, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use std::collections::VecDeque;
use std::fs::OpenOptions;

use parking_lot::{Condvar, Mutex, RwLock};

use super::parser::{parse_response, Response, ResponseType};
use super::serial::Serial;
use super::state::*;

struct SendQueue {
    capacity: usize,
    pending: VecDeque<String>,
    in_flight: VecDeque<usize>,
    used: usize,
}

impl SendQueue {
    fn new(capacity: usize) -> Self {
        Self { capacity, pending: VecDeque::new(), in_flight: VecDeque::new(), used: 0 }
    }

    fn enqueue(&mut self, line: String) {
        self.pending.push_back(line);
    }

    fn flush(&mut self) -> Vec<String> {
        let mut out = Vec::new();
        while let Some(front) = self.pending.front() {
            let size = front.len() + 1;
            if self.used + size > self.capacity && !self.in_flight.is_empty() { break; }
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

/// Shared state for the send pipeline. All serial writes go through here.
struct SendPipe {
    queue: Mutex<SendQueue>,
    buf_ready: Condvar,
    write_port: Mutex<Option<Box<dyn serialport::SerialPort + Send>>>,
    on_log: OnLog,
    file_log: Mutex<std::fs::File>,
}

impl SendPipe {
    fn new() -> Self {
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
            on_log: Arc::new(Mutex::new(None)),
            file_log: Mutex::new(file),
        }
    }

    /// Enqueue a line and flush what fits to serial. Non-blocking.
    fn send(&self, line: &str) {
        let line = strip_gcode_comments(line);
        if line.is_empty() { return; }
        let to_send = {
            let mut q = self.queue.lock();
            q.enqueue(line);
            q.flush()
        };
        self.write_to_serial(&to_send);
    }

    /// Called when GRBL acks a command (ok/error). Frees buffer space and flushes.
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

    /// Block until the queue has space for `line`, enqueue it, flush. Used by job streamer.
    fn send_blocking(&self, line: &str, cancel: &dyn Fn() -> bool) -> bool {
        let line = strip_gcode_comments(line);
        if line.is_empty() { return true; }
        let to_send = {
            let mut q = self.queue.lock();
            while !q.has_space_for(line.len()) {
                if cancel() { return false; }
                self.buf_ready.wait(&mut q);
            }
            q.enqueue(line);
            q.flush()
        };
        self.write_to_serial(&to_send);
        true
    }

    /// Block until all in-flight commands are ack'd.
    fn wait_idle(&self, cancel: &dyn Fn() -> bool) {
        let mut q = self.queue.lock();
        while !q.is_idle() {
            if cancel() { return; }
            self.buf_ready.wait(&mut q);
        }
    }

    /// Send a realtime character (not queued, bypasses buffer).
    fn realtime(&self, b: u8) {
        let mut wp = self.write_port.lock();
        if let Some(ref mut port) = *wp {
            let _ = port.write_all(&[b]);
        }
    }

    fn clear(&self) {
        let mut q = self.queue.lock();
        q.clear();
        self.buf_ready.notify_all();
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

    /// The single place that writes to serial and logs sent commands.
    fn write_to_serial(&self, lines: &[String]) {
        if lines.is_empty() { return; }
        {
            let mut wp = self.write_port.lock();
            if let Some(ref mut port) = *wp {
                for line in lines {
                    let _ = port.write_all(line.as_bytes());
                    let _ = port.write_all(b"\n");
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
    }
}

pub struct Engine {
    pub state: Arc<RwLock<MachineState>>,
    pub job: Arc<RwLock<JobState>>,
    pipe: Arc<SendPipe>,
    stop_flag: Mutex<Option<Arc<AtomicBool>>>,
}

impl Engine {
    pub fn new(state: Arc<RwLock<MachineState>>, job: Arc<RwLock<JobState>>) -> Self {
        Self {
            state,
            job,
            pipe: Arc::new(SendPipe::new()),
            stop_flag: Mutex::new(None),
        }
    }

    pub fn set_on_log(&self, f: impl Fn(String) + Send + Sync + 'static) {
        *self.pipe.on_log.lock() = Some(Arc::new(f));
    }

    pub fn connect(&self, port: &str, baud: u32) -> std::io::Result<()> {
        let serial = Serial::open(port, baud)?;
        let (write_port, reader) = serial.into_parts();

        *self.pipe.write_port.lock() = Some(write_port);
        self.pipe.clear();

        {
            let mut s = self.state.write();
            s.port = port.to_string();
            s.baud = baud;
            s.connected = true;
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

        // Just read current settings, don't overwrite them
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

    pub fn realtime(&self, b: u8) { self.pipe.realtime(b); }
    pub fn feed_hold(&self) { self.realtime(b'!'); }
    pub fn resume(&self) { self.realtime(b'~'); }

    pub fn soft_reset(&self) {
        self.realtime(0x18);
        self.pipe.clear();
    }

    pub fn start_job(self: &Arc<Self>) {
        {
            let mut j = self.job.write();
            if j.status == JobStatus::Running { return; }
            j.status = JobStatus::Running;
            j.current_line = 0;
        }
        let engine = self.clone();
        std::thread::spawn(move || engine.stream_job());
    }

    pub fn pause_job(&self) {
        self.job.write().status = JobStatus::Paused;
        self.feed_hold();
    }

    pub fn resume_job(&self) {
        self.job.write().status = JobStatus::Running;
        self.resume();
    }

    pub fn stop_job(&self) {
        self.job.write().status = JobStatus::Idle;
        self.soft_reset();
    }

    pub fn step_line(&self) {
        loop {
            let mut j = self.job.write();
            if j.current_line >= j.lines.len() { return; }
            if j.current_line < j.violated_lines.len() && j.violated_lines[j.current_line] {
                let msg = format!("SOFT LIMIT at line {}: blocked", j.current_line + 1);
                drop(j);
                self.pipe.log(msg);
                return;
            }
            let line = j.lines[j.current_line].clone();
            let z_locked = j.z_locked;
            j.current_line += 1;
            drop(j);
            let mut stripped = strip_gcode_comments(&line).trim().to_string();
            if z_locked { stripped = strip_z_words(&stripped); }
            if !stripped.is_empty() {
                self.pipe.send(&stripped);
                return;
            }
        }
    }

    pub fn reset_job(&self) {
        let mut j = self.job.write();
        j.current_line = 0;
        j.status = JobStatus::Idle;
    }

    fn stream_job(&self) {
        let j = self.job.read();
        let lines = j.lines.clone();
        let z_locked = j.z_locked;
        let violated_lines = j.violated_lines.clone();
        drop(j);

        let cancel = || self.job.read().status != JobStatus::Running;

        for (i, line) in lines.iter().enumerate() {
            if cancel() { return; }
            if i < violated_lines.len() && violated_lines[i] {
                self.pipe.log(format!("SOFT LIMIT at line {}: {}", i + 1, line.trim()));
                self.job.write().status = JobStatus::Idle;
                return;
            }
            let mut stripped = strip_gcode_comments(line).trim().to_string();
            if z_locked { stripped = strip_z_words(&stripped); }
            if stripped.is_empty() {
                self.job.write().current_line = i + 1;
                continue;
            }
            if !self.pipe.send_blocking(&stripped, &cancel) {
                return;
            }
            self.job.write().current_line = i + 1;
        }

        self.pipe.wait_idle(&cancel);
        if !cancel() {
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
        if stop.load(Ordering::Relaxed) { return; }
        buf.clear();
        match reader.read_line(&mut buf) {
            Ok(0) => return,
            Ok(_) => {
                let line = buf.trim().to_string();
                if line.is_empty() { continue; }
                let r = parse_response(&line);
                apply_response(&r, &state, &job, &pipe);
                pipe.log(line);
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => return,
        }
    }
}

fn poll_loop(stop: Arc<AtomicBool>, pipe: Arc<SendPipe>) {
    loop {
        std::thread::sleep(Duration::from_millis(200));
        if stop.load(Ordering::Relaxed) { return; }
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
            if r.has_wco { s.wco = r.wco; }
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
            if r.feed_ovr != 0 { s.feed_ovr = r.feed_ovr; }
            if r.spindle_ovr != 0 { s.spindle_ovr = r.spindle_ovr; }
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
        _ => {}
    }
}

fn strip_z_words(line: &str) -> String {
    let bytes = line.as_bytes();
    let mut out = String::with_capacity(line.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'Z' || bytes[i] == b'z' {
            i += 1;
            while i < bytes.len() && (bytes[i] == b'-' || bytes[i] == b'.' || (bytes[i] >= b'0' && bytes[i] <= b'9')) {
                i += 1;
            }
            while i < bytes.len() && bytes[i] == b' ' {
                i += 1;
            }
        } else {
            out.push(bytes[i] as char);
            i += 1;
        }
    }
    out.trim().to_string()
}

fn strip_gcode_comments(line: &str) -> String {
    let mut out = String::new();
    let mut depth = 0i32;
    for c in line.chars() {
        match c {
            '(' => depth += 1,
            ')' if depth > 0 => depth -= 1,
            ';' => return out.trim().to_string(),
            _ if depth == 0 => out.push(c),
            _ => {}
        }
    }
    out.trim().to_string()
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
        assert_eq!(strip_gcode_comments("$$"), "$$");
        assert_eq!(strip_gcode_comments("$20=0"), "$20=0");
    }
}

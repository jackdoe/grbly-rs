use std::io::{BufRead, BufReader, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use std::collections::VecDeque;

use parking_lot::{Mutex, RwLock};

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
            if self.used + size > self.capacity { break; }
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

    #[cfg(test)]
    fn in_flight_bytes(&self) -> usize {
        self.used
    }

    fn clear(&mut self) {
        self.pending.clear();
        self.in_flight.clear();
        self.used = 0;
    }
}

type WritePort = Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>;

pub struct Engine {
    pub state: Arc<RwLock<MachineState>>,
    pub job: Arc<RwLock<JobState>>,
    write_port: WritePort,
    queue: Arc<Mutex<SendQueue>>,
    on_log: Arc<Mutex<Option<Arc<dyn Fn(String) + Send + Sync>>>>,
    stop_flag: Mutex<Option<Arc<AtomicBool>>>,
}

impl Engine {
    pub fn new(state: Arc<RwLock<MachineState>>, job: Arc<RwLock<JobState>>) -> Self {
        Self {
            state,
            job,
            write_port: Arc::new(Mutex::new(None)),
            queue: Arc::new(Mutex::new(SendQueue::new(128))),
            on_log: Arc::new(Mutex::new(None)),
            stop_flag: Mutex::new(None),
        }
    }

    pub fn set_on_log(&self, f: impl Fn(String) + Send + Sync + 'static) {
        *self.on_log.lock() = Some(Arc::new(f));
    }

    pub fn connect(&self, port: &str, baud: u32) -> std::io::Result<()> {
        let serial = Serial::open(port, baud)?;
        let (write_port, reader) = serial.into_parts();

        *self.write_port.lock() = Some(write_port);
        *self.queue.lock() = SendQueue::new(128);

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
            let queue = self.queue.clone();
            let write_port = self.write_port.clone();
            let on_log = self.on_log.clone();
            let stop = stop.clone();
            std::thread::spawn(move || read_loop(reader, stop, state, queue, write_port, on_log));
        }

        {
            let write_port = self.write_port.clone();
            let stop = stop.clone();
            std::thread::spawn(move || poll_loop(stop, write_port));
        }

        Ok(())
    }

    pub fn disconnect(&self) {
        if let Some(stop) = self.stop_flag.lock().take() {
            stop.store(true, Ordering::Relaxed);
        }
        *self.write_port.lock() = None;
        self.queue.lock().clear();
        let mut s = self.state.write();
        s.connected = false;
        s.status = Status::Disconnected;
    }

    pub fn send(&self, line: &str) {
        let stripped = strip_gcode_comments(line);
        let to_send = {
            let mut q = self.queue.lock();
            q.enqueue(stripped);
            q.flush()
        };
        write_lines(&self.write_port, &to_send);
    }

    pub fn realtime(&self, b: u8) {
        let mut wp = self.write_port.lock();
        if let Some(ref mut port) = *wp {
            let _ = port.write_all(&[b]);
        }
    }

    pub fn feed_hold(&self) { self.realtime(b'!'); }
    pub fn resume(&self) { self.realtime(b'~'); }
    pub fn soft_reset(&self) {
        self.realtime(0x18);
        self.queue.lock().clear();
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
        let mut j = self.job.write();
        if j.current_line >= j.lines.len() { return; }
        let line = j.lines[j.current_line].clone();
        let z_locked = j.z_locked;
        j.current_line += 1;
        drop(j);
        let mut stripped = strip_gcode_comments(&line).trim().to_string();
        if z_locked { stripped = strip_z_words(&stripped); }
        if !stripped.is_empty() {
            self.send(&stripped);
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
        drop(j);
        for (i, line) in lines.iter().enumerate() {
            if self.job.read().status != JobStatus::Running { return; }
            let mut stripped = strip_gcode_comments(line).trim().to_string();
            if z_locked { stripped = strip_z_words(&stripped); }
            if stripped.is_empty() {
                self.job.write().current_line = i + 1;
                continue;
            }
            let to_send = {
                let mut q = self.queue.lock();
                q.enqueue(stripped);
                q.flush()
            };
            write_lines(&self.write_port, &to_send);
            self.job.write().current_line = i + 1;
        }
        self.job.write().status = JobStatus::Complete;
    }
}

fn write_lines(write_port: &WritePort, lines: &[String]) {
    let mut wp = write_port.lock();
    if let Some(ref mut port) = *wp {
        for line in lines {
            let _ = port.write_all(line.as_bytes());
            let _ = port.write_all(b"\n");
        }
    }
}

type OnLog = Arc<Mutex<Option<Arc<dyn Fn(String) + Send + Sync>>>>;

fn read_loop(
    mut reader: BufReader<Box<dyn serialport::SerialPort>>,
    stop: Arc<AtomicBool>,
    state: Arc<RwLock<MachineState>>,
    queue: Arc<Mutex<SendQueue>>,
    write_port: WritePort,
    on_log: OnLog,
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
                apply_response(&r, &state, &queue, &write_port);
                if let Some(ref cb) = *on_log.lock() {
                    cb(line);
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => return,
        }
    }
}

fn poll_loop(stop: Arc<AtomicBool>, write_port: WritePort) {
    loop {
        std::thread::sleep(Duration::from_millis(200));
        if stop.load(Ordering::Relaxed) { return; }
        let mut wp = write_port.lock();
        if let Some(ref mut port) = *wp {
            let _ = port.write_all(&[b'?']);
        }
    }
}

fn apply_response(
    r: &Response,
    state: &Arc<RwLock<MachineState>>,
    queue: &Arc<Mutex<SendQueue>>,
    write_port: &WritePort,
) {
    match r.resp_type {
        ResponseType::Ok | ResponseType::Error => {
            let to_send = {
                let mut q = queue.lock();
                q.ack();
                q.flush()
            };
            write_lines(write_port, &to_send);
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
            let mut s = state.write();
            s.status = Status::Alarm;
            s.alarm_code = r.alarm_code;
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
}

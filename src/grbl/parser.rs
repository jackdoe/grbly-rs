use super::state::{Status, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResponseType {
    Ok,
    Error,
    Alarm,
    Status,
    Msg,
    Setting,
    Welcome,
    Unknown,
}

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct Response {
    pub resp_type: ResponseType,
    pub error_code: i32,
    pub alarm_code: i32,
    pub status: Status,
    pub mpos: Vec3,
    pub wpos: Vec3,
    pub wco: Vec3,
    pub has_mpos: bool,
    pub has_wpos: bool,
    pub has_wco: bool,
    pub feed: f32,
    pub spindle: f32,
    pub feed_ovr: i32,
    pub rapid_ovr: i32,
    pub spindle_ovr: i32,
    pub message: String,
    pub setting_num: i32,
    pub setting_val: f32,
}

impl Default for Response {
    fn default() -> Self {
        Self {
            resp_type: ResponseType::Unknown,
            error_code: 0,
            alarm_code: 0,
            status: Status::Disconnected,
            mpos: Vec3::default(),
            wpos: Vec3::default(),
            wco: Vec3::default(),
            has_mpos: false,
            has_wpos: false,
            has_wco: false,
            feed: 0.0,
            spindle: 0.0,
            feed_ovr: 0,
            rapid_ovr: 0,
            spindle_ovr: 0,
            message: String::new(),
            setting_num: 0,
            setting_val: 0.0,
        }
    }
}

pub fn parse_response(line: &str) -> Response {
    if line == "ok" {
        return Response { resp_type: ResponseType::Ok, ..Default::default() };
    }
    if let Some(rest) = line.strip_prefix("error:") {
        return Response {
            resp_type: ResponseType::Error,
            error_code: rest.parse().unwrap_or(0),
            ..Default::default()
        };
    }
    if let Some(rest) = line.strip_prefix("ALARM:") {
        return Response {
            resp_type: ResponseType::Alarm,
            alarm_code: rest.parse().unwrap_or(0),
            ..Default::default()
        };
    }
    if line.len() > 2 && line.starts_with('<') && line.ends_with('>') {
        return parse_status(&line[1..line.len() - 1]);
    }
    if let Some(inner) = line.strip_prefix("[MSG:").and_then(|s| s.strip_suffix(']')) {
        return Response {
            resp_type: ResponseType::Msg,
            message: inner.to_string(),
            ..Default::default()
        };
    }
    if line.starts_with("Grbl ") {
        return Response { resp_type: ResponseType::Welcome, ..Default::default() };
    }
    if line.starts_with('$') && line.contains('=') {
        if let Some((num_str, val_str)) = line[1..].split_once('=') {
            let val_clean = val_str.split_whitespace().next().unwrap_or("");
            if let (Ok(num), Ok(val)) = (num_str.parse::<i32>(), val_clean.parse::<f32>()) {
                return Response {
                    resp_type: ResponseType::Setting,
                    setting_num: num,
                    setting_val: val,
                    ..Default::default()
                };
            }
        }
    }
    Response::default()
}

fn parse_status(s: &str) -> Response {
    let fields: Vec<&str> = s.split('|').collect();
    let mut r = Response { resp_type: ResponseType::Status, ..Default::default() };
    if fields.is_empty() {
        return r;
    }
    r.status = parse_status_word(fields[0]);
    for f in &fields[1..] {
        if let Some((k, v)) = f.split_once(':') {
            match k {
                "MPos" => {
                    r.mpos = parse_vec3(v);
                    r.has_mpos = true;
                }
                "WPos" => {
                    r.wpos = parse_vec3(v);
                    r.has_wpos = true;
                }
                "WCO" => {
                    r.wco = parse_vec3(v);
                    r.has_wco = true;
                }
                "FS" => {
                    let parts: Vec<&str> = v.splitn(2, ',').collect();
                    if parts.len() == 2 {
                        r.feed = parts[0].parse().unwrap_or(0.0);
                        r.spindle = parts[1].parse().unwrap_or(0.0);
                    }
                }
                "Ov" => {
                    let parts: Vec<&str> = v.splitn(3, ',').collect();
                    if parts.len() == 3 {
                        r.feed_ovr = parts[0].parse().unwrap_or(0);
                        r.rapid_ovr = parts[1].parse().unwrap_or(0);
                        r.spindle_ovr = parts[2].parse().unwrap_or(0);
                    }
                }
                _ => {}
            }
        }
    }
    r
}

fn parse_status_word(s: &str) -> Status {
    let word = s.split(':').next().unwrap_or("");
    match word {
        "Idle" => Status::Idle,
        "Run" => Status::Run,
        "Hold" => Status::Hold,
        "Alarm" => Status::Alarm,
        "Home" => Status::Home,
        "Check" => Status::Check,
        "Jog" => Status::Jog,
        "Door" => Status::Door,
        "Sleep" => Status::Sleep,
        _ => Status::Disconnected,
    }
}

fn parse_vec3(s: &str) -> Vec3 {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() < 3 {
        return Vec3::default();
    }
    Vec3 {
        x: parts[0].parse().unwrap_or(0.0),
        y: parts[1].parse().unwrap_or(0.0),
        z: parts[2].parse().unwrap_or(0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_ok() {
        let r = parse_response("ok");
        assert_eq!(r.resp_type, ResponseType::Ok);
    }

    #[test]
    fn parse_error() {
        let r = parse_response("error:20");
        assert_eq!(r.resp_type, ResponseType::Error);
        assert_eq!(r.error_code, 20);
    }

    #[test]
    fn parse_alarm() {
        let r = parse_response("ALARM:2");
        assert_eq!(r.resp_type, ResponseType::Alarm);
        assert_eq!(r.alarm_code, 2);
    }

    #[test]
    fn parse_status_full() {
        let r = parse_response("<Run|MPos:1.000,2.000,3.000|FS:500,8000|Ov:100,100,100>");
        assert_eq!(r.resp_type, ResponseType::Status);
        assert_eq!(r.status, Status::Run);
        assert_eq!(r.mpos, Vec3 { x: 1.0, y: 2.0, z: 3.0 });
        assert_eq!(r.feed, 500.0);
        assert_eq!(r.spindle, 8000.0);
        assert_eq!(r.feed_ovr, 100);
        assert_eq!(r.spindle_ovr, 100);
    }

    #[test]
    fn parse_status_with_wpos() {
        let r = parse_response("<Idle|WPos:10.000,20.000,-5.000|FS:0,0>");
        assert_eq!(r.wpos, Vec3 { x: 10.0, y: 20.0, z: -5.0 });
    }

    #[test]
    fn parse_status_hold() {
        let r = parse_response("<Hold:0|MPos:0.000,0.000,0.000|FS:0,0>");
        assert_eq!(r.status, Status::Hold);
    }

    #[test]
    fn parse_status_door() {
        let r = parse_response("<Door:0|MPos:0.000,0.000,0.000,0.000|Bf:35,111|FS:0,9000>");
        assert_eq!(r.resp_type, ResponseType::Status);
        assert_eq!(r.status, Status::Door);
        assert_eq!(r.mpos, Vec3 { x: 0.0, y: 0.0, z: 0.0 });
    }

    #[test]
    fn parse_msg() {
        let r = parse_response("[MSG:Caution: Unlocked]");
        assert_eq!(r.resp_type, ResponseType::Msg);
        assert_eq!(r.message, "Caution: Unlocked");
    }

    #[test]
    fn parse_welcome() {
        let r = parse_response("Grbl 1.1h ['$' for help]");
        assert_eq!(r.resp_type, ResponseType::Welcome);
    }

    #[test]
    fn parse_unknown() {
        let r = parse_response("something unexpected");
        assert_eq!(r.resp_type, ResponseType::Unknown);
    }

    #[test]
    fn parse_setting_with_description() {
        let r = parse_response("$20=1 (soft limits,bool)");
        assert_eq!(r.resp_type, ResponseType::Setting);
        assert_eq!(r.setting_num, 20);
        assert_eq!(r.setting_val, 1.0);
    }

    #[test]
    fn parse_setting_float_with_description() {
        let r = parse_response("$130=150.000 (X aixs max travel:mm)");
        assert_eq!(r.resp_type, ResponseType::Setting);
        assert_eq!(r.setting_num, 130);
        assert_eq!(r.setting_val, 150.0);
    }
}

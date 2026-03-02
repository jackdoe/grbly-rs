use std::io::{self, BufReader};
use std::time::Duration;

pub struct Serial {
    port: Box<dyn serialport::SerialPort>,
    reader: BufReader<Box<dyn serialport::SerialPort>>,
}

pub fn list_ports() -> Vec<String> {
    let ports = serialport::available_ports().unwrap_or_default();
    ports
        .into_iter()
        .filter_map(|p| {
            let name = p.port_name.clone();
            if cfg!(target_os = "linux") {
                if name.starts_with("/dev/ttyUSB") || name.starts_with("/dev/ttyACM") {
                    return Some(name);
                }
            } else if cfg!(target_os = "macos") {
                if name.contains("usbserial") || name.contains("usbmodem") {
                    return Some(name);
                }
            } else {
                return Some(name);
            }
            None
        })
        .collect()
}

impl Serial {
    pub fn open(port_name: &str, baud: u32) -> io::Result<Self> {
        let port = serialport::new(port_name, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
        let reader_port = port
            .try_clone()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
        Ok(Self {
            port,
            reader: BufReader::new(reader_port),
        })
    }

    pub fn into_parts(self) -> (Box<dyn serialport::SerialPort>, BufReader<Box<dyn serialport::SerialPort>>) {
        (self.port, self.reader)
    }
}

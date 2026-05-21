use std::io;
use std::sync::Mutex;
use std::time::{Duration, Instant};

pub mod wifi;

use serialport::{SerialPort, SerialPortType};

// Stores the port *name* of the detected LoRa module.
static LORA_PORT_NAME: Mutex<Option<String>> = Mutex::new(None);

// const LORA_SERIAL: &str = "08:92:72:85:8B:F0";
const LORA_SERIAL: &str = "08:92:72:85:0B:B8";
const BAUD_RATE: u32 = 115_200;

// SLIP special bytes
const SLIP_END: u8 = 0xC0;
const SLIP_ESC: u8 = 0xDB;
const SLIP_ESC_END: u8 = 0xDC;
const SLIP_ESC_ESC: u8 = 0xDD;

#[derive(Debug)]
pub enum SerialError {
    Timeout,
    Detached,
    Io(io::Error),
    Port(serialport::Error),
}

impl From<serialport::Error> for SerialError {
    fn from(e: serialport::Error) -> Self {
        match e.kind {
            serialport::ErrorKind::NoDevice => SerialError::Detached,
            serialport::ErrorKind::Io(k) => SerialError::Io(io::Error::from(k)),
            _ => SerialError::Port(e),
        }
    }
}

// Scans available ports and caches the name of the LoRa module if found.
// Returns the port name if found.
fn scan_for_lora() -> Option<String> {
    let ports = serialport::available_ports().ok()?;

    for port in ports {
        if let SerialPortType::UsbPort(usb) = port.port_type {
            if usb.serial_number.as_deref() == Some(LORA_SERIAL) {
                let name = port.port_name.clone();
                *LORA_PORT_NAME.lock().unwrap() = Some(name.clone());
                return Some(name);
            }
        }
    }

    None
}

// Returns the cached port name, or scans if not yet cached.
fn get_lora_port_name() -> Option<String> {
    // if let Some(name) = LORA_PORT_NAME.lock().unwrap().clone() {
    //     return Some(name);
    // }
    scan_for_lora()
}

// Opens a fresh serial connection to the LoRa module.
fn open_lora_port() -> Result<Box<dyn SerialPort>, SerialError> {
    let name = get_lora_port_name().ok_or(SerialError::Detached)?;

    serialport::new(&name, BAUD_RATE)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .exclusive(false)
        .open()
        .map_err(|e| match e.kind {
            serialport::ErrorKind::NoDevice => {
                // Cached name is stale — clear it.
                *LORA_PORT_NAME.lock().unwrap() = None;
                SerialError::Detached
            }
            _ => SerialError::from(e),
        })
}

pub fn is_lora_module_attached() -> bool {
    get_lora_port_name().is_some()
}

// SLIP-encodes `data` and writes it to `port`.
fn slip_send(port: &mut dyn SerialPort, data: &[u8]) -> Result<(), SerialError> {
    let mut frame: Vec<u8> = Vec::with_capacity(data.len() + 2);

    frame.push(SLIP_END);
    for &byte in data {
        match byte {
            SLIP_END => {
                frame.push(SLIP_ESC);
                frame.push(SLIP_ESC_END);
            }
            SLIP_ESC => {
                frame.push(SLIP_ESC);
                frame.push(SLIP_ESC_ESC);
            }
            b => frame.push(b),
        }
    }
    frame.push(SLIP_END);

    port.write_all(&frame).map_err(SerialError::Io)
}

// Reads and SLIP-decodes one packet from `port`, blocking until `timeout_ms`.
// An initial SLIP_END (flush marker) is skipped per RFC 1055.
fn slip_recv(port: &mut dyn SerialPort, timeout_ms: u32) -> Result<Vec<u8>, SerialError> {
    port.set_timeout(Duration::from_millis(timeout_ms as u64))
        .map_err(SerialError::from)?;

    let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
    let mut packet: Vec<u8> = Vec::new();
    let mut in_escape = false;
    let mut started = false;
    let mut buf = [0u8; 1];

    loop {
        if Instant::now() >= deadline {
            return Err(SerialError::Timeout);
        }

        match port.read(&mut buf) {
            Ok(0) => continue,
            Ok(_) => {}
            Err(e) if e.kind() == io::ErrorKind::TimedOut => return Err(SerialError::Timeout),
            Err(e) => return Err(SerialError::Io(e)),
        }

        dbg!("getting bytes");

        let byte = buf[0];

        match byte {
            SLIP_END if !started => {
                // Leading END — RFC 1055 flush marker, skip.
                started = true;
            }
            SLIP_END => {
                // Trailing END — packet complete.
                if !packet.is_empty() {
                    return Ok(packet);
                }
                // Empty packet between two ENDs — skip.
            }
            SLIP_ESC => {
                in_escape = true;
            }
            SLIP_ESC_END if in_escape => {
                in_escape = false;
                packet.push(SLIP_END);
            }
            SLIP_ESC_ESC if in_escape => {
                in_escape = false;
                packet.push(SLIP_ESC);
            }
            b => {
                in_escape = false;
                if started {
                    packet.push(b);
                }
            }
        }
    }
}

/// Blocks until a SLIP packet is received or the timeout elapses.
pub fn recv_lora_packet(timeout_ms: u32) -> Result<Vec<u8>, SerialError> {
    let mut port = open_lora_port()?;
    slip_recv(port.as_mut(), timeout_ms)
}

/// SLIP-encodes and sends `data` to the LoRa module.
pub fn send_lora_packet(data: &[u8]) -> Result<(), SerialError> {
    let mut port = open_lora_port()?;
    slip_send(port.as_mut(), data)
}

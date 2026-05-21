use serialport::{SerialPort, SerialPortType};
use std::sync::{
    Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;

const SERIAL: &str = "0001";
const BAUD_RATE: u32 = 115_200;
const RECV_TIMEOUT_MS: u64 = 100;

const SLIP_END: u8 = 0xC0;
const SLIP_ESC: u8 = 0xDB;
const SLIP_ESC_END: u8 = 0xDC;
const SLIP_ESC_ESC: u8 = 0xDD;

struct PortPair {
    reader: Box<dyn SerialPort>,
    writer: Box<dyn SerialPort>,
}

pub struct WifiModule {
    read_port: Mutex<Option<Box<dyn SerialPort>>>,
    write_port: Mutex<Option<Box<dyn SerialPort>>>,
    detached: AtomicBool,
}

#[derive(Debug)]
enum RecvError {
    Timeout,
    Io,
}

impl WifiModule {
    pub fn new() -> Self {
        Self {
            read_port: Mutex::new(None),
            write_port: Mutex::new(None),
            detached: AtomicBool::new(false),
        }
    }

    fn scan_for_port_name() -> Option<String> {
        let ports = serialport::available_ports().ok()?;

        for port in ports {
            if let SerialPortType::UsbPort(usb) = port.port_type {
                if usb.serial_number.as_deref() == Some(SERIAL) {
                    return Some(port.port_name);
                }
            }
        }

        None
    }

    fn open_port_pair() -> Option<PortPair> {
        let name = Self::scan_for_port_name()?;

        let writer = serialport::new(&name, BAUD_RATE)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .exclusive(true)
            .timeout(Duration::from_millis(RECV_TIMEOUT_MS))
            .open()
            .ok()?;

        let reader = writer.try_clone().ok()?;
        Some(PortPair { reader, writer })
    }

    pub fn is_attached(&self) -> bool {
        Self::scan_for_port_name().is_some()
    }

    fn ensure_open(&self) -> bool {
        let mut rg = self.read_port.lock().unwrap();
        let mut wg = self.write_port.lock().unwrap();

        if rg.is_some() && wg.is_some() {
            return true;
        }

        match Self::open_port_pair() {
            Some(pair) => {
                *rg = Some(pair.reader);
                *wg = Some(pair.writer);
                true
            }
            None => false,
        }
    }

    /// Drops both port handles. Call `reconnect()` to re-enable.
    pub fn detach(&self) {
        self.detached.store(true, Ordering::Release);
        let mut rg = self.read_port.lock().unwrap();
        let mut wg = self.write_port.lock().unwrap();
        *rg = None;
        *wg = None;
    }

    /// Clears the detached flag, allowing `send_packet`/`recv_packet` to reopen the port.
    pub fn reconnect(&self) {
        self.detached.store(false, Ordering::Release);
    }

    pub fn send_packet(&self, data: &[u8]) -> Option<()> {
        if self.detached.load(Ordering::Acquire) {
            return None;
        }

        if !self.ensure_open() {
            return None;
        }

        let mut wg = self.write_port.lock().unwrap();
        let port = wg.as_mut()?;

        match slip_send(port.as_mut(), data) {
            Ok(()) => Some(()),
            Err(()) => {
                *wg = None;
                None
            }
        }
    }

    /// Blocks until a SLIP packet is received. Wakes every `RECV_TIMEOUT_MS` to
    /// check the detached flag, releasing the lock in between so `detach()` can proceed.
    pub fn recv_packet(&self) -> Option<Vec<u8>> {
        loop {
            if self.detached.load(Ordering::Acquire) {
                dbg!("Detached");
                return None;
            }

            if !self.ensure_open() {
                dbg!("Not open");
                return None;
            }

            let mut rg = self.read_port.lock().unwrap();
            let port = match rg.as_mut() {
                Some(p) => p,
                None => {
                    dbg!("audience");
                    continue;
                }
            };

            match slip_recv(port.as_mut()) {
                Ok(packet) => return Some(packet),
                Err(RecvError::Timeout) => continue,
                Err(RecvError::Io) => {
                    *rg = None;
                    return None;
                }
            }
        }
    }
}

fn slip_send(port: &mut dyn SerialPort, data: &[u8]) -> Result<(), ()> {
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

    port.write_all(&frame).map_err(|_| ())
}

fn slip_recv(port: &mut dyn SerialPort) -> Result<Vec<u8>, RecvError> {
    let mut packet: Vec<u8> = Vec::new();
    let mut in_escape = false;
    let mut started = false;
    let mut buf = [0u8; 1];

    loop {
        match port.read(&mut buf) {
            Ok(0) => continue,
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Err(RecvError::Timeout),
            Err(_) => return Err(RecvError::Io),
        }

        let byte = buf[0];

        match byte {
            SLIP_END if !started => {
                started = true;
            }
            SLIP_END => {
                if !packet.is_empty() {
                    return Ok(packet);
                }
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

use std::{
    collections::VecDeque, num::{NonZeroU8, NonZeroU16}, time::{Duration, Instant}
};

use control_protocol::{DeviceId, last};
use crossbeam_channel::{Receiver, RecvError, Sender, TryRecvError};

use crate::drivers::StatusError;

use super::crc::*;

pub mod sender;


// ------------------------- DEVICE I/O ABSTRACTION ------------------------- //


mod hal {
    use crate::drivers::{StatusError, from_ffi};


    pub trait LoRaInterface: Send {
        fn is_module_attached(&self) -> bool;
        fn initialize_module(&self) -> Result<(), StatusError>;
        fn shutdown_module(&self);
        fn get_lora_byterate(&self) -> u32;
        fn send_packet(&self, bytes: &[u8]) -> Result<(), StatusError>;
        fn recv_packet(&self, buffer: &mut [u8; control_sys::MAX_LORA_RECV_PACKET_LEN], len: &mut usize, timeout_ms: u32) -> Result<(), StatusError>;
    }
    pub struct LoRaSys;

    impl LoRaInterface for LoRaSys {
        fn is_module_attached(&self) -> bool {
            control_sys::is_lora_module_attached()
        }

        fn initialize_module(&self) -> Result<(), StatusError> {
            from_ffi(control_sys::initialize_lora_module())
        }

        fn shutdown_module(&self) {
            control_sys::shutdown_lora_module();
        }

        fn get_lora_byterate(&self) -> u32 {
            control_sys::get_lora_byterate()
        }

        fn send_packet(&self, bytes: &[u8]) -> Result<(), StatusError> {
            from_ffi(control_sys::send_lora_packet(bytes.as_ptr(), bytes.len()))
        }

        fn recv_packet(&self, buffer: &mut [u8; control_sys::MAX_LORA_RECV_PACKET_LEN], len: &mut usize, timeout_ms: u32) -> Result<(), StatusError> {
            from_ffi(control_sys::recv_lora_packet(buffer.as_mut_ptr().cast(), len, timeout_ms))
        }
    }
}

pub struct LoRaDev {
    controller_id: DeviceId,
    hal: Box<dyn hal::LoRaInterface>,
}

impl Drop for LoRaDev {
    fn drop(&mut self) {
        self.hal.shutdown_module();
    }
}

impl LoRaDev {
    pub fn new() -> Self {
        Self {
            controller_id: rand::random(),
            hal: Box::new(hal::LoRaSys)
        }
    }

    pub fn send(&self, payload: &[u8]) -> Result<(), StatusError> {
        self.hal.send_packet(&payload)
    }

    pub fn recv(&self, payload: &mut Vec<u8>, timeout_ms: u32) -> Result<(), StatusError> {
        payload.clear();
        payload.extend_from_slice(&[0; control_sys::MAX_LORA_RECV_PACKET_LEN]);

        let mut len = 0;
        let result = self.hal.recv_packet(
            payload.first_chunk_mut::<{control_sys::MAX_LORA_RECV_PACKET_LEN}>().unwrap(),
            &mut len,
            timeout_ms,
        );
        
        match result {
            Ok(()) => payload.truncate(len),
            Err(_) => payload.truncate(0),
        }

        result
    }
}

// ------------------------- TRANSPORT, NETWORK, and MAC ------------------------- //

pub struct LoraCommand(Vec<u8>);

pub enum LoraEvent {
    Attached,
    Detached,
    Connected(DeviceId),
    Disconnected(DeviceId),
}


pub fn start_lora_thread(
    device_id: DeviceId,
    commands: crossbeam_channel::Receiver<LoraCommand>,
) -> crossbeam_channel::Receiver<LoraEvent> {
    start_lora_thread_with_hal(device_id, commands, Box::new(hal::LoRaSys))
}

fn start_lora_thread_with_hal(
    device_id: DeviceId,
    commands: crossbeam_channel::Receiver<LoraCommand>,
    hal: Box<dyn hal::LoRaInterface>,
) -> crossbeam_channel::Receiver<LoraEvent> {
    let (sender, receiver) = crossbeam_channel::unbounded();
    std::thread::spawn(|| lora_thread(device_id, commands, sender, hal));
    receiver
}

pub enum LoRaState {
    Detached,
    Attached,
}

// impl WiFiState {
//     pub fn detached() -> Self {
//         Self::NotAttached { poll_timer: Timer::interval(Duration::from_millis(500)) }
//     }
// }


fn lora_thread(
    device_id: DeviceId,
    commands: crossbeam_channel::Receiver<LoraCommand>,
    events: crossbeam_channel::Sender<LoraEvent>,
    hal: Box<dyn hal::LoRaInterface>,
) {
    let mut dev_state = LoRaState::Detached;
    let mut command_queue = VecDeque::new();


    'lora_loop: loop {
        // Collect all the outstanding commands into the queue.
        loop {
            match commands.try_recv() {
                Ok(cmd) => command_queue.push_back(cmd),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break 'lora_loop,
            }
        }
        if command_queue.is_empty() {
            match commands.recv() {
                Ok(cmd) => command_queue.push_back(cmd),
                Err(RecvError) => break 'lora_loop,
            }
        }

        // LoRa does not support fragmentation. All commands must fit.
        let x = command_queue.pop_front().unwrap().0;



        let mut next_dev_state = None;

        match dev_state {
            LoRaState::Detached => {
                if hal.is_module_attached() {
                    if let Err(_) = events.send(WiFiEvent::Attached) {
                        return;
                    }

                    if let Err(e) = hal.initialize_module() {

                    } else {
                        next_wifi_state = Some(LoRaState::Attached);
                    }
                }

                std::thread::sleep(Duration::from_millis(500));
            }
            LoRaState::Attached => {
                let hal_recv = hal.clone();

                let to_send = smol::future::race(
                    async {
                        Either::A(smol::unblock(move || {
                            let mut b = Box::new([0u8; MAX_WIFI_RECV_PACKET_LEN]);
                            hal_recv.recv_packet(&mut b).map(|r| (b, r))
                        }).await)
                    },
                    async {
                        let _ = receiver.timer.next().await;
                        Either::B(())
                    },
                ).await;

                let x = match to_send {
                    Either::A(Ok((recv_data, DevRecv { src_mac, len }))) => {
                        Ok(receiver.on_recv(
                            &device_id,
                            src_mac,
                            &recv_data[..len],
                            |mac| events.send_blocking(WiFiEvent::Connected(mac)).unwrap(),
                            |mac| events.send_blocking(WiFiEvent::Disconnected(mac)).unwrap(),
                            |m| events.send_blocking(WiFiEvent::ReceiveMessage(m)).unwrap(),
                        ))
                    }
                    Either::B(_) => {
                        Ok(Some(receiver.on_timeout()))
                    }
                    Either::A(Err(e)) => Err(e)
                };

                let mut state = WiFiState::Attached { receiver };

                match x {
                    Ok(None) => {},
                    Ok(Some((mac, ack))) => {
                        let hal_send = hal.clone();
                        smol::unblock(move || hal_send.send_packet(mac, &ack)).detach();
                    },
                    Err(StatusError::ModuleDetached) => {
                        if let Err(_) = events.send(WiFiEvent::Detached).await {
                            return;
                        }

                        state = WiFiState::detached();
                    },
                    Err(StatusError::ReceiveTimeout) => unreachable!(),
                    Err(StatusError::Unknown(u)) => unreachable!("Unknown error: {u}"),
                }

                state
            }
        }
    }
}






enum ParseNetworkHeaderError {
    UnexpectedController { expected: DeviceId, read: DeviceId },
    TooShort { buffer_len: usize, required: usize },
}


pub struct LoRaTransport {
    module_id: NonZeroU16,

    est_rtt: f64,
    var_rtt: f64,
    sequence_flag: bool,
}

impl LoRaTransport {
    const TRANSMISSION_CONTROL_ADAPTION: f64 = 0.5;
    const PACKET_RETRIES: usize = 5;

    pub fn transport_net_and_mac(&mut self, to_send: Receiver<Vec<u8>>, to_process: Sender<Vec<u8>>, lora_dev: &LoRaDev) {
        let mut packet_buffer = Vec::with_capacity(
            control_sys::MAX_LORA_RECV_PACKET_LEN.max(control_sys::MAX_LORA_SEND_PACKET_LEN)
        );

        'disconnected: loop {
            let Ok(payload) = to_send.recv() else {
                break 'disconnected;
            };

            let req_header = NetTransHeader {
                sequence_flag: self.sequence_flag,
                controller_id: lora_dev.controller_id,
                module_id: self.module_id.get(),
            };

            packet_buffer.clear();
            req_header.append_network_header(&mut packet_buffer);
            packet_buffer.extend_from_slice(&payload);
            let crc = compute_crc(&packet_buffer);
            packet_buffer.extend_from_slice(&crc.to_le_bytes());            

            for i in 0.. {
                if i > Self::PACKET_RETRIES {
                    break 'disconnected;
                }

                lora_dev.send(&packet_buffer);

                let toa = packet_buffer.len() as f64 / lora_dev.hal.get_lora_byterate() as f64;
                let wait = toa + self.est_rtt + 4.0 * self.var_rtt;
                let timeout_ms = (wait / 1000.0) as u32;
                let t_send = Instant::now();

                match lora_dev.recv(&mut packet_buffer, timeout_ms) {
                    Ok(()) => (),
                    Err(StatusError::ReceiveTimeout) => continue,
                    Err(StatusError::ModuleDetached) => break 'disconnected,
                    Err(StatusError::Unknown(e)) => panic!("Unknown error code {e}"),
                }

                let (body, crc) = last!(u16, packet_buffer);

                if compute_crc(&body) == crc {
                    if let Ok((res_header, bytes)) = NetTransHeader::parse_network_header(&body, lora_dev.controller_id) {
                        if res_header == req_header {
                            if i == 0 {
                                let sample_rtt = (Instant::now() - t_send).as_secs_f64() - toa;
                                let diff_rtt: f64 = sample_rtt - self.est_rtt;
                                self.est_rtt += Self::TRANSMISSION_CONTROL_ADAPTION * diff_rtt;
                                self.var_rtt += Self::TRANSMISSION_CONTROL_ADAPTION * (diff_rtt.abs() - self.var_rtt);
                            }

                            let mut response = Vec::new();
                            response.extend_from_slice(bytes);
                            to_process.send(response);
                        }
                    }
                }
            }
        }
    }
}


// ------------- APPLICATION -------------- //


pub struct PingState {
    controller_id: u16,

    module_id: Option<NonZeroU16>,
    enable_wifi: bool,
    stream_records: bool,

    preserve_old_records: bool,


}

pub struct Command {

}

pub enum ControlEvent {
    SetWifiEnabled { enabled: bool },
    SetStreamRecords { streaming: bool },
    SetPreserveOldRecords { preserve: bool },

}

// impl PingState {
//     // 
//     fn to_bytes(&self, module_id:  buffer: &mut Vec<u8>, commands: &mut VecDeque<Command>) {
//         buffer.extend_from_slice(other);
//     }
// }

pub struct Pong {

    device_id: NonZeroU8,
    is_wifi_enabled: bool,
    is_streaming_records: bool,

    is_preserving_old_records: bool,

    gps: Option<()>
}

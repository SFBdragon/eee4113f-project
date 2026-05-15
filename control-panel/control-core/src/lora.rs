use std::{
    collections::VecDeque,
    sync::Arc,
    thread,
    time::{Duration, Instant},
};

use control_protocol::{
    LoRaAddr,
    lora::{self, LoRaFrame},
    rtt::RttEstimator,
};
use crossbeam_channel::{Receiver, Sender, TryRecvError};

use crate::{drivers::StatusError, utils::Disconnected};

use super::crc::*;

const PACKET_RETRANSMISSIONS: usize = 5;

// ------------------------- DEVICE I/O ABSTRACTION ------------------------- //

pub mod hal {
    use control_protocol::phy::*;

    use crate::drivers::{StatusError, from_ffi};

    pub trait LoRaInterface: Send + Sync {
        fn is_module_attached(&self) -> bool;
        fn initialize_module(&self) -> Result<(), StatusError>;
        fn shutdown_module(&self);
        fn get_lora_byterate(&self) -> u32;
        fn send_packet(&self, bytes: &[u8]) -> Result<(), StatusError>;
        fn recv_packet(
            &self,
            buffer: &mut [u8; MAX_LORA_RECV_PACKET_LEN],
            len: &mut usize,
            timeout_ms: u32,
        ) -> Result<(), StatusError>;
    }
    pub struct LoRaRadio;

    impl LoRaInterface for LoRaRadio {
        fn is_module_attached(&self) -> bool {
            control_data_link::is_lora_module_attached()
        }

        fn initialize_module(&self) -> Result<(), StatusError> {
            from_ffi(control_data_link::initialize_lora_module())
        }

        fn shutdown_module(&self) {
            control_data_link::shutdown_lora_module();
        }

        fn get_lora_byterate(&self) -> u32 {
            control_data_link::get_lora_byterate()
        }

        fn send_packet(&self, bytes: &[u8]) -> Result<(), StatusError> {
            from_ffi(control_data_link::send_lora_packet(
                bytes.as_ptr(),
                bytes.len(),
            ))
        }

        fn recv_packet(
            &self,
            buffer: &mut [u8; MAX_LORA_RECV_PACKET_LEN],
            len: &mut usize,
            timeout_ms: u32,
        ) -> Result<(), StatusError> {
            from_ffi(control_data_link::recv_lora_packet(
                buffer.as_mut_ptr().cast(),
                len,
                timeout_ms,
            ))
        }
    }
}

pub struct LoraCommand(pub LoRaAddr, pub Vec<u8>);

#[derive(Debug, Clone)]
pub enum LoraEvent {
    Attached,
    Detached,
    Discovered(LoRaAddr),
    TimedOut(LoRaAddr),

    Message(Vec<u8>),
}

pub fn start_lora_thread(controller_addr: LoRaAddr) -> (Sender<LoraCommand>, Receiver<LoraEvent>) {
    start_lora_thread_with_hal(controller_addr, Arc::new(hal::LoRaRadio))
}

pub fn start_lora_thread_with_hal(
    controller_addr: LoRaAddr,
    hal: Arc<dyn hal::LoRaInterface>,
) -> (Sender<LoraCommand>, Receiver<LoraEvent>) {
    let (cmd_sender, cmd_receiver) = crossbeam_channel::unbounded();
    let (event_sender, event_receiver) = crossbeam_channel::unbounded();
    std::thread::spawn(move || {
        connector(controller_addr, cmd_receiver, event_sender, hal);
        tracing::error!("LoRa thread returned");
    });
    (cmd_sender, event_receiver)
}

fn connector(
    controller_addr: LoRaAddr,
    commands: Receiver<LoraCommand>,
    events: Sender<LoraEvent>,
    hal: Arc<dyn hal::LoRaInterface>,
) -> Result<(), Disconnected> {
    let _span = tracing::debug_span!("lora_attacher").entered();

    const ATTACHED_POLL_PERIOD: Duration = Duration::from_millis(200);

    loop {
        if hal.is_module_attached() {
            hal.initialize_module();

            events.send(LoraEvent::Attached)?;

            let _ = lora_thread(controller_addr, &commands, &events, hal.as_ref());

            events.send(LoraEvent::Detached)?;
        }

        thread::sleep(ATTACHED_POLL_PERIOD);
    }
}

fn lora_thread(
    controller_addr: LoRaAddr,
    commands: &Receiver<LoraCommand>,
    events: &Sender<LoraEvent>,
    hal: &dyn hal::LoRaInterface,
) -> Result<(), Disconnected> {
    let _span = tracing::debug_span!("lora_comms").entered();

    let mut cmd_queue = VecDeque::new();
    let mut rtt_est = RttEstimator::lora();
    let mut sequence = false;

    loop {
        // Collect all the outstanding commands into the queue.
        loop {
            match commands.try_recv() {
                Ok(cmd) => {
                    tracing::trace!(to = ?cmd.0, len = %cmd.1.len(), "Got LoRa command.");
                    cmd_queue.push_back(cmd);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return Err(Disconnected),
            }
        }
        if cmd_queue.is_empty() {
            cmd_queue.push_back(commands.recv()?);
        }

        let (module_addr, lora_frame) = {
            let LoraCommand(target_dev, cmds) = cmd_queue.pop_front().unwrap();
            let mut lora_frame =
                Vec::with_capacity(cmds.len() + control_protocol::lora::FRAME_OVERHEAD);

            sequence ^= true;
            lora_frame
                .extend_from_slice(&LoRaFrame::first_u16(controller_addr, sequence).to_le_bytes());
            lora_frame.extend_from_slice(&target_dev.to_le_bytes());
            lora_frame.extend_from_slice(&cmds);

            // LoRa does not support fragmentation. All commands must fit.
            while let Some(f) = cmd_queue.pop_front_if(|f| {
                f.0 == target_dev && lora_frame.len() + f.1.len() < lora::MAX_PAYLOAD
            }) {
                lora_frame.extend_from_slice(&f.1);
            }
            let crc = compute_crc(&lora_frame);
            lora_frame.extend_from_slice(&crc.to_le_bytes());

            (target_dev, lora_frame)
        };

        tracing::debug!(%module_addr, len = %lora_frame.len(), "Made LoRa frame");

        'retrans: for i in 0.. {
            if i > 0 {
                tracing::trace!(%i, "Retransmitting LoRa packet.")
            }

            if i > PACKET_RETRANSMISSIONS {
                events.send(LoraEvent::TimedOut(module_addr))?;
                break;
            }

            if let Err(_) = hal.send_packet(&lora_frame) {
                return Err(Disconnected);
            }

            let timeout_ms = rtt_est.rto();
            let t_send = Instant::now();

            loop {
                let mut buf = [0u8; control_protocol::lora::MAX_FRAME];
                let mut len = 0;
                tracing::debug!("LoRa waiting on recv packet");
                match hal.recv_packet(&mut buf, &mut len, timeout_ms) {
                    Ok(()) => (),
                    Err(StatusError::ReceiveTimeout) => {
                        rtt_est.on_timeout();
                        break;
                    }
                    Err(StatusError::ModuleDetached) => return Err(Disconnected),
                    Err(StatusError::Unknown(e)) => panic!("Unknown error code {e}"),
                }

                match LoRaFrame::parse(&buf[..len], compute_crc_raw) {
                    Ok(f) => {
                        if f.con_addr != controller_addr
                            || (!module_addr.is_bcast() && f.mod_addr != module_addr)
                        {
                            tracing::warn!(?f, "Address mismatch, dropping frame.");
                            continue;
                        }

                        if f.sequence_flag != sequence {
                            tracing::warn!(?f, "Sequence mismatch, dropping frame.");
                            continue;
                        }

                        tracing::debug!(?f, "Got seemingly fine frame back.");

                        // Karn's algorithm: only sample if no retransmit.
                        if i == 0 {
                            let rtt = t_send.elapsed().as_millis() as u32;
                            rtt_est.update(rtt);
                        }

                        let mut response = Vec::new();
                        response.extend_from_slice(&f.payload);
                        events.send(LoraEvent::Discovered(f.mod_addr))?;
                        events.send(LoraEvent::Message(response))?;

                        break 'retrans;
                    }
                    Err(e) => {
                        tracing::warn!(error = ?e, "Failed to parse LoRa frame.");
                        continue 'retrans;
                    }
                }
            }
        }
    }
}

use std::{
    sync::{Mutex, MutexGuard},
    time::{Duration, Instant},
};

use control_protocol::{
    LoRaAddr, app,
    lora::LoRaFrame,
    phy::*,
    wifi::{Mac, common::DataFrame, ping::PING_BYTES},
};
use tracing::trace;

use crate::{crc::compute_crc_raw, wifi::hal};

#[derive(Debug)]
pub struct MockModule {
    pub controller_mac: Mac,

    pub state: Mutex<MockModuleInner>,

    pub lora_reply_sender: crossbeam_channel::Sender<Vec<u8>>,
    pub lora_reply_receiver: crossbeam_channel::Receiver<Vec<u8>>,

    pub wifi_try_send_sender: crossbeam_channel::Sender<()>,
    pub wifi_try_send_receiver: crossbeam_channel::Receiver<()>,
    pub wifi_ping_receiver: crossbeam_channel::Receiver<Instant>,
}

#[derive(Debug)]
pub struct MockModuleInner {
    pub addr: LoRaAddr,
    pub mac: Mac,

    pub is_laptop_lora_module_attached: bool,
    pub is_laptop_wifi_module_attached: bool,

    pub is_module_wifi_on: bool,
    pub module_wifi_data_dump: Option<control_protocol::app::StartDataDump>,
    pub wifi_sender_state: control_protocol::wifi::Sender,

    pub storage_mode: app::StoragePolicy,
    pub storage: app::StorageInfo,
    pub lora_recv_window: app::LoRaRecvWindow,
    pub gps: app::GpsInfo,
}

impl MockModuleInner {
    pub fn new() -> Self {
        let addr = LoRaAddr::from_raw(0x33AD);
        let mac = Mac::from(rand::random::<u64>());
        trace!(%addr, %mac, "Created mock module.");

        Self {
            addr,
            mac,
            is_laptop_lora_module_attached: true,
            is_laptop_wifi_module_attached: true,
            is_module_wifi_on: false,
            module_wifi_data_dump: None,
            wifi_sender_state: control_protocol::wifi::Sender::new(),
            storage_mode: control_protocol::app::StoragePolicy::Preserve,
            storage: control_protocol::app::StorageInfo {
                total_blocks: 100,
                available_begin: 70,
                available_end: 30,
                overwritable_end: 11,
                generation: 1,
            },
            lora_recv_window: control_protocol::app::LoRaRecvWindow {
                on_period: 1,
                total_period: 1,
            },
            gps: control_protocol::app::GpsInfo { lat: 0, lon: 0 },
        }
    }

    pub fn lora_state(&self) -> control_protocol::app::LoRaModuleState {
        use control_protocol::app::LoRaModuleState;

        control_protocol::app::LoRaModuleState {
            status_flags: LoRaModuleState::STATUS_WIFI_ON * self.is_module_wifi_on as u8
                | LoRaModuleState::STATUS_WIFI_DUMPING * self.module_wifi_data_dump.is_some() as u8,
            // | LoRaModuleState::STATUS_WIFI_CONN
            //     * matches!(
            //         self.wifi_sender_state.state,
            //         control_protocol::wifi::sender::SenderState::Connected { .. }
            //     ) as u8,
            storage_policy: self.storage_mode,
            storage_info: self.storage,
            lora_recv_window: self.lora_recv_window,
            gps_info: self.gps,
        }
    }
}

impl MockModule {
    pub fn new() -> Self {
        let lora_channel = crossbeam_channel::unbounded();
        let wifi_channel = crossbeam_channel::unbounded();
        let wifi_pings = crossbeam_channel::tick(Duration::from_millis(
            control_protocol::wifi::common::PING_PERIOD_MS as _,
        ));

        Self {
            controller_mac: Mac::from(rand::random::<u64>()),
            state: Mutex::new(MockModuleInner::new()),
            lora_reply_sender: lora_channel.0,
            lora_reply_receiver: lora_channel.1,
            wifi_try_send_sender: wifi_channel.0,
            wifi_try_send_receiver: wifi_channel.1,
            wifi_ping_receiver: wifi_pings,
        }
    }

    pub fn state(&self) -> MutexGuard<'_, MockModuleInner> {
        self.state.lock().unwrap()
    }
}

impl crate::lora::hal::LoRaInterface for MockModule {
    fn is_module_attached(&self) -> bool {
        self.state().is_laptop_lora_module_attached
    }

    fn initialize_module(&self) -> Result<(), crate::drivers::StatusError> {
        Ok(())
    }

    fn shutdown_module(&self) {}

    fn get_lora_byterate(&self) -> u32 {
        100
    }

    fn send_packet(&self, bytes: &[u8]) -> Result<(), crate::drivers::StatusError> {
        fn set_timer_no_op(_t_ms: u32) {}
        fn cancel_timer_no_op() {}
        fn get_time() -> u32 {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u32
        }

        let state = self.state();

        if !state.is_laptop_lora_module_attached {
            return Err(crate::drivers::StatusError::ModuleDetached);
        }

        let lora_frame = LoRaFrame::parse(bytes, compute_crc_raw).unwrap();
        tracing::debug!(?lora_frame, "mock: got lora frame");

        if !lora_frame.mod_addr.is_bcast() && lora_frame.mod_addr != state.addr {
            tracing::warn!("LoRa: wrong module address, dropping the packet.");
            return Ok(());
        }

        drop(state);

        let mut pos = 0;
        while pos < lora_frame.payload.len() {
            let cmd = control_protocol::app::LoRaCmd::parse(lora_frame.payload, &mut pos).unwrap();

            match cmd {
                app::LoRaCmd::EnableWiFi => {
                    let mut state = self.state();
                    state.is_module_wifi_on = true;
                }
                app::LoRaCmd::DisableWiFi => {
                    let mut state = self.state();
                    state.is_module_wifi_on = false;
                }
                app::LoRaCmd::StartDataDump(start_data_dump) => {

                    let mut state = self.state();
                    state.is_module_wifi_on = true;
                    state.wifi_sender_state.connect(
                        lora_frame.con_addr,
                        set_timer_no_op,
                        cancel_timer_no_op,
                        get_time,
                        compute_crc_raw,
                    );
                    drop(state);

                    tracing::trace!("Sending SYN packet to WiFi interface.");

                    self.wifi_try_send_sender.send(()).unwrap();

                    self.state().module_wifi_data_dump = Some(start_data_dump)
                }
                app::LoRaCmd::CancelDataDump => self.state().module_wifi_data_dump = None,
                app::LoRaCmd::SetLoRaRecvWindow(win) => self.state().lora_recv_window = win,
                app::LoRaCmd::SetOverwritable(ow) => {
                    self.state().storage.overwritable_end = ow.up_to
                }
                app::LoRaCmd::SetStoragePolicy(storage_policy) => {
                    self.state().storage_mode = storage_policy
                }
            }
        }

        // Create the reply
        let state = self.state();
        let mut pos = 0;
        let mut buf = [0u8; control_protocol::lora::MAX_FRAME];
        state.lora_state().serialize(&mut buf, &mut pos).unwrap();
        let f = LoRaFrame {
            con_addr: lora_frame.con_addr,
            mod_addr: state.addr,
            sequence_flag: lora_frame.sequence_flag, // ack this seq
            payload: &buf[..pos],
        };
        let mut buf = [0u8; control_protocol::lora::MAX_FRAME];
        let len = f.serialize(&mut buf, compute_crc_raw);
        self.lora_reply_sender.send(buf[..len].to_vec()).unwrap();

        Ok(())
    }

    fn recv_packet(
        &self,
        buffer: &mut [u8; MAX_LORA_RECV_PACKET_LEN],
        len: &mut usize,
        timeout_ms: u32,
    ) -> Result<(), crate::drivers::StatusError> {
        tracing::trace!("mock: recv_packet");

        let r = self
            .lora_reply_receiver
            .recv_timeout(Duration::from_millis(timeout_ms as _));

        match r {
            Ok(r) => {
                tracing::trace!("mock: recv_packet: got message");

                let state = self.state();
                if state.is_laptop_lora_module_attached {
                    buffer[..r.len()].copy_from_slice(&r);
                    *len = r.len();
                    Ok(())
                } else {
                    Err(crate::drivers::StatusError::ModuleDetached)
                }
            }
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                tracing::trace!("mock: recv_packet: timeout");

                Err(crate::drivers::StatusError::ReceiveTimeout)
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                Err(crate::drivers::StatusError::ModuleDetached)
            }
        }
    }
}

impl hal::WiFiInterface for MockModule {
    fn is_module_attached(&self) -> bool {
        self.state().is_laptop_wifi_module_attached
    }

    fn initialize_module(&self) -> Result<(), crate::drivers::StatusError> {
        Ok(())
    }

    fn shutdown_module(&self) {}

    fn get_byterate(&self) -> u32 {
        todo!()
    }

    fn send_packet(&self, dst_mac: Mac, bytes: &[u8]) -> Result<(), crate::drivers::StatusError> {
        let mut state = self.state();

        if !state.is_laptop_wifi_module_attached {
            return Err(crate::drivers::StatusError::ModuleDetached);
        }

        tracing::debug!(%dst_mac, ?bytes, "for module WiFI.");

        if dst_mac.is_bcast() || dst_mac == state.mac {
            let try_send = state
                .wifi_sender_state
                .on_recv_ack(self.controller_mac, bytes);

            if try_send {
                self.wifi_try_send_sender.send(()).unwrap();
            }
        }

        Ok(())
    }

    fn recv_packet(&self) -> Result<hal::WiFiPacket, crate::drivers::StatusError> {
        loop {
            loop {
                if !self.state().is_laptop_wifi_module_attached {
                    return Err(crate::drivers::StatusError::ModuleDetached);
                }

                let mut sel = crossbeam_channel::Select::new();
                let msg = sel.recv(&self.wifi_try_send_receiver);
                let ping = sel.recv(&self.wifi_ping_receiver);

                match sel.ready_timeout(Duration::from_millis(50)) {
                    Err(_) => {}
                    Ok(i) if i == msg => {
                        self.wifi_try_send_receiver.recv().unwrap();
                        break;
                    }
                    Ok(i) if i == ping => {
                        self.wifi_ping_receiver.recv().unwrap();

                        let state = self.state();

                        if state.is_laptop_wifi_module_attached {
                            let mut ping_buf = [0u8; PING_BYTES];
                            unsafe {
                                control_protocol::wifi::ping::write_ping(
                                    state.addr,
                                    compute_crc_raw,
                                    ping_buf.as_mut_ptr(),
                                );
                            }

                            return Ok(hal::WiFiPacket {
                                from: state.mac,
                                data: ping_buf.to_vec(),
                            });
                        }
                    }
                    Ok(_) => unreachable!(),
                }
            }

            tracing::trace!("mock: wifi module will try to send");

            let mut state = self.state();

            let mut mac = Mac::from(0);
            let mut ptr = std::ptr::null();
            let mut len = 0;
            let should_send = state
                .wifi_sender_state
                .send_next(&mut mac, &mut ptr, &mut len);

            if !should_send {
                tracing::trace!("mock: wifi module did not send");
                continue;
            } else {
                tracing::trace!("mock: wifi module sending");
            }

            let buf = unsafe { std::slice::from_raw_parts(ptr, len as _) };

            if state.is_laptop_wifi_module_attached {
                return Ok(hal::WiFiPacket {
                    from: state.mac,
                    data: buf.to_vec(),
                });
            } else {
                return Err(crate::drivers::StatusError::ModuleDetached);
            }
        }
    }
}

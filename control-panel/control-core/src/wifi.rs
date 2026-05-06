use std::{
    sync::Arc, time::Duration
};

use control_protocol::{DeviceIdAtomic, wifi::common::Mac};
use control_sys::MAX_WIFI_RECV_PACKET_LEN;
use smol::{Timer, channel, stream::StreamExt};

use crate::{drivers::StatusError, wifi::{hal::DevRecv, receiver::Receiver}};

mod receiver;


// ------------------------- DEVICE I/O ABSTRACTION ------------------------- //


mod hal {
    use control_protocol::wifi::common::Mac;

use crate::drivers::{StatusError, from_ffi};

    pub trait WiFiInterface: Send + Sync {
        fn is_module_attached(&self) -> bool;
        fn initialize_module(&self) -> Result<(), StatusError>;
        fn shutdown_module(&self);
        fn get_byterate(&self) -> u32;
        fn send_packet(&self, dst_mac: Mac, bytes: &[u8]) -> Result<(), StatusError>;
        fn recv_packet(&self, buffer: &mut [u8; control_sys::MAX_WIFI_RECV_PACKET_LEN]) -> Result<DevRecv, StatusError>;
    }
    pub struct WiFiModule;

    pub struct DevRecv {
        pub src_mac: Mac,
        pub len: usize,
    }

    impl WiFiInterface for WiFiModule {
        fn is_module_attached(&self) -> bool {
            control_sys::is_lora_module_attached()
        }

        fn initialize_module(&self) -> Result<(), StatusError> {
            from_ffi(control_sys::initialize_lora_module())
        }

        fn shutdown_module(&self) {
            control_sys::shutdown_lora_module();
        }

        fn get_byterate(&self) -> u32 {
            control_sys::get_lora_byterate()
        }

        fn send_packet(&self, dst_mac: Mac, bytes: &[u8]) -> Result<(), StatusError> {
            from_ffi(control_sys::send_wifi_packet(dst_mac.to_u64(), bytes.as_ptr(), bytes.len() as _))
        }

        fn recv_packet(&self, buffer: &mut [u8; control_sys::MAX_WIFI_RECV_PACKET_LEN]) -> Result<DevRecv, StatusError> {
            let mut src_mac = 0u64;
            let mut len = 0u16;
            from_ffi(control_sys::recv_wifi_packet(&mut src_mac, buffer.as_mut_ptr().cast(), &mut len))
                .map(|_| DevRecv { src_mac: src_mac.into(), len: len as _ })
        
        }
    }
}

pub enum WiFiEvent {
    /// WiFi hardware module detected.
    Attached,
    /// WiFi hardware module was not found.
    Detached,
    /// Received WiFi packet recently.
    Connected(Mac),
    /// No WiFi packets in a while.
    Disconnected(Mac),

    /// Got a message over WiFi.
    ReceiveMessage(Vec<u8>),
}

pub fn start_wifi_listener_thread(device_id: Arc<DeviceIdAtomic>) -> channel::Receiver<WiFiEvent> {
    start_wifi_listener_thread_with_hal(device_id, Arc::new(hal::WiFiModule))
}

fn start_wifi_listener_thread_with_hal(device_id: Arc<DeviceIdAtomic>, hal: Arc<dyn hal::WiFiInterface>) -> channel::Receiver<WiFiEvent> {
    let (sender, receiver) = channel::unbounded();
    std::thread::spawn(|| wifi_listener(hal, device_id, sender));
    receiver
}

pub enum WiFiState {
    NotAttached { poll_timer: Timer },
    Attached { receiver: Receiver },
}

impl WiFiState {
    pub fn detached() -> Self {
        Self::NotAttached { poll_timer: Timer::interval(Duration::from_millis(500)) }
    }
}


async fn wifi_listener(hal: Arc<dyn hal::WiFiInterface>, device_id: Arc<DeviceIdAtomic>, sender: channel::Sender<WiFiEvent>) {
    // Simple union to handle the racing async block return values.
    enum Either<T, U> {
        A(T),
        B(U)
    }

    let mut dev_state: WiFiState = WiFiState::detached();

    loop {
        dev_state = match dev_state {
            WiFiState::NotAttached { mut poll_timer } => {
                poll_timer.next().await;

                if hal.is_module_attached() {
                    if let Err(_) = sender.send(WiFiEvent::Attached).await {
                        return;
                    }

                    WiFiState::Attached { receiver: Receiver::new() }
                } else {
                    WiFiState::NotAttached { poll_timer }
                }
            }
            WiFiState::Attached { mut receiver } => {
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
                            |mac| sender.send_blocking(WiFiEvent::Connected(mac)).unwrap(),
                            |mac| sender.send_blocking(WiFiEvent::Disconnected(mac)).unwrap(),
                            |m| sender.send_blocking(WiFiEvent::ReceiveMessage(m)).unwrap(),
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
                        if let Err(_) = sender.send(WiFiEvent::Detached).await {
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


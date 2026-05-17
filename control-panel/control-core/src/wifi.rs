use std::{sync::Arc, thread, time::Duration};

use control_protocol::{LoRaAddr, wifi::Mac};
use crossbeam_channel::{Receiver, Sender, never};

use crate::{drivers::StatusError, utils::Disconnected, wifi::rx::WiFiRx};

mod rx;

// ------------------------- DEVICE I/O ABSTRACTION ------------------------- //

pub mod hal {
    use control_protocol::{phy::*, wifi::Mac};

    use crate::drivers::{StatusError, from_ffi};

    pub struct WiFiPacket {
        pub from: Mac,
        pub data: Vec<u8>,
    }

    pub trait WiFiInterface: Send + Sync {
        fn is_module_attached(&self) -> bool;
        fn initialize_module(&self) -> Result<(), StatusError>;
        fn shutdown_module(&self);
        fn send_packet(&self, dst_mac: Mac, bytes: &[u8]) -> Result<(), StatusError>;
        fn recv_packet(&self) -> Result<WiFiPacket, StatusError>;
    }
    pub struct WiFiRadio;

    // impl WiFiInterface for WiFiRadio {
    //     fn is_module_attached(&self) -> bool {
    //         control_data_link::is_lora_module_attached()
    //     }

    //     fn initialize_module(&self) -> Result<(), StatusError> {
    //         from_ffi(control_data_link::initialize_wifi_module())
    //     }

    //     fn shutdown_module(&self) {
    //         control_data_link::shutdown_lora_module();
    //     }

    //     fn send_packet(&self, dst_mac: Mac, bytes: &[u8]) -> Result<(), StatusError> {
    //         from_ffi(control_data_link::send_wifi_packet(
    //             dst_mac.to_u64(),
    //             bytes.as_ptr(),
    //             bytes.len() as _,
    //         ))
    //     }

    //     fn recv_packet(&self) -> Result<WiFiPacket, StatusError> {
    //         let mut src_mac = 0u64;
    //         let mut len = 0u16;
    //         let mut vec = vec![0u8; MAX_WIFI_RECV_PACKET_LEN];
    //         from_ffi(control_data_link::recv_wifi_packet(
    //             &mut src_mac,
    //             vec.as_mut_array::<MAX_WIFI_RECV_PACKET_LEN>().unwrap(),
    //             &mut len,
    //         ))
    //         .map(|_| {
    //             vec.truncate(len as _);

    //             WiFiPacket {
    //                 from: Mac::from(src_mac),
    //                 data: vec,
    //             }
    //         })
    //     }
    // }
}

#[derive(Debug)]
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

    /// Got a ping over WiFi.
    Ping((Mac, LoRaAddr)),
}

pub fn start_wifi_listener_thread_with_hal(
    controller_addr: LoRaAddr,
    hal: Arc<dyn hal::WiFiInterface>,
) -> Receiver<WiFiEvent> {
    let (sender, receiver) = crossbeam_channel::unbounded();
    thread::spawn(move || wifi_attacher(controller_addr, hal, sender));
    receiver
}

fn wifi_attacher(
    controller_addr: LoRaAddr,
    hal: Arc<dyn hal::WiFiInterface>,
    sender: Sender<WiFiEvent>,
) -> Result<(), Disconnected> {
    let _span = tracing::debug_span!("wifi_attacher").entered();

    const ATTACHED_POLL_PERIOD: Duration = Duration::from_millis(200);

    loop {
        if hal.is_module_attached() {
            hal.initialize_module();

            sender.send(WiFiEvent::Attached)?;

            let _ = wifi_listener(controller_addr, hal.as_ref(), &sender);

            sender.send(WiFiEvent::Detached)?;
        }

        thread::sleep(ATTACHED_POLL_PERIOD);
    }
}

fn wifi_listener(
    controller_addr: LoRaAddr,
    hal: &dyn hal::WiFiInterface,
    sender: &Sender<WiFiEvent>,
) -> Result<(), Disconnected> {
    let _span = tracing::debug_span!("wifi_listener").entered();

    thread::scope::<_, Result<(), Disconnected>>(move |s| {
        // Create a listener thread dedicated to listening for WiFi packets.
        // This allows more flexibility in terms of what this thread does, including handling timeouts.
        let (listen_sender, listen_receiver) =
            crossbeam_channel::unbounded::<Result<hal::WiFiPacket, StatusError>>();

        s.spawn(move || {
            while let Ok(_) = listen_sender.send(hal.recv_packet()) {
                tracing::debug!("WiFi: received a packet")
            }
        });

        let mut rx = WiFiRx::new(controller_addr);

        loop {
            let either = crossbeam_channel::select! {
                recv(listen_receiver) -> msg => Either::A(msg.unwrap()),
                recv(rx.timeout.as_ref().unwrap_or(&never())) -> msg => Either::B(msg.unwrap()),
            };

            let to_send = match either {
                Either::A(Ok(packet)) => {
                    tracing::debug!(?packet.data, "Received a packet");

                    rx.on_recv(
                        packet.from,
                        &packet.data,
                        |mac| {
                            let _ = sender.send(WiFiEvent::Connected(mac));
                        },
                        |mac| {
                            let _ = sender.send(WiFiEvent::Disconnected(mac));
                        },
                        |m| {
                            let _ = sender.send(WiFiEvent::ReceiveMessage(m));
                        },
                        |(m, a)| {
                            let _ = sender.send(WiFiEvent::Ping((m, a)));
                        },
                    )
                }
                Either::A(Err(status)) => match status {
                    StatusError::ModuleDetached => return Err(Disconnected),
                    StatusError::ReceiveTimeout => unreachable!(),
                    StatusError::Unknown(u) => unreachable!("{u}"),
                },
                Either::B(_) => Some(rx.on_timeout()),
            };

            if let Some((mac, ack)) = to_send {
                if let Err(StatusError::ModuleDetached) = hal.send_packet(mac, &ack) {
                    return Err(Disconnected);
                }
            }
        }
    })
}

// Simple union to handle the select.
enum Either<T, U> {
    A(T),
    B(U),
}

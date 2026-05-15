use std::{
    io::ErrorKind,
    net::UdpSocket,
    sync::{Mutex, MutexGuard},
    time::Duration,
};

use control_protocol::{phy::*, wifi::Mac};
use tracing::trace;

use crate::wifi::hal;

#[derive(Debug)]
pub struct UdpMockRadios {
    pub controller_mac: Mac,
    pub lora_sock: UdpSocket,
    pub wifi_sock: UdpSocket,
    pub state: Mutex<MockModule2Inner>,
}

#[derive(Debug)]
pub struct MockModule2Inner {
    pub is_laptop_lora_module_attached: bool,
    pub is_lora_send_failing: bool,
    pub is_lora_recv_failing: bool,
    pub is_laptop_wifi_module_attached: bool,
    pub is_wifi_send_failing: bool,
    pub is_wifi_recv_failing: bool,
}

impl MockModule2Inner {
    pub fn new() -> Self {
        trace!("Created mock module.");

        Self {
            is_laptop_lora_module_attached: false,
            is_lora_send_failing: false,
            is_lora_recv_failing: false,
            is_laptop_wifi_module_attached: false,
            is_wifi_send_failing: false,
            is_wifi_recv_failing: false,
        }
    }
}

impl UdpMockRadios {
    pub fn new() -> Self {
        let lora_sock = UdpSocket::bind("127.0.0.1:12000").unwrap();
        lora_sock.set_nonblocking(false).unwrap();
        let wifi_sock = UdpSocket::bind("127.0.0.1:12010").unwrap();
        wifi_sock.set_nonblocking(false).unwrap();

        Self {
            controller_mac: Mac::from(rand::random::<u64>()),
            state: Mutex::new(MockModule2Inner::new()),
            lora_sock,
            wifi_sock,
        }
    }

    pub fn state(&self) -> MutexGuard<'_, MockModule2Inner> {
        self.state.lock().unwrap()
    }
}

impl crate::lora::hal::LoRaInterface for UdpMockRadios {
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
        let state = self.state();

        if !state.is_laptop_lora_module_attached {
            return Err(crate::drivers::StatusError::ModuleDetached);
        }

        if state.is_lora_send_failing {
            return Ok(());
        }

        drop(state);

        match self.lora_sock.send_to(bytes, "127.0.0.1:12001") {
            Ok(len) => {
                if len != bytes.len() {
                    panic!();
                }

                tracing::debug!("Sent LoRa packet");

                Ok(())
            }
            Err(err) => {
                tracing::error!(?err, "LoRa send failed");
                Err(crate::drivers::StatusError::ModuleDetached)
            }
        }
    }

    fn recv_packet(
        &self,
        buffer: &mut [u8; MAX_LORA_RECV_PACKET_LEN],
        len: &mut usize,
        timeout_ms: u32,
    ) -> Result<(), crate::drivers::StatusError> {
        {
            let state = self.state();
            if !state.is_laptop_lora_module_attached {
                return Err(crate::drivers::StatusError::ModuleDetached);
            }
        }

        self.lora_sock
            .set_read_timeout(Some(Duration::from_millis(timeout_ms as _)))
            .unwrap();

        *len = loop {
            match self.lora_sock.recv(buffer) {
                Ok(d) => {
                    if self.state().is_lora_recv_failing {
                        continue;
                    } else {
                        break d;
                    }
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => {
                    // timeout on linux
                    return Err(crate::drivers::StatusError::ReceiveTimeout);
                }
                Err(err) if err.kind() == ErrorKind::TimedOut => {
                    return Err(crate::drivers::StatusError::ReceiveTimeout);
                }
                Err(err) => panic!("{:?}", err),
            };
        };

        Ok(())
    }
}

impl hal::WiFiInterface for UdpMockRadios {
    fn is_module_attached(&self) -> bool {
        self.state().is_laptop_wifi_module_attached
    }

    fn initialize_module(&self) -> Result<(), crate::drivers::StatusError> {
        Ok(())
    }

    fn shutdown_module(&self) {}

    fn send_packet(&self, dst_mac: Mac, bytes: &[u8]) -> Result<(), crate::drivers::StatusError> {
        let state = self.state();

        if !state.is_laptop_wifi_module_attached {
            return Err(crate::drivers::StatusError::ModuleDetached);
        }

        if state.is_wifi_send_failing {
            return Ok(());
        }

        drop(state);

        tracing::debug!(%dst_mac, ?bytes, "for module WiFI.");

        let mut buf = [0u8; MAX_WIFI_SEND_PACKET_LEN + 16];
        buf[..8].copy_from_slice(&self.controller_mac.to_u64().to_le_bytes());
        buf[8..16].copy_from_slice(&dst_mac.to_u64().to_le_bytes());
        buf[16..][..bytes.len()].copy_from_slice(bytes);

        match self
            .wifi_sock
            .send_to(&buf[..(bytes.len() + 16)], "127.0.0.1:12011")
        {
            Ok(_) => Ok(()),
            Err(err) => {
                tracing::error!(?err, "WiFi send failed");
                Err(crate::drivers::StatusError::ModuleDetached)
            }
        }
    }

    fn recv_packet(&self) -> Result<hal::WiFiPacket, crate::drivers::StatusError> {
        let state = self.state();
        if !state.is_laptop_wifi_module_attached {
            return Err(crate::drivers::StatusError::ModuleDetached);
        }

        drop(state);

        let mut buf = [0u8; MAX_WIFI_RECV_PACKET_LEN + 8];

        loop {
            let data_len = match self.wifi_sock.recv(&mut buf) {
                Ok(d) => d,
                Err(e) => panic!("{:?}", e),
            };

            let mac_src = Mac::from(u64::from_le_bytes(buf[..8].try_into().unwrap()));
            let mac_dst = Mac::from(u64::from_le_bytes(buf[8..16].try_into().unwrap()));

            if mac_dst != self.controller_mac && !mac_dst.is_bcast() {
                continue;
            }

            let payload = &buf[16..data_len];

            if self.state().is_wifi_recv_failing {
                continue;
            }

            return Ok(hal::WiFiPacket {
                from: mac_src,
                data: payload.to_vec(),
            });
        }
    }
}

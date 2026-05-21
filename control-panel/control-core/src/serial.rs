use control_protocol::wifi::Mac;
use control_serial_ports::{SerialError, wifi::WifiModule};

use crate::{
    drivers::StatusError,
    lora::hal::LoRaInterface,
    wifi::hal::{WiFiInterface, WiFiPacket},
};

pub struct LoRaSerial;

impl LoRaInterface for LoRaSerial {
    fn is_module_attached(&self) -> bool {
        control_serial_ports::is_lora_module_attached()
    }

    fn initialize_module(&self) -> Result<(), crate::drivers::StatusError> {
        Ok(())
    }

    fn shutdown_module(&self) {}

    fn get_lora_byterate(&self) -> u32 {
        40
    }

    fn send_packet(&self, bytes: &[u8]) -> Result<(), crate::drivers::StatusError> {
        control_serial_ports::send_lora_packet(bytes).map_err(|e| match e {
            SerialError::Timeout => StatusError::ReceiveTimeout,
            SerialError::Detached => StatusError::ModuleDetached,
            SerialError::Io(_) => StatusError::ModuleDetached,
            SerialError::Port(_) => StatusError::ModuleDetached,
        })
    }

    fn recv_packet(
        &self,
        buffer: &mut [u8; control_protocol::phy::MAX_LORA_RECV_PACKET_LEN],
        len: &mut usize,
        timeout_ms: u32,
    ) -> Result<(), crate::drivers::StatusError> {
        let data = control_serial_ports::recv_lora_packet(timeout_ms).map_err(|e| match e {
            SerialError::Timeout => StatusError::ReceiveTimeout,
            SerialError::Detached => StatusError::ModuleDetached,
            SerialError::Io(_) => StatusError::ModuleDetached,
            SerialError::Port(_) => StatusError::ModuleDetached,
        })?;

        buffer[..data.len()].copy_from_slice(&data);
        *len = data.len();
        Ok(())
    }
}

pub struct WiFiSerial(WifiModule);

impl WiFiSerial {
    pub fn new() -> Self {
        Self(WifiModule::new())
    }
}

impl WiFiInterface for WiFiSerial {
    fn is_module_attached(&self) -> bool {
        self.0.is_attached()
    }

    fn initialize_module(&self) -> Result<(), crate::drivers::StatusError> {
        Ok(())
    }

    fn shutdown_module(&self) {
        self.0.detach();
    }

    fn send_packet(&self, _dstmac: Mac, bytes: &[u8]) -> Result<(), crate::drivers::StatusError> {
        self.0.send_packet(bytes).ok_or(StatusError::ModuleDetached)
    }

    fn recv_packet(&self) -> Result<WiFiPacket, crate::drivers::StatusError> {
        self.0
            .recv_packet()
            .map(|data| WiFiPacket {
                from: Mac::bcast(),
                data,
            })
            .ok_or(StatusError::ModuleDetached)
    }
}

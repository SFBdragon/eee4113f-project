use control_serial_ports::SerialError;

use crate::{drivers::StatusError, lora::hal::LoRaInterface};

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
        dbg!("hiiiiiiiiiiii");
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

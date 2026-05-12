use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender};
use protocol::LoRaAddr;

use crate::mock::MockModule;

pub use control_protocol as protocol;

mod crc;
mod drivers;
pub mod lora;
pub mod mock;
pub mod utils;
pub mod wifi;

#[derive(Debug, Clone)]
pub struct Controller {
    pub addr: LoRaAddr,
    pub wifi_receiver: Receiver<wifi::WiFiEvent>,
    pub lora_commands: Sender<lora::LoraCommand>,
    pub lora_events: Receiver<lora::LoraEvent>,
}

impl Controller {
    pub fn new() -> Self {
        let addr = LoRaAddr::from_raw(rand::random());

        let wifi_receiver = wifi::start_wifi_listener_thread(addr);
        let (lora_commands, lora_events) = lora::start_lora_thread(addr);

        Self {
            addr,
            wifi_receiver,
            lora_commands,
            lora_events,
        }
    }

    pub fn mocked() -> (Self, Arc<MockModule>) {
        let addr = LoRaAddr::from_raw(0x1CAD);
        let mock_module = Arc::new(MockModule::new());

        let wifi_receiver = wifi::start_wifi_listener_thread_with_hal(addr, mock_module.clone());

        let (lora_commands, lora_events) =
            lora::start_lora_thread_with_hal(addr, mock_module.clone());

        (
            Self {
                addr,
                wifi_receiver,
                lora_commands,
                lora_events,
            },
            mock_module,
        )
    }
}

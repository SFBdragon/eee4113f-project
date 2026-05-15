use std::sync::Arc;

use crossbeam_channel::{Receiver, Sender};
use protocol::LoRaAddr;

use crate::{mock::MockModule, mock2::UdpMockRadios};

pub use control_protocol as protocol;

mod crc;
mod drivers;
pub mod lora;
pub mod mock;
pub mod mock2;
pub mod utils;
pub mod wifi;

pub struct Controller {
    pub addr: LoRaAddr,
    pub wifi_hal: Arc<dyn wifi::hal::WiFiInterface>,
    pub wifi_receiver: Receiver<wifi::WiFiEvent>,
    pub lora_commands: Sender<lora::LoraCommand>,
    pub lora_events: Receiver<lora::LoraEvent>,
}

impl Controller {
    pub fn new() -> Self {
        let addr = LoRaAddr::from_raw(rand::random());

        let wifi_hal = Arc::new(wifi::hal::WiFiRadio);
        let wifi_receiver = wifi::start_wifi_listener_thread_with_hal(addr, wifi_hal.clone());
        let (lora_commands, lora_events) = lora::start_lora_thread(addr);

        Self {
            addr,
            wifi_hal,
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
                wifi_hal: mock_module.clone(),
                wifi_receiver,
                lora_commands,
                lora_events,
            },
            mock_module.clone(),
        )
    }

    pub fn mocked2() -> (Self, Arc<UdpMockRadios>) {
        let addr = LoRaAddr::from_raw(0x1CAD);
        let mock_module = Arc::new(UdpMockRadios::new());

        let wifi_receiver = wifi::start_wifi_listener_thread_with_hal(addr, mock_module.clone());

        let (lora_commands, lora_events) =
            lora::start_lora_thread_with_hal(addr, mock_module.clone());

        (
            Self {
                addr,
                wifi_hal: mock_module.clone(),
                wifi_receiver,
                lora_commands,
                lora_events,
            },
            mock_module.clone(),
        )
    }
}



pub struct LoRaDev {
    hal: Box<dyn hal::LoRaInterface>,
}

impl Drop for LoRaDev {
    fn drop(&mut self) {
        self.hal.shutdown_module();
    }
}

impl LoRaDev {
    pub fn new() -> Self {
        Self { hal: Box::new(hal::LoRaSys) }
    }
}


mod hal {
    use crate::drivers::{StatusError, from_ffi};


    pub trait LoRaInterface {
        fn is_module_attached(&self) -> bool;
        fn initialize_module(&self) -> Result<(), StatusError>;
        fn shutdown_module(&self);
        fn send_packet(&self, bytes: &[u8]) -> Result<(), StatusError>;
        fn recv_packet(&self, buffer: &mut [u8; control_sys::MAX_LORA_RECV_PACKET_LEN], len: &mut usize) -> Result<(), StatusError>;
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

        fn send_packet(&self, bytes: &[u8]) -> Result<(), StatusError> {
            from_ffi(control_sys::send_lora_packet(bytes.as_ptr(), bytes.len()))
        }

        fn recv_packet(&self, buffer: &mut [u8; control_sys::MAX_LORA_RECV_PACKET_LEN], len: &mut usize) -> Result<(), StatusError> {
            from_ffi(control_sys::recv_lora_packet(buffer.as_mut_ptr().cast(), len))
        }
    }
}

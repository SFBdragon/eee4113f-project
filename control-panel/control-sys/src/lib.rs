#![no_std]

pub type Status = i32;

pub const MAX_LORA_RECV_PACKET_LEN: usize = 64;
pub const MAX_LORA_SEND_PACKET_LEN: usize = 64;

pub const MAX_WIFI_RECV_PACKET_LEN: usize = 255;
pub const MAX_WIFI_SEND_PACKET_LEN: usize = 255;

pub const STATUS_SUCCESSFUL: Status = 0;
pub const STATUS_RECEIVE_TIMEOUT: Status = -3;
pub const STATUS_MODULE_DETACHED: Status = -5;

unsafe extern "C" {
    pub safe fn is_lora_module_attached() -> bool;

    pub safe fn initialize_lora_module() -> Status;
    pub safe fn shutdown_lora_module();

    pub safe fn get_lora_byterate() -> u32;

    pub safe fn send_lora_packet(data: *const u8, len: usize) -> Status;
    pub safe fn recv_lora_packet(data: *mut [u8; MAX_LORA_RECV_PACKET_LEN], len: *mut usize, timeout_ms: u32) -> Status;


    pub safe fn is_wifi_module_attached() -> bool;

    pub safe fn initialize_wifi_module() -> Status;
    pub safe fn shutdown_wifi_module();

    pub safe fn send_wifi_packet(dst_mac: u64, data: *const u8, len: u16) -> Status;
    pub safe fn recv_wifi_packet(src_mac: *mut u64, data: *mut [u8; MAX_WIFI_RECV_PACKET_LEN], len: *mut u16) -> Status;
}

#[cfg(test)]
mod tests {
    use super::*;

    // Ensure the compiler attempts to link all the defined functions to
    // catch certain linking issues.
    #[test]
    fn call_every_function() {
        if is_lora_module_attached() {
            initialize_lora_module();
            shutdown_lora_module();
            let data = [1, 2, 3, 4];
            send_lora_packet(data.as_ptr_range().start, data.len());
            let mut out_data = [0; MAX_LORA_RECV_PACKET_LEN];
            let mut len = 0;
            recv_lora_packet(&raw mut out_data, &mut len, 0);
        }

        if is_wifi_module_attached() {
            initialize_wifi_module();
            shutdown_wifi_module();
            let data = [1, 2, 3, 4];
            send_wifi_packet(123, data.as_ptr_range().start, data.len() as _);
            let mut out_data = [0; MAX_WIFI_RECV_PACKET_LEN];
            let mut len = 0;
            let mut src_mac = 0;
            recv_wifi_packet(&mut src_mac, &raw mut out_data, &mut len);
        }
    }
}

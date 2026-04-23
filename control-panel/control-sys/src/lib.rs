
pub type Status = i32;

pub const MAX_LORA_RECV_PACKET_LEN: usize = 64;
pub const MAX_LORA_SEND_PACKET_LEN: usize = 64;

pub const MAX_WIFI_RECV_PACKET_LEN: usize = 1500;
pub const MAX_WIFI_SEND_PACKET_LEN: usize = 1500;

pub const STATUS_SUCCESSFUL: Status = 0;
pub const STATUS_MODULE_DETACHED: Status = -5;

unsafe extern "C" {
    pub safe fn is_lora_module_attached() -> bool;

    pub safe fn initialize_lora_module() -> Status;
    pub safe fn shutdown_lora_module();

    pub safe fn send_lora_packet(data: *const u8, len: usize) -> Status;
    pub safe fn recv_lora_packet(data: *mut [u8; MAX_LORA_RECV_PACKET_LEN], len: *mut usize) -> Status;


    pub safe fn is_wifi_module_attached() -> bool;

    pub safe fn initialize_wifi_module() -> Status;
    pub safe fn shutdown_wifi_module();

    pub safe fn send_wifi_packet(mac_dst: u64, data: *const u8, len: usize) -> Status;
    pub safe fn recv_wifi_packet(mac_src: u64, data: *mut [u8; MAX_LORA_RECV_PACKET_LEN], len: *mut usize) -> Status;
}

#[cfg(test)]
mod tests {

}

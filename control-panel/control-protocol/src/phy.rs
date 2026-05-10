pub type Status = i32;

pub const MAX_LORA_RECV_PACKET_LEN: usize = 64;
pub const MAX_LORA_SEND_PACKET_LEN: usize = 64;

pub const MAX_WIFI_RECV_PACKET_LEN: usize = 255;
pub const MAX_WIFI_SEND_PACKET_LEN: usize = 255;

pub const STATUS_SUCCESSFUL: Status = 0;
pub const STATUS_RECEIVE_TIMEOUT: Status = -3;
pub const STATUS_MODULE_DETACHED: Status = -5;

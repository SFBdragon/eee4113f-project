


pub enum LoRaRequest {
    TurnOnWiFi,
    TurnOffWiFi,
    DataDump
}


impl LoRaRequest {
    pub const TURN_ON_WIFI: u8 = 1;
    pub const TURN_OFF_WIFI: u8 = 2;
    pub const DATA_DUMP: u8 = 3;
    pub const DATA_STREAM: u8 = 4;

    pub fn discriminator(&self) -> u8 {
        match self {
            LoRaResponse::StorageInfo(_) => LORA_EVENT_STORAGE,
        }
    }

    pub fn parse() -> Result<Self, ParseError> {

    }
}


pub struct DumpRequest {
    from_block: u32,
    to_block: u32,
}



pub enum LoRaResponse {
    StorageInfo(StorageInfo),
}


impl LoRaResponse {
    pub const STORAGE: u8 = 1;

    pub fn discriminator(&self) -> u8 {
        match self {
            LoRaResponse::StorageInfo(_) => LORA_EVENT_STORAGE,
        }
    }

    pub fn parse() -> Result<Self, ParseError> {

    }
}

pub struct LoRaResponse {

}

pub const FLAG_WIFI_ON: u8 = 1;
pub const FLAG_WIFI_CONN: u8 = 2;
pub const FLAG_WIFI_STREAMING: u8 = 3;

pub struct Status {
    pub flags: u8,
}

pub struct StorageInfo {
    pub total_blocks: u32,
    pub available_begin: u32,
    pub available_end: u32,
    pub overwritable_end: u32,
}

pub struct GpsInfo {
    pub lat: u32,
    pub lon: u32,
}


pub enum ParseError {

}

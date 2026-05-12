#[macro_export]
macro_rules! read_next {
    ($t:ty, $buf:expr, $pos:expr) => {{
        $buf[*$pos..]
            .first_chunk::<{ size_of::<$t>() }>()
            .map(|chunk| {
                *$pos += chunk.len();
                <$t>::from_le_bytes(*chunk)
            })
            .ok_or(ParseError::NotEnoughBytes)
    }};
}

#[macro_export]
macro_rules! write_next {
    ($v:expr, $buf:expr, $pos:expr) => {{
        let le_bytes = $v.to_le_bytes();
        let r = $buf[*$pos..]
            .get_mut(..le_bytes.len())
            .map(|s| s.copy_from_slice(&le_bytes))
            .ok_or(NotEnoughBytes);
        *$pos += le_bytes.len();
        r
    }};
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    NotEnoughBytes,
    UnknownRequest(u8),
    UnknownStoragePolicy(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotEnoughBytes;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoRaCmd {
    /// Power up WiFi and start pinging.
    EnableWiFi,
    /// Power down WiFi and stop pinging.
    DisableWiFi,
    /// Establish a connection over WiFi and transfer the specified blocks.
    StartDataDump(StartDataDump),
    /// Stop mass-transferring blocks.
    CancelDataDump,
    SetLoRaRecvWindow(LoRaRecvWindow),
    SetOverwritable(Overwritable),
    SetStoragePolicy(StoragePolicy),
}

impl LoRaCmd {
    pub const ENABLE_WIFI: u8 = 1;
    pub const DISABLE_WIFI: u8 = 2;
    // gap
    pub const START_DATA_DUMP: u8 = 5;
    pub const CANCEL_DATA_DUMP: u8 = 6;
    pub const SET_LORA_RECV_WINDOW: u8 = 7;
    pub const SET_OVERWRITABLE: u8 = 8;
    pub const SET_STORAGE_POLICY: u8 = 9;

    pub fn discriminator(&self) -> u8 {
        match self {
            LoRaCmd::EnableWiFi => Self::ENABLE_WIFI,
            LoRaCmd::DisableWiFi => Self::DISABLE_WIFI,
            LoRaCmd::StartDataDump(_) => Self::START_DATA_DUMP,
            LoRaCmd::CancelDataDump => Self::CANCEL_DATA_DUMP,
            LoRaCmd::SetLoRaRecvWindow(_) => Self::SET_LORA_RECV_WINDOW,
            LoRaCmd::SetOverwritable(_) => Self::SET_OVERWRITABLE,
            LoRaCmd::SetStoragePolicy(_) => Self::SET_STORAGE_POLICY,
        }
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let discriminator = read_next!(u8, buf, pos)?;

        match discriminator {
            Self::ENABLE_WIFI => Ok(Self::EnableWiFi),
            Self::DISABLE_WIFI => Ok(Self::DisableWiFi),
            Self::START_DATA_DUMP => StartDataDump::parse(buf, pos).map(Self::StartDataDump),
            Self::CANCEL_DATA_DUMP => Ok(Self::CancelDataDump),
            Self::SET_LORA_RECV_WINDOW => {
                LoRaRecvWindow::parse(buf, pos).map(Self::SetLoRaRecvWindow)
            }
            Self::SET_OVERWRITABLE => Overwritable::parse(buf, pos).map(Self::SetOverwritable),
            Self::SET_STORAGE_POLICY => StoragePolicy::parse(buf, pos).map(Self::SetStoragePolicy),
            u => Err(ParseError::UnknownRequest(u)),
        }
    }

    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.discriminator(), buf, pos)?;

        match self {
            LoRaCmd::StartDataDump(req) => req.serialize(buf, pos),
            LoRaCmd::SetLoRaRecvWindow(req) => req.serialize(buf, pos),
            LoRaCmd::SetOverwritable(req) => req.serialize(buf, pos),
            LoRaCmd::SetStoragePolicy(req) => req.serialize(buf, pos),
            LoRaCmd::EnableWiFi | LoRaCmd::DisableWiFi | LoRaCmd::CancelDataDump => Ok(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StartDataDump {
    pub from_block: u32,
    pub to_block: u32,
}

impl StartDataDump {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.from_block, buf, pos)?;
        write_next!(self.to_block, buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let from_block = read_next!(u32, buf, pos)?;
        let to_block = read_next!(u32, buf, pos)?;

        Ok(Self {
            from_block,
            to_block,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overwritable {
    pub up_to: u32,
}

impl Overwritable {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.up_to, buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let up_to = read_next!(u32, buf, pos)?;

        Ok(Self { up_to })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StoragePolicy {
    Overwrite = 1u8,
    Preserve = 2u8,
    Readonly = 3u8,
}

impl TryFrom<u8> for StoragePolicy {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(StoragePolicy::Overwrite),
            2 => Ok(StoragePolicy::Preserve),
            3 => Ok(StoragePolicy::Readonly),
            u => Err(u),
        }
    }
}

impl StoragePolicy {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(*self as u8, buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let policy = read_next!(u8, buf, pos)?;
        Self::try_from(policy).map_err(ParseError::UnknownStoragePolicy)
    }

    pub fn to_str(self) -> &'static str {
        match self {
            Self::Overwrite => "Overwrite",
            Self::Preserve => "Preserve",
            Self::Readonly => "Readonly",
        }
    }

    pub fn from_str(text: &str) -> Self {
        match text {
            "Overwrite" => Self::Overwrite,
            "Preserve" => Self::Preserve,
            "Readonly" => Self::Readonly,
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoRaRecvWindow {
    pub on_period: u16,
    pub total_period: u16,
}

impl LoRaRecvWindow {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.on_period, buf, pos)?;
        write_next!(self.total_period, buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let on_period = read_next!(u16, buf, pos)?;
        let total_period = read_next!(u16, buf, pos)?;

        Ok(Self {
            on_period,
            total_period,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoRaModuleState {
    pub status_flags: u8,                 // 1B
    pub storage_policy: StoragePolicy,    // 1B
    pub storage_info: StorageInfo,        // 16B
    pub lora_recv_window: LoRaRecvWindow, // 4B
    pub gps_info: GpsInfo,                // 8B
}

impl LoRaModuleState {
    pub const STATUS_WIFI_ON: u8 = 1 << 0;
    // gap
    pub const STATUS_WIFI_DUMPING: u8 = 1 << 3;

    pub const STATUS_STORAGE_FULL: u8 = 1 << 4;
}

impl LoRaModuleState {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.status_flags, buf, pos)?;
        self.storage_policy.serialize(buf, pos)?;
        self.storage_info.serialize(buf, pos)?;
        self.lora_recv_window.serialize(buf, pos)?;
        self.gps_info.serialize(buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let status_flags = read_next!(u8, buf, pos)?;
        let storage_policy = StoragePolicy::parse(buf, pos)?;
        let storage_info = StorageInfo::parse(buf, pos)?;
        let lora_recv_window = LoRaRecvWindow::parse(buf, pos)?;
        let gps_info = GpsInfo::parse(buf, pos)?;

        Ok(Self {
            status_flags,
            storage_policy,
            storage_info,
            lora_recv_window,
            gps_info,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageInfo {
    pub total_blocks: u32,
    pub available_begin: u32,
    pub available_end: u32,
    pub overwritable_end: u32,
    pub generation: u16,
}

impl StorageInfo {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.total_blocks, buf, pos)?;
        write_next!(self.available_begin, buf, pos)?;
        write_next!(self.available_end, buf, pos)?;
        write_next!(self.overwritable_end, buf, pos)?;
        write_next!(self.generation, buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let total_blocks = read_next!(u32, buf, pos)?;
        let available_begin = read_next!(u32, buf, pos)?;
        let available_end = read_next!(u32, buf, pos)?;
        let overwritable_end = read_next!(u32, buf, pos)?;
        let generation = read_next!(u16, buf, pos)?;

        Ok(Self {
            total_blocks,
            available_begin,
            available_end,
            overwritable_end,
            generation,
        })
    }

    pub fn coherent(&self) -> bool {
        if self.available_end >= self.available_begin {
            self.available_begin <= self.overwritable_end
                && self.overwritable_end <= self.available_end
        } else {
            self.overwritable_end <= self.available_begin
                || self.available_end <= self.overwritable_end
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsInfo {
    pub lat: u32,
    pub lon: u32,
}

impl GpsInfo {
    pub fn serialize(&self, buf: &mut [u8], pos: &mut usize) -> Result<(), NotEnoughBytes> {
        write_next!(self.lat, buf, pos)?;
        write_next!(self.lon, buf, pos)
    }

    pub fn parse(buf: &[u8], pos: &mut usize) -> Result<Self, ParseError> {
        let lat = read_next!(u32, buf, pos)?;
        let lon = read_next!(u32, buf, pos)?;

        Ok(Self { lat, lon })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusError {
    ModuleDetached,
    ReceiveTimeout,
    Unknown(i32),
}

pub fn from_ffi(s: i32) -> Result<(), StatusError> {
    use control_protocol::phy::*;

    match s {
        STATUS_SUCCESSFUL => Ok(()),
        STATUS_MODULE_DETACHED => Err(StatusError::ModuleDetached),
        STATUS_RECEIVE_TIMEOUT => Err(StatusError::ReceiveTimeout),
        unknown => Err(StatusError::Unknown(unknown)),
    }
}

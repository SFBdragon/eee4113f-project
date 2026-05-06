
pub enum StatusError {
    ModuleDetached,
    ReceiveTimeout,
    Unknown(i32),
}


pub fn from_ffi(s: i32) -> Result<(), StatusError> {
    match s {
        control_sys::STATUS_SUCCESSFUL => Ok(()),
        control_sys::STATUS_MODULE_DETACHED => Err(StatusError::ModuleDetached),
        control_sys::STATUS_RECEIVE_TIMEOUT => Err(StatusError::ReceiveTimeout),
        unknown => Err(StatusError::Unknown(unknown))
    }
}

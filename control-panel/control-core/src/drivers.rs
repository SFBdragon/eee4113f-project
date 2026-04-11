
pub enum StatusError {
    ModuleDetached,
    Unknown(i32),
}


pub fn from_ffi(s: i32) -> Result<(), StatusError> {
    match s {
        control_sys::STATUS_SUCCESSFUL => Ok(()),
        control_sys::STATUS_MODULE_DETACHED => Err(StatusError::ModuleDetached),
        unknown => Err(StatusError::Unknown(unknown))
    }
}

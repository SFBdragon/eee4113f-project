#[derive(Debug, Clone, Copy)]
pub struct Disconnected;

impl<T> From<crossbeam_channel::SendError<T>> for Disconnected {
    fn from(_: crossbeam_channel::SendError<T>) -> Self {
        Self
    }
}

impl From<crossbeam_channel::RecvError> for Disconnected {
    fn from(_: crossbeam_channel::RecvError) -> Self {
        Self
    }
}

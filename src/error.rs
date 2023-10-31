#[derive(Debug, Clone)]
pub enum Error<E> {
    I2c(E),
    InvalidData,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Self::I2c(error)
    }
}

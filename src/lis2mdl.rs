use crate::error::Error;
use crate::i2c_abstraction::{ConfigurableRegister, Offset, Output};
use crate::register_map::RegisterMap;
use crate::{IntSourceReg, ModeOfOperation, Status};
use core::marker::PhantomData;
#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayUs, i2c::I2c};

#[derive(Debug)]
pub struct Lis2mdl<I, D> {
    i2c: I,
    _delay: PhantomData<D>,
}

#[cfg(feature = "async")]
impl<I, E, D> Lis2mdl<I, D>
where
    I: I2c<Error = E>,
    D: DelayUs,
{
    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            _delay: PhantomData,
        }
    }

    pub async fn who_am_i(&mut self) -> Result<u8, Error<E>> {
        let write = [RegisterMap::WhoAmI.addr()];
        let mut read = [0];
        self.i2c
            .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
            .await?;
        Ok(*read.first().ok_or(Error::InvalidData)?)
    }

    pub async fn get_hard_iron_offset(&mut self) -> Result<Offset, Error<E>> {
        let mut write: [u8; 1];
        let mut read: [u8; 1] = [0];
        let mut lsb_x: u8 = 0;
        let mut msb_x: u8 = 0;
        let mut lsb_y: u8 = 0;
        let mut msb_y: u8 = 0;
        let mut lsb_z: u8 = 0;
        let mut msb_z: u8 = 0;

        for (addr, offset) in [
            (RegisterMap::OffsetXRegL, &mut lsb_x),
            (RegisterMap::OffsetXRegH, &mut msb_x),
            (RegisterMap::OffsetYRegL, &mut lsb_y),
            (RegisterMap::OffsetYRegH, &mut msb_y),
            (RegisterMap::OffsetZRegL, &mut lsb_z),
            (RegisterMap::OffsetZRegH, &mut msb_z),
        ] {
            write = [addr.addr()];
            self.i2c
                .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
                .await?;
            *offset = *read.first().ok_or(Error::InvalidData)?;
        }

        Ok(Offset::parse(lsb_x, msb_x, lsb_y, msb_y, lsb_z, msb_z))
    }

    pub async fn get_output(&mut self) -> Result<Output, Error<E>> {
        let mut write: [u8; 1];
        let mut read: [u8; 1] = [0];
        let mut lsb_x: u8 = 0;
        let mut msb_x: u8 = 0;
        let mut lsb_y: u8 = 0;
        let mut msb_y: u8 = 0;
        let mut lsb_z: u8 = 0;
        let mut msb_z: u8 = 0;

        for (addr, offset) in [
            (RegisterMap::OutxLReg, &mut lsb_x),
            (RegisterMap::OutxHReg, &mut msb_x),
            (RegisterMap::OutyLReg, &mut lsb_y),
            (RegisterMap::OutyHReg, &mut msb_y),
            (RegisterMap::OutzLReg, &mut lsb_z),
            (RegisterMap::OutzHReg, &mut msb_z),
        ] {
            write = [addr.addr()];
            self.i2c
                .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
                .await?;
            *offset = *read.first().ok_or(Error::InvalidData)?;
        }

        Ok(Output::parse(lsb_x, msb_x, lsb_y, msb_y, lsb_z, msb_z))
    }

    /// Returns temperature from the internal temperature sensor in degC. The
    /// output value is expressed as a signed 16-bit byte in 2’s complement. The four most
    /// significant bits contain a copy of the sign bit.
    ///
    /// The nominal sensitivity is 8 LSB/°C.
    pub async fn get_temperature(&mut self) -> Result<f32, Error<E>> {
        let mut write: [u8; 1];
        let mut read: [u8; 1] = [0];
        let mut lsb: u8 = 0;
        let mut msb: u8 = 0;

        for (addr, offset) in [
            (RegisterMap::TempOutLReg, &mut lsb),
            (RegisterMap::TempOutHReg, &mut msb),
        ] {
            write = [addr.addr()];
            self.i2c
                .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
                .await?;
            *offset = *read.first().ok_or(Error::InvalidData)?;
        }

        Ok((((msb as i16) << 8) | (lsb as i16)) as f32 * 0.125)
    }

    pub async fn read_cfg_reg<R: ConfigurableRegister>(&mut self) -> Result<R, Error<E>> {
        let write: [u8; 1] = [<R as ConfigurableRegister>::register_addr()];
        let mut read: [u8; 1] = [0];
        self.i2c
            .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
            .await?;

        Ok(<R as ConfigurableRegister>::parse(
            *read.first().ok_or(Error::InvalidData)?,
        ))
    }

    pub async fn write_cfg_reg<R: ConfigurableRegister>(
        &mut self,
        cfg_reg: R,
    ) -> Result<(), Error<E>> {
        let write: [u8; 2] = [
            <R as ConfigurableRegister>::register_addr(),
            cfg_reg.to_register(),
        ];
        self.i2c.write(crate::LIS2MDL_I2C_ADDR, &write).await?;
        Ok(())
    }

    pub async fn update_cfg_reg<R: ConfigurableRegister>(
        &mut self,
        f: impl FnOnce(&mut R),
    ) -> Result<(), Error<E>> {
        let mut cfg_reg = self.read_cfg_reg::<R>().await?;
        f(&mut cfg_reg);
        self.write_cfg_reg(cfg_reg).await?;
        Ok(())
    }

    pub async fn get_interrupt_level(&mut self) -> Result<u16, Error<E>> {
        let mut write: [u8; 1];
        let mut read: [u8; 1] = [0];
        let mut lsb: u8 = 0;
        let mut msb: u8 = 0;

        for (addr, offset) in [
            (RegisterMap::IntThsLReg, &mut lsb),
            (RegisterMap::IntThsHReg, &mut msb),
        ] {
            write = [addr.addr()];
            self.i2c
                .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
                .await?;
            *offset = *read.first().ok_or(Error::InvalidData)?;
        }

        Ok(((msb as u16) << 8) | (lsb as u16))
    }

    pub async fn write_interrupt_level(&mut self, int: u16) -> Result<(), Error<E>> {
        let write: [u8; 2] = [RegisterMap::IntThsLReg.addr(), int as u8];
        self.i2c.write(crate::LIS2MDL_I2C_ADDR, &write).await?;
        let write: [u8; 2] = [RegisterMap::IntThsHReg.addr(), (int >> 8) as u8];
        self.i2c.write(crate::LIS2MDL_I2C_ADDR, &write).await?;
        Ok(())
    }

    pub async fn get_interrupt_status(&mut self) -> Result<IntSourceReg, Error<E>> {
        let write: [u8; 1] = [RegisterMap::IntSourceReg.addr()];
        let mut read: [u8; 1] = [0];
        self.i2c
            .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
            .await?;
        Ok(IntSourceReg::parse(
            *read.first().ok_or(Error::InvalidData)?,
        ))
    }

    pub async fn get_status(&mut self) -> Result<Status, Error<E>> {
        let write: [u8; 1] = [RegisterMap::StatusReg.addr()];
        let mut read: [u8; 1] = [0];
        self.i2c
            .write_read(crate::LIS2MDL_I2C_ADDR, &write, &mut read)
            .await?;
        Ok(Status::parse(*read.first().ok_or(Error::InvalidData)?))
    }

    pub async fn enable_continuous_conversion(&mut self) -> Result<(), Error<E>> {
        self.update_cfg_reg::<crate::CfgRegA>(|cfg_reg| {
            cfg_reg.mode = ModeOfOperation::Continuous;
        })
        .await?;
        Ok(())
    }

    pub async fn take_single_measurement(&mut self) -> Result<(), Error<E>> {
        self.update_cfg_reg::<crate::CfgRegA>(|cfg_reg| {
            cfg_reg.mode = ModeOfOperation::Single;
        })
        .await?;
        Ok(())
    }

    pub async fn enter_idle_mode(&mut self) -> Result<(), Error<E>> {
        self.update_cfg_reg::<crate::CfgRegA>(|cfg_reg| {
            cfg_reg.mode = ModeOfOperation::Idle;
        })
        .await?;
        Ok(())
    }

    pub async fn set_update_rate(&mut self, rate: crate::OutputDataRate) -> Result<(), Error<E>> {
        self.update_cfg_reg::<crate::CfgRegA>(|cfg_reg| {
            cfg_reg.data_rate = rate;
        })
        .await?;
        Ok(())
    }
}

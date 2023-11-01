#![no_std]

mod error;
mod i2c_abstraction;
mod lis2mdl;
mod register_map;

pub use error::Error;
pub use i2c_abstraction::{
    CfgRegA, CfgRegB, CfgRegC, ConfigurableRegister, IntCtrlReg, IntSourceReg, IntThs,
    ModeOfOperation, Offset, OffsetX, OffsetY, OffsetZ, OutX, OutY, OutZ, Output, OutputDataRate,
    Status,
};
pub use lis2mdl::Lis2mdl;

pub const LIS2MDL_I2C_ADDR: u8 = 0x1E;

/// 1.5 milligauss/LSB * 100 nanotesla/milligauss
const MAG_SCALE: i32 = 150;

//! Register mapping for internal use

#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum RegisterMap {
    /// Upper byte of `OFFSET_X_REG`
    ///
    /// Represent X hard-iron offset in order to compensate environmental effects
    /// (data in two’s complement). These values act on the magnetic output data value
    /// in order to delete the environmental offset.
    ///
    /// Read/write
    OffsetXRegL = 0x45,
    /// Lower byte of `OFFSET_X_REG`
    ///
    /// Represent X hard-iron offset in order to compensate environmental effects
    /// (data in two’s complement). These values act on the magnetic output data value
    /// in order to delete the environmental offset.
    ///
    /// Read/write
    OffsetXRegH = 0x46,

    /// Upper byte of `OFFSET_Y_REG`
    ///
    /// Represent Y hard-iron offset in order to compensate environmental effects
    /// (data in two’s complement). These values act on the magnetic output data value
    /// in order to delete the environmental offset.
    ///
    /// Read/write
    OffsetYRegL = 0x47,
    /// Lower byte of `OFFSET_Y_REG`
    ///
    /// Represent Y hard-iron offset in order to compensate environmental effects
    /// (data in two’s complement). These values act on the magnetic output data value
    /// in order to delete the environmental offset.
    ///
    /// Read/write
    OffsetYRegH = 0x48,

    /// Upper byte of `OFFSET_Z_REG`
    ///
    /// Represent Z hard-iron offset in order to compensate environmental effects
    /// (data in two’s complement). These values act on the magnetic output data value
    /// in order to delete the environmental offset.
    ///
    /// Read/write
    OffsetZRegL = 0x49,
    /// Lower byte of `OFFSET_Z_REG`
    ///
    /// Represent Z hard-iron offset in order to compensate environmental effects
    /// (data in two’s complement). These values act on the magnetic output data value
    /// in order to delete the environmental offset.
    ///
    /// Read/write
    OffsetZRegH = 0x4A,

    /// `WhoAmI` register
    ///
    /// Identifies device.
    /// Expected value is `0b01000000`
    ///
    /// Read-only
    WhoAmI = 0x4F,

    /// Byte 1 of `CFG_REG`
    ///
    /// Contains `COMP_TEMP_EN`, `REBOOT`, `SOFT_RST`, `LP`, `ODR1`, `ODR0`, `MD1`, `MD0`
    ///
    /// Read/write
    CfgRegA = 0x60,

    /// Byte 2 of `CFG_REG`
    ///
    /// Contains three unused bits, followed by `OFF_CANC_ONE_SHOT`,
    /// `INT_on_DataOFF`, `Set_FREQ`, `OFF_CANC`, `LPF`
    ///
    /// Read/write
    CfgRegB = 0x61,

    /// Byte 3 of `CFG_REG`
    ///
    /// Contains one unused bit, followed by `INT_on_PIN`, `I2C_DIS`, `BDU`, `BLE`,
    /// `4WSPI`, `Self_test`, `DRDY_on_PIN`
    ///
    /// Read/write
    CfgRegC = 0x62,

    /// `IntCrtlReg` register
    ///
    /// Contains `XIEN`, `YIEN`, `ZIEN`, two unused bits, `IEA`, `IEL`, `IEN`
    ///
    /// Read/write
    IntCrtlReg = 0x63,

    /// `IntSourceReg` register
    ///
    /// Contains `P_TH_S_X`, `P_TH_S_Y`, `P_TH_S_Z`, `N_TH_S_X`, `N_TH_S_Y`, `N_TH_S_Z`,
    /// `MROI`, `INT`
    ///
    /// Read-only
    IntSourceReg = 0x64,

    /// `IntThsLReg` register
    ///
    /// Contains LSB of `INT_THS_REG`
    ///
    /// Read/write
    IntThsLReg = 0x65,

    /// `IntThsHReg` register
    ///
    /// Contains MSB of `INT_THS_REG`
    ///
    /// Read/write
    IntThsHReg = 0x66,

    /// `StatusReg` register
    ///
    /// Contains `Zyxor`, `zor`, `yor`, `xor`, `Zyxda`, `zda`, `yda`, `xda`
    ///
    /// Read-only
    StatusReg = 0x67,

    /// `OutxLReg` register
    ///
    /// Contains LSB of `OUTX_REG`
    ///
    /// The output data represents the raw magnetic data only if OFFSET_X_REG is equal to zero,
    /// otherwise hard-iron calibration is included
    ///
    /// Read-only
    OutxLReg = 0x68,

    /// `OutxHReg` register
    ///
    /// Contains MSB of `OUTX_REG`
    ///
    /// The output data represents the raw magnetic data only if OFFSET_X_REG is equal to zero,
    /// otherwise hard-iron calibration is included
    ///
    /// Read-only
    OutxHReg = 0x69,

    /// `OutyLReg` register
    ///
    /// Contains LSB of `OUTY_REG`
    ///
    /// The output data represents the raw magnetic data only if OFFSET_Y_REG is equal to zero,
    /// otherwise hard-iron calibration is included
    ///
    /// Read-only
    OutyLReg = 0x6A,

    /// `OutyHReg` register
    ///
    /// Contains MSB of `OUTY_REG`
    ///
    /// The output data represents the raw magnetic data only if OFFSET_Y_REG is equal to zero,
    /// otherwise hard-iron calibration is included
    ///
    /// Read-only
    OutyHReg = 0x6B,

    /// `OutzLReg` register
    ///
    /// Contains LSB of `OUTZ_REG`
    ///
    /// The output data represents the raw magnetic data only if OFFSET_Z_REG is equal to zero,
    /// otherwise hard-iron calibration is included
    ///
    /// Read-only
    OutzLReg = 0x6C,

    /// `OutzHReg` register
    ///
    /// Contains MSB of `OUTZ_REG`
    ///
    /// The output data represents the raw magnetic data only if OFFSET_Z_REG is equal to zero,
    /// otherwise hard-iron calibration is included
    ///
    /// Read-only
    OutzHReg = 0x6D,

    /// `TempOutLReg` register
    ///
    /// Contains LSB of `TEMP_OUT_REG`
    ///
    /// These registers contain temperature values from the internal temperature sensor. The
    /// output value is expressed as a signed 16-bit byte in 2’s complement. The four most
    /// significant bits contain a copy of the sign bit.
    ///
    /// The nominal sensitivity is 8 LSB/°C.
    ///
    /// Read-only
    TempOutLReg = 0x6E,

    /// `TempOutHReg` register
    ///
    /// Contains MSB of `TEMP_OUT_REG`
    ///
    /// These registers contain temperature values from the internal temperature sensor. The
    /// output value is expressed as a signed 16-bit byte in 2’s complement. The four most
    /// significant bits contain a copy of the sign bit.
    ///
    /// The nominal sensitivity is 8 LSB/°C.
    ///
    /// Read-only
    TempOutHReg = 0x6F,
}

impl RegisterMap {
    pub const fn addr(&self) -> u8 {
        *self as u8
    }
}

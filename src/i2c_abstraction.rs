//! Abstracts away the parsing of the I2C data, just needs to be fed the raw data from the
//! registers.

use crate::register_map::RegisterMap;

/// Raw data from register `OFFSET_X_REG`
pub struct OffsetX(pub i16);
impl OffsetX {
    fn parse(lsb: u8, msb: u8) -> Self {
        Self(i16::from_le_bytes([lsb, msb]))
    }
}

/// Raw data from register `OFFSET_Y_REG`
pub struct OffsetY(pub i16);
impl OffsetY {
    fn parse(lsb: u8, msb: u8) -> Self {
        Self(i16::from_le_bytes([lsb, msb]))
    }
}
/// Raw data from register `OFFSET_Z_REG`
pub struct OffsetZ(pub i16);
impl OffsetZ {
    fn parse(lsb: u8, msb: u8) -> Self {
        Self(i16::from_le_bytes([lsb, msb]))
    }
}

pub struct Offset {
    pub x: OffsetX,
    pub y: OffsetY,
    pub z: OffsetZ,
}
impl Offset {
    pub(crate) fn parse(lsb_x: u8, msb_x: u8, lsb_y: u8, msb_y: u8, lsb_z: u8, msb_z: u8) -> Self {
        Self {
            x: OffsetX::parse(lsb_x, msb_x),
            y: OffsetY::parse(lsb_y, msb_y),
            z: OffsetZ::parse(lsb_z, msb_z),
        }
    }
}

/// Raw data from register `CfgRegA`
#[derive(Debug, PartialEq)]
pub struct CfgRegA {
    /// Enables the magnetometer temperature compensation. Default value: false
    ///
    /// Bit 7
    pub comp_temp_en: bool,
    /// Reboot magnetometer memory content. Default value: false
    ///
    /// Bit 6
    pub reboot: bool,
    /// When this bit is set, the configuration registers and user registers are reset.
    /// Flash registers keep their values.
    ///
    /// Bit 5
    pub soft_rst: bool,
    /// Enables low-power mode. Default value: false
    ///
    /// Bit 4
    pub lp: bool,
    /// Sets the data rate. Default value: 10Hz
    ///
    /// Bits 3:2
    pub data_rate: OutputDataRate,
    /// Operation mode. Default value: idle.
    ///
    /// Bits 1:0
    pub mode: ModeOfOperation,
}

impl CfgRegA {
    const COMP_TEMP_EN: u8 = 0b1000_0000;
    const REBOOT: u8 = 0b0100_0000;
    const SOFT_RST: u8 = 0b0010_0000;
    const LP: u8 = 0b0001_0000;
}

impl ConfigurableRegister for CfgRegA {
    fn parse(register: u8) -> Self {
        let comp_temp_en = (register & Self::COMP_TEMP_EN) != 0;
        let reboot = (register & Self::REBOOT) != 0;
        let soft_rst = (register & Self::SOFT_RST) != 0;
        let lp = (register & Self::LP) != 0;
        let data_rate = match (register & 0b0000_1100) >> 2 {
            0b00 => OutputDataRate::_10Hz,
            0b01 => OutputDataRate::_20Hz,
            0b10 => OutputDataRate::_50Hz,
            0b11 => OutputDataRate::_100Hz,
            _ => unreachable!(),
        };
        let mode = match register & 0b0000_0011 {
            0b00 => ModeOfOperation::Continuous,
            0b01 => ModeOfOperation::Single,
            0b10 => ModeOfOperation::Idle,
            0b11 => ModeOfOperation::Idle,
            _ => unreachable!(),
        };

        Self {
            comp_temp_en,
            reboot,
            soft_rst,
            lp,
            data_rate,
            mode,
        }
    }

    fn to_register(&self) -> u8 {
        let mut register = 0b0000_0000;
        if self.comp_temp_en {
            register |= Self::COMP_TEMP_EN;
        }
        if self.reboot {
            register |= Self::REBOOT;
        }
        if self.soft_rst {
            register |= Self::SOFT_RST;
        }
        if self.lp {
            register |= Self::LP;
        }
        register |= (self.data_rate as u8) << 2;
        register |= self.mode as u8;
        register
    }

    fn register_addr() -> u8 {
        RegisterMap::CfgRegA.addr()
    }
}

/// Raw data from `CfgRegB`
#[derive(Debug, PartialEq)]
pub struct CfgRegB {
    // Upper 3 bits are unused
    /// Enables offset cancellation in single measurement mode.
    pub off_canc_one_shot: bool,
    /// If true, the interrupt block recognition checks data after the hard-iron correction to
    /// discover the interrupt
    pub int_on_data_off: bool,
    /// Sets the frequency of the set pulse. Default: false.
    /// * false: set pulse is released every 63 ODR;
    /// * true: set pulse is released only at power-on after PD condition
    pub set_freq: bool,
    /// Enables offset cancellation in single measurement mode.
    pub off_canc: bool,
    /// Enables low-pass filter. Default value: false
    ///
    /// * false: low-pass filter is disabled; bandwidth is one half of the ODR
    /// * true: low-pass filter is enabled; bandwidth is one quarter of the ODR
    pub lpf: bool,
}

impl CfgRegB {
    const OFF_CANC_ONE_SHOT: u8 = 0b0001_0000;
    const INT_ON_DATA_OFF: u8 = 0b0000_1000;
    const SET_FREQ: u8 = 0b0000_0100;
    const OFF_CANC: u8 = 0b0000_0010;
    const LPF: u8 = 0b0000_0001;
}

impl ConfigurableRegister for CfgRegB {
    fn parse(register: u8) -> Self {
        let off_canc_one_shot = (register & Self::OFF_CANC_ONE_SHOT) != 0;
        let int_on_data_off = (register & Self::INT_ON_DATA_OFF) != 0;
        let set_freq = (register & Self::SET_FREQ) != 0;
        let off_canc = (register & Self::OFF_CANC) != 0;
        let lpf = (register & Self::LPF) != 0;

        Self {
            off_canc_one_shot,
            int_on_data_off,
            set_freq,
            off_canc,
            lpf,
        }
    }

    fn to_register(&self) -> u8 {
        let mut register = 0b0000_0000;
        if self.off_canc_one_shot {
            register |= Self::OFF_CANC_ONE_SHOT;
        }
        if self.int_on_data_off {
            register |= Self::INT_ON_DATA_OFF;
        }
        if self.set_freq {
            register |= Self::SET_FREQ;
        }
        if self.off_canc {
            register |= Self::OFF_CANC;
        }
        if self.lpf {
            register |= Self::LPF;
        }
        register
    }

    fn register_addr() -> u8 {
        RegisterMap::CfgRegB.addr()
    }
}

/// Raw data from `CfgRegC`
#[derive(Debug, PartialEq)]
pub struct CfgRegC {
    // Upper bit is unused
    /// If true, the INTERRUPT signal (INT bit in INT_SOURCE) is driven on the INT/DRDY pin.
    /// The pin is configured as push-pull output.
    ///
    /// Default value: false
    pub int_on_pin: bool,
    /// If true, the I2C bus is disabled. Only SPI is functional.
    ///
    /// Defaults to false.
    ///
    /// You probably don't want to use this.
    pub i2c_dis: bool,
    /// If enabled, reading of incorrect data is avoided when the user reads asynchronously.
    /// In fact, if the read request arrives during an update of the output data, a
    /// latch is possible, reading incoherent high and low parts of the same register. Only
    /// one part is updated and the other one remains old.
    ///
    /// Default value: true
    pub bdu: bool,
    /// If true, the low and high parts of the data are swapped. In other words, the
    /// MSB is written into the LSB register and the LSB is written into the MSB register.
    ///
    /// Default value: false
    ///
    /// You probably don't want to use this.
    pub ble: bool,
    /// If true, the four-wire SPI interface is enabled.
    /// Enables the SDO line on pin 7.
    ///
    /// Default value: false
    pub four_wire_spi: bool,
    /// If true, the self-test is enabled.
    pub self_test: bool,
    /// If true, the DRDY signal (Zxyda bit in StatusReg) is driven on the INT/DRDY pin.
    /// The pin is configured as push-pull output.
    ///
    /// Default value: false
    pub drdy_on_pin: bool,
}

impl CfgRegC {
    const INT_ON_PIN: u8 = 0b0100_0000;
    const I2C_DIS: u8 = 0b0010_0000;
    const BDU: u8 = 0b0001_0000;
    const BLE: u8 = 0b0000_1000;
    const FOUR_WIRE_SPI: u8 = 0b0000_0100;
    const SELF_TEST: u8 = 0b0000_0010;
    const DRDY_ON_PIN: u8 = 0b0000_0001;
}

impl ConfigurableRegister for CfgRegC {
    fn parse(register: u8) -> Self {
        let int_on_pin = (register & 0b0100_0000) != 0;
        let i2c_dis = (register & 0b0010_0000) != 0;
        let bdu = (register & 0b0001_0000) != 0;
        let ble = (register & 0b0000_1000) != 0;
        let four_wire_spi = (register & 0b0000_0100) != 0;
        let self_test = (register & 0b0000_0010) != 0;
        let drdy_on_pin = (register & 0b0000_0001) != 0;

        Self {
            int_on_pin,
            i2c_dis,
            bdu,
            ble,
            four_wire_spi,
            self_test,
            drdy_on_pin,
        }
    }

    fn to_register(&self) -> u8 {
        let mut register = 0b0000_0000;
        if self.int_on_pin {
            register |= Self::INT_ON_PIN;
        }
        if self.i2c_dis {
            register |= Self::I2C_DIS;
        }
        if self.bdu {
            register |= Self::BDU;
        }
        if self.ble {
            register |= Self::BLE;
        }
        if self.four_wire_spi {
            register |= Self::FOUR_WIRE_SPI;
        }
        if self.self_test {
            register |= Self::SELF_TEST;
        }
        if self.drdy_on_pin {
            register |= Self::DRDY_ON_PIN;
        }
        register
    }

    fn register_addr() -> u8 {
        RegisterMap::CfgRegC.addr()
    }
}

/// Raw data from `INT_CTRL_REG`
#[derive(Debug, PartialEq)]
pub struct IntCtrlReg {
    /// Enables interrupt detection for the X-axis. Default value: true
    pub x_interrupt_enable: bool,
    /// Enables interrupt detection for the Y-axis. Default value: true
    pub y_interrupt_enable: bool,
    /// Enables interrupt detection for the Z-axis. Default value: true
    pub z_interrupt_enable: bool,

    // two unused bits
    // these should be set to 0 when writing to the register
    // may want to double-check on read
    /// Polarity of the input signal. Default value: true
    ///
    /// * false: active low (interrupt pin is low when interrupt is active)
    /// * true: active high (interrupt pin is high when interrupt is active)
    pub interrupt_active_high: bool,
    /// Controls whether the INT pin is latched or pulsed. Default value: true
    ///
    /// * false: interrupt pulsed
    /// * true: interrupt latched (remains active until IntSourceReg is read)
    pub interrupt_latched: bool,
    /// Enables interrupt generation. Default value: false
    pub interrupt_enabled: bool,
}

impl IntCtrlReg {
    const XIEN: u8 = 0b1000_0000;
    const YIEN: u8 = 0b0100_0000;
    const ZIEN: u8 = 0b0010_0000;
    const IEA: u8 = 0b0000_0100;
    const IEL: u8 = 0b0000_0010;
    const IEN: u8 = 0b0000_0001;
}

impl ConfigurableRegister for IntCtrlReg {
    fn parse(register: u8) -> Self {
        let xien = (register & Self::XIEN) != 0;
        let yien = (register & Self::YIEN) != 0;
        let zien = (register & Self::ZIEN) != 0;
        let iea = (register & Self::IEA) != 0;
        let iel = (register & Self::IEL) != 0;
        let ien = (register & Self::IEN) != 0;

        Self {
            x_interrupt_enable: xien,
            y_interrupt_enable: yien,
            z_interrupt_enable: zien,
            interrupt_active_high: iea,
            interrupt_latched: iel,
            interrupt_enabled: ien,
        }
    }

    fn to_register(&self) -> u8 {
        let mut register = 0b0000_0000;
        if self.x_interrupt_enable {
            register |= Self::XIEN;
        }
        if self.y_interrupt_enable {
            register |= Self::YIEN;
        }
        if self.z_interrupt_enable {
            register |= Self::ZIEN;
        }
        if self.interrupt_active_high {
            register |= Self::IEA;
        }
        if self.interrupt_latched {
            register |= Self::IEL;
        }
        if self.interrupt_enabled {
            register |= Self::IEN;
        }
        register
    }

    fn register_addr() -> u8 {
        RegisterMap::IntCrtlReg.addr()
    }
}

pub trait ConfigurableRegister: sealed::Sealed {
    fn parse(register: u8) -> Self;
    fn to_register(&self) -> u8;
    fn register_addr() -> u8;
}

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::CfgRegA {}
    impl Sealed for super::CfgRegB {}
    impl Sealed for super::CfgRegC {}
    impl Sealed for super::IntCtrlReg {}
}

/// Raw data from `IntSourceReg`
#[derive(Debug, PartialEq)]
pub struct IntSourceReg {
    /// X-axis value exceeds threshold on positive side. Default value: false
    pub p_th_s_x: bool,
    /// Y-axis value exceeds threshold on positive side. Default value: false
    pub p_th_s_y: bool,
    /// Z-axis value exceeds threshold on positive side. Default value: false
    pub p_th_s_z: bool,
    /// X-axis value exceeds threshold on negative side. Default value: false
    pub n_th_s_x: bool,
    /// Y-axis value exceeds threshold on negative side. Default value: false
    pub n_th_s_y: bool,
    /// Z-axis value exceeds threshold on negative side. Default value: false
    pub n_th_s_z: bool,
    /// Always true. Flag is reset by reading this register.
    pub mroi: bool,
    /// Interrupt active. Default value: false
    pub int: bool,
}

impl IntSourceReg {
    pub fn parse(register: u8) -> Self {
        let p_th_s_x = (register & 0b1000_0000) != 0;
        let p_th_s_y = (register & 0b0100_0000) != 0;
        let p_th_s_z = (register & 0b0010_0000) != 0;
        let n_th_s_x = (register & 0b0001_0000) != 0;
        let n_th_s_y = (register & 0b0000_1000) != 0;
        let n_th_s_z = (register & 0b0000_0100) != 0;
        let mroi = (register & 0b0000_0010) != 0;
        let int = (register & 0b0000_0001) != 0;

        Self {
            p_th_s_x,
            p_th_s_y,
            p_th_s_z,
            n_th_s_x,
            n_th_s_y,
            n_th_s_z,
            mroi,
            int,
        }
    }
}

/// Raw data from `INT_THS_REG`
///
/// These registers set the threshold value for the output to generate the interrupt (INT bit in
/// IntSourceReg (64h)). This threshold is common to all three (axes) output values and
/// is unsigned unipolar. The threshold value is correlated to the current gain, and it is unsigned
/// because the threshold is considered as an absolute value, but crossing the threshold is
/// detected for both positive and negative sides.
pub type IntThs = u16;

/// Raw data from `StatusReg`
#[derive(Debug, PartialEq)]
pub struct Status {
    /// All three axes have new data which overwrote the previous data that was never read.
    pub zyxor: bool,
    /// Z-axis has new data which overwrote the previous data that was never read.
    pub zor: bool,
    /// Y-axis has new data which overwrote the previous data that was never read.
    pub yor: bool,
    /// X-axis has new data which overwrote the previous data that was never read.
    pub xor: bool,
    /// All three axes have new data.
    pub zyxda: bool,
    /// Z-axis has new data.
    pub zda: bool,
    /// Y-axis has new data.
    pub yda: bool,
    /// X-axis has new data.
    pub xda: bool,
}

impl Status {
    pub fn parse(register: u8) -> Self {
        let zyxor = (register & 0b1000_0000) != 0;
        let zor = (register & 0b0100_0000) != 0;
        let yor = (register & 0b0010_0000) != 0;
        let xor = (register & 0b0001_0000) != 0;
        let zyxda = (register & 0b0000_1000) != 0;
        let zda = (register & 0b0000_0100) != 0;
        let yda = (register & 0b0000_0010) != 0;
        let xda = (register & 0b0000_0001) != 0;

        Self {
            zyxor,
            zor,
            yor,
            xor,
            zyxda,
            zda,
            yda,
            xda,
        }
    }
}

/// Raw data from `OUTX_REG`
///
/// The output data represents the raw magnetic data only if OFFSET_X_REG is equal to zero,
/// otherwise hard-iron calibration is included.
pub struct OutX(pub i16);
impl OutX {
    fn parse(lsb: u8, msb: u8) -> Self {
        Self(i16::from_le_bytes([lsb, msb]))
    }
}

/// Raw data from `OUTY_REG`
///
/// The output data represents the raw magnetic data only if OFFSET_Y_REG is equal to zero,
/// otherwise hard-iron calibration is included.
pub struct OutY(pub i16);
impl OutY {
    fn parse(lsb: u8, msb: u8) -> Self {
        Self(i16::from_le_bytes([lsb, msb]))
    }
}

/// Raw data from `OUTZ_REG`
///
/// The output data represents the raw magnetic data only if OFFSET_Z_REG is equal to zero,
/// otherwise hard-iron calibration is included.
pub struct OutZ(pub i16);
impl OutZ {
    fn parse(lsb: u8, msb: u8) -> Self {
        Self(i16::from_le_bytes([lsb, msb]))
    }
}

pub struct Output {
    pub x: OutX,
    pub y: OutY,
    pub z: OutZ,
}
impl Output {
    pub(crate) fn parse(lsb_x: u8, msb_x: u8, lsb_y: u8, msb_y: u8, lsb_z: u8, msb_z: u8) -> Self {
        Self {
            x: OutX::parse(lsb_x, msb_x),
            y: OutY::parse(lsb_y, msb_y),
            z: OutZ::parse(lsb_z, msb_z),
        }
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u8)]
pub enum OutputDataRate {
    _100Hz = 0b11,
    _50Hz = 0b10,
    _20Hz = 0b01,
    _10Hz = 0b00,
}

#[derive(Debug, PartialEq, Copy, Clone)]
#[repr(u8)]
pub enum ModeOfOperation {
    /// In continuous mode the device continuously
    /// performs measurements and places the result in the data register. The
    /// data-ready signal is generated when a new data set is ready to be
    /// read. This signal can be available on the external pin by setting the
    /// DRDY_on_PIN bit in CfgRegC (62h).
    Continuous = 0b00,
    /// When single mode is selected, the device performs a
    /// single measurement, sets DRDY high and returns to idle mode.
    Single = 0b01,
    /// Device is placed in idle mode. I2C and SPI active.
    Idle = 0b10,
    // 0b11 is functionally the same as 0b10
}

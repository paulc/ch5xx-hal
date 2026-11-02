//! Peripheral marker traits.
//!
//! ch32-hal's `RccPeripheral` has no CH5xx counterpart: there is no RCC to
//! gate a peripheral's clock with (the `SYS` sleep-clock bits stand in, see
//! `crate::sys`). [`RemapPeripheral`] keeps ch32-hal's shape for the pin remap,
//! which CH5xx selects per peripheral in `SYS.PIN_ALTERNATE` instead of AFIO.

use embassy_hal_internal::PeripheralType;

/// A peripheral this HAL can hand out.
pub trait Peripheral: PeripheralType + 'static {}

/// A peripheral whose pins can be remapped (`R16_PIN_ALTERNATE`).
///
/// `remap` 0 selects the default pins and 1 the alternate (`_`-suffixed in the
/// datasheet) pins. Peripherals without a remap bit accept only 0.
pub(crate) trait RemapPeripheral {
    fn set_remap(remap: u8);
}

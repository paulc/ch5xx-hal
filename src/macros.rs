#![macro_use]
#![allow(unused_macros)]

/// `panic!` that routes through `defmt` when that feature is on.
#[collapse_debuginfo(yes)]
macro_rules! panic {
    ($($x:tt)*) => {
        {
            #[cfg(not(feature = "defmt"))]
            ::core::panic!($($x)*);
            #[cfg(feature = "defmt")]
            ::defmt::panic!($($x)*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::trace!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::debug!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::info!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::warn!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt")]
            ::defmt::error!($s $(, $x)*);
            #[cfg(not(feature = "defmt"))]
            let _ = ($( & $x ),*);
        }
    };
}

/// Declare a pin trait for a peripheral signal.
///
/// Same shape as ch32-hal's PCFR-remap form: `trait Signal<T, const REMAP: u8 =
/// 0>`. On CH5xx `REMAP` is the peripheral's `PIN_ALTERNATE` bit.
macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: u8 = 0>: crate::gpio::Pin {}
    };
}

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $n:expr) => {
        if_pin!($pin, impl crate::$mod::$trait<crate::peripherals::$instance, $n> for crate::peripherals::$pin {});
    };
}

/// Write the remap bit for the `REMAP` const in scope.
macro_rules! apply_remap {
    () => {{
        fn apply<P: crate::peripheral::RemapPeripheral, const R: u8>() {
            P::set_remap(R);
        }
        apply::<T, REMAP>();
    }};
}

/// Configure a pin for peripheral use and consume it into
/// `Option<Peri<'d, AnyPin>>`.
macro_rules! new_pin {
    ($name:ident, $af_type:expr) => {{
        let pin = $name;
        crate::gpio::SealedPin::set_as_af(&*pin, $af_type);
        Some(pin.into())
    }};
}

# ch5xx-hal

Rust HAL for WCH CH5xx BLE microcontrollers, with blocking APIs and optional
Embassy support.

[Documentation](https://docs.rs/ch5xx-hal) · [Examples](examples/ch585)

> **Note**
> This crate is under development and its API may change. CH585 and CH584
> are exercised on hardware; CH592 support builds, and was tested on an
> earlier port.

## Supported devices

| Chip | Core | Status |
| --- | --- | --- |
| CH585 | QingKe V3C | GPIO, clocks, UART, and Embassy time drivers tested on hardware |
| CH584 | QingKe V3C | Same die as CH585; every driver and BLE tested on a WeAct CH584F board (`examples/ch584`, `examples/ch584-ble`) |
| CH592 | QingKe V4C | Builds; the drivers were tested on an earlier port (`examples/ch592`) |

Select exactly one part feature: `ch584m`, `ch584x`, `ch584f`, `ch585m`,
`ch585f`, `ch585c`, `ch585d`, `ch592x`, `ch592f`, `ch592a` or `ch592d`. The package decides which pins exist: only
bonded pins get a singleton. Register access comes from
[`ch5xx-metapac`](https://github.com/ch32-rs/ch5xx-data), allowing shared driver
code across chips with compatible peripherals.

## Features

- `gpio`: input, output, flex pins, pulls, and drive strength.
- `rcc`: HSI/HSE, dividers, PLL configuration, and clock reporting.
- `uart`: blocking and interrupt-driven async UART0–UART3, typed pin remapping,
  split halves, exact and short reads, and `embedded-io`/`embedded-io-async`
  traits.
- `delay`: `embedded-hal` `DelayNs` using SysTick or the Embassy timebase.
- Embassy time driver on SysTick or TMR0–TMR3, plus typed interrupt bindings.
- `timer`: counter, external counter, PWM and input capture (with a
  `Decoder` for whole measurements) on TMR0-TMR3, timer-local DMA, and
  `timer::pwmx`, the PWM4-PWM11 block.
- `exti`: GPIO interrupts (`ExtiInput`, async edge and level waits).
- `spi`: SPI0 master, blocking and async (DMA).
- `i2c`: I2C master, blocking and async; `soft_i2c`, a bit-banged master on
  any two pins clocked by a TMR (with clock stretching).
- `adc`: the 12-bit ADC with its PGA, differential pairs, battery and
  temperature channels, blocking, async, DMA and ring-buffer reads.
- `usb`: device driver for `embassy-usb` (USB on CH592, USBFS on
  CH584/CH585).
- `ledc`: the LED/display controller (CH584/CH585).
- `rtc`: the RTC counter, alarms and periodic interrupt; `wdg`: the
  watchdog.
- `pwr`: DC-DC, idle/halt/sleep/shutdown with their wake-up sources, the
  battery monitor.
- `ble` (feature): WCH's BLE library (vendored in `vendor/wch-ble`) with
  TMOS driven from Embassy, advertising, a peripheral role and a GATT server.
  With the `executor` feature, `embassy::Executor` runs TMOS and puts the
  chip to sleep between BLE events.
- `debug`: SDI print (`println!` over the WCH-LinkE debug link).

Timer-local DMA is not yet qualified for arbitrary bus loads, lossless
streaming, or NFC transport.

## Cargo features

| Feature | Effect |
| --- | --- |
| a part (`ch584f`, `ch592f`, ...) | Selects the die, core and bonded pins; exactly one is required |
| `rt` | Interrupt support through `qingke-rt` (default) |
| `embassy`, `time-driver-*` | Embassy time driver (see below) |
| `executor` | `embassy::Executor`, a thread-mode executor that sleeps when idle |
| `exti` | Built-in GPIO interrupt handlers for `ExtiInput` |
| `highcode` | Interrupt entry and `.highcode` functions run from RAM |
| `ble` | BLE; binaries add `-Tble.x` after `-Tlink.x`. Sleep needs `executor` and `time-driver-systick` |
| `memory-x` | Emit `memory.x` (enable in the binary crate) |

## Architecture

The API and ownership model follow `ch32-hal` and Embassy, with CH5xx-specific
hardware differences:

- Clocks and pin muxing live in `SYS`, not separate RCC/AFIO peripherals.
- One GPIO block contains per-port register groups.
- Each timer has one 26-bit counter/FIFO, no prescaler, and peripheral-local DMA.

A generated PAC singleton does not imply that a HAL driver exists.

## Quick start

```rust
let p = ch5xx_hal::init(ch5xx_hal::rcc::Config::HSI_PLL_24MHZ.into());
```

The default clock is HSI 16 MHz divided by 3 (5.333 MHz). It cannot generate an
in-spec 115200 baud for external UART communication; `uart_echo` therefore uses
`HSE_PLL_78MHZ`. HSE presets require a 32 MHz crystal.

Prefer selecting the final clock during initialization. Runtime clock changes
do not rebuild UART dividers, the Embassy timebase, or other clock-dependent
drivers.

See [`examples/ch585`](examples/ch585) for complete programs and wiring notes.

## Embassy time driver

| Feature | Counter | Effect |
| --- | --- | --- |
| `time-driver-systick` | Core SysTick | `delay` uses the Embassy timebase |
| `time-driver-tmr0` … `time-driver-tmr3` | Selected TMR | SysTick remains available for `delay` |

With `embassy` enabled, exactly one time driver must be selected; none or
multiple selections produce a compile error. The HAL has no default time driver,
while the example crate defaults to SysTick. A selected TMR is removed from
`Peripherals`, preventing a second owner.

`init()` starts the selected driver, and enables interrupts (with `rt`), so an Embassy
executor can be started directly from `main`.

## Target and atomics

Build for `riscv32imc-unknown-none-elf` on every supported chip, not
`riscv32imac`, and use `portable-atomic` with the `critical-section` feature
for shared atomics:

- The QingKe V3C (CH584/CH585) is RV32IMBC with **no A extension**: `amoadd.w`
  and friends raise an exception.
- The QingKe V4C (CH592) has the A extension, but a thread-mode LR/SC loop
  loses updates when an interrupt writes the same word between the `lr.w` and
  the `sc.w`. On `riscv32imac` Embassy's run queue is such a loop, so a task
  woken from an interrupt can drop out of the queue for good. Single AMOs and
  critical sections are not affected. `examples/ch592`'s `wake_race` shows the
  failure when built for `riscv32imac`.

The example crates already have the required target and dependency
configuration.

## Running examples

```console
cd examples/ch585
cargo run --release --bin blinky
```

`examples/ch584` (WeAct CH584F) and `examples/ch592` hold a hardware test
per driver, needing no wiring beyond the board, reporting PASS/FAIL over SDI
print; `examples/hw-test.sh` runs them and summarises:

```console
examples/hw-test.sh examples/ch584
```

`examples/ch584-ble` has the BLE examples, on the low-power executor.

The test crates run Embassy's `platform-spin` executor by default;
`--no-default-features --features platform-riscv32` (for example in
`CARGO_ARGS`) selects the one that sleeps in `wfi` between polls. The CH584
suite passes on both, except `soft_i2c` at 100 kHz under `platform-riscv32`.

Tests that enter low-power modes or need something beyond the board (`pwr`,
`sleep_current`, `usb_cdc`) are marked `hw-test: manual` and run only when
named:

- `pwr`, `sleep_current` and the BLE examples log on UART0 TX (PB7, 115200
  8N1), as SDI print stops in halt: wire PB7 to the WCH-LinkE RX and flash
  without `--enable-sdi-print`.
- `sleep_current` and the BLE examples switch the debug interface off after a
  2 s pause at reset, and a chip in halt or sleep does not answer the
  debugger. To reflash, power-cycle the board and flash at once, with the
  board's own USB unplugged (otherwise it stays powered).
- Measure current from the 5 V pin: fed from the LinkE's 3.3 V, the WeAct
  CH584F's regulator draws about 0.55 mA backwards.
- `usb_cdc` needs a host on the board's USB port; the BLE examples need a
  scanner or central.

The runner uses `probe-rs run --chip CH585` to flash, reset, and stream `defmt`
over RTT. The example manifest uses the local metapac build
(`../ch5xx-data`). To build every example, select a TMR time driver:

```console
cargo build --release --bins --no-default-features --features time-driver-tmr0
```

## License

Licensed under either of Apache License, Version 2.0 or MIT license, at your
option.

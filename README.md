# ch5xx-hal

Rust HAL (Hardware Abstraction Layer) crate for WCH's CH5xx BLE microcontrollers.

> **Note**
> This project is under development. Most peripheral drivers exist for CH592
> only; CH585 has the core (GPIO, delay, clocks). Expect the API to change.

This HAL follows the [ch32-hal](https://github.com/ch32-rs/ch32-hal) layout and
the metapac approach: register access comes from a single
[`ch5xx-metapac`](https://github.com/ch32-rs/ch5xx-data) crate, so the same HAL
code serves every supported chip in the family.

## Supported devices

| Chip  | Core      | Status                          |
| ----- | --------- | ------------------------------- |
| CH585 | QingKe V3C | Bring-up target; blinky verified on hardware |
| CH592 | QingKe V4C | Peripheral drivers below, tested on a WeAct CH592F board |

Chip support is selected with a Cargo feature (`ch585`, `ch592`). Exactly one
must be enabled. The chip feature also selects the QingKe core for `qingke` /
`qingke-rt`. Pin singletons only exist for bonded pins; on CH592 that is the
QFN32 (CH592X) set, PA4–PA15, PB0, PB4, PB6, PB7, PB10–PB15, PB22 and PB23.
The QFN28 CH592F lacks PA6, PA7, PB0 and PB6, which still get singletons.

## Building

The HAL depends on sibling checkouts, not crates.io releases (yet):

- `../ch5xx-data`, with the metapac generated (`./d gen`) into
  `build/ch5xx-metapac`. CH592 needs the data fixes on its `ch592-ds-v1.9`
  branch.
- `../qingke`, for the `v3c`/`v4c` core features, which postdate qingke
  0.8.2 (see the `[patch.crates-io]` blocks).

Builds need nightly with `rust-src` (`-Zbuild-std=core`); the example crates
set this up in `.cargo/config.toml`:

```console
cargo +nightly build -Zbuild-std=core --target riscv32imac-unknown-none-elf --release --features ch592
cargo +nightly doc -Zbuild-std=core --target riscv32imac-unknown-none-elf --features ch592,embassy,exti --no-deps
```

## Implemented so far

All chips:

- `init()`: clock/frequency setup and GPIO bring-up
- `gpio`: input, output, flex pins, internal pull up/down, drive strength
- `delay`: `embedded-hal` `DelayNs`, backed by the core SysTick
- `rcc`: system clock (CH592: HSE/PLL selection; CH585: measured)

CH592 only. The APIs follow ch32-hal where one exists. "HW" marks what has
been checked on a WeAct CH592F board by the test examples in `examples/ch592`.

| Module | Driver | HW |
| ------ | ------ | -- |
| `debug` | SDI print (`println!` over the debug link) | yes |
| `embassy` | `embassy-time` driver on SysTick (feature `embassy`) | yes |
| `exti` | `ExtiInput` pin interrupts, async edge/level waits (feature `exti`) | yes |
| `uart` | UART0–3, blocking and async, `embedded-io(-async)` | yes |
| `spi` | SPI0 master, blocking and async, `embedded-hal(-async)` | yes (alternate pins PB12–PB15 untested) |
| `adc` | single-ended and differential, PGA, calibration, temperature, VBAT | yes |
| `timer::simple_pwm` | `SimplePwm` on TMR0–3 | yes (TMR1/TMR2 remap pins untested) |
| `timer::pwmx` | 8/16-bit PWMX channels 4–11 | yes |
| `rtc` | counter, alarm, periodic flag, LSI or LSE, LSI calibration | yes |
| `wdg` | `IndependentWatchdog` (IWDG) | yes |
| `i2c` | I2C0 master, blocking and async, `embedded-hal(-async)`; on PB15/PB14, so it turns the debug interface off | yes (bus scan) |
| `usb` | full-speed device for `embassy-usb` | yes (CDC-ACM echo; stall and remote wakeup untested) |

Not implemented: BLE, low-power modes, timer capture, CH585 peripherals.

## What differs from CH32

The CH5xx register model is not a CH32 one, so a few things look different:

- **No RCC.** Clocks, power and the pin mux live in `SYS` / `System_control` at
  `0x4000_1000`.
- **No AFIO.** Peripheral signals are routed with the `SYS.PIN_ALTERNATE`
  "alternate pin enable" bits rather than a remap register.
- **One GPIO block** with per-port register groups (`pa_out`, `pb_dir`, …)
  instead of one peripheral per port.

## System clock

- **CH592**: every HCLK source derives from the 32 MHz crystal, so the
  frequency is computed exactly from `CLK_CFG`. The boot ROM leaves HSE / 5 =
  6.4 MHz; `rcc::Config` presets select up to 60 MHz from the PLL (USB needs
  the PLL, e.g. `SYSCLK_FREQ_60MHZ`).
- **CH585**: the boot ROM leaves `CLK_CFG` PLL-sourced (`0x014D` rather than
  the documented default; about 5.33 MHz on the bring-up part). Rather than
  model the PLL, init measures SysTick against the RTC's independent 32 kHz
  counter (the same idea as WCH's `OSC_CAL`) and reports the result through
  `rcc::clocks()`. PLL setup is not implemented.

## Examples

`examples/ch585` blinks LED0 (PB16) and LED1 (PB17) in opposite phase. Its
runner is `probe-rs run --chip CH585`, which flashes, resets and streams
`defmt` output in one step:

```console
cd examples/ch585
cargo run --release --bin blinky
```

`examples/ch592` targets the WeAct CH592F core board (LED on PA8; on the WCH
EVT board, wire P2-18 (PA8) to P2-22 (LED0)). probe-rs has no CH59x target
yet, so the runner is `wlink --chip CH59X --speed low flash`. Both options
are required: wlink cannot auto-detect CH5xx parts, and at the default speed
the probe fails to start the flash routine on CH592 (`protocol error: 0x55`).

The two-wire debug interface must first be enabled in the user config
(`CFG_DEBUG_EN`, via `wchisp config enable-debug`). On CH592, USB-ISP
flashing with `wchisp` fails verify while debug is enabled
([wchisp#93](https://github.com/ch32-rs/wchisp/issues/93)), so pick one or the
other.

Besides `blinky`, `sdi_print`, `defmt_rtt` and `clocks` (which reports the
clock registers), the CH592 examples are hardware tests that print
`PASS`/`FAIL` lines over SDI print: `adc`, `embassy_time`, `exti`, `pwm`,
`rtc`, `uart_loopback`, `wdg` (which resets the chip on purpose), and `spi`
and `uart_async`, which need a PA14–PA15 jumper.
`usb_cdc` is a CDC-ACM echo on the USB port; on macOS open
`/dev/cu.usbmodem*` (the `tty.` node waits for carrier detect, which the
class never reports). `i2c_scan` reports over UART1 TX (PA9).

### PB14/PB15

With `CFG_DEBUG_EN` set, PB14/PB15 belong to the debug link (TIO/TCK) and
work as nothing else. The I2C pins have no alternate, so the I2C driver
turns the debug interface off until reset
(`debug::set_debug_interface_enabled`); the same call frees the pins for
GPIO or SPI's alternate pins. The datasheet says only parts with a
lot-number fifth digit of 1 or more implement it. Leave a delay after reset
before calling it (as `i2c_scan` does) so the debugger can still attach.

> **Careful when observing with a debugger:** every `wlink`/`probe-rs` memory
> read halts the core and does not resume it, so polling a live counter will
> freeze the firmware. Read state once after waiting, or record a history into
> RAM and read it afterwards.

## Console output

CH585 has no SDI printf (the debug-module `DATA0`/`DATA1` handshake is not
present). CH592 has it, but wlink needs a one-line patch to use it; see
`examples/ch592/src/bin/sdi_print.rs`. Otherwise use `defmt` over SEGGER RTT
— no special chip support is needed, and `probe-rs` handles it natively:

```console
cd examples/ch585
cargo run --release --bin defmt_rtt
```

This uses the `probe-rs run --chip CH585` runner from `.cargo/config.toml`. If
you want to flash and attach separately instead:

```console
probe-rs download --chip CH585 target/riscv32imac-unknown-none-elf/release/defmt_rtt
probe-rs attach --chip CH585 target/riscv32imac-unknown-none-elf/release/defmt_rtt
```

Add `defmt` and `defmt-rtt` to your binary's dependencies. The `DEFMT_LOG` env
variable controls log level at compile time (default is `error`; `.cargo/config.toml`
sets `info` for the examples).

## License

Licensed under either of Apache License, Version 2.0 or MIT license at your
option.

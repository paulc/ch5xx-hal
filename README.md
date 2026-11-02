# ch5xx-hal

Rust HAL (Hardware Abstraction Layer) crate for WCH's CH5xx BLE microcontrollers.

> **Note**
> This project is under development. Only a small part of the peripheral set is
> implemented, and only CH585 has been exercised on real hardware. Expect the
> API to change.

This HAL follows the [ch32-hal](https://github.com/ch32-rs/ch32-hal) layout and
the metapac approach: register access comes from a single
[`ch5xx-metapac`](https://github.com/ch32-rs/ch5xx-data) crate, so the same HAL
code serves every supported chip in the family.

## Supported devices

| Chip  | Core      | Status                          |
| ----- | --------- | ------------------------------- |
| CH585 | QingKe V3C | Bring-up target; blinky verified on hardware |
| CH592 | QingKe V4C | Builds and links; not yet run on hardware |

Chip support is selected with a Cargo feature (`ch585`, `ch592`). Exactly one
must be enabled. The chip feature also selects the QingKe core for `qingke` /
`qingke-rt`. Pin singletons only exist for bonded pins; on CH592 that is the
QFN32 (CH592X) set, PA4–PA15, PB0, PB4, PB6, PB7, PB10–PB15, PB22 and PB23.

## Implemented so far

- `init()` — clock/frequency setup and GPIO bring-up
- `gpio` — input, output, flex pins, internal pull up/down, drive strength
- `delay` — `embedded-hal` `DelayNs`, backed by the core SysTick
- `rcc` — reports the measured system clock

Everything else (UART, SPI, I2C, timers, BLE, USB, …) is not implemented yet.

## What differs from CH32

The CH5xx register model is not a CH32 one, so a few things look different:

- **No RCC.** Clocks, power and the pin mux live in `SYS` / `System_control` at
  `0x4000_1000`.
- **No AFIO.** Peripheral signals are routed with the `SYS.PIN_ALTERNATE`
  "alternate pin enable" bits rather than a remap register.
- **One GPIO block** with per-port register groups (`pa_out`, `pb_dir`, …)
  instead of one peripheral per port.

## System clock

The datasheet documents a 32 MHz `CLK_CFG` reset default, but the boot ROM does
not leave it there: on the CH585 used for bring-up `CLK_CFG` reads `0x014D`
(`CLK_SYS_MOD = 01`, PLL-sourced) and the actual system clock is
**5,333,696 Hz**.

Rather than model the PLL, `delay::init` measures SysTick against the RTC's
independent 32 kHz counter at startup (the same idea as WCH's `OSC_CAL`) and
publishes the result through `rcc::clocks()`. The measurement is deterministic
across resets. PLL setup itself is not implemented.

## Example

`examples/ch585` blinks LED0 (PB16) and LED1 (PB17) in opposite phase:

```console
cd examples/ch585
cargo run --release --bin blinky
```

`examples/ch592` blinks the LED on PA8 of the WeAct CH592F core board (on the
WCH EVT board, wire P2-18 (PA8) to P2-22 (LED0)). probe-rs has no CH59x target
yet, so its runner is `wlink --chip CH59X --speed low flash`. Both options are
required: wlink cannot auto-detect CH5xx parts, and at the default speed the
probe fails to start the flash routine on CH592 (`protocol error: 0x55`).

The two-wire debug interface must first be enabled in the user config
(`CFG_DEBUG_EN`, via `wchisp config enable-debug`). Note that on CH592 USB-ISP
flashing with `wchisp` fails verify while debug is enabled
([wchisp#93](https://github.com/ch32-rs/wchisp/issues/93)), so pick one or the
other.

The runner is `probe-rs run --chip CH585`, so `cargo run` flashes, resets, and
streams `defmt` output in one step. A `wlink flash` runner is kept commented out
in `.cargo/config.toml` if you only want to program the part.

> **Careful when observing with a debugger:** every `wlink`/`probe-rs` memory
> read halts the core and does not resume it, so polling a live counter will
> freeze the firmware. Read state once after waiting, or record a history into
> RAM and read it afterwards.

## Console output (defmt + RTT)

CH5xx has no SDI printf (the debug-module `DATA0`/`DATA1` handshake is not
present). Use `defmt` over SEGGER RTT instead — no special chip support is
needed, and `probe-rs` handles it natively:

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

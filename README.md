# ch5xx-hal

Rust HAL (Hardware Abstraction Layer) crate for WCH's CH5xx BLE microcontrollers.

> **Note**
> This project is under development. The peripheral drivers below exist for
> CH592 and CH584/CH585. Expect the API to change.

This HAL follows the [ch32-hal](https://github.com/ch32-rs/ch32-hal) layout and
the metapac approach: register access comes from a single
[`ch5xx-metapac`](https://github.com/ch32-rs/ch5xx-data) crate, so the same HAL
code serves every supported chip in the family.

## Supported devices

| Chip  | Core      | Status                          |
| ----- | --------- | ------------------------------- |
| CH584 | QingKe V3C | Peripheral drivers below, tested on a WeAct CH584F board |
| CH585 | QingKe V3C | As CH584 (CH585 adds USBHS and SPI1, not covered); not tested on a CH585 part |
| CH592 | QingKe V4C | Peripheral drivers below, tested on a WeAct CH592F board |

The part is selected with a Cargo feature named after it, as in ch32-hal;
exactly one must be enabled. It also selects the QingKe core for `qingke` /
`qingke-rt`, and the package decides which pins exist: only bonded pins get a
singleton, and pin trait impls for the others are left out, so a driver
cannot be given a pin the package lacks.

| Feature | Package | GPIO |
| ------- | ------- | ---- |
| `ch584m`, `ch584x`, `ch585m` | QFN48 | PA0-PA15, PB0-PB23 |
| `ch584f`, `ch585f` | QFN32 | PA4, PA5, PA9-PA15, PB4-PB8, PB10-PB17, PB22, PB23 |
| `ch585c` | QFN26 | PA9, PA12-PA15, PB4, PB8, PB10-PB17, PB22, PB23 |
| `ch585d` | QFN20 | PA9, PA12-PA15, PB10-PB15, PB22 |
| `ch592x` | QFN32 | PA4-PA15, PB0, PB4, PB6, PB7, PB10-PB15, PB22, PB23 |
| `ch592f`, `ch592a` | QFN28 | PA4, PA5, PA8-PA15, PB4, PB7, PB10-PB15, PB22, PB23 |
| `ch592d` | QFN20 | PA10-PA15, PB10-PB15 |

On the CH58xF and CH585C, PB9 shares PA9's pad (DS note 4); only PA9 gets a
singleton. Peripherals are not trimmed per package yet: a peripheral whose
pins are all missing still has a singleton (CH585C/D, for example, have no
I2C pins).

PB23 is also the RST# input; with `CFG_RESET_EN` set in the user config (as on
the WeAct boards) it reads high whatever drives it.

## Building

The HAL depends on a sibling checkout of `../ch5xx-data`, not a crates.io
release (yet), with the metapac generated (`./d gen`) into
`build/ch5xx-metapac`. CH592 needs the data fixes on its `ch592-ds-v1.9`
branch. `qingke` / `qingke-rt` come from crates.io (0.8.3, the first release
with the `v3c`/`v4c` core features).

Builds need nightly with `rust-src` (`-Zbuild-std=core`); the example crates
set this up in `.cargo/config.toml`:

```console
cargo +nightly build -Zbuild-std=core --target riscv32imc-unknown-none-elf --release --features ch592f
cargo +nightly build -Zbuild-std=core --target riscv32imc-unknown-none-elf --release --features ch584f
```

Build for `riscv32imc`: the CH584/CH585 core (V3C) has no A extension, and on
CH592 an LR/SC loop is not safe against interrupts (see the examples below).

### API docs

The modules shown depend on the chip feature, so build the docs per chip.
Both builds write to the same directory, so give the second one its own
`--target-dir` to keep both:

```console
# CH592: target/riscv32imc-unknown-none-elf/doc/ch5xx_hal/index.html
cargo +nightly doc -Zbuild-std=core --target riscv32imc-unknown-none-elf --features ch592f,embassy,exti --no-deps

# CH584: target/doc-ch584/riscv32imc-unknown-none-elf/doc/ch5xx_hal/index.html
cargo +nightly doc -Zbuild-std=core --target riscv32imc-unknown-none-elf --features ch584f,embassy,exti --no-deps --target-dir target/doc-ch584
```

Add `--open` to open the result in a browser, or serve the `doc` directory
(`python3 -m http.server`).

## Implemented so far

All chips:

- `init()`: clock/frequency setup and GPIO bring-up
- `gpio`: input, output, flex pins, internal pull up/down, drive strength
- `delay`: `embedded-hal` `DelayNs`, backed by the core SysTick
- `rcc`: system clock: HSE, HSI (CH584/CH585) or PLL

CH592 and CH584/CH585. The APIs follow ch32-hal where one exists. The HW
columns mark what has been checked on a WeAct CH592F and a WeAct CH584F
board by the test examples in `examples/ch592` and `examples/ch584`.

| Module | Driver | HW CH592 | HW CH584 |
| ------ | ------ | -------- | -------- |
| `debug` | SDI print (`println!` over the debug link; `sdi_writeln!` for raw bytes) | yes | yes |
| `embassy` | `embassy-time` driver on SysTick (feature `embassy`) | yes | yes |
| `exti` | `ExtiInput` pin interrupts, async edge/level waits (feature `exti`) | yes | yes |
| `uart` | UART0-3, blocking and async, `embedded-io(-async)` | yes | yes |
| `spi` | SPI0 master, blocking and async, `embedded-hal(-async)` | yes (alternate pins PB12-PB15 untested) | yes (alternate pins untested) |
| `adc` | single-ended and differential, PGA, calibration, temperature, VBAT; async (interrupt) reads, DMA reads (back to back or timed), ring-buffered DMA | yes | yes |
| `timer::simple_pwm` | `SimplePwm` on TMR0-3 | yes (TMR1/TMR2 remap pins untested) | yes (TMR1/TMR2 remap pins untested) |
| `timer::input_capture` | `InputCapture` on TMR0-3: period or high/low times, blocking and async; DMA on TMR1/TMR2 (all four on CH584/CH585) | yes | yes (DMA on TMR0/TMR3 untested) |
| `timer::counter` | `Counter` on TMR0-3: edge counting with a wrap period, blocking and async | yes | yes |
| `timer::pwmx` | 8/16-bit PWMX channels 4-11 | yes | yes (remap pins untested) |
| `rtc` | counter, alarm, periodic flag, LSI or LSE, LSI calibration | yes | yes |
| `wdg` | `IndependentWatchdog` (IWDG) | yes | yes |
| `i2c` | I2C0 master, blocking and async, `embedded-hal(-async)`; on CH592 on PB15/PB14, so it turns the debug interface off | yes (bus scan) | yes (no device: NACKs, stuck-line errors and recovery) |
| `soft_i2c` | bit-bang I2C master on any two pins, clocked by TMR0-3, blocking and async, `embedded-hal(-async)`; up to 100 kHz with feature `highcode` | yes (waveform, stretching, errors; devices not yet) | yes (as CH592) |
| `usb` | full-speed device for `embassy-usb` (`USBFS` on CH584/CH585) | yes (CDC-ACM echo; stall and remote wakeup untested) | enumerates (echo untested) |
| `ledc` | LED screen controller (CH584/CH585): clock plus 1/2/4/8 data lanes, blocking (FIFO) and async (DMA) | - | yes (clock and lane 4, the only data pin the CH584F bonds; WCH supports the block on QFN48 parts only) |
| `pwr` | DC-DC converter; idle, halt, sleep and shutdown modes with GPIO/RTC/USB/battery wake-up; battery voltage monitor; reset reason | not yet (`pwr` example) | modes, RTC and GPIO wake-up, battery monitor, reset reason (not yet: DC-DC) |

Not implemented: BLE, low-power modes, timer PWM DMA, and on CH584/CH585
USBHS, SPI1, LCD and touch-key.

## What differs from CH32

The CH5xx register model is not a CH32 one, so a few things look different:

- **No RCC.** Clocks, power and the pin mux live in `SYS` / `System_control` at
  `0x4000_1000`.
- **No AFIO.** Peripheral signals are routed with the `SYS.PIN_ALTERNATE`
  "alternate pin enable" bits rather than a remap register.
- **One GPIO block** with per-port register groups (`pa_out`, `pb_dir`, ...)
  instead of one peripheral per port.

## System clock

- **CH592**: every HCLK source derives from the 32 MHz crystal, so the
  frequency is computed exactly from `CLK_CFG`. The boot ROM leaves HSE / 5 =
  6.4 MHz; `rcc::Config` presets select up to 60 MHz from the PLL (USB needs
  the PLL, e.g. `SYSCLK_FREQ_60MHZ`).
- **CH584/CH585**: HCLK comes from the 32 MHz crystal, the 16 MHz RC (+-1%)
  or the 312 MHz PLL, each divided, and again the frequency is computed from
  `CLK_CFG`. The boot ROM leaves HSI / 3 = 5.33 MHz; presets select up to
  78 MHz from the PLL (`SYSCLK_FREQ_62_4MHZ` is WCH's default). A wlink attach
  sets `CLK_CFG` to `0x014D` (24 MHz) on the running program.

## Examples

`examples/ch584` targets the WeAct CH584F core board (LED on PB6) and holds
the same hardware tests as `examples/ch592`, plus `i2c` and `ledc`; build them for
`riscv32imc` (the default there), since V3C has no A extension and the
first atomic instruction of an `imac` build traps. The runner is
`wlink --chip CH585 flash`: wlink reports a CH584 as a CH585. They need the
PA14-PA15 jumper too. On that board PB10/PB11 go to the USB connector (with
a host attached they read low through the pull-ups, so `soft_i2c` uses
PA10/PA11 there) and PB22 has a strong pull-up.

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
`PASS`/`FAIL` lines over SDI print: `adc`, `capture`, `counter`,
`embassy_time`, `exti`, `pwm`, `rtc`, `uart_loopback`, `wake_race`, `wdg`
(which resets the chip on purpose), and `soft_i2c`, `spi` and `uart_async`,
which need a PA14-PA15 jumper. `soft_i2c_sensors` reads an AHT20 and a BMP280 on PB13
(SCL) / PB12 (SDA).

`usb_cdc` is a CDC-ACM echo on the USB port; on macOS open
`/dev/cu.usbmodem*` (the `tty.` node waits for carrier detect, which the
class never reports). `i2c_scan` reports over UART1 TX (PA9).

The embassy examples build with either executor platform: `platform-spin`
(the default; never sleeps) or `platform-riscv32`, which sleeps between
polls:

```console
cd examples/ch592
cargo run --release --no-default-features --features platform-riscv32 --bin uart_async
```

`platform-riscv32` sleeps in `wfi` inside a critical section, which wakes on
QingKe only because qingke-rt turns `wfi` into `wfe` and `init()` sets
`PFIC_SCTLR.SEVONPEND`, making a pending (masked) interrupt a wake-up event.

The CH592 examples build for `riscv32imc`, not `riscv32imac`, although the
QingKe V4C core implements the A extension. Its AMOs hold, but an LR/SC loop
(`compare_exchange`) loses updates when an interrupt writes the same word
between the LR and the SC, and embassy's run queue is such a loop: a task
woken by an interrupt while another task wakes a third can be dropped from the
queue for good. Without A, embassy and the HAL (through `portable-atomic`)
make atomic read-modify-writes in a critical section. `wake_race` checks this;
built with `--target riscv32imac-unknown-none-elf` it fails. Applications on
CH592 should use `riscv32imc` too.

### Running the hardware tests

`examples/hw-test.sh` flashes each test of an example crate, captures its SDI
print output until it reports a result, and prints a summary; its exit
status is the number of tests that failed or reported nothing:

```console
examples/hw-test.sh examples/ch584                # every test
examples/hw-test.sh examples/ch584 ledc spi       # some
CARGO_ARGS="--no-default-features --features platform-riscv32" \
    examples/hw-test.sh examples/ch584            # the other executor
```

It uses the crate's runner with `--enable-sdi-print --watch-serial` added;
set `RUNNER` to use another (cargo appends the ELF path). The SDI console
sometimes fails to start, so a test that prints nothing is retried
(`TRIES`, default 3). Logs go to `<crate>/target/hw-test`. The tests that
need wiring (the PA14-PA15 jumper) say so in their source. Tests marked
`hw-test: manual` run only when named: `pwr` enters the low-power modes (and,
with its `DCDC` constant set, the DC-DC converter, which needs the board's
inductor), and waits up to 10 s for a key press, so give it `SECS=60`. SDI
print stops at its first halt, so it writes to UART0 TX on PB7 (115200)
instead: wire PB7 to WCH-LinkE RX and read the LinkE's serial port. Without
`--enable-sdi-print`, `wlink flash --watch-serial` leaves the core halted
(wlink only releases it when it detaches), so flash with plain `wlink flash`
and open the port with a serial terminal; the test waits 2 s before printing.

The examples enable the HAL's `highcode` feature, which runs interrupt entry
and the functions placed in `.highcode` from RAM: CH592 flash runs code
roughly ten times slower than RAM (an empty loop iteration takes 32 cycles).
It works the same on CH584 (V3C).

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

CH592 and CH584/CH585 both have SDI print (the debug-module `DATA0`/`DATA1`
handshake, at `0xE000_0380` on V4C and `0xE000_0340` on V3C), but wlink
0.1.3 refuses to arm it for either: add `CH59X` and `CH585` to
`support_sdi_print()` in wlink's `src/lib.rs`. See `src/debug.rs`. Otherwise
use `defmt` over SEGGER RTT, which needs no special chip support and which
`probe-rs` handles natively:

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

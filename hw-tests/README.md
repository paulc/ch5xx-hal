# Hardware tests

On-target tests for the HAL drivers, one crate per chip:

- `ch584/` for the WeAct CH584F core board (CH584/CH585 die).
- `ch592/` for a CH592F board.

Each binary in `src/bin` exercises one driver on the board itself. Most need
no wiring: they drive a pin with a pull or as an output and read it back with
the peripheral under test, or loop a UART back internally. Results are lines
such as

```text
uart0 loopback 115200 baud: PASS
uart0 loopback 7E2: PASS
any edge: SKIP (wait_for_any_edge panics in this HAL)
uart_loopback done: 5 pass, 0 fail
```

and they are printed over the debug link ("SDI print"), so a WCH-LinkE is all
the setup a test needs. The last line repeats once a second. `SKIP` marks a
check that the HAL cannot run yet, with the reason.

`common/hwtest.rs` holds what the crates share: the SDI console, the PASS/FAIL
counts and the summary line. Each crate's `lib.rs` includes it and adds the
chip's debug-module address and its safe-access sequence.

## Running

`hw-test.sh` builds, flashes and watches each test in turn, then prints a
table:

```console
$ hw-tests/hw-test.sh hw-tests/ch584
test            pass  fail  result
adc               15     0  ok
...
```

By default it flashes with the crate's `.cargo/config.toml` runner (`wlink`)
plus `--enable-sdi-print --watch-serial`. Set `RUNNER` to use a different
flash-and-watch command; cargo appends the ELF path. Name tests to run only
those:

```console
hw-tests/hw-test.sh hw-tests/ch584 adc spi
```

The executor and time driver are Cargo features of the test crates:

```console
# The executor sleeps in WFI between polls, as the HAL's examples do.
CARGO_ARGS="--no-default-features --features platform-riscv32,time-driver-systick" \
    hw-tests/hw-test.sh hw-tests/ch584

# Only `clocks` with the TMR0 time driver: the timer tests use TMR0.
CARGO_ARGS="--no-default-features --features platform-spin,time-driver-tmr0" \
    hw-tests/hw-test.sh hw-tests/ch584 clocks
```

One test can also be run by hand, from its crate directory:

```console
cd hw-tests/ch584
cargo run --release --bin adc -- --enable-sdi-print --watch-serial
```

wlink needs `--chip` for these parts (`CH585` for the CH584, `CH59X` for the
CH592), and the CH592 also needs `--speed low`; the crates' runners have both.
The 0.1.x releases of wlink refuse to arm SDI print for these chips: add them
to the chips `--enable-sdi-print` accepts. The console sometimes fails to
start, so `hw-test.sh` retries a test that printed nothing (`TRIES`).

## Tests

| Test | Covers | Wiring |
| --- | --- | --- |
| `adc` | temperature, battery, a pin driven and pulled, digital input off while analog, calibration, conversion time; CH584: scan with DMA | none |
| `blinky` | the LED (CH584F PB6, CH592F PA8) | none |
| `capture` | timer input capture on TMR0..3: the three edge modes, long levels and the timeout, FIFO overrun, DMA | none |
| `clocks` | `rcc::set_config` presets, `rcc::clocks()`, `Delay` against TMR1 | none |
| `counter` | external-pin counting on TMR0 and the internal counter on TMR1 (`elapsed`, `restart`, `wait`) | none |
| `embassy_time` | `with_timeout`, short timers, then `Instant` printed every 500 ms | none |
| `exti` | every `Input::wait_for_*` on port A and port B; CH592: PB22, which shares interrupt bit 8 | none |
| `i2c` | (CH584) NACKs, address range, SDA or SCL held low, the timeout, the clock limits | none |
| `pwm` | `SimplePwm` on TMR0..3 and PWMX channels, period and duty measured on the pin | none |
| `rtc` | 32 kHz rate, calendar round trip and rollovers, refused dates, alarm, interval timer | none |
| `sdi_print` | prints a counter forever (a demo, no PASS lines) | none |
| `spi` | SPI0 blocking, async and DMA timing, `set_config`, refused configs; full transfers with a jumper | optional PA14-PA15 |
| `uart_async` | async UART TX and RX over the internal loopback; more with a jumper | optional PA14-PA15 |
| `uart_loopback` | blocking UART at three rates and 7E2 over the internal loopback, a refused rate | none |
| `wake_race` | executor wakes from tasks racing wakes from an interrupt | none |
| `wdg` | window watchdog period, overflow flag, reset; reset cause after a watchdog and a software reset | none |

These are run only when named, because they leave the debug link or need a
host:

| Test | Covers | Needs |
| --- | --- | --- |
| `pwr` | battery detector, halt and sleep woken by the RTC, sleep woken by the KEY button (PB22) | PB7 (UART0 TX) to the WCH-LinkE RX, a runner without SDI print, `SECS=60` |
| `sleep_current` | halt and sleep for 10 s each, for a current meter | a meter; board powered from its 5 V pin |
| `usb_cdc` | USB CDC-ACM echo (VID:PID c0de:cafe) | a USB host |

## Not ported

Tests from the original suite that have no counterpart in this HAL yet:

- `ledc` (CH585 LED controller), `soft_i2c`, `soft_i2c_sensors`, `i2c_scan`.
- The CH584 BLE examples (`ble_adv`, `ble_peripheral`, `ble_sleep`).
- `i2c` on CH592, which has no hardware I2C test in the original suite.
- Checks within ported tests that are marked `SKIP` in their output, with
  the reason.

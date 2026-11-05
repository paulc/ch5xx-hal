//! Generates the peripheral singletons, interrupt module and pin table for
//! `ch5xx-hal` from `ch5xx-metapac`'s metadata.
//!
//! This mirrors `ch32-hal/build.rs`, with the CH5xx differences folded in:
//!
//! - There is no RCC and no AFIO. The chip's clock/power/pin-mux registers live
//!   in the `SYS` (and `System_control`) blocks at `0x4000_1000`, so the
//!   `RccPeripheral` / `RemapPeripheral` impls ch32-hal emits have no
//!   counterpart here.
//! - GPIO is **one** peripheral (`GPIO`) split into per-port accessor registers
//!   (`PA_DIR`, `PB_OUT`, …), not one peripheral per port. Pins are still
//!   individual singletons (`PA0`, … `PB23`) so the driver API is unchanged.
//! - There are no per-pin EXTI channels; `GPIO_A` / `GPIO_B` are whole-port
//!   interrupts.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::{env, fs};

use ch5xx_metapac::metadata::METADATA;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Exactly one part feature, e.g. `ch584f`. Its die (`ch584`) and the part
    // itself both become cfgs; drivers use the die, or better the block
    // versions below.
    let part = env::vars()
        .map(|(a, _)| a)
        .filter(|x| x.starts_with("CARGO_FEATURE_CH5"))
        .collect::<Vec<_>>();
    let part = match part.len() {
        0 => panic!("no ch5xx part Cargo feature enabled (e.g. `ch584f`)"),
        1 => part[0].strip_prefix("CARGO_FEATURE_").unwrap().to_ascii_lowercase(),
        _ => panic!("multiple ch5xx part Cargo features enabled; pick one"),
    };
    let chip_name = match part.as_str() {
        "ch584m" | "ch584x" | "ch584f" => "ch584",
        "ch585m" | "ch585f" | "ch585c" | "ch585d" => "ch585",
        "ch592x" | "ch592f" | "ch592a" | "ch592d" => "ch592",
        other => panic!("unknown ch5xx part `{other}`"),
    };
    println!("cargo:rustc-cfg={chip_name}");
    println!("cargo:rustc-cfg={part}");

    // One cfg per register block version, e.g. `adc_ch592`, `sys_ch585`, as
    // embassy-stm32 does (`adc_v3`): drivers gate on the block they program,
    // not the chip, so chips sharing a block share the code. CH584 and CH585
    // are one die and use the same blocks, for example.
    let mut block_cfgs = BTreeSet::new();
    for p in METADATA.peripherals {
        if let Some(r) = &p.registers {
            block_cfgs.insert(format!("{}_{}", r.kind, r.version));
        }
    }
    for cfg in &block_cfgs {
        println!("cargo:rustc-cfg={cfg}");
    }

    // Bonded GPIO pins per package, from the datasheets' pin definition
    // tables (CH585/CH584 DS V1.6, CH592
    // DS V1.9). Metadata only lists pins that carry a
    // peripheral signal, so pads with no alternate function would not be
    // discovered, and it does not say which pads a package bonds out. Only these pins get a singleton, and pin trait impls for
    // any other pin are left out (`if_pin!`).
    let ports: &[(char, &[u8])] = match part.as_str() {
        // QFN48: all 40.
        "ch584m" | "ch584x" | "ch585m" => &[
            ('A', &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
            (
                'B',
                &[
                    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                    22, 23,
                ],
            ),
        ],
        // QFN32: 24. PB9 shares PA9's pad (DS note 4), so only PA9 is listed.
        "ch584f" | "ch585f" => &[
            ('A', &[4, 5, 9, 10, 11, 12, 13, 14, 15]),
            ('B', &[4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 22, 23]),
        ],
        // QFN26: 17, with PB9 again on PA9's pad.
        "ch585c" => &[
            ('A', &[9, 12, 13, 14, 15]),
            ('B', &[4, 8, 10, 11, 12, 13, 14, 15, 16, 17, 22, 23]),
        ],
        // QFN20: 12.
        "ch585d" => &[('A', &[9, 12, 13, 14, 15]), ('B', &[10, 11, 12, 13, 14, 15, 22])],
        // QFN32: 24.
        "ch592x" => &[
            ('A', &[4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
            ('B', &[0, 4, 6, 7, 10, 11, 12, 13, 14, 15, 22, 23]),
        ],
        // QFN28: 20.
        "ch592f" | "ch592a" => &[
            ('A', &[4, 5, 8, 9, 10, 11, 12, 13, 14, 15]),
            ('B', &[4, 7, 10, 11, 12, 13, 14, 15, 22, 23]),
        ],
        // QFN20: 12.
        "ch592d" => &[('A', &[10, 11, 12, 13, 14, 15]), ('B', &[10, 11, 12, 13, 14, 15])],
        other => panic!("no pin list for part `{other}`"),
    };

    // Pads on the die that the package does not bond out (PA0-PA15, PB0-PB23
    // on every part). `gpio::init` pulls them up: left floating they draw
    // current, about 27 uA in shutdown on a CH584F, which is why WCH's
    // examples configure `GPIO_Pin_All`. Where PB9 shares PA9's pad (DS note
    // 4: the two must not both be outputs), PB9 is left to PA9.
    let shared_pads: &[(char, u8)] = match part.as_str() {
        "ch584f" | "ch585f" | "ch585c" => &[('B', 9)],
        _ => &[],
    };
    let unbonded: Vec<u32> = [('A', 16u32), ('B', 24)]
        .iter()
        .map(|&(port, width)| {
            let mut mask = (1u32 << width) - 1;
            for (p, bonded) in ports {
                if *p == port {
                    for &n in *bonded {
                        mask &= !(1 << n);
                    }
                }
            }
            for &(p, n) in shared_pads {
                if p == port {
                    mask &= !(1 << n);
                }
            }
            mask
        })
        .collect();

    // A `time-driver-tmr*` feature takes that TMR for the timebase. Drop it from
    // the singletons so `init()` cannot hand the same peripheral to a second
    // user; the driver reaches it through the PAC anyway. Exactly one such
    // feature may be enabled (`embassy::mod` enforces that), so a lookup is
    // enough.
    let time_driver_peripheral: Option<String> = ["TMR0", "TMR1", "TMR2", "TMR3"]
        .into_iter()
        .find(|n| env::var_os(format!("CARGO_FEATURE_TIME_DRIVER_{n}")).is_some())
        .map(str::to_owned);

    // Peripheral singletons. Skip the register-less placeholders (BLE) and the
    // core-internal blocks (PFIC, SysTick) — the HAL never hands those out.
    const SKIP_KINDS: &[&str] = &["pfic", "systick"];
    let mut singletons: Vec<String> = Vec::new();
    for p in METADATA.peripherals {
        if p.registers.is_none() {
            continue;
        }
        if SKIP_KINDS.contains(&p.registers.as_ref().unwrap().kind) {
            continue;
        }
        if p.registers.as_ref().unwrap().kind == "gpio" {
            // Pins are the user-facing singletons; the port block itself is not.
            continue;
        }
        let name = normalize(&p.name);
        if time_driver_peripheral.as_deref() == Some(name.as_str()) {
            continue;
        }
        singletons.push(name);
    }

    // Blocks without a peripheral entry of their own in the metadata, which
    // drivers still take as a singleton: the independent watchdog is two
    // registers at the start of `SYS`.
    if block_cfgs.contains("sys_ch592") || block_cfgs.contains("sys_ch585") {
        singletons.push("IWDG".to_string());
    }

    // One singleton per pin.
    let mut pins: Vec<(String, char, u8, u8)> = Vec::new();
    for (port, bonded) in ports {
        for &n in *bonded {
            pins.push((format!("P{port}{n}"), *port, (*port as u8) - b'A', n));
        }
    }

    // ---- _generated.rs -------------------------------------------------
    let mut g = TokenStream::new();

    let singleton_tokens: Vec<_> = singletons.iter().map(|s| format_ident!("{}", s)).collect();
    let pin_tokens: Vec<_> = pins.iter().map(|(p, ..)| format_ident!("{}", p)).collect();

    g.extend(quote! {
        crate::peripherals_definition!(#(#singleton_tokens,)* #(#pin_tokens),*);
        crate::peripherals_struct!(#(#singleton_tokens,)* #(#pin_tokens),*);
    });

    let (unbonded_a, unbonded_b) = (unbonded[0], unbonded[1]);
    g.extend(quote! {
        /// Pads the package does not bond out, per port (A, B).
        pub(crate) const UNBONDED_PADS: [u32; 2] = [#unbonded_a, #unbonded_b];
    });

    // Interrupt declarations go into their own file: `interrupt_mod!` expands
    // to `pub mod interrupt`, whose methods name `crate::interrupt`, which does
    // not resolve from inside the `_generated` module.
    let irqs: Vec<_> = METADATA
        .interrupts
        .iter()
        .map(|irq| format_ident!("{}", irq.name))
        .collect();
    let irqs_file = quote! {
        crate::interrupt_mod!(#(#irqs,)*);
    };
    fs::write(out_dir.join("_interrupts.rs"), irqs_file.to_string()).unwrap();

    // ---- _macros.rs ----------------------------------------------------
    let mut m = String::new();

    // foreach_peripheral: kind, name
    let peripherals_table: Vec<Vec<String>> = METADATA
        .peripherals
        .iter()
        .filter_map(|p| {
            p.registers
                .as_ref()
                .map(|r| vec![r.kind.to_string(), normalize(&p.name)])
        })
        .collect();
    make_table(&mut m, "foreach_peripheral", &peripherals_table);

    // foreach_pin: pin, port_name, port_num, pin_num
    let pins_table: Vec<Vec<String>> = pins
        .iter()
        .map(|(pin, port, port_num, pin_num)| {
            vec![
                pin.clone(),
                format!("GPIO{port}"),
                port_num.to_string(),
                pin_num.to_string(),
            ]
        })
        .collect();
    make_table(&mut m, "foreach_pin", &pins_table);

    // foreach_interrupt: peripheral, kind, block, signal, interrupt.
    // Register-less placeholders (BLE, NFC) are skipped: the table's `block`
    // column names a register block, which they do not have. Their vectors are
    // still in `pac::Interrupt`, so a driver can bind them directly.
    let interrupts_table: Vec<Vec<String>> = METADATA
        .peripherals
        .iter()
        .filter_map(|p| {
            let r = p.registers.as_ref()?;
            (!p.interrupts.is_empty()).then(|| {
                p.interrupts
                    .iter()
                    .map(|irq| {
                        vec![
                            normalize(&p.name),
                            r.kind.to_string(),
                            r.block.to_string(),
                            irq.signal.to_string(),
                            irq.interrupt.to_ascii_uppercase(),
                        ]
                    })
                    .collect::<Vec<_>>()
            })
        })
        .flatten()
        .collect();
    make_table(&mut m, "foreach_interrupt", &interrupts_table);

    // ---- write ---------------------------------------------------------
    fs::write(out_dir.join("_generated.rs"), g.to_string()).unwrap();
    // if_pin!(PA0, item): `item` if the part bonds PA0, else nothing. Drivers
    // wrap their pin trait impls in it, so a package without a pin simply
    // lacks the impl, as ch32-hal's generated impls do.
    m.push_str("#[allow(unused_macros)]\nmacro_rules! if_pin {\n");
    for (p, ..) in &pins {
        writeln!(m, "    ({p}, $($t:tt)*) => {{ $($t)* }};").unwrap();
    }
    m.push_str("    ($p:ident, $($t:tt)*) => {};\n}\n");

    fs::write(out_dir.join("_macros.rs"), m).unwrap();

    // BLE: WCH's library for the die, and `ble.x`, a linker script fragment
    // for the binary (`-Tble.x` after `-Tlink.x`) placing what qingke-rt's
    // `link.x` does not: `.highcode_lp` (time-critical library code, run
    // from RAM like `.highcode`; `ble::init` copies it) and the C common
    // symbols (the CH59x library has them; `ble::init` zeroes them).
    if env::var_os("CARGO_FEATURE_BLE").is_some() {
        let lib = match chip_name {
            "ch584" | "ch585" => "CH58xBLE",
            _ => "CH59xBLE",
        };
        let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
        println!("cargo:rustc-link-search=native={manifest_dir}/vendor/wch-ble");
        println!("cargo:rustc-link-lib=static={lib}");
        println!("cargo:rerun-if-changed=vendor/wch-ble/lib{lib}.a");
        fs::write(out_dir.join("ble.x"), BLE_X).unwrap();
        println!("cargo:rustc-link-search={}", out_dir.display());
    }

    println!("cargo:rerun-if-changed=build.rs");
}

/// Metadata names are free-form (`System_control`, `PWMx`); make them valid,
/// conventional Rust singleton idents (`SYSTEM_CONTROL`, `PWMX`).
fn normalize(name: &str) -> String {
    name.to_ascii_uppercase().replace('-', "_")
}

fn make_table(out: &mut String, name: &str, rows: &[Vec<String>]) {
    // Deduplicate: one row per (peripheral, signal, irq) is already unique, but
    // the generic helper is also used for tables where a chip can repeat a key.
    let mut seen = BTreeSet::new();
    write!(
        out,
        "#[allow(unused)]
macro_rules! {name} {{
    ($($pat:tt => $code:tt;)*) => {{
        macro_rules! __{name}_inner {{
            $(($pat) => $code;)*
            ($_:tt) => {{}}
        }}
"
    )
    .unwrap();

    for row in rows {
        if !seen.insert(row.join(",")) {
            continue;
        }
        writeln!(out, "        __{name}_inner!(({}));", row.join(",")).unwrap();
    }

    write!(out, "    }};\n}}").unwrap();
}

/// Linker script fragment for the `ble` feature (see `main`).
const BLE_X: &str = r#"/* ch5xx-hal `ble`: place the WCH BLE library's own sections.
   Use after qingke-rt's link.x: -Tlink.x -Tble.x */
SECTIONS
{
    .highcode_lp : ALIGN(4)
    {
        __ble_highcode_lp_lma = LOADADDR(.highcode_lp);
        __ble_highcode_lp_start = .;
        *(.highcode_lp .highcode_lp.*);
        . = ALIGN(4);
        __ble_highcode_lp_end = .;
    } > RAM AT > FLASH
}
INSERT AFTER .highcode;

SECTIONS
{
    .ble_common (NOLOAD) : ALIGN(4)
    {
        __ble_common_start = .;
        *(COMMON);
        . = ALIGN(4);
        __ble_common_end = .;
    } > RAM
}
INSERT AFTER .bss;
"#;

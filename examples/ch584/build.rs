fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // The BLE library's sections (see `ch5xx_hal::ble`).
    if std::env::var_os("CARGO_FEATURE_BLE").is_some() {
        println!("cargo:rustc-link-arg-bins=-Tble.x");
    }
}

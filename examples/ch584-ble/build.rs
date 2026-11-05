fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // The BLE library's sections (see `ch5xx_hal::ble`).
    println!("cargo:rustc-link-arg-bins=-Tble.x");
}

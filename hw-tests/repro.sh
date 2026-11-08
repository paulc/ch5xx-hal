#!/bin/sh
# Run example files from a HAL checkout on the CH584, over SDI print.
#
# usage: hw-tests/repro.sh <HAL dir> <example.rs> ...
#
#   hw-tests/repro.sh ../ch5xx-hal-fix \
#       ../ch5xx-hal-fix/examples/ch585/src/bin/repro_flash_timing.rs
#
# Upstream's examples log with defmt over RTT, which wlink does not show. Each
# file is copied into a scratch copy of hw-tests/ch584 (ch584/target/repro)
# that builds against the HAL at <HAL dir>, with `defmt_rtt` swapped for this
# crate's SDI console and panic handler, `panic_halt` dropped, the defmt log
# macros turned into `hwtest::println!`, and `hwtest::sdi_enable()` after the
# `= hal::init(` line. hw-test.sh then runs them, so its environment (RUNNER,
# SECS, BEFORE, ...) applies.
#
# A file finishes when it prints "<name> done: N pass, M fail", and may only
# use format strings that both defmt and core::fmt accept ({} and {:#x}).

set -eu

here=$(cd "$(dirname "$0")" && pwd)
hal=$(cd "${1:?usage: $0 <HAL dir> <example.rs> ...}" && pwd)
shift
[ $# -gt 0 ] || { echo "usage: $0 <HAL dir> <example.rs> ..." >&2; exit 1; }

src=$here/ch584
crate=$src/target/repro
mkdir -p "$crate/src/bin" "$crate/.cargo"
rm -f "$crate"/src/bin/*.rs

sed -e "s|path = \"\.\./\.\./\"|path = \"$hal\"|" \
    -e 's|^name = "ch584-hw-tests"|name = "ch584-repro"|' \
    "$src/Cargo.toml" >"$crate/Cargo.toml"
sed "s|\.\./\.\./common/hwtest\.rs|$here/common/hwtest.rs|" "$src/src/lib.rs" >"$crate/src/lib.rs"
cp "$src/build.rs" "$crate/"
cp "$src/.cargo/config.toml" "$crate/.cargo/"
for f in rust-toolchain.toml Cargo.lock; do
    [ -f "$src/$f" ] && [ ! -f "$crate/$f" ] && cp "$src/$f" "$crate/"
done

names=
for f in "$@"; do
    n=$(basename "$f" .rs)
    sed -e 's|^use defmt_rtt as _;|use hwtest as _; // SDI console and panic handler|' \
        -e '/^use panic_halt as _;/d' \
        -e 's/defmt::\(trace\|debug\|info\|warn\|error\)!/hwtest::println!/g' \
        -e '/= hal::init(/a\' -e '    hwtest::sdi_enable();' \
        "$f" >"$crate/src/bin/$n.rs"
    names="$names $n"
done

exec "$here/hw-test.sh" "$crate" $names

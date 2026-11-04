#!/bin/sh
# Run the hardware tests of an example crate and summarise them.
#
# usage: examples/hw-test.sh <crate dir> [test ...]
#
#   examples/hw-test.sh examples/ch584
#   examples/hw-test.sh examples/ch584 ledc spi
#   CARGO_ARGS="--no-default-features --features platform-riscv32" \
#       examples/hw-test.sh examples/ch584
#
# Each test is flashed with SDI print on and its output captured until it
# reports a result (or SECS seconds pass). The tests are the binaries in
# src/bin that print PASS/FAIL lines, unless named on the command line.
#
# Environment:
#   RUNNER      flash-and-watch command; cargo appends the ELF path. Default:
#               the crate's .cargo/config.toml runner (wlink ... flash) plus
#               --enable-sdi-print --watch-serial.
#   CARGO_ARGS  extra cargo arguments, e.g. executor features.
#   SECS        per-test time limit (default 40).
#   OUT         directory for the logs (default target/hw-test).
#   TRIES       attempts per test when nothing is captured (default 3); the
#               SDI console sometimes fails to start.
#
# The exit status is the number of tests that failed or reported nothing.

set -u

dir=${1:?usage: $0 <crate dir> [test ...]}
shift
cd "$dir" || exit 1

if [ -z "${RUNNER:-}" ]; then
    RUNNER=$(sed -n 's/^runner *= *"\(.*\)"/\1/p' .cargo/config.toml | head -n 1)
    [ -n "$RUNNER" ] || { echo "no runner in $dir/.cargo/config.toml; set RUNNER" >&2; exit 1; }
    RUNNER="$RUNNER --enable-sdi-print --watch-serial"
fi
CARGO_ARGS=${CARGO_ARGS:-}
SECS=${SECS:-40}
TRIES=${TRIES:-3}
OUT=${OUT:-target/hw-test}
mkdir -p "$OUT"

target=$(sed -n 's/^target *= *"\(.*\)"/\1/p' .cargo/config.toml | head -n 1)
runner_var=CARGO_TARGET_$(echo "$target" | tr 'a-z-' 'A-Z_')_RUNNER

if [ $# -gt 0 ]; then
    tests="$*"
else
    tests=$(grep -l 'PASS' src/bin/*.rs | sed 's|.*/||; s|\.rs$||' | sort)
fi

# The line that ends a test's run.
done_re() {
    case $1 in
        wdg) echo 'WDG: done' ;;
        # Checks once, then prints a timestamp every 500 ms.
        embassy_time) echo 'now [0-9]+ us' ;;
        *) echo '[0-9]+ pass(ed)?, [0-9]+ fail(ed)?' ;;
    esac
}

# Has the test finished? embassy_time: once a few timestamps are in.
finished() {
    case $t in
        embassy_time) [ "$(grep -aEc "$re" "$log")" -ge 4 ] ;;
        *) grep -aEq "$re" "$log" ;;
    esac
}

# Run one attempt, stopping once the result is in.
run_once() {
    env "$runner_var=$RUNNER" timeout "$SECS" cargo run -q --release $CARGO_ARGS --bin "$t" >"$log" 2>&1 &
    pid=$!
    while kill -0 $pid 2>/dev/null; do
        finished && break
        sleep 1
    done
    # `timeout` passes the signal on to the runner.
    kill $pid 2>/dev/null
    wait $pid 2>/dev/null
}

bad=0
printf '%-14s %5s %5s  %s\n' test pass fail result
for t in $tests; do
    log=$OUT/$t.log
    if ! cargo build -q --release $CARGO_ARGS --bin "$t" >"$log" 2>&1; then
        printf '%-14s %5s %5s  %s\n' "$t" - - "BUILD FAILED (see $log)"
        bad=$((bad + 1))
        continue
    fi
    re=$(done_re "$t")
    i=0
    while [ $i -lt "$TRIES" ]; do
        i=$((i + 1))
        run_once
        grep -aq 'PASS\|FAIL' "$log" && break
    done
    # Prefer the test's own summary; otherwise count the lines.
    sum=$(grep -aEo '[0-9]+ pass(ed)?, [0-9]+ fail(ed)?' "$log" | tail -n 1)
    if [ -n "$sum" ]; then
        p=$(echo "$sum" | sed 's/^\([0-9]*\).*/\1/')
        f=$(echo "$sum" | sed 's/.* \([0-9]*\) fail.*/\1/')
    else
        p=$(grep -ac 'PASS' "$log")
        f=$(grep -ac 'FAIL' "$log")
    fi
    if ! finished; then
        result="INCOMPLETE (see $log)"
        bad=$((bad + 1))
    elif [ "$f" -gt 0 ] || [ "$p" -eq 0 ]; then
        result="FAILED: $(grep -a 'FAIL' "$log" | sed 's/^[0-9: .-]*//' | tr '\n' '|')"
        bad=$((bad + 1))
    else
        result=ok
    fi
    printf '%-14s %5s %5s  %s\n' "$t" "$p" "$f" "$result"
done
exit $bad

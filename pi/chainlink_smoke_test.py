#!/usr/bin/env python3
"""
Chainlink Driver smoke test for a Raspberry Pi 4 -- steps all six modules at once
so the peak motor-supply current can be measured.

Wiring (Chainlink "Input" 2x4 header -> Pi, BCM numbering):
    1 CLOCK      -> GPIO21 (pin 40)
    3 MOTOR      -> GPIO20 (pin 38)
    5 SENSOR_OUT -> GPIO19 (pin 35)
    7 LATCH      -> GPIO16 (pin 36)
    even pins    -> GND
    "Logic 3.3-5V" terminal -> Pi 3V3 (pin 1/17)   -- NOT 5V
    "Motor" terminal        -> bench supply V+, supply GND shared with the Pi

The bit layout mirrors firmware/src/spi_io_config.h (CHAINLINK, NUM_MODULES=6):
4 bytes are shifted out MSB-first, byte 0 first, so byte 3 lands in the
74HC595 nearest the input header. One LATCH line drives both the 595 RCLK
(rising edge = outputs update) and the 74HC165 SH/LD (low = parallel load,
high = shift), so after latching high we clock in one sensor byte.

Usage (needs python3-lgpio, preinstalled on Raspberry Pi OS Bookworm):
    python3 chainlink_smoke_test.py                 # step all 6 for 10 s
    python3 chainlink_smoke_test.py -t 30 -p 1800   # 30 s at 1.8 ms/step
    python3 chainlink_smoke_test.py --hold 5        # hold coils on 5 s (static worst case)
    python3 chainlink_smoke_test.py --modules 0 3   # only modules A and D
"""

import argparse
import signal
import sys
import time

import lgpio

PIN_CLK = 21
PIN_DATA = 20
PIN_SENSOR = 19
PIN_LATCH = 16

NUM_MODULES = 6
MOTOR_BYTES = 4

# (byte index, bit shift) of each module's 4 coil bits -- from initialize_modules()
MOTOR_SLOT = [(3, 0), (3, 4), (2, 0), (1, 4), (0, 0), (0, 4)]
# (byte index, bit mask) of each module's LED -- from chainlink_set_led()
LED_SLOT = [(2, 1 << 4), (2, 1 << 5), (2, 1 << 6), (1, 1 << 0), (1, 1 << 1), (1, 1 << 2)]

# Two-phase-on full stepping, same as the firmware (two coils energized per step)
STEP_PATTERN = [0b1100, 0b0110, 0b0011, 0b1001]

# Firmware starts at 10 ms/step and accelerates to ~1.6 ms/step
START_PERIOD_US = 10000
RAMP_STEPS = 72


class Chainlink:
    def __init__(self):
        self.h = lgpio.gpiochip_open(0)
        for pin in (PIN_CLK, PIN_DATA, PIN_LATCH):
            lgpio.gpio_claim_output(self.h, pin, 0)
        lgpio.gpio_claim_input(self.h, PIN_SENSOR)

    def io(self, motor_bytes):
        """Shift out motor bytes, latch them, and read back one sensor byte."""
        h = self.h
        write = lgpio.gpio_write
        write(h, PIN_LATCH, 0)
        for byte in motor_bytes:
            for bit in range(7, -1, -1):
                write(h, PIN_DATA, (byte >> bit) & 1)
                write(h, PIN_CLK, 1)
                write(h, PIN_CLK, 0)
        # Rising edge: 595 outputs update, 165 freezes its inputs for shifting
        write(h, PIN_LATCH, 1)
        sensors = 0
        for _ in range(8):
            sensors = (sensors << 1) | lgpio.gpio_read(h, PIN_SENSOR)
            write(h, PIN_CLK, 1)
            write(h, PIN_CLK, 0)
        return sensors

    def all_off(self):
        self.io([0] * MOTOR_BYTES)

    def close(self):
        try:
            self.all_off()
        finally:
            lgpio.gpiochip_close(self.h)


def build_frame(modules, phase, leds=0):
    buf = [0] * MOTOR_BYTES
    for m in modules:
        idx, shift = MOTOR_SLOT[m]
        buf[idx] |= STEP_PATTERN[phase] << shift
    for m in range(NUM_MODULES):
        if leds & (1 << m):
            idx, mask = LED_SLOT[m]
            buf[idx] |= mask
    return buf


def wait_until(deadline_ns):
    while time.perf_counter_ns() < deadline_ns:
        pass


def run_stepping(link, modules, seconds, period_us):
    print(f"Stepping modules {[chr(65 + m) for m in modules]} for {seconds:.1f} s "
          f"at {period_us} us/step (~{60e6 / (period_us * 2048):.1f} RPM)")
    phase = 0
    leds = 0
    last_home = 0
    home_hits = [0] * NUM_MODULES
    overruns = 0
    worst_io_us = 0
    steps = 0

    start = time.perf_counter_ns()
    end = start + int(seconds * 1e9)
    deadline = start
    while deadline < end:
        # Linear ramp from START_PERIOD_US down to the target period
        if steps < RAMP_STEPS:
            p = START_PERIOD_US + (period_us - START_PERIOD_US) * steps // RAMP_STEPS
        else:
            p = period_us
        deadline += p * 1000

        t0 = time.perf_counter_ns()
        sensors = link.io(build_frame(modules, phase, leds))
        worst_io_us = max(worst_io_us, (time.perf_counter_ns() - t0) // 1000)

        # Home sensor bit set = flap at home; light the module's LED and count rising edges
        home = sensors & 0x3F
        for m in range(NUM_MODULES):
            if home & ~last_home & (1 << m):
                home_hits[m] += 1
        last_home = home
        leds = home

        phase = (phase + 1) % 4
        steps += 1
        if time.perf_counter_ns() > deadline:
            overruns += 1
        wait_until(deadline)

    link.all_off()
    revs = steps / 2048
    print(f"Done: {steps} steps (~{revs:.2f} rev), worst shift-register IO {worst_io_us} us, "
          f"{overruns} late steps")
    print("Home sensor hits per module (expect ~1 per revolution on connected modules):")
    print("  " + "  ".join(f"{chr(65 + m)}={home_hits[m]}" for m in range(NUM_MODULES)))


def run_hold(link, modules, seconds):
    print(f"Holding phase AB on modules {[chr(65 + m) for m in modules]} for {seconds:.1f} s "
          "(static worst-case current)")
    link.io(build_frame(modules, 0))
    time.sleep(seconds)
    link.all_off()
    print("Coils off.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-t", "--time", type=float, default=10.0, help="seconds to step (default 10)")
    ap.add_argument("-p", "--period", type=int, default=2000,
                    help="microseconds per step after ramp-up (default 2000; firmware max speed is ~1600)")
    ap.add_argument("--hold", type=float, metavar="SECONDS",
                    help="instead of stepping, hold coils energized this long (max 10)")
    ap.add_argument("--modules", type=int, nargs="+", default=list(range(NUM_MODULES)),
                    choices=range(NUM_MODULES), metavar="N", help="module indices 0-5 (A-F), default all")
    ap.add_argument("-y", "--yes", action="store_true", help="skip the 'turn on motor power' prompt")
    args = ap.parse_args()

    if args.period < 1000:
        ap.error("period below 1000 us will just stall a 28BYJ-48")
    if args.hold is not None and not 0 < args.hold <= 10:
        ap.error("--hold must be between 0 and 10 seconds")

    link = Chainlink()

    def bail(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, bail)

    try:
        link.all_off()
        link.all_off()
        print("Shift registers cleared (all coils off).")
        if not args.yes:
            input("Turn on motor power now, then press Enter... ")
        if args.hold is not None:
            run_hold(link, args.modules, args.hold)
        else:
            run_stepping(link, args.modules, args.time, args.period)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        link.close()
        print("All coils off. Safe to cut motor power.")


if __name__ == "__main__":
    sys.exit(main())

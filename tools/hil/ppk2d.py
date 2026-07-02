#!/usr/bin/env python3
"""ppk2d — Nordic PPK2 current-measurement sidecar for the HIL harness (STUB).

Protocol: **line-JSON over stdio**. Read one JSON command per line on stdin, write one JSON reply
per line on stdout. The Rust harness (`tools/hil/src/lib.rs`, `Ppk2`) drives this.

Commands
--------
    {"cmd":"on","mv":<mV>}      enable source-measure at <mV>          -> {"ok":true}
    {"cmd":"off"}               disable the supply                    -> {"ok":true}
    {"cmd":"cycle","mv":<mV>}   power-cycle (off, settle, on) at <mV>  -> {"ok":true}
    {"cmd":"avg","ms":<ms>}     average current over <ms>              -> {"ua":<microamps>}

Why a sidecar (not a Rust PPK2 driver): the PPK2 host protocol + calibration live in Nordic's
Python tooling (`ppk2-api`); wrapping it in a line-JSON process keeps that dependency out of the
Rust harness and lets the confounder handling live in one documented place.

The three known confounders this sidecar is responsible for (see the low-power measurement notes):

 1. **Debug-domain residual (+~200 µA).** A debug probe (J-Link/ST-Link) attached over SWD leaves
    the STM32 debug power domain energised, adding ~200 µA that never goes away until a clean
    power cycle. So `avg` ALWAYS power-cycles first (or requires a prior `cycle`) — never measure
    without cycling.
 2. **~2 V regulator/brown-out knee.** Measured at 3 V the deep-sleep current can read
    artificially low (below the knee the part behaves differently); the honest STOP floor is taken
    at **1.8 V**. This sidecar refuses `on`/`cycle`/`avg` above a sane supply ceiling and the
    harness fixture pins `supply_mv = 1800`.
 3. **PPK2 CDC desync during SWD traffic.** While probe-rs is hammering SWD, the PPK2's USB CDC can
    desync and report bursts of tens of mA of garbage. So this sidecar NEVER samples mid-flash and
    REJECTS any averaged reading above 50 mA as noise (the Rust side enforces the same ceiling).

THIS IS A STUB: it does not open a real PPK2. It models the protocol + the confounder *policy* so
the harness compiles and its plumbing can be unit-tested; wire in `ppk2-api` where marked to drive
real hardware. The hardware HIL tests are `#[ignore]`d, so nothing here runs in CI.
"""

import json
import sys
import time

# Sanity ceilings (mirror the Rust `PPK2_SANE_MAX_UA`).
SANE_MAX_UA = 50_000.0  # >50 mA averaged => CDC desync (confounder #3), never a valid STOP floor.
MAX_SUPPLY_MV = 3600    # never source above the board's abs-max; the bench uses 1800 (knee, #2).

# Settle time after toggling the supply, so the debug-domain residual (confounder #1) fully
# discharges before a measurement is taken.
CYCLE_SETTLE_S = 0.30


class Ppk2Stub:
    """Stand-in for a real PPK2 (via ppk2-api). Tracks whether the supply is on and enforces the
    confounder policy; returns a plausible deep-sleep floor for `avg`."""

    def __init__(self):
        self.on = False
        self.mv = 0
        self.cycled_since_measure = False

    def set_on(self, mv):
        if mv <= 0 or mv > MAX_SUPPLY_MV:
            raise ValueError(f"supply {mv} mV out of range (0, {MAX_SUPPLY_MV}] — see confounder #2")
        # TODO(real hw): ppk2.set_source_voltage(mv); ppk2.toggle_DUT_power("ON");
        #                ppk2.start_measuring()
        self.on = True
        self.mv = mv

    def set_off(self):
        # TODO(real hw): ppk2.toggle_DUT_power("OFF"); ppk2.stop_measuring()
        self.on = False

    def cycle(self, mv):
        # Confounder #1: a clean power cycle clears the debug-domain residual current. Always the
        # preamble to a measurement.
        self.set_off()
        time.sleep(CYCLE_SETTLE_S)
        self.set_on(mv)
        self.cycled_since_measure = True

    def avg_ua(self, ms):
        if not self.on:
            raise RuntimeError("avg requested with supply off — `on`/`cycle` first")
        if not self.cycled_since_measure:
            # Enforce confounder #1: refuse a measurement that wasn't preceded by a power cycle.
            raise RuntimeError("avg requested without a prior power-cycle (confounder #1: debug "
                               "domain adds ~200 µA until cycled) — call `cycle` first")
        # TODO(real hw): samples = ppk2.get_samples() collected over `ms`; return their mean in µA.
        # Stub value: a believable Core-Module STOP floor at 1.8 V with the FTDI unplugged.
        ua = 12.0
        # Guard mirrors the Rust ceiling: a real desynced read would exceed this and must be
        # rejected as noise rather than reported (confounder #3).
        if ua > SANE_MAX_UA:
            raise RuntimeError(f"averaged {ua} µA > {SANE_MAX_UA} µA — CDC desync mid-flash (#3)")
        # A fresh measurement "consumes" the cycle: the next avg must cycle again.
        self.cycled_since_measure = False
        return ua


def main():
    dev = Ppk2Stub()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            cmd = req.get("cmd")
            if cmd == "on":
                dev.set_on(int(req["mv"]))
                reply = {"ok": True}
            elif cmd == "off":
                dev.set_off()
                reply = {"ok": True}
            elif cmd == "cycle":
                dev.cycle(int(req["mv"]))
                reply = {"ok": True}
            elif cmd == "avg":
                reply = {"ua": dev.avg_ua(int(req.get("ms", 100)))}
            else:
                reply = {"error": f"unknown cmd {cmd!r}"}
        except Exception as e:  # surface any error as a JSON line the harness can see
            reply = {"error": str(e)}
        sys.stdout.write(json.dumps(reply) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Check that the arm controller can be driven the way the tracker drives it.

Read-only by default:
  - telnet (TCP 23) reachable, banner
  - firmware is FluidNC ($I), its kinematics and arm lengths ($CD)
  - status reports parse (MPos or WPos+WCO), round-trip latency of '?'
  - machine state (Alarm = not homed yet)

With --jog-test D it also MOVES THE ARM (asks first):
  1. $J=G91 X+D, wait Idle, measure; then back
  2. a slow long jog, cancelled with 0x85 after 0.3 s: the arm must stop
     short of the target and return to Idle
  3. return to the start position

  ./fluidnc_check.py 192.168.0.1
  ./fluidnc_check.py 192.168.0.1 --jog-test 10 --feed 600
Standard library only.
"""

import argparse
import re
import socket
import statistics
import sys
import time


class Telnet:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port), timeout=3)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.settimeout(0.05)
        self.buf = b""
        self.wco = (0.0, 0.0, 0.0)

    def lines(self, duration):
        """Yield lines received within duration seconds."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            try:
                chunk = self.s.recv(4096)
                if not chunk:
                    raise ConnectionError("controller closed the connection")
                self.buf += chunk
            except socket.timeout:
                pass
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                line = line.strip(b"\r").decode(errors="replace")
                if line:
                    yield line

    def rt(self, byte):
        self.s.sendall(bytes([byte]) if isinstance(byte, int) else byte.encode())

    def command(self, cmd, timeout=3.0):
        """Send a line, collect output up to ok/error."""
        self.s.sendall((cmd + "\n").encode())
        out = []
        for line in self.lines(timeout):
            if line == "ok" or line.startswith("error:"):
                return out, line
            if not line.startswith("<"):
                out.append(line)
        return out, None

    def status(self, timeout=1.0):
        """Ask '?' and return (state, mpos, seconds)."""
        t0 = time.monotonic()
        self.rt("?")
        for line in self.lines(timeout):
            if line.startswith("<"):
                return self.parse(line) + (time.monotonic() - t0,)
        return None, None, None

    def parse(self, line):
        fields = line.strip("<>").split("|")
        state = fields[0].split(":")[0]
        pos, is_w = None, False
        for f in fields[1:]:
            k, _, v = f.partition(":")
            if k in ("MPos", "WPos"):
                pos, is_w = [float(x) for x in v.split(",")[:3]], k == "WPos"
            elif k == "WCO":
                self.wco = tuple(float(x) for x in v.split(",")[:3])
        if pos and is_w:
            pos = [p + w for p, w in zip(pos, self.wco)]
        return state, pos

    def wait_state(self, want, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            st, pos, _ = self.status()
            if st == want:
                return pos
            time.sleep(0.05)
        return None


def ok(msg):
    print(f"  [ok]   {msg}")


def warn(msg):
    print(f"  [WARN] {msg}")


def fail(msg):
    print(f"  [FAIL] {msg}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host")
    ap.add_argument("--port", type=int, default=23)
    ap.add_argument("--jog-test", type=float, metavar="D", help="also jog +D/-D on the axis (MOVES THE ARM)")
    ap.add_argument("--axis", default="X", choices=["X", "Y"])
    ap.add_argument("--feed", type=float, default=600, help="jog feed for the test, mm/min (default 600)")
    ap.add_argument("--yes", action="store_true", help="don't ask before moving")
    a = ap.parse_args()

    print(f"connecting to {a.host}:{a.port}")
    try:
        t = Telnet(a.host, a.port)
    except OSError as e:
        fail(f"telnet: {e}  (is FluidNC on WiFi with telnet enabled? $Telnet/Enable)")
        sys.exit(1)
    ok("telnet connected")
    banner = list(t.lines(0.8))
    if banner:
        print("  banner: " + " / ".join(banner))

    info, res = t.command("$I")
    text = " ".join(info)
    m = re.search(r"FluidNC\s+v?([\w.\-]+)", text)
    if m:
        ok(f"FluidNC {m.group(1)}")
    else:
        warn(f"firmware not identified as FluidNC: {text or res}")

    dump, res = t.command("$CD", timeout=5.0)
    if res == "ok" and dump:
        cfg = "\n".join(dump)
        km = re.search(r"^kinematics:\s*\n\s+(\w+):", cfg, re.M)
        kin = km.group(1) if km else "?"
        if kin.upper() == "SCARA":
            ok("kinematics SCARA: jogs are cartesian mm  -> config.h JOG_CARTESIAN 1")
            for key in ("arm1_mm", "arm2_mm", "elbow_positive", "r_min_mm", "r_max_mm"):
                km = re.search(rf"^\s+{key}:\s*(\S+)", cfg, re.M)
                if km:
                    print(f"         {key}: {km.group(1)}")
        else:
            warn(f"kinematics {kin}: X/Y jog the joints (degrees) -> config.h JOG_CARTESIAN 0")
        for ax in "xyz":
            sm = re.search(rf"^  {ax}:\s*\n(?:    .*\n)*?    soft_limits:\s*(\w+)", cfg + "\n", re.M)
            if sm:
                print(f"         axis {ax} soft_limits: {sm.group(1)}")
        if "stallguard" in cfg.lower():
            ok("TMC StallGuard configured (sensorless homing)")
    else:
        warn("could not read config ($CD)")

    lat, state, pos = [], None, None
    for _ in range(20):
        st, p, dt = t.status()
        if st:
            state, pos = st, p
            lat.append(dt * 1000)
        time.sleep(0.05)
    if not lat:
        fail("no status report in reply to '?'")
        sys.exit(1)
    ok(f"status reports: state {state}, MPos {pos}")
    ok(f"'?' round trip: median {statistics.median(lat):.0f} ms, max {max(lat):.0f} ms over {len(lat)}")
    if max(lat) > 150:
        warn("latency spikes: check WiFi power save / signal; the tracker needs < ~100 ms")

    if state == "Alarm":
        warn("state Alarm: home the machine ($H) before jogging; the tracker refuses to run in Alarm")

    if not a.jog_test:
        print("\nread-only checks done. Add --jog-test 10 to test jogging and jog cancel.")
        return
    if state != "Idle":
        fail(f"jog test needs Idle, state is {state}")
        sys.exit(1)

    d, ax, f = a.jog_test, a.axis, a.feed
    if not a.yes:
        r = input(f"\nThe arm will jog {ax} +{d} and back, then a cancelled jog. Area clear? [y/N] ")
        if r.strip().lower() != "y":
            return
    i = "XY".index(ax)
    start = pos

    # 1. plain jog
    _, res = t.command(f"$J=G91 G21 {ax}{d:.3f} F{f:.0f}")
    if res != "ok":
        fail(f"jog rejected: {res}")
        sys.exit(1)
    p = t.wait_state("Idle", d / f * 60 + 10)
    if p is None:
        fail("did not return to Idle after jog")
        t.rt(0x85)
        sys.exit(1)
    moved = p[i] - start[i]
    (ok if abs(moved - d) < 0.05 * abs(d) + 0.1 else warn)(f"jog {ax}+{d}: moved {moved:.3f}")
    t.command(f"$J=G91 G21 {ax}{-d:.3f} F{f:.0f}")
    t.wait_state("Idle", d / f * 60 + 10)

    # 2. jog cancel
    long = 5 * d
    t0 = time.monotonic()
    t.command(f"$J=G91 G21 {ax}{long:.3f} F{f:.0f}")
    time.sleep(0.3)
    t.rt(0x85)
    p = t.wait_state("Idle", 5)
    if p is None:
        fail("jog cancel (0x85) did not bring the machine back to Idle")
        sys.exit(1)
    moved = p[i] - start[i]
    if moved < 0.8 * long:
        ok(f"jog cancel works: stopped after {moved:.2f} of {long:.0f} in {time.monotonic() - t0:.2f}s")
    else:
        fail(f"jog cancel did not stop the move ({moved:.2f} of {long:.0f})")
    # 3. back to start
    t.command(f"$J=G91 G21 {ax}{start[i] - p[i]:.3f} F{f:.0f}")
    p = t.wait_state("Idle", long / f * 60 + 10)
    print(f"  back at {p}")


if __name__ == "__main__":
    main()

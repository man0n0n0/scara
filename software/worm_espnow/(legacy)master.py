####
# MASTER / COGNITION  -  ESP32-S2 mini
# Broadcasts the 4 axis targets in ONE packet so every node acts
# on the same radio frame, and accepts serial commands so each
# slave can be driven independently from a terminal.
#
# Packet: <B B 4h H> = cmd, mask, x,y,a,b (0.1 mm), feedrate (mm/min) = 12 bytes
#   cmd 0 MOVE    masked axes follow their target (0.1 mm units)
#   cmd 1 HOME    masked axes re-home locally
#   cmd 2 STOP    masked axes go limp (sticky)
#   cmd 3 ENABLE  masked axes wake up
####
import sys
import select
import struct
import network
import espnow
from time import sleep_ms, ticks_ms, ticks_diff, ticks_add

CHANNEL   = 1
BROADCAST = b'\xff\xff\xff\xff\xff\xff'
SEND_MS   = 100          # heartbeat period: also feeds the nodes' watchdog
ALL       = 0x0F
FEED      = 666         # mm/min, same units as cognition's F_TRACKING

MOVE, HOME, STOP, ENABLE = 0, 1, 2, 3

HEAD = 50
TAIL = 50
BORDER   = (HEAD, HEAD, TAIL, TAIL)   # sweep endpoints, mm either side of centre
ACCEL    = 50       # mm/s^2, same as the nodes' ACCEL_MM_S2
DWELL_MS = 300       # pause at each border before turning back

HELP = """
m <axis> <mm> [f]       move one axis      (axis 0=x 1=y 2=a 3=b)
M <x> <y> <a> <b> [f]   move all four      (f = mm/min)
h [axis]                home  (all if omitted)
s [axis]                stop / go limp
e [axis]                enable
a                       toggle auto sweep
?                       status
"""

'''radio init'''
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()         # stay off any AP, ESP-NOW only
try:
    sta.config(channel=CHANNEL)
except Exception as err:
    print("channel set failed:", err)

e = espnow.ESPNow()
e.active(True)
e.add_peer(BROADCAST)

print("master up, mac:", sta.config('mac'))
print(HELP)

'''state'''
targets = [0, 0, 0, 0]           # mm, resent every SEND_MS
feed    = FEED                   # mm/min
auto    = False                  # demo sweep off until asked
side    = [0, 0, 0, 0]           # sweep: which border each axis heads for
flip_at = [0, 0, 0, 0]           # sweep: ticks_ms when that axis turns back

def leg_ms(span, f):
    """Time for a node to cover span mm at f mm/min, ramps included."""
    v = f / 60.0
    return int((span / v + v / ACCEL) * 1000) + DWELL_MS

def start_sweep():
    # sides are pre-flipped: the first pass of the loop turns them round.
    # y lags x by half a leg, b always opposes a.
    now = ticks_ms()
    for i, d in enumerate((-1, -1, -1, 1)):
        side[i] = d
        flip_at[i] = now
    flip_at[1] = ticks_add(now, leg_ms(2 * HEAD, FEED) // 2)

def send(cmd, mask, tg, f):
    # targets travel as 0.1 mm units: whole-mm packing made a visible
    # staircase at slow speeds (1 mm = 200 steps)
    e.send(BROADCAST, struct.pack("<BB4hH", cmd, mask,
                                  int(tg[0] * 10), int(tg[1] * 10),
                                  int(tg[2] * 10), int(tg[3] * 10),
                                  int(f)), False)

def mask_of(arg):
    """No axis given -> all four."""
    return ALL if arg is None else (1 << int(arg))

'''non-blocking serial'''
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
line = ""

def handle(cmd_line):
    global auto, feed
    p = cmd_line.split()
    if not p:
        return
    c = p[0]

    if c == "m" and len(p) >= 3:
        ax = int(p[1])
        targets[ax] = float(p[2])
        feed = int(p[3]) if len(p) > 3 else FEED
        print("axis", ax, "->", targets[ax], "mm at", feed, "mm/min")

    elif c == "M" and len(p) >= 5:
        for i in range(4):
            targets[i] = float(p[1 + i])
        feed = int(p[5]) if len(p) > 5 else FEED
        print("all ->", targets, "at", feed, "mm/min")

    elif c == "h":
        ax = p[1] if len(p) > 1 else None
        send(HOME, mask_of(ax), targets, feed)
        print("home", ax if ax else "all")

    elif c == "s":
        ax = p[1] if len(p) > 1 else None
        send(STOP, mask_of(ax), targets, feed)
        print("stop", ax if ax else "all")

    elif c == "e":
        ax = p[1] if len(p) > 1 else None
        send(ENABLE, mask_of(ax), targets, feed)
        print("enable", ax if ax else "all")

    elif c == "a":
        auto = not auto
        if auto:
            start_sweep()
        print("auto sweep", "on" if auto else "off")

    elif c == "?":
        print("targets", targets, "feed", feed, "auto", auto)

    else:
        print(HELP)

'''execution'''
while True:
    # --- serial, one char at a time, never blocks ---
    while poll.poll(0):
        ch = sys.stdin.read(1)
        if ch in ("\n", "\r"):
            if line:
                try:
                    handle(line)
                except Exception as err:
                    print("bad command:", err)
                line = ""
        else:
            line += ch

    # --- optional demo sweep, overwrites targets ---
    # only the borders are sent: a goal that creeps forward every packet
    # keeps the node inside its braking distance, so it never cruises.
    # A fixed far goal lets it ramp up once, cruise, and ramp down once.
    if auto:
        feed = FEED
        now = ticks_ms()
        for i in range(4):
            if ticks_diff(now, flip_at[i]) >= 0:
                side[i] = -side[i]
                flip_at[i] = ticks_add(now, leg_ms(2 * BORDER[i], feed))
            targets[i] = side[i] * BORDER[i]

    # --- heartbeat: absolute targets, so an unchanged axis simply doesn't move
    send(MOVE, ALL, targets, feed)
    sleep_ms(SEND_MS)

####
# MASTER / COGNITION  -  ESP32-S2 mini
# Broadcasts the 4 axis targets in ONE packet so every node acts
# on the same radio frame. Talks to the nodes over ESP-NOW only; on UDP
# LIGHT_PORT it hears the light info (sets the feed) and broadcasts
# "human 1" / "human 0" from the LD2450 radar (sweep on / motors off).
# Serial commands drive each slave independently from a terminal.
#
# Packet: <B B 4h H> = cmd, mask, x,y,a,b (0.1 mm), feedrate (mm/min) = 12 bytes
#   cmd 0 MOVE    masked axes follow their target (0.1 mm units)
#   cmd 1 HOME    masked axes re-home locally
#   cmd 2 STOP    masked axes go limp (sticky)
#   cmd 3 ENABLE  masked axes wake up
####
import sys
import select
import socket
import struct
import random
import network
import espnow
from machine import UART, Pin
from time import sleep_ms, ticks_ms, ticks_diff, ticks_add

CHANNEL   = 1
BROADCAST = b'\xff\xff\xff\xff\xff\xff'
SEND_MS   = 100          # heartbeat period: also feeds the nodes' watchdog
ALL       = 0x0F
FEED      = 2000         # mm/min, same units as cognition's F_TRACKING
NODE_MAX_FEED = 1800     # mm/min, nodes' MAX_SPS / STEPS_PER_MM * 60

# --- Wi-Fi: only to hear the light info -----------------------------------
# ESP-NOW and the STA connection share ONE radio, so they live on ONE
# channel. Associating with an AP hands channel control to that AP, so the
# AP MUST be locked to CHANNEL (= the nodes' channel) or ESP-NOW packets
# never reach them. Set your router to channel 1, or change CHANNEL on every
# node to match the AP.
# The light commands are sent to the light controller; the master only hears
# them if the sender broadcasts (x.x.x.255) or also sends to the master's IP.
WIFI_SSID = "ok"
WIFI_PASS = "123456789"
LIGHT_PORT = 4210        # light controller's port: "light 1" -> FEED_LIGHT_ON
FEED_LIGHT_ON  = 6666    # mm/min while the light is on
FEED_LIGHT_OFF = 666     # mm/min while it is off

# --- HLK-LD2450 radar: UART1 256000 8N1, ~10 frames/s ----------------------
# Any target -> "human 1" and the sweep at once. Nobody for HUMAN_HOLD_MS ->
# "human 0" and the motors cut. No frames at all counts as nobody.
RADAR_TX, RADAR_RX = 3, 4    # GPIO3 -> sensor RX, GPIO4 <- sensor TX
HUMAN_HOLD_MS = 13000
HUMAN_MAX_MM  = 3000         # targets farther than this from the sensor are ignored
BUTTON_PIN    = 9            # to GND, active low: pressed counts as a human
DATA_HEAD = b"\xaa\xff\x03\x00"   # data frame header
DATA_TAIL = b"\x55\xcc"          # data frame tail
FRAME_LEN = 30

MOVE, HOME, STOP, ENABLE = 0, 1, 2, 3

HEAD = 50
TAIL = 50
BORDER   = (HEAD, HEAD, TAIL, TAIL)   # sweep endpoints, mm either side of centre
SWEEP_MIN = 0.67     # each stroke ends at random within SWEEP_MIN..1 x BORDER
ACCEL    = 100       # mm/s^2, same as the nodes' ACCEL_MM_S2
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

'''radio init  -  associate with the AP, THEN start ESP-NOW on its channel'''
sta = network.WLAN(network.STA_IF)
sta.active(True)
try:
    sta.config(pm=sta.PM_NONE)   # power save drops and delays ESP-NOW frames
except Exception as err:
    print("power save off failed:", err)
if not sta.isconnected():
    sta.connect(WIFI_SSID, WIFI_PASS)
    t0 = ticks_ms()
    while not sta.isconnected() and ticks_diff(ticks_ms(), t0) < 10000:
        sleep_ms(100)

if sta.isconnected():
    print("wifi up:", sta.ifconfig()[0])
    ch = sta.config('channel')
    if ch != CHANNEL:
        # nodes are fixed to CHANNEL; a mismatched AP channel = silent nodes
        print("WARNING: AP on channel", ch, "but nodes expect", CHANNEL,
              "- lock the router to channel", CHANNEL)
else:
    print("WARNING: wifi not connected; no light info, ESP-NOW only")

e = espnow.ESPNow()
e.active(True)
e.add_peer(BROADCAST)

'''non-blocking UDP: listens in on the light commands, only the feedrate
   follows them; the light controller itself still answers the sender.
   The same socket broadcasts the human state to every client on the port.'''
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.setblocking(False)
udp.bind(('0.0.0.0', LIGHT_PORT))
try:
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except Exception:
    pass                         # not every port has it; lwIP sends anyway

'''radar: timeout=0 so a read never stalls the heartbeat'''
radar = UART(1, baudrate=256000, bits=8, parity=None, stop=1,
             tx=RADAR_TX, rx=RADAR_RX, rxbuf=512, timeout=0)
radar_buf = bytearray()
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

print("master up, mac:", sta.config('mac'), "udp port:", LIGHT_PORT)
print(HELP)

'''state'''
targets = [0, 0, 0, 0]           # mm, resent every SEND_MS
feed    = FEED                   # mm/min
base_feed = FEED                 # default feed, set by the light commands
auto    = False                  # demo sweep off until asked
side    = [0, 0, 0, 0]           # sweep: which border each axis heads for
flip_at = [0, 0, 0, 0]           # sweep: ticks_ms when that axis turns back
amp     = [0, 0, 0, 0]           # sweep: mm from centre of the current end point
leg_span = [0, 0, 0, 0]          # sweep: mm the current stroke covers
human  = False                  # radar: someone seen within HUMAN_HOLD_MS
last_seen = ticks_ms()           # radar: last frame with a target in it

def leg_ms(span, f):
    """Time for a node to cover span mm at f mm/min, ramps included.
       The nodes cap the speed, so a faster f doesn't shorten the leg."""
    v = min(f, NODE_MAX_FEED) / 60.0
    return int((span / v + v / ACCEL) * 1000) + DWELL_MS

def start_sweep():
    # sides are pre-flipped: the first pass of the loop turns them round.
    # y starts half a leg behind x, b starts opposite a; the random stroke
    # lengths then let all four drift apart.
    # The axes may sit anywhere up to the far end, so the first stroke is
    # timed for the longest case: a longer wait at the end, never an early turn.
    now = ticks_ms()
    for i, d in enumerate((-1, -1, -1, 1)):
        side[i] = d
        flip_at[i] = now
        amp[i] = BORDER[i]
        leg_span[i] = 2 * BORDER[i]
    flip_at[1] = ticks_add(now, leg_ms(2 * HEAD, base_feed) // 2)

def send(cmd, mask, tg, f):
    # targets travel as 0.1 mm units: whole-mm packing made a visible
    # staircase at slow speeds (1 mm = 200 steps)
    e.send(BROADCAST, struct.pack("<BB4hH", cmd, mask,
                                  int(tg[0] * 10), int(tg[1] * 10),
                                  int(tg[2] * 10), int(tg[3] * 10),
                                  int(f)), False)

def mask_of(arg):
    """No axis given -> all four."""
    if arg is None:
        return ALL
    ax = int(arg)
    if not 0 <= ax <= 3:
        raise ValueError("axis must be 0-3")
    return 1 << ax

'''non-blocking serial'''
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
line = ""

def stop_sweep():
    """A manual move would be overwritten by the sweep on the next pass."""
    global auto
    if auto:
        auto = False
        print("auto sweep off")

def handle(cmd_line):
    global auto, feed
    p = cmd_line.split()
    if not p:
        return
    c = p[0]

    if c == "m" and len(p) >= 3:
        ax = int(p[1])
        mask_of(ax)                      # validates the axis
        stop_sweep()
        targets[ax] = float(p[2])
        feed = int(p[3]) if len(p) > 3 else base_feed
        print("axis", ax, "->", targets[ax], "mm at", feed, "mm/min")

    elif c == "M" and len(p) >= 5:
        stop_sweep()
        for i in range(4):
            targets[i] = float(p[1 + i])
        feed = int(p[5]) if len(p) > 5 else base_feed
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

def handle_light(cmd_line):
    """light 1 -> fast, light 0 -> slow. Everything else is ignored."""
    global feed, base_feed
    p = cmd_line.lower().split()
    if p == ["light", "1"]:
        base_feed = FEED_LIGHT_ON
    elif p == ["light", "0"]:
        base_feed = FEED_LIGHT_OFF
    else:
        return
    old, feed = feed, base_feed
    if auto:
        # legs in flight were timed at the old feed: stretch what is left
        # of each by the new/old leg ratio, which keeps y's lag behind x
        now = ticks_ms()
        for i in range(4):
            left = ticks_diff(flip_at[i], now)
            if left > 0:
                span = leg_span[i]
                flip_at[i] = ticks_add(now, left * leg_ms(span, feed)
                                       // leg_ms(span, old))
    print("light cmd", cmd_line, "-> feed", feed, "mm/min")
    if feed > NODE_MAX_FEED:
        print("note: nodes cap at", NODE_MAX_FEED, "mm/min")

def decode_signed(raw):
    # 16-bit value: MSB = sign flag (1 = positive). lower 15 bits = magnitude.
    return (raw - 0x8000) if (raw & 0x8000) else -raw

def radar_seen():
    """True/False from the newest complete frame, None if none arrived.
       Older frames in the buffer are skipped: no stale backlog."""
    global radar_buf
    chunk = radar.read()
    if chunk:
        radar_buf += chunk
    seen = None
    while True:
        i = radar_buf.find(DATA_HEAD)
        if i < 0:
            if len(radar_buf) > 3:       # keep tail bytes (header may be split)
                radar_buf = radar_buf[-3:]
            break
        if len(radar_buf) - i < FRAME_LEN:   # incomplete frame, wait for more
            radar_buf = radar_buf[i:]
            break
        frame = bytes(radar_buf[i:i + FRAME_LEN])
        radar_buf = radar_buf[i + FRAME_LEN:]
        if frame[28:30] != DATA_TAIL:
            continue                     # bad tail, resync on next header
        seen = False
        for k in range(3):               # 3 target slots, empty = all zero
            x, y, spd, res = struct.unpack_from("<HHHH", frame, 4 + k * 8)
            x, y, spd = decode_signed(x), decode_signed(y), decode_signed(spd)
            if (x or y or spd) and x * x + y * y <= HUMAN_MAX_MM * HUMAN_MAX_MM:
                seen = True
    return seen

def bcast_ip():
    """Subnet broadcast from the STA address, e.g. 192.168.4.255."""
    ip, mask = sta.ifconfig()[:2]
    return ".".join(str(int(a) | (~int(m) & 255))
                    for a, m in zip(ip.split("."), mask.split(".")))

def announce(msg):
    print("udp", LIGHT_PORT, "->", msg)
    if not sta.isconnected():
        return
    try:
        udp.sendto(msg.encode(), (bcast_ip(), LIGHT_PORT))
    except OSError as err:
        print("udp send failed:", err)

def set_human(on):
    """Sweep on, or motors cut, then tell every client on LIGHT_PORT.
       ESP-NOW broadcast has no ack, so ENABLE/STOP go out three times;
       the nodes treat a repeat as a no-op."""
    global human, auto
    human = on
    if on:
        for _ in range(3):
            send(ENABLE, ALL, targets, feed)
        auto = True
        start_sweep()
    else:
        auto = False
        for _ in range(3):
            send(STOP, ALL, targets, feed)
    announce("human 1" if on else "human 0")

'''execution'''
set_human(False)                 # motors off until someone shows up
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

    # --- light info, one datagram = one command, never blocks ---
    # drains the socket each pass so a burst can't lag the heartbeat
    my_ip = sta.ifconfig()[0]
    while True:
        try:
            pkt, addr = udp.recvfrom(64)
        except OSError:
            break                        # EAGAIN: nothing queued
        if not pkt:
            break
        if addr[0] == my_ip:
            continue                     # our own "human" broadcast
        print("udp", LIGHT_PORT, "from", addr, ":", pkt)
        try:
            handle_light(pkt.decode().strip())
        except Exception as err:
            print("bad light command from", addr, ":", err)

    # --- radar: on at the first target, off after HUMAN_HOLD_MS of nobody ---
    # the button stands in for a target while it is held
    seen = radar_seen() or button.value() == 0
    now = ticks_ms()
    if seen:
        last_seen = now
        if not human:
            set_human(True)
    elif human and ticks_diff(now, last_seen) >= HUMAN_HOLD_MS:
        set_human(False)

    # --- optional demo sweep, overwrites targets ---
    # only the borders are sent: a goal that creeps forward every packet
    # keeps the node inside its braking distance, so it never cruises.
    # A fixed far goal lets it ramp up once, cruise, and ramp down once.
    # Each new end point is random, SWEEP_MIN..1 x BORDER: shorter, never
    # longer. The stroke is timed on its real length, old end to new end.
    if auto:
        feed = base_feed
        now = ticks_ms()
        for i in range(4):
            if ticks_diff(now, flip_at[i]) >= 0:
                side[i] = -side[i]
                new = BORDER[i] * random.uniform(SWEEP_MIN, 1.0)
                leg_span[i] = amp[i] + new
                amp[i] = new
                flip_at[i] = ticks_add(now, leg_ms(leg_span[i], feed))
            targets[i] = side[i] * amp[i]

    # --- heartbeat: absolute targets, so an unchanged axis simply doesn't move
    send(MOVE, ALL, targets, feed)
    sleep_ms(SEND_MS)

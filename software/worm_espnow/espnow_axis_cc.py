####
# AXIS NODE  -  ESP32-C3 supermini  (one per driver)
# Follows the broadcast target for its own axis, owns its switches
# and its homing. MOVE / HOME / STOP / ENABLE are addressed by bit mask.
####
import struct
import network
import espnow
import micropython
from machine import Pin
from stepper import Stepper
from time import sleep_ms, ticks_ms, ticks_diff

'''per-node config  -  CHANGE MY_AXIS ON EACH BOARD'''
MY_AXIS = 3              # 0=x 1=y 2=a 3=b

STEP_PIN, DIR_PIN, EN_PIN = 1, 2, 0
LIM_NEG_PIN, LIM_POS_PIN  = 4, 3
HAS_POS_LIMIT = True     # an UNWIRED input floats to the pull-up and looks
                         # permanently triggered - only True once wired
INVERT_DIR    = True
SWITCH_ACTIVE = 1
HOME_DIR      = -1       # which way is "toward the negative switch"

CHANNEL      = 1
STEPS_PER_MM = 200
MAX_SPS      = 6000      # <-- MEASURE THIS. python-level stepping ceiling.
MIN_SPS      = 60
HOME_SPS     = 800
RETURN_SPS   = 3000
PULLOFF_MM   = 3
HOME_MPOS_MM = -70      # position AT the switch; 0 is the CENTRE
TRAVEL_MM    = 65        # soft limit around centre 
WATCHDOG_MS  = 600
DEBOUNCE_MS  = 25        # level must hold this long to count
IRQ_LOCKOUT  = 50

DEADBAND_STEPS = 4       # close enough: stop chasing, kill dither
ACCEL_MM_S2    = 100     # same figure the FluidNC config used
SPS_HYST       = 0.05    # only re-arm the timer on a >5% change
ACCEL_SPS2     = ACCEL_MM_S2 * STEPS_PER_MM

PACKET_LEN = 12
MOVE, HOME, STOP, ENABLE = 0, 1, 2, 3
MY_BIT = 1 << MY_AXIS

micropython.alloc_emergency_exception_buf(100)

'''init'''
led = Pin(8, Pin.OUT)
lim_neg = Pin(LIM_NEG_PIN, Pin.IN, Pin.PULL_UP)
lim_pos = Pin(LIM_POS_PIN, Pin.IN, Pin.PULL_UP)
s = Stepper(STEP_PIN, DIR_PIN, EN_PIN, invert_dir=INVERT_DIR)

sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()
try:
    sta.config(channel=CHANNEL)
except Exception as err:
    print("channel set failed:", err)

e = espnow.ESPNow()
e.active(True)

print("axis", MY_AXIS, "up  rest: neg", lim_neg.value(), " pos", lim_pos.value())

def raw(pin):
    return pin.value() == SWITCH_ACTIVE

def hit(pin, ms=DEBOUNCE_MS):
    """Active for ms straight. Noise cannot fake a hit."""
    t = ticks_ms()
    while ticks_diff(ticks_ms(), t) < ms:
        if not raw(pin):
            return False
        sleep_ms(1)
    return True

def clear(pin, ms=DEBOUNCE_MS):
    """Inactive for ms straight. hit() returns False on the first idle
       sample, so it cannot be used to confirm a release."""
    t = ticks_ms()
    while ticks_diff(ticks_ms(), t) < ms:
        if raw(pin):
            return False
        sleep_ms(1)
    return True

def pos_raw():
    return HAS_POS_LIMIT and raw(lim_pos)

def pos_hit():
    return HAS_POS_LIMIT and hit(lim_pos)

def blink(n):
    for _ in range(n):
        led.value(1); sleep_ms(60); led.value(0); sleep_ms(60)

'''limits on hardware interrupt: the ISR halts stepping at once but keeps
   the driver energised, so the axis cannot slump. Edges only fire on
   transition, so home() still reads by level.'''
limit_flag    = 0
homing_active = False
_last_irq     = 0

def _limit_isr(pin):
    global limit_flag, _last_irq
    t = ticks_ms()
    if ticks_diff(t, _last_irq) < IRQ_LOCKOUT:
        return
    _last_irq  = t
    limit_flag = 1 if pin is lim_neg else 2
    if not homing_active:
        s.stop()

TRIG = Pin.IRQ_RISING if SWITCH_ACTIVE else Pin.IRQ_FALLING
lim_neg.irq(trigger=TRIG, handler=_limit_isr)
if HAS_POS_LIMIT:
    lim_pos.irq(trigger=TRIG, handler=_limit_isr)

'''motion state: the node follows an absolute goal, it does not replay a
   move. The master resends the same goal every 100 ms, so anything that
   restarts a profile per packet judders at 10 Hz.'''
goal_steps = 0
feed_sps   = 0.0         # cruise speed from the commanded feedrate
cur_sps    = 0.0
last_sps   = -1.0
last_goal  = None
t_prev     = ticks_ms()
moving     = False
last_rx    = ticks_ms()
limp       = False       # watchdog, clears on any packet
disabled   = False       # STOP, sticky until ENABLE

def set_goal(target_mm, feed_mm_min):
    global goal_steps, feed_sps
    target_mm  = max(-TRAVEL_MM, min(TRAVEL_MM, target_mm))
    goal_steps = int(target_mm * STEPS_PER_MM)
    feed_sps   = feed_mm_min / 60.0 * STEPS_PER_MM      # mm/min -> steps/s
    if feed_sps > MAX_SPS:
        feed_sps = MAX_SPS
    elif feed_sps < MIN_SPS:
        feed_sps = MIN_SPS

'''homing: seek the negative switch, call it -max travel, then go to centre.
   Bounded only by the switch.'''
def home():
    global moving, last_rx, homing_active, limit_flag
    global goal_steps, cur_sps, last_sps, last_goal
    moving = False
    homing_active = True
    blink(2)
    s.enable(True)
    s.speed(HOME_SPS)

    if hit(lim_neg):                 # already on it: get off first
        s.free_run(-HOME_DIR)
        while not clear(lim_neg):
            sleep_ms(2)
        s.stop()

    s.free_run(HOME_DIR)             # seek
    while not hit(lim_neg):
        sleep_ms(2)
    s.stop()

    s.overwrite_pos(int(HOME_MPOS_MM * STEPS_PER_MM))
    s.target(s.get_pos())
    s.track_target()

    s.speed(HOME_SPS)                # crawl off the switch
    s.target(int((HOME_MPOS_MM + PULLOFF_MM) * STEPS_PER_MM))
    while not clear(lim_neg):
        sleep_ms(5)

    s.speed(RETURN_SPS)              # run to centre
    s.target(0)
    while s.get_pos() != 0:
        sleep_ms(5)

    goal_steps = 0
    cur_sps    = 0.0
    last_sps   = -1.0
    last_goal  = None
    limit_flag    = 0                # edges fired during the seek
    homing_active = False
    last_rx = ticks_ms()             # homing blocks; don't trip the watchdog
    blink(1)

def wake():
    global limp
    limp = False
    s.enable(True)
    s.target(s.get_pos())            # hold position: never lurch on wake
    s.track_target()

'''execution'''
home()
s.track_target()

while True:
    host, msg = e.recv(0)
    if msg and len(msg) == PACKET_LEN:
        cmd, mask, x, y, a, b, feed = struct.unpack("<BB4hH", msg)
        last_rx = ticks_ms()
        mine = mask & MY_BIT

        if cmd == ENABLE and mine:
            disabled = False
            wake()

        elif cmd == STOP and mine:
            disabled = True
            moving   = False
            s.stop()
            s.enable(False)
            led.value(0)

        elif cmd == HOME and mine and not disabled:
            if limp:
                wake()
            home()
            s.track_target()

        elif cmd == MOVE and mine and not disabled:
            if limp:
                wake()
            set_goal((x, y, a, b)[MY_AXIS] / 10.0, feed)  # 0.1 mm, mm/min

    # --- watchdog: master gone -> stop driving ---
    if not limp and not disabled and ticks_diff(ticks_ms(), last_rx) > WATCHDOG_MS:
        limp    = True
        moving  = False
        cur_sps = 0.0
        s.stop()
        s.enable(False)
        led.value(0)

    # --- limits: the ISR stopped us, confirm before re-homing ---
    if not limp and not disabled and (limit_flag or raw(lim_neg) or pos_raw()):
        limit_flag = 0
        if hit(lim_neg):
            print("axis", MY_AXIS, "neg limit -> rehome")
            moving = False
            s.stop()
            home()
        elif pos_hit():
            print("axis", MY_AXIS, "pos limit -> rehome")
            moving = False
            s.stop()
            home()
        s.track_target()

    # --- follower: continuous, never restarts a profile ---
    if not limp and not disabled:
        _now   = ticks_ms()
        dt     = ticks_diff(_now, t_prev) / 1000.0
        t_prev = _now
        if dt <= 0:
            dt = 0.005

        remaining = goal_steps - s.get_pos()
        if abs(remaining) > DEADBAND_STEPS:
            far  = abs(remaining)
            want = feed_sps                           # commanded cruise speed
            stop = (2.0 * ACCEL_SPS2 * far) ** 0.5    # fast enough to still stop
            if want > stop:
                want = stop
            if want < MIN_SPS:
                want = MIN_SPS

            dv = ACCEL_SPS2 * dt                      # same limit both ways
            if want > cur_sps:
                cur_sps = min(want, cur_sps + dv)
            else:
                cur_sps = max(want, cur_sps - dv)

            if abs(cur_sps - last_sps) > last_sps * SPS_HYST + 1:
                s.speed(int(cur_sps))        # re-arms the timer: keep it rare
                last_sps = cur_sps
            if goal_steps != last_goal:
                s.target(goal_steps)
                last_goal = goal_steps
            moving = True
        else:
            if moving:                       # arrived: settle
                s.target(s.get_pos())
                last_goal = None
                cur_sps   = 0.0
                last_sps  = -1.0
            moving = False

    if not disabled:
        led.value(1 if moving else 0)
    sleep_ms(5)

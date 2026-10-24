# Wormbot visual tracking — XIAO ESP32S3 Sense → FluidNC SCARA

A XIAO ESP32S3 Sense rides on the SCARA forearm. It runs a FOMO model
(Edge Impulse, int8) that finds the wormbot. It then jogs the arm over
FluidNC telnet so that the wormbot stays centred in the image.

```
camera ─► JPEG decode ─► centre crop/resize ─► FOMO ─► centroid (u,v)      core 1
                                                          │
             EMA ─► deadband ─► v = -kp·R(φ-φ0)·J0⁻¹·e ─► slew limit       core 0
                                                          │
             short $J=G91 jogs (≈ horizon_s each) ─► telnet :23 ─► FluidNC
```

## Layout

| Path | What |
| --- | --- |
| `platformio.ini` | envs `capture`, `tracker`, `tracker_stub` |
| `lib/xiao_cam/` | camera pins + sensor settings, shared by both firmwares (NVS `xcam`) |
| `src/capture/` | dataset-capture firmware: live view, `/capture`, sensor controls |
| `src/tracker/` | tracking firmware: `vision` (FOMO), `fluidnc` (telnet), `controller` (IBVS, calibration, safety), `web` (dashboard) |
| `tools/grab_frames.py` | frame grabber → `dataset/<session>/` |
| `tools/fluidnc_check.py` | validates telnet, kinematics, status, jog and jog cancel |
| `include/secrets.h` | WiFi + FluidNC address (copy from `secrets.h.example`) |

Build/flash: `pio run -e <env> -t upload`, monitor with `pio device monitor`.
On Ubuntu, add yourself to `dialout` if the port is not accessible.
If upload fails, hold BOOT while plugging the XIAO in.

---

## Step 1 — validate the controller (do this first)

```sh
tools/fluidnc_check.py 192.168.0.1                 # read-only
tools/fluidnc_check.py 192.168.0.1 --jog-test 10   # moves the arm, asks first
```

It reports the FluidNC version and the kinematics (`SCARA` or `Cartesian`),
along with the arm lengths and the `?` round-trip latency. The jog test checks
that `$J=` works and that `0x85` jog cancel really stops a move.

Then set `src/tracker/config.h` to match:

- `JOG_CARTESIAN 1` if kinematics is SCARA (jogs in mm), `0` if the joints are
  plain rotary axes (jogs in degrees).
- `ARM_L1_MM`, `ARM_L2_MM`, `ELBOW_SIGN`. **These need checking.** The brief says
  1000 / 660 mm. `../dlc32/scara_dlc32_kinematics.yaml` says 1090 / 1050 and
  `elbow_positive: false`. Use whatever the running controller reports.
- **Soft limits** `LIM_*` for the exhibit layout. StallGuard homing means there
  are no limit switches once homed, so these limits, together with the SCARA
  module's own envelope check, are what keep the arm in its area.

Telnet needs FluidNC on WiFi with `$Telnet/Enable=ON` (the default).
Accepting jogs requires a homed machine, i.e. not in `Alarm`.

## Step 2 — validate detectability of transparent PETG

Transparent PETG is mostly visible through its edges, specular highlights and
the Bowden cables, so lighting changes it a lot. Test that before investing in
the tracker.

1. `pio run -e capture -t upload`, open `http://<xiao>/`. The green square is
   what the model sees.
2. Set the sensor for the hall and press **Save settings for tracker**. The
   tracker loads the same settings from NVS. Under spotlights, fixed exposure
   usually beats auto (`aec=0`, tune `aec_value`; `agc=0`), because auto
   exposure hunts when the arm swings past a lamp.
3. Grab one session per lighting condition, mounted at the real camera height
   over the real floor or background:
   ```sh
   tools/grab_frames.py <xiao-ip> --lighting spot_warm --every 0.5 --count 150
   tools/grab_frames.py <xiao-ip> --lighting spot_warm --negatives --count 60
   ```
   Vary the wormbot's pose, its position in the frame (edges included),
   rotation and actuation state. Include negatives with reflections and glare.
4. Edge Impulse: object detection, image 96×96 (start grayscale, then try RGB),
   **resize mode "Fit shortest axis"** (matches the firmware's centre crop),
   FOMO MobileNetV2 0.35, int8. Label the class **`wormbot`**, or change
   `TARGET_LABEL`. Draw one box over the whole body. FOMO lights several cells
   along it, and the firmware takes their confidence-weighted centroid.
5. **Go/no-go:** keep an entire lighting session out as the test set rather
   than a random split. If F1 on unseen lighting is poor, fix it physically
   before tuning the model. Options: a matte or contrasting floor, a few
   opaque or coloured markers on the segments, coloured cables, polarised or
   diffuse light.

## Step 3 — tracker

1. Studio → Deployment → Arduino library → unzip into `lib/`. If the project is
   not named `wormbot`, edit the include in `src/tracker/ei_model.h`.
2. `pio run -e tracker -t upload`. Open `http://wormbot-tracker/` (or the IP
   printed on serial).
3. **Calibrate.** Home the machine, put the wormbot still in view near the
   centre, and press Calibrate. The arm makes small moves of ±`cal_step` in X,
   then in Y, and back. The step is doubled if the image barely moves and
   halved if the target leaves view. The resulting 2×2 image Jacobian, the
   forearm angle φ0 and Z are stored in NVS. Calibration fails, safely, if the
   target moved, the columns are degenerate, or a soft limit is in the way.
   Recalibrate after changing camera mount, Z or lens.
4. **Enable.** With the stub build (`tracker_stub`, no model) you can test the
   whole command path first: `/fake?u=0.3&v=0&ttl=500` makes the tracker
   chase a fake target for 0.5 s. After that the target counts as lost and the
   jog is cancelled. That cancel is the loss path to verify.

### Fixed camera on a wall (`CAMERA_FIXED 1`, the default now)

The camera is fixed on a wall, does not move with the arm or with Z, and sees
the workspace. No Jacobian, no
rotation compensation, no arm geometry in the loop:

- **Floor map.** A homography H turns the target's image position (u, v) into
  arm X/Y mm. It is taught on the dashboard, nothing moves on its own:
  1. head on the floor in view → *Mark from detection* (or click its spot);
  2. jog the tool exactly over it with FluidNC → *Record arm position*;
  3. ≥ 4 spots spread over the workspace (6–8 to check the fit) → *Solve*.
  Each point's error after the fit is shown, in mm. The map is drawn on the
  image (arm X/Y grid, soft limits, tool position): if it is right, the orange
  tool marker sits on the tool. Valid at any Z (the camera does not move);
  teach again only if the camera is moved.
- **Control.** target = H(u, v) + (`off_x`, `off_y`); v = `kp` × (target −
  MPos), capped at `vmax`, slew-limited by `amax`, deadband `db_mm_in` /
  `db_mm_out` (mm). Target lost: the arm keeps going to the last point where
  the head was seen and stays there until it is seen again (the arm often
  hides it once over it). Jog streaming and the other safety checks are unchanged.
- Resolution: FOMO 12×12 cells over the whole view → a few cm. The model must
  be retrained with frames from this viewpoint.

`CAMERA_FIXED 0` restores the forearm camera (Calibrate / manual Jacobian below).

### Control law (forearm camera, `CAMERA_FIXED 0`)

- `e` = target centroid, normalised −1..1, smoothed by EMA (`alpha`).
- Deadband with hysteresis: stop correcting below `db_in`, resume above `db_out`.
- `d = −J(φ)⁻¹·e` is the move that would centre the target. Because the camera
  turns with the forearm (no wrist), `J(φ) = J0·R(φ0−φ)` and φ = θ1+θ2 comes
  from the IK of the reported MPos (`CAMERA_ROTATES_WITH_FOREARM`).
- `v = kp·d`, capped at `vmax` and slew-limited by `amax`, so the 5 kg payload
  never sees a velocity step.
- Velocity becomes relative jogs of `horizon_s` duration. A new jog is queued
  only when the last one is half done, so at most about 2 segments are ever
  queued.

Tuning, live from the dashboard (saved in NVS): start with `kp 0.5–1`,
`vmax 30`. If it oscillates, lower `kp` or smooth more (lower `alpha`). The loop latency is roughly one frame plus `horizon_s`. Keep
`kp × latency` under about 0.4.

### Safety measures

| Hazard | Measure |
| --- | --- |
| Target lost / low confidence | after `lost_ms` without detection: jog cancel (0x85), hold |
| Tracker crash, WiFi drop, hang | jogs are short (`vmax·horizon_s`, about 15 mm), so motion ends on its own; WiFi loss also disables tracking; on reconnect a jog cancel is sent first |
| Leaving the work area | each jog is clipped to the box + annulus in `config.h`; FluidNC's SCARA module rejects unreachable targets |
| Controller not ready | tracking only in `Idle`/`Jog`; Alarm, Hold, Door, stale status (>600 ms) → disable + cancel |
| Wrong scale after Z change (forearm camera only) | refuses to track if Z differs from calibration by > 5 mm |
| Violent motion | `vmax`, `amax` slew limit, deadband |
| Operator stop | XIAO BOOT button or dashboard STOP → disable + jog cancel; tracking is off at every boot |

None of this replaces a hardware E-stop that cuts motor power. A 5 kg payload
at 1.6 m reach is dangerous in a public space. Use a physical barrier or
reduced speed in the visitor zone.

## Known limits / open points

- The brief names a FYSETC E4, while `../dlc32` is configured for an MKS DLC32.
  Both run FluidNC, and the tracker only talks telnet, so it does not care.
  The board only matters for which FluidNC config is live.
- The calibration target must stay still for about 10 s. If the wormbot can't
  be held still, calibrate on a printed marker trained as a second class, or
  on a parked wormbot.
- FOMO gives centroids, not orientation or size, so tracking is XY only and Z
  is never jogged.
- The real-model path of `vision.cpp` follows the Edge Impulse ESP32 camera
  example (`fmt2rgb888` BGR swap, `crop_and_interpolate_rgb888`). It compiles
  only once the exported library is in `lib/`, so it has not been built here yet.

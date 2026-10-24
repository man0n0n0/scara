// Pole tracker for AI-Thinker ESP32-CAM (ESP32 + OV2640).
//
// Finds a thin vertical cylinder (33 mm dia, 1000 mm tall) in a fixed scene and
// reports its image position over serial.  The host turns that into floor XY.
//
// Method: per-column gradient energy profile, background-subtracted in 1-D, then
// a matched filter for "rising edge, w px of pole, falling edge" over the range
// of widths the object can subtend.  No frame-sized background buffer: the
// background model is 2 x 640 int32 in internal RAM.
//
// Serial protocol (921600), one line per frame:
//   P <x_px> <ybase_px> <w_px> <score> <est_z_mm>   detection
//   N                                               nothing found
//   S <best_score> <floor> <thresh> <fps>           status, every STATUS_EVERY frames
//   B <frames> floor=<n>                            background (re)learned
//   I <key>=<value> ...                             info, at boot
// Commands:
//   b  relearn the background (scene must be empty)
//   r  reboot
//   f  dump one full frame:    "F <w> <h> <nbytes>\n" then nbytes of raw grayscale
//   g  dump one quarter frame: same, decimated 2x in both axes (4x faster)
//   p  dump the background-subtracted column profiles used by the detector:
//      "D <640 ints>" (rising), "E <640 ints>" (falling)

#include "esp_camera.h"

// ---- AI-Thinker pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ---- geometry / target
static const int   IMG_W = 640, IMG_H = 480;
static const float FOCAL_PX = 554.0f;   // 640 px over ~60 deg HFOV; refine by calibration
static const float TARGET_DIA_MM = 33.0f;
static const float Z_MIN_MM = 500.0f, Z_MAX_MM = 3000.0f;

// width in px the target subtends at the near/far limits
static int W_MIN, W_MAX;

// ---- background model + working profiles (internal RAM, ~10 kB)
static int32_t bg_pos[640], bg_neg[640];
static int32_t pos[640], neg[640];
static int32_t dpos[640], dneg[640];
static int32_t score_floor = 0;         // learned noise ceiling of the empty scene
static bool    have_bg = false;

static const int   LEARN_FRAMES = 30;
static const float SCORE_MARGIN = 1.8f; // detection must beat the empty-scene floor by this
static const int   BG_UPDATE_SHIFT = 6; // slow drift tracking when nothing is detected
static const int   STATUS_EVERY = 20;   // heartbeat so a silent setup is diagnosable

static const uint32_t SERIAL_BAUD = 921600;
static volatile char pending_cmd = 0;   // dump requested, served once a frame is held
static int32_t last_score = 0;

static bool camera_init() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;   c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;   c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // no JPEG decode on our side
  c.frame_size = FRAMESIZE_VGA;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) return false;

  // Background subtraction cannot survive auto-exposure hunting: lock everything.
  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_aec2(s, 0);
  s->set_gain_ctrl(s, 0);
  s->set_agc_gain(s, 4);        // tune for your lighting
  s->set_aec_value(s, 300);     // tune for your lighting
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_lenc(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  return true;
}

// Accumulate signed per-column gradient energy over the whole frame.
static void frame_profiles(const uint8_t *buf) {
  memset(pos, 0, sizeof(pos));
  memset(neg, 0, sizeof(neg));
  for (int y = 0; y < IMG_H; y++) {
    const uint8_t *r = buf + (size_t)y * IMG_W;
    int prev = r[0];
    for (int x = 1; x < IMG_W; x++) {
      int cur = r[x];
      int g = cur - prev;
      if (g > 0) pos[x - 1] += g; else neg[x - 1] -= g;
      prev = cur;
    }
  }
}

// Best "edge pair w apart" over both polarities. Returns score; sets x/w out.
static int32_t match(float *x_out, int *w_out) {
  int32_t best = 0;
  *x_out = -1; *w_out = 0;
  for (int w = W_MIN; w <= W_MAX; w++) {
    for (int pol = 0; pol < 2; pol++) {
      const int32_t *a = pol ? dneg : dpos;   // leading edge
      const int32_t *b = pol ? dpos : dneg;   // trailing edge
      for (int x = 0; x + w < IMG_W; x++) {
        int32_t s = a[x] + b[x + w];
        if (s > best) { best = s; *x_out = x + w * 0.5f; *w_out = w; }
      }
    }
  }
  return best;
}

// Sub-pixel: energy centroid of the two edges, +-2 px around the match.
static float refine_x(float x, int w) {
  float c = 0, m = 0;
  int l = (int)(x - w * 0.5f), r = (int)(x + w * 0.5f);
  for (int i = -2; i <= 2; i++) {
    int xl = l + i, xr = r + i;
    if (xl >= 0 && xl < IMG_W) { int32_t e = dpos[xl] + dneg[xl]; c += e * (xl - w * 0.5f); m += e; }
    if (xr >= 0 && xr < IMG_W) { int32_t e = dpos[xr] + dneg[xr]; c += e * (xr + w * 0.5f); m += e; }
  }
  return m > 0 ? c / m : x;
}

// Lowest row where the edge pair still exists = where the pole meets the floor.
// Scans up from the bottom and returns the FIRST such row, confirmed by RUN_MIN
// consecutive rows above it, so noise is not mistaken for the base.
static int find_base(const uint8_t *buf, float xc, int w) {
  const int RUN_MIN = 5, thresh = 12;
  int xl = (int)(xc - w * 0.5f), xr = (int)(xc + w * 0.5f);
  if (xl < 1 || xr >= IMG_W - 1) return -1;
  int cand = -1, run = 0;
  for (int y = IMG_H - 1; y >= 0; y--) {
    const uint8_t *r = buf + (size_t)y * IMG_W;
    int el = abs((int)r[xl + 1] - (int)r[xl - 1]);
    int er = abs((int)r[xr + 1] - (int)r[xr - 1]);
    if (el + er > thresh) {
      if (cand < 0) cand = y;
      if (++run >= RUN_MIN) return cand;
    } else { cand = -1; run = 0; }
  }
  return cand;
}

static void dump_frame(const uint8_t *buf, int step) {
  static uint8_t line[640];
  int w = IMG_W / step, h = IMG_H / step;
  Serial.printf("F %d %d %d\n", w, h, w * h);
  for (int y = 0; y < IMG_H; y += step) {
    const uint8_t *r = buf + (size_t)y * IMG_W;
    int n = 0;
    for (int x = 0; x < IMG_W; x += step) line[n++] = r[x];
    Serial.write(line, n);
  }
  Serial.println();
}

// The two profiles the matched filter actually sees. If these are flat, the
// problem is upstream of the detector (exposure, focus, or an empty scene).
static void dump_profiles() {
  Serial.print("D");
  for (int x = 0; x < IMG_W; x++) Serial.printf(" %ld", (long)dpos[x]);
  Serial.println();
  Serial.print("E");
  for (int x = 0; x < IMG_W; x++) Serial.printf(" %ld", (long)dneg[x]);
  Serial.println();
}

static void learn_background() {
  // static: 10 kB of accumulators would blow the 8 kB Arduino loop stack
  static int64_t ap[640], an[640];
  memset(ap, 0, sizeof(ap));
  memset(an, 0, sizeof(an));
  for (int i = 0; i < LEARN_FRAMES; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;
    frame_profiles(fb->buf);
    esp_camera_fb_return(fb);
    for (int x = 0; x < IMG_W; x++) { ap[x] += pos[x]; an[x] += neg[x]; }
  }
  for (int x = 0; x < IMG_W; x++) {
    bg_pos[x] = ap[x] / LEARN_FRAMES;
    bg_neg[x] = an[x] / LEARN_FRAMES;
  }
  // Residual score on an empty scene sets the detection floor.
  int32_t worst = 0;
  for (int i = 0; i < 5; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;
    frame_profiles(fb->buf);
    esp_camera_fb_return(fb);
    for (int x = 0; x < IMG_W; x++) {
      dpos[x] = pos[x] > bg_pos[x] ? pos[x] - bg_pos[x] : 0;
      dneg[x] = neg[x] > bg_neg[x] ? neg[x] - bg_neg[x] : 0;
    }
    float x; int w;
    int32_t s = match(&x, &w);
    if (s > worst) worst = s;
  }
  score_floor = worst;
  have_bg = true;
  Serial.printf("B %d floor=%ld\n", LEARN_FRAMES, (long)score_floor);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  W_MIN = (int)(FOCAL_PX * TARGET_DIA_MM / Z_MAX_MM); if (W_MIN < 2) W_MIN = 2;
  W_MAX = (int)(FOCAL_PX * TARGET_DIA_MM / Z_MIN_MM) + 1;

  if (!camera_init()) { Serial.println("I error=camera_init"); while (true) delay(1000); }
  Serial.printf("I board=esp32cam res=%dx%d focal_px=%.0f dia_mm=%.0f w_px=%d..%d psram=%u\n",
                IMG_W, IMG_H, FOCAL_PX, TARGET_DIA_MM, W_MIN, W_MAX,
                (unsigned)ESP.getFreePsram());
  Serial.println("I learning background, keep the scene empty");
  learn_background();
}

void loop() {
  static uint32_t frames = 0;
  static int32_t  best_since_status = 0;
  static uint32_t t_status = 0;

  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == 'b') { Serial.println("I relearning"); learn_background(); }
    else if (ch == 'r') ESP.restart();
    else if (ch == 'f' || ch == 'g' || ch == 'p') pending_cmd = (char)ch;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("N"); return; }
  frame_profiles(fb->buf);

  for (int x = 0; x < IMG_W; x++) {
    dpos[x] = pos[x] > bg_pos[x] ? pos[x] - bg_pos[x] : 0;
    dneg[x] = neg[x] > bg_neg[x] ? neg[x] - bg_neg[x] : 0;
  }

  float xc; int w;
  int32_t s = match(&xc, &w);
  last_score = s;
  int32_t thresh = (int32_t)(score_floor * SCORE_MARGIN);
  bool hit = have_bg && xc >= 0 && s > thresh;

  if (hit) {
    xc = refine_x(xc, w);
    int base = find_base(fb->buf, xc, w);
    float z = FOCAL_PX * TARGET_DIA_MM / (float)w;
    Serial.printf("P %.2f %d %d %ld %.0f\n", xc, base, w, (long)s, z);
  } else {
    Serial.println("N");
    for (int x = 0; x < IMG_W; x++) {
      bg_pos[x] += (pos[x] - bg_pos[x]) >> BG_UPDATE_SHIFT;
      bg_neg[x] += (neg[x] - bg_neg[x]) >> BG_UPDATE_SHIFT;
    }
  }

  if (s > best_since_status) best_since_status = s;
  if (++frames % STATUS_EVERY == 0) {
    uint32_t now = millis();
    float fps = t_status ? STATUS_EVERY * 1000.0f / (now - t_status) : 0.0f;
    t_status = now;
    Serial.printf("S %ld %ld %ld %.1f\n",
                  (long)best_since_status, (long)score_floor, (long)thresh, fps);
    best_since_status = 0;
  }

  if (pending_cmd) {
    char c = pending_cmd;
    pending_cmd = 0;
    if (c == 'p') dump_profiles();
    else dump_frame(fb->buf, c == 'g' ? 2 : 1);
  }

  esp_camera_fb_return(fb);
}

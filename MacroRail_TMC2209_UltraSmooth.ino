// ============================================================================
// ESP32-S3 + TMC2209 Ultra-Smooth Macro Rail Controller
// Paste this WHOLE file as your ONLY .ino content.
//
// Features:
// - Web panel over Wi-Fi
// - X and optional Z axis TMC2209 STEP/DIR control
// - Smooth S-curve start/stop for macro photography
// - Manual, Semi-Auto, Full Auto stack modes
// - Nikon IR shutter output
// - Physical joystick jogging
// - Fixed TMC2209 hardware microstep setting; code assumes 1/32 by default
//
// Board target in Arduino IDE:
//   Board: ESP32S3 Dev Module
//   USB CDC On Boot: Enabled
//   USB Mode: Hardware CDC and JTAG
// ============================================================================

// ====================== TYPEDEFS FIRST ======================
struct AxisState {
  long cur     = 0;
  long start   = 0;
  long end     = 0;
  long softMin = -100000;
  long softMax =  100000;
  int  stepPin = -1;
  int  dirPin  = -1;
  int  enPin   = -1;
  int  lastDir = 0;
};

enum CmdType { CMD_NONE=0, CMD_MANUAL=1, CMD_SEMI=2, CMD_FULL=3 };

struct MotionCmd {
  CmdType type = CMD_NONE;
  char axis = 'X';
  int dir = +1;

  bool irOn = true;
  float preIR = 1.0f;
  float shutter = 1.0f;
  float postRest = 1.0f;

  long moveSteps = 50;
  long photos = 2;

  long start = 0;
  long end = 0;
  long stepsPerPhoto = 10;
  long backlashSteps = 100;
  float interGapSec = 0.5f;
  long softMin = -100000;
  long softMax = 100000;

  int lens = 0;
  float aperture = 0.0f;
  float naOverride = 0.0f;
};

// ====================== INCLUDES ======================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>

// ====================== WIFI ======================
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* HOSTNAME      = "esp32";

// ====================== MECHANICS ======================
constexpr float LEAD_MM_REV = 2.0f;  // 2 mm travel per motor revolution
static int   gMicrostep   = 32;
static long  gStepsPerRev = 6400;    // 200 * microstep
static float gUmPerStep   = 0.3125f; // derived for 2 mm lead screw at 1/32 microstep

static void recalcResolution() {
  gStepsPerRev = 200L * (long)gMicrostep;
  gUmPerStep = (LEAD_MM_REV * 1000.0f) / (float)gStepsPerRev;
}

// ====================== ESP32-S3 SAFE PIN MAP ======================
// Avoid GPIO19/20 on many ESP32-S3 boards because they are USB pins.
// Avoid GPIO0/45/46 because they are boot/strapping pins.
// Avoid GPIO26-32 because some ESP32-S3 modules use them internally.

// X axis TMC2209
constexpr int X_STEP = 4;
constexpr int X_DIR  = 5;
constexpr int X_EN   = 6;

// Z axis TMC2209, optional second driver
constexpr int Z_STEP = 7;
constexpr int Z_DIR  = 15;
constexpr int Z_EN   = 16;

// TMC2209 simple STEP/DIR mode wiring:
// MS1 = 3V3, MS2 = GND for 1/32 microstepping on common TMC2209 modules.
// SPREAD = GND/open for quiet StealthChop.
// PDN_UART = 3V3 for this no-UART beginner version.
// GPIO8-GPIO12 are NOT connected to the TMC2209 in this version.

// Nikon IR LED driver signal. Use a transistor/MOSFET, not the ESP32 pin directly for strong IR.
constexpr int PIN_IR = 18;

// Joystick: VCC=3V3, GND=GND, VRx=GPIO1, VRy=GPIO2, SW=GPIO42-to-GND when pressed
constexpr int JOY_VRX = 1;
constexpr int JOY_VRY = 2;
constexpr int JOY_SW  = 42;

// ====================== GLOBALS ======================
WebServer server(80);
Preferences prefs;

AxisState X{0,0,0,-100000,100000, X_STEP,X_DIR,X_EN, 0};
AxisState Z{0,0,0,-100000,100000, Z_STEP,Z_DIR,Z_EN, 0};

static bool gBusy = false;
static volatile bool gStopRequested = false;
static bool gHasQueuedCmd = false;
static MotionCmd gQueuedCmd;

static float gSpeedScale = 0.10f;  // 10% default for ultra-smooth macro movement

static int gJoyCenterX = 2048;
static int gJoyCenterY = 2048;
static float gJoyFiltX = 0.0f;
static float gJoyFiltY = 0.0f;
static uint32_t gNextJoySampleMs = 0;
static uint32_t gNextStepDueX = 0;
static uint32_t gNextStepDueZ = 0;
static float gJogSpsX = 0.0f;
static float gJogSpsZ = 0.0f;
static int gJogDirX = 0;
static int gJogDirZ = 0;
static uint32_t gLastJogUpdateMs = 0;

static int gCurrentPhoto = 0;
static int gTotalPhotos = 0;

// ====================== CONSTANTS ======================
constexpr float DEFAULT_PRE_IR_WAIT_SEC  = 1.0f;
constexpr float DEFAULT_SHUTTER_SEC      = 1.0f;
constexpr float DEFAULT_POST_REST_SEC    = 1.0f;
constexpr bool  DEFAULT_IR_ENABLED       = true;

constexpr uint32_t STEP_PULSE_US = 4;       // TMC2209 accepts short STEP pulses; 4us is safe
constexpr uint32_t DIR_SETUP_US  = 10;

// Two motion profiles:
// PHOTO = slow, ultra-smooth stacking movement.
// FAST  = fast rail adjustment, still ramped so it does not kick.
static bool gFastAdjust = false;

constexpr float PHOTO_BASE_MAX_SPS = 90.0f;     // photography movement max at 100%
constexpr float FAST_BASE_MAX_SPS  = 1600.0f;   // fast rail adjustment max at 100%
constexpr float PHOTO_JOG_MAX_SPS  = 70.0f;
constexpr float FAST_JOG_MAX_SPS   = 1800.0f;
constexpr float JOG_MIN_SPS        = 0.6f;      // crawl speed; prevents launch kick
constexpr float PHOTO_JOG_ACCEL_SPS2 = 18.0f;
constexpr float PHOTO_JOG_DECEL_SPS2 = 28.0f;
constexpr float FAST_JOG_ACCEL_SPS2  = 650.0f;  // fast but still ramped
constexpr float FAST_JOG_DECEL_SPS2  = 900.0f;
constexpr float JOY_DEADZONE = 0.32f;           // wider deadzone prevents tiny vibration
constexpr float JOY_EMA_ALPHA = 0.06f;          // stronger filtering = smoother joystick
constexpr uint32_t JOY_SAMPLE_MS = 10;

static inline float baseMaxSps() { return (gFastAdjust ? FAST_BASE_MAX_SPS : PHOTO_BASE_MAX_SPS); }
static inline float jogMaxSps()  { return (gFastAdjust ? FAST_JOG_MAX_SPS  : PHOTO_JOG_MAX_SPS); }
static inline float jogAccelSps2(){ return (gFastAdjust ? FAST_JOG_ACCEL_SPS2 : PHOTO_JOG_ACCEL_SPS2); }
static inline float jogDecelSps2(){ return (gFastAdjust ? FAST_JOG_DECEL_SPS2 : PHOTO_JOG_DECEL_SPS2); }

// ====================== LOW-LEVEL DRIVER CONTROL ======================
// TMC2209 EN/ENN is active LOW on common modules: LOW = enabled, HIGH = disabled.
// This code keeps the drivers enabled/holding all the time to prevent position snap.

static void driversWake() {
  // No sleep/reset pins are used for TMC2209 in this simple STEP/DIR version.
}

static void driversSleep() {
  // Intentionally empty. Do not sleep the driver during macro work.
}

static inline void enWrite(int pin, bool enable) {
  if (pin < 0) return;
  digitalWrite(pin, enable ? LOW : HIGH);
}

static void driversEnable() {
  enWrite(X.enPin, true);
  enWrite(Z.enPin, true);
}

static void driversDisable() {
  // Intentionally not used by Stop or Reset. Releasing the motor can cause a snap later.
  enWrite(X.enPin, false);
  enWrite(Z.enPin, false);
}

static void applyMicrostepPins(int ms) {
  // TMC2209 microstep is set by hardware pins in this no-UART version.
  // Wire MS1=3V3 and MS2=GND for 1/32.
  // This value is only used for distance calculation in the web app.
  if (!(ms==1 || ms==2 || ms==4 || ms==8 || ms==16 || ms==32)) ms = 32;
  gMicrostep = ms;
  recalcResolution();
}

static void saveSysCfg() {
  prefs.begin("sys", false);
  prefs.putInt("microstep", gMicrostep);
  prefs.putBool("fast", gFastAdjust);
  prefs.end();
}

static void loadSysCfg() {
  prefs.begin("sys", true);
  int ms = prefs.getInt("microstep", 32);
  bool fast = prefs.getBool("fast", false);
  prefs.end();
  gFastAdjust = fast;
  applyMicrostepPins(ms);
}

// ====================== GENERAL HELPERS ======================
static inline AxisState& pickAxis(char axis) {
  return (axis == 'Z') ? Z : X;
}

static inline bool withinSoft(const AxisState& A, long t) {
  return (t >= A.softMin && t <= A.softMax);
}

static float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

static void clampSpeedScale() {
  if (gSpeedScale < 0.01f) gSpeedScale = 0.01f;
  if (gSpeedScale > 1.00f) gSpeedScale = 1.00f;
}

static void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while ((millis() - t0) < ms) {
    server.handleClient();
    if (gStopRequested) break;
    delay(2);
  }
}

static void waitSeconds(float s) {
  if (s > 0.0f) waitMs((uint32_t)(s * 1000.0f));
}

static inline void waitUntilUs(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    // Do not break here on gStopRequested.
    // Smooth Stop needs the timing delay so deceleration remains smooth.
    server.handleClient();
    delayMicroseconds(40);
  }
}

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// smootherstep: zero slope at both ends
static inline float smootherStep01(float x) {
  x = clamp01(x);
  return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f);
}

// ====================== IR SHUTTER ======================
constexpr int IR_FREQ = 38000;
constexpr int IR_RES_BITS = 8;
constexpr uint32_t IR_DUTY = 128;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static inline bool pwmAttachIR() {
  return ledcAttach(PIN_IR, IR_FREQ, IR_RES_BITS);
}
static inline void pwmWriteIR(uint32_t duty) {
  ledcWrite(PIN_IR, duty);
}
#else
constexpr int IR_CH = 0;
static inline bool pwmAttachIR() {
  ledcSetup(IR_CH, IR_FREQ, IR_RES_BITS);
  ledcAttachPin(PIN_IR, IR_CH);
  return true;
}
static inline void pwmWriteIR(uint32_t duty) {
  ledcWrite(IR_CH, duty);
}
#endif

static inline void irOn()  { pwmWriteIR(IR_DUTY); }
static inline void irOff() { pwmWriteIR(0); }

static void irMark(uint32_t usec) {
  irOn();
  delayMicroseconds(usec);
}

static void irSpace(uint32_t usec) {
  irOff();
  delayMicroseconds(usec);
}

static void sendNikonIR() {
  const uint16_t nikonTimings[] = {
    2000, 27830, 390, 1580, 410, 3580, 400, 63200,
    2000, 27830, 390, 1580, 410, 3580, 400
  };
  const int N = sizeof(nikonTimings) / sizeof(nikonTimings[0]);

  for (int i = 0; i < N; i++) {
    if (gStopRequested) break;
    if ((i % 2) == 0) irMark(nikonTimings[i]);
    else              irSpace(nikonTimings[i]);
  }
  irOff();
}

static void fireIRWithRests(bool useIR, float preIR, float shutter, float postRest) {
  waitSeconds(preIR);
  if (useIR) sendNikonIR();
  waitSeconds(shutter);
  waitSeconds(postRest);
}

// ====================== MOTION ======================
static inline void setDir(AxisState& A, int dir) {
  if (dir != A.lastDir) {
    digitalWrite(A.dirPin, (dir >= 0) ? HIGH : LOW);
    A.lastDir = dir;
    delayMicroseconds(DIR_SETUP_US);
  }
}

static inline void pulseStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(2);
}

static inline void axisStep(AxisState& A, int dir) {
  long nextStep = A.cur + ((dir >= 0) ? 1 : -1);
  if (!withinSoft(A, nextStep)) return;
  setDir(A, dir);
  pulseStep(A.stepPin);
  A.cur = nextStep;
}

static void moveTo(AxisState& A, long target) {
  if (target == A.cur) return;

  if (!withinSoft(A, target)) {
    target = (target < A.softMin) ? A.softMin : A.softMax;
  }

  const long totalSteps = llabs(target - A.cur);
  if (totalSteps <= 0) return;

  const int dir = (target > A.cur) ? +1 : -1;

  // Ultra-smooth motion tuning.
  const float VMIN_SPS = 0.6f;
  const uint32_t START_SETTLE_MS = 350;
  const uint32_t END_SETTLE_MS   = 350;

  // Photo profile uses a long ramp. Fast profile allows high speed but still ramps.
  const long RAMP_MIN_STEPS_LOCAL = gFastAdjust ? 1800 : 2400;
  const long STOP_RAMP_MIN_STEPS  = gFastAdjust ? 1400 : 900;
  const float STOP_DECEL_SPS2     = gFastAdjust ? 900.0f : 22.0f;

  float vMax = baseMaxSps() * gSpeedScale;
  if (vMax < 1.5f) vMax = 1.5f;

  float vMin = VMIN_SPS;
  if (vMin > vMax * 0.50f) vMin = vMax * 0.50f;
  if (vMin < 0.2f) vMin = 0.2f;

  long rampSteps = RAMP_MIN_STEPS_LOCAL;
  if (2 * rampSteps > totalSteps) rampSteps = totalSteps / 2;
  if (rampSteps < 1) rampSteps = 1;

  driversEnable();
  setDir(A, dir);
  waitMs(START_SETTLE_MS);

  uint32_t nextT = micros();
  bool stopMode = false;
  long stopStartI = 0;
  long stopRampSteps = STOP_RAMP_MIN_STEPS;
  float stopStartV = vMin;
  float lastV = vMin;

  for (long i = 0; i < totalSteps; i++) {
    if (gStopRequested && !stopMode) {
      stopMode = true;
      stopStartI = i;
      stopStartV = lastV;
      long physicsSteps = (long)((stopStartV * stopStartV) / (2.0f * STOP_DECEL_SPS2));
      if (physicsSteps < STOP_RAMP_MIN_STEPS) physicsSteps = STOP_RAMP_MIN_STEPS;
      stopRampSteps = physicsSteps;
      long remainingToTarget = totalSteps - i;
      if (stopRampSteps > remainingToTarget) stopRampSteps = remainingToTarget;
      if (stopRampSteps < 1) stopRampSteps = 1;
    }

    float v = vMax;

    if (stopMode) {
      long si = i - stopStartI;
      if (si >= stopRampSteps) break;
      float x = (float)si / (float)stopRampSteps;
      v = vMin + (stopStartV - vMin) * (1.0f - smootherStep01(x));
    }
    else {
      if (i < rampSteps) {
        float x = (float)i / (float)rampSteps;
        v = vMin + (vMax - vMin) * smootherStep01(x);
      }
      else if (i >= (totalSteps - rampSteps)) {
        float x = (float)(totalSteps - 1 - i) / (float)rampSteps;
        v = vMin + (vMax - vMin) * smootherStep01(x);
      }
    }

    if (v < vMin) v = vMin;
    lastV = v;

    uint32_t intervalUs = (uint32_t)(1000000.0f / v);

    waitUntilUs(nextT);

    long nextStep = A.cur + ((dir >= 0) ? 1 : -1);
    if (!withinSoft(A, nextStep)) break;

    pulseStep(A.stepPin);
    A.cur = nextStep;

    nextT += intervalUs;

    if ((i & 0x1FF) == 0) {
      server.handleClient();
      delay(0);
    }
  }

  // Keep the driver enabled and let mechanical vibration die before the next photo/move.
  driversEnable();
  waitMs(END_SETTLE_MS);
}

static void preloadBacklash(AxisState& A, long target, long backlashSteps) {
  if (!withinSoft(A, target)) return;
  const int dir = (target > A.cur) ? +1 : (target < A.cur ? -1 : 0);
  if (dir == 0) return;

  long over = target + ((dir > 0) ? backlashSteps : -backlashSteps);
  if (!withinSoft(A, over)) over = target;

  moveTo(A, over);
  if (!gStopRequested && over != target) moveTo(A, target);
}

// ====================== OPTICS ======================
constexpr float LAMBDA_UM = 0.55f;

static int computeStepsPerPhotoFromLens(int lens, float fnum, float naOverride) {
  float na = 0.0f;

  if (naOverride > 0.0f) {
    na = naOverride;
  }
  else if (lens == 1 || lens == 2) {
    if (fnum <= 0.0f) return 0;
    float Neff = fnum * (1.0f + lens);
    na = 1.0f / (2.0f * Neff);
  }
  else {
    switch (lens) {
      case 4:  na = 0.10f; break;
      case 5:  na = 0.15f; break;
      case 10: na = 0.25f; break;
      case 20: na = 0.40f; break;
      case 40: na = 0.65f; break;
      case 60: na = 0.85f; break;
      default: na = 0.0f;  break;
    }
  }

  if (na <= 0.0f) return 0;

  const float stepUm = 0.5f * LAMBDA_UM / (na * na);
  int steps = (int)ceilf(stepUm / gUmPerStep);
  if (steps < 1) steps = 1;
  return steps;
}

// ====================== COMMANDS ======================
static void runManual(const MotionCmd& c) {
  AxisState& A = pickAxis(c.axis);
  if (c.moveSteps > 0) moveTo(A, A.cur + c.dir * c.moveSteps);
  if (!gStopRequested) fireIRWithRests(c.irOn, c.preIR, c.shutter, c.postRest);
}

static void runSemi(const MotionCmd& c) {
  AxisState& A = pickAxis(c.axis);
  long photos = c.photos;
  if (photos < 1) photos = 1;

  gCurrentPhoto = 0;
  gTotalPhotos = (int)photos;

  for (long i = 0; i < photos; i++) {
    if (gStopRequested) break;
    gCurrentPhoto = (int)i + 1;

    fireIRWithRests(c.irOn, c.preIR, c.shutter, c.postRest);
    if (gStopRequested || i == photos - 1) break;

    moveTo(A, A.cur + c.dir * c.moveSteps);
    waitSeconds(0.10f);
  }

  gCurrentPhoto = 0;
  gTotalPhotos = 0;
}

static void runFull(const MotionCmd& c) {
  AxisState& A = pickAxis(c.axis);
  A.softMin = c.softMin;
  A.softMax = c.softMax;

  if (!withinSoft(A, c.start) || !withinSoft(A, c.end)) return;

  long spp = c.stepsPerPhoto;
  if (spp < 1) {
    spp = computeStepsPerPhotoFromLens(c.lens, c.aperture, c.naOverride);
  }
  if (spp < 1) spp = 10;

  int dir = (c.end >= c.start) ? +1 : -1;
  preloadBacklash(A, c.start, c.backlashSteps);

  long span = llabs(c.end - c.start);
  long stations = (span / spp) + 1;
  if (stations < 1) stations = 1;

  gCurrentPhoto = 0;
  gTotalPhotos = (int)stations;

  long pos = c.start;
  for (long i = 0; i < stations; i++) {
    if (gStopRequested) break;
    gCurrentPhoto = (int)i + 1;

    fireIRWithRests(c.irOn, c.preIR, c.shutter, c.postRest);
    if (gStopRequested || i == stations - 1) break;

    long next = pos + dir * spp;
    if ((dir > 0 && next > c.end) || (dir < 0 && next < c.end)) next = c.end;
    if (!withinSoft(A, next)) break;

    moveTo(A, next);
    pos = next;
    waitSeconds(c.interGapSec);
  }

  gCurrentPhoto = 0;
  gTotalPhotos = 0;
}

static void executeCommand(const MotionCmd& c) {
  gBusy = true;
  gStopRequested = false;
  driversEnable();

  if (c.type == CMD_MANUAL) runManual(c);
  else if (c.type == CMD_SEMI) runSemi(c);
  else if (c.type == CMD_FULL) runFull(c);

  gBusy = false;
  // Keep drivers enabled to hold position. Use Stop button if you want motors released.
  driversEnable();
}

// ====================== JOYSTICK ======================
static float joyNorm(int raw, int center) {
  int d = raw - center;
  if (d == 0) return 0.0f;

  if (d > 0) {
    float denom = (float)(4095 - center);
    if (denom < 1.0f) denom = 1.0f;
    return (float)d / denom;
  }
  else {
    float denom = (float)center;
    if (denom < 1.0f) denom = 1.0f;
    return (float)d / denom;
  }
}

static float applyDeadzone(float v) {
  if (fabsf(v) < JOY_DEADZONE) return 0.0f;
  float s = (fabsf(v) - JOY_DEADZONE) / (1.0f - JOY_DEADZONE);
  s = clampf(s, 0.0f, 1.0f);
  return (v < 0.0f) ? -s : s;
}

static float shapeJoy(float v) {
  float a = fabsf(v);
  float s = a * a;
  return (v < 0.0f) ? -s : s;
}

static void calibrateJoystickCenters(uint32_t ms = 700) {
  uint32_t t0 = millis();
  uint32_t sx = 0, sy = 0, n = 0;

  while (millis() - t0 < ms) {
    sx += (uint32_t)analogRead(JOY_VRX);
    sy += (uint32_t)analogRead(JOY_VRY);
    n++;
    delay(5);
  }

  if (n == 0) n = 1;
  gJoyCenterX = (int)(sx / n);
  gJoyCenterY = (int)(sy / n);
}

static float approachFloat(float current, float target, float maxDelta) {
  if (current < target) {
    current += maxDelta;
    if (current > target) current = target;
  } else if (current > target) {
    current -= maxDelta;
    if (current < target) current = target;
  }
  return current;
}

static void smoothJogAxis(AxisState& A, float cmd, float& currentSps, int& currentDir,
                          uint32_t& nextStepDue, uint32_t nowUs, float dtSec) {
  int desiredDir = 0;
  float targetSps = 0.0f;

  if (cmd != 0.0f) {
    desiredDir = (cmd > 0.0f) ? +1 : -1;
    targetSps = JOG_MIN_SPS + (jogMaxSps() * gSpeedScale - JOG_MIN_SPS) * fabsf(cmd);
    if (targetSps < JOG_MIN_SPS) targetSps = JOG_MIN_SPS;
  }

  // If direction changes, decelerate to zero first. Never reverse instantly.
  if (desiredDir != 0 && currentDir != 0 && desiredDir != currentDir) {
    currentSps = approachFloat(currentSps, 0.0f, jogDecelSps2() * dtSec);
    if (currentSps <= 0.05f) {
      currentSps = 0.0f;
      currentDir = desiredDir;
      nextStepDue = nowUs;
    }
  } else {
    if (desiredDir != 0) currentDir = desiredDir;
    float accel = (targetSps > currentSps) ? jogAccelSps2() : jogDecelSps2();
    currentSps = approachFloat(currentSps, targetSps, accel * dtSec);
  }

  if (desiredDir == 0 && currentSps <= 0.05f) {
    currentSps = 0.0f;
    currentDir = 0;
    nextStepDue = nowUs;
    return;
  }

  if (currentSps <= 0.05f || currentDir == 0) return;

  driversEnable();
  uint32_t interval = (uint32_t)(1000000.0f / currentSps);
  uint32_t minInterval = (uint32_t)(1000000.0f / jogMaxSps());
  if (interval < minInterval) interval = minInterval;

  if ((int32_t)(nowUs - nextStepDue) >= 0) {
    axisStep(A, currentDir);
    nextStepDue = nowUs + interval;
  }
}

static void processJoystick() {
  if (gBusy) return;

  bool joystickStop = (digitalRead(JOY_SW) == LOW);
  if (joystickStop) {
    gHasQueuedCmd = false;
    // Do not disable drivers and do not zero the speed instantly.
    // The command becomes zero and smoothJogAxis decelerates to a stop.
  }

  uint32_t nowMs = millis();
  if (gLastJogUpdateMs == 0) gLastJogUpdateMs = nowMs;
  float dtSec = (float)(nowMs - gLastJogUpdateMs) / 1000.0f;
  if (dtSec < 0.001f) dtSec = 0.001f;
  if (dtSec > 0.050f) dtSec = 0.050f;
  gLastJogUpdateMs = nowMs;

  if (!joystickStop && (int32_t)(nowMs - gNextJoySampleMs) >= 0) {
    gNextJoySampleMs = nowMs + JOY_SAMPLE_MS;

    int rx = analogRead(JOY_VRX);
    int ry = analogRead(JOY_VRY);

    float nx = clampf(joyNorm(rx, gJoyCenterX), -1.0f, 1.0f);
    float ny = clampf(joyNorm(ry, gJoyCenterY), -1.0f, 1.0f);

    gJoyFiltX = (1.0f - JOY_EMA_ALPHA) * gJoyFiltX + JOY_EMA_ALPHA * nx;
    gJoyFiltY = (1.0f - JOY_EMA_ALPHA) * gJoyFiltY + JOY_EMA_ALPHA * ny;
  }

  float vx = 0.0f;
  float vy = 0.0f;

  if (!joystickStop) {
    vx = shapeJoy(applyDeadzone(gJoyFiltX));
    vy = shapeJoy(applyDeadzone(gJoyFiltY));
  }

  uint32_t nowUs = micros();

  smoothJogAxis(X, vx, gJogSpsX, gJogDirX, gNextStepDueX, nowUs, dtSec);
  smoothJogAxis(Z, vy, gJogSpsZ, gJogDirZ, gNextStepDueZ, nowUs, dtSec);

  if (gJogSpsX <= 0.05f && gJogSpsZ <= 0.05f) {
    gStopRequested = false;
  }
}

// ====================== WEB UI ======================
static const char HTML_INDEX[] PROGMEM = R"=====(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>ESP32-S3 TMC2209 Macro Rail</title>
<style>
:root{--bg:#0b0f14;--card:#111821;--line:#263445;--ink:#e6edf3;--muted:#9fb0c0;--field:#0d141b;--ok:#22c55e;--warn:#f59e0b;--danger:#ef4444;--blue:#2563eb}
body{margin:0;padding:12px;background:var(--bg);color:var(--ink);font-family:system-ui,-apple-system,Segoe UI,Arial,sans-serif}
.card{max-width:900px;margin:auto;background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px}
h1{font-size:19px;margin:0 0 10px}.badges{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:12px}.pill{display:inline-block;padding:5px 10px;border-radius:999px;border:1px solid var(--line);background:var(--field);font-size:12px}
.row{display:grid;grid-template-columns:160px 1fr;gap:10px;align-items:center;margin:8px 0}select,input{width:100%;box-sizing:border-box;background:var(--field);color:var(--ink);border:1px solid var(--line);border-radius:10px;padding:10px;font-size:14px}
.compact{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}button{border:0;border-radius:12px;padding:10px 16px;font-weight:700;cursor:pointer;color:white;background:var(--blue)}.start{background:var(--ok)}.stop{background:var(--warn);color:#111}.reset{background:var(--danger)}.muted{color:var(--muted);font-size:12px}.box{border:1px dashed var(--line);border-radius:10px;padding:10px;margin-top:8px}
@media(max-width:650px){.row{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="card">
<h1>ESP32-S3 TMC2209 Ultra-Smooth Macro Rail</h1>
<div class="badges">
<span class="pill">IP: {{IP}}</span>
<span class="pill">RSSI: {{RSSI}} dBm</span>
<span class="pill" id="pos">X/Z: {{CURX}} / {{CURZ}}</span>
<span class="pill" id="res">{{UMSTEP}} µm/step</span>
<span class="pill" id="ms">1/{{MS}}</span>
<span class="pill" id="busy">Ready</span>
<span class="pill" id="prog">Photo: 0/0</span>
</div>

<div class="row"><label>Speed</label><div class="compact"><input id="speed" type="range" min="1" max="100" value="10"><span class="pill" id="speedTxt">10%</span></div></div>
<div class="row"><label>Motion profile</label><select id="profile"><option value="0" selected>Photo floating / slow</option><option value="1">Fast rail adjustment</option></select></div>
<div class="row"><label>Microstep in code</label><select id="micro"><option value="16">1/16</option><option value="32" selected>1/32</option></select></div>

<div class="box muted">TMC2209 wiring: X STEP=GPIO4, DIR=GPIO5, EN/ENN=GPIO6. Z STEP=GPIO7, DIR=GPIO15, EN/ENN=GPIO16. VIO/VCC_IO=3V3. MS1=3V3, MS2=GND for 1/32. SPREAD=GND/open. PDN_UART=3V3. IR=GPIO18. Joystick=GPIO1/GPIO2/SW42. Motors stay enabled and holding after Stop.</div>

<div class="row"><label>Mode</label><select id="mode"><option>Manual</option><option>Semi-Auto</option><option>Full Auto</option></select></div>
<div class="row"><label>Axis</label><select id="axis"><option value="X">X-Axis</option><option value="Z">Z-Axis</option></select></div>
<div class="row"><label>Direction</label><select id="dir"><option value="1">Forward / Up</option><option value="-1">Backward / Down</option></select></div>

<div id="manualBox" class="box">
<div class="row"><label>Move steps</label><input id="mSteps" type="number" value="50" min="1"></div>
<div class="row"><label>IR</label><select id="mIR"><option value="1">On</option><option value="0">Off</option></select></div>
</div>

<div id="semiBox" class="box" style="display:none">
<div class="row"><label>Photos</label><input id="sPhotos" type="number" value="2" min="1"></div>
<div class="row"><label>Move steps</label><input id="sSteps" type="number" value="50" min="1"></div>
<div class="row"><label>IR</label><select id="sIR"><option value="1">On</option><option value="0">Off</option></select></div>
</div>

<div id="fullBox" class="box" style="display:none">
<div class="row"><label>Start point</label><div class="compact"><button onclick="setPoint('start')">Use current</button><span class="pill" id="startVal">0</span></div></div>
<div class="row"><label>End point</label><div class="compact"><button onclick="setPoint('end')">Use current</button><span class="pill" id="endVal">0</span></div></div>
<div class="row"><label>Steps/photo</label><input id="fSPP" type="number" value="10" min="1"></div>
<div class="row"><label>Backlash steps</label><input id="fBacklash" type="number" value="100" min="0"></div>
<div class="row"><label>Inter-shot gap sec</label><input id="fGap" type="number" value="0.5" min="0" step="0.1"></div>
<div class="row"><label>Soft min</label><input id="fMin" type="number" value="-100000"></div>
<div class="row"><label>Soft max</label><input id="fMax" type="number" value="100000"></div>
<div class="row"><label>IR</label><select id="fIR"><option value="1">On</option><option value="0">Off</option></select></div>
</div>

<div class="box">
<div class="row"><label>Pre-IR wait sec</label><input id="preIR" type="number" value="1.0" min="0" step="0.1"></div>
<div class="row"><label>Shutter wait sec</label><input id="shutter" type="number" value="1.0" min="0" step="0.1"></div>
<div class="row"><label>Post-rest sec</label><input id="postRest" type="number" value="1.0" min="0" step="0.1"></div>
</div>

<div class="btns"><button class="start" onclick="startCmd()">Start</button><button class="stop" onclick="cmd('stop')">Smooth Stop + Hold</button><button class="reset" onclick="cmd('reset')">Reset positions</button></div>
<p class="muted" id="status">Status: ready</p>
</div>

<script>
const $=id=>document.getElementById(id);
function enc(v){return encodeURIComponent(v)}
function showMode(){let m=$('mode').value;$('manualBox').style.display=m==='Manual'?'block':'none';$('semiBox').style.display=m==='Semi-Auto'?'block':'none';$('fullBox').style.display=m==='Full Auto'?'block':'none'}
$('mode').addEventListener('change',showMode);showMode();
async function refresh(){try{let r=await fetch('/state',{cache:'no-store'});let s=await r.json();$('pos').textContent='X/Z: '+s.curX+' / '+s.curZ;$('res').textContent=s.umPerStep.toFixed(4)+' µm/step';$('ms').textContent='1/'+s.microstep;$('busy').textContent=s.busy?'BUSY':'Ready';$('prog').textContent='Photo: '+s.currentPhoto+'/'+s.totalPhotos;$('startVal').textContent=($('axis').value==='X')?s.startX:s.startZ;$('endVal').textContent=($('axis').value==='X')?s.endX:s.endZ;if(String($('micro').value)!==String(s.microstep))$('micro').value=String(s.microstep);if(String($('profile').value)!==String(s.fastAdjust))$('profile').value=String(s.fastAdjust);}catch(e){}}
setInterval(refresh,1000);refresh();
async function cmd(action,extra=''){let r=await fetch('/cmd?action='+action+extra);let t=await r.text();$('status').textContent='Status: '+t;refresh();}
function common(){return '&axis='+$('axis').value+'&dir='+$('dir').value+'&preIR='+$('preIR').value+'&shutter='+$('shutter').value+'&postRest='+$('postRest').value}
function startCmd(){let m=$('mode').value;let q=common()+'&mode='+enc(m);if(m==='Manual'){q+='&moveSteps='+$('mSteps').value+'&ir='+$('mIR').value}else if(m==='Semi-Auto'){q+='&photos='+$('sPhotos').value+'&moveSteps='+$('sSteps').value+'&ir='+$('sIR').value}else{q+='&stepsPerPhoto='+$('fSPP').value+'&backlash='+$('fBacklash').value+'&gap='+$('fGap').value+'&softMin='+$('fMin').value+'&softMax='+$('fMax').value+'&ir='+$('fIR').value}cmd('start',q)}
function setPoint(which){cmd(which==='start'?'setStart':'setEnd','&axis='+$('axis').value)}
$('speed').addEventListener('input',async()=>{let p=$('speed').value;$('speedTxt').textContent=p+'%';await fetch('/speed?value='+(p/100.0));});
$('micro').addEventListener('change',async()=>{await fetch('/microstep?value='+$('micro').value);refresh();});
$('profile').addEventListener('change',async()=>{await fetch('/profile?fast='+$('profile').value);refresh();});
</script>
</body></html>)=====";

static String htmlPage() {
  String html(HTML_INDEX);
  html.replace("{{IP}}", WiFi.localIP().toString());
  html.replace("{{RSSI}}", String(WiFi.RSSI()));
  html.replace("{{CURX}}", String(X.cur));
  html.replace("{{CURZ}}", String(Z.cur));
  html.replace("{{MS}}", String(gMicrostep));
  html.replace("{{UMSTEP}}", String(gUmPerStep, 4));
  return html;
}

static void handleRoot() {
  server.send(200, "text/html; charset=utf-8", htmlPage());
}

static void handleState() {
  String json = "{";
  json += "\"busy\":" + String(gBusy ? 1 : 0) + ",";
  json += "\"curX\":" + String(X.cur) + ",";
  json += "\"curZ\":" + String(Z.cur) + ",";
  json += "\"startX\":" + String(X.start) + ",";
  json += "\"startZ\":" + String(Z.start) + ",";
  json += "\"endX\":" + String(X.end) + ",";
  json += "\"endZ\":" + String(Z.end) + ",";
  json += "\"microstep\":" + String(gMicrostep) + ",";
  json += "\"stepsPerRev\":" + String(gStepsPerRev) + ",";
  json += "\"umPerStep\":" + String(gUmPerStep, 6) + ",";
  json += "\"speedScale\":" + String(gSpeedScale, 2) + ",";
  json += "\"fastAdjust\":" + String(gFastAdjust ? 1 : 0) + ",";
  json += "\"currentPhoto\":" + String(gCurrentPhoto) + ",";
  json += "\"totalPhotos\":" + String(gTotalPhotos);
  json += "}";
  server.send(200, "application/json", json);
}

static double argNum(const char* name, double def) {
  if (!server.hasArg(name)) return def;
  return server.arg(name).toFloat();
}

static String argStr(const char* name, const String& def) {
  if (!server.hasArg(name)) return def;
  return server.arg(name);
}

static void handleSpeed() {
  gSpeedScale = (float)argNum("value", 0.10);
  clampSpeedScale();
  server.send(200, "text/plain", "speed set");
}

static void handleMicrostep() {
  if (gBusy) {
    server.send(409, "text/plain", "busy - stop first");
    return;
  }
  int ms = (int)argNum("value", 16);
  if (!(ms==1 || ms==2 || ms==4 || ms==8 || ms==16 || ms==32)) {
    server.send(400, "text/plain", "microstep must be 1,2,4,8,16,32");
    return;
  }
  applyMicrostepPins(ms);
  saveSysCfg();
  server.send(200, "text/plain", "microstep value set in code; hardware MS1/MS2 must match");
}

static void handleProfile() {
  gFastAdjust = ((int)argNum("fast", 0)) != 0;
  saveSysCfg();
  server.send(200, "text/plain", gFastAdjust ? "fast rail adjustment profile" : "photo floating profile");
}

static void handleCmd() {
  String action = argStr("action", "");

  if (action == "stop") {
    gStopRequested = true;
    gHasQueuedCmd = false;
    driversEnable();   // hold position; do not release motors, because releasing causes the next-start snap
    server.send(200, "text/plain", "smooth stop requested; motors will hold position");
    return;
  }

  if (action == "reset") {
    if (gBusy) {
      gStopRequested = true;
      driversEnable();
      server.send(409, "text/plain", "smooth stop first, then reset positions");
      return;
    }
    gHasQueuedCmd = false;
    X.cur = X.start = X.end = 0;
    Z.cur = Z.start = Z.end = 0;
    gCurrentPhoto = 0;
    gTotalPhotos = 0;
    driversEnable();
    gStopRequested = false;
    server.send(200, "text/plain", "positions reset; motors holding");
    return;
  }

  char axis = (argStr("axis", "X") == "Z") ? 'Z' : 'X';
  AxisState& A = pickAxis(axis);

  if (action == "setStart") {
    A.start = A.cur;
    server.send(200, "text/plain", "start set");
    return;
  }

  if (action == "setEnd") {
    A.end = A.cur;
    server.send(200, "text/plain", "end set");
    return;
  }

  if (action != "start") {
    server.send(400, "text/plain", "unknown action");
    return;
  }

  if (gBusy || gHasQueuedCmd) {
    server.send(409, "text/plain", "busy");
    return;
  }

  MotionCmd c;
  c.axis = axis;
  c.dir = (int)argNum("dir", 1);
  if (c.dir >= 0) c.dir = +1; else c.dir = -1;

  c.preIR = (float)argNum("preIR", DEFAULT_PRE_IR_WAIT_SEC);
  c.shutter = (float)argNum("shutter", DEFAULT_SHUTTER_SEC);
  c.postRest = (float)argNum("postRest", DEFAULT_POST_REST_SEC);
  c.irOn = ((int)argNum("ir", DEFAULT_IR_ENABLED ? 1 : 0)) != 0;

  String mode = argStr("mode", "Manual");

  if (mode == "Manual") {
    c.type = CMD_MANUAL;
    c.moveSteps = (long)argNum("moveSteps", 50);
  }
  else if (mode == "Semi-Auto") {
    c.type = CMD_SEMI;
    c.photos = (long)argNum("photos", 2);
    c.moveSteps = (long)argNum("moveSteps", 50);
  }
  else {
    c.type = CMD_FULL;
    c.start = A.start;
    c.end = A.end;
    c.stepsPerPhoto = (long)argNum("stepsPerPhoto", 10);
    c.backlashSteps = (long)argNum("backlash", 100);
    c.interGapSec = (float)argNum("gap", 0.5);
    c.softMin = (long)argNum("softMin", A.softMin);
    c.softMax = (long)argNum("softMax", A.softMax);
  }

  gQueuedCmd = c;
  gHasQueuedCmd = true;
  gStopRequested = false;
  server.send(200, "text/plain", "queued");
}

static void startServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/state", HTTP_GET, handleState);
  server.on("/speed", HTTP_GET, handleSpeed);
  server.on("/microstep", HTTP_GET, handleMicrostep);
  server.on("/profile", HTTP_GET, handleProfile);
  server.on("/cmd", HTTP_GET, handleCmd);
  server.on("/health", HTTP_GET, [](){ server.send(200, "text/plain", "ok"); });
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  server.begin();
}

// ====================== WIFI ======================
static bool connectWiFi(uint8_t maxAttempts = 30) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  for (uint8_t i = 0; WiFi.status() != WL_CONNECTED && i < maxAttempts; i++) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  return WiFi.status() == WL_CONNECTED;
}

// ====================== SETUP / LOOP ======================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== ESP32-S3 TMC2209 MACRO RAIL STARTED =====");

  pinMode(X.stepPin, OUTPUT);
  pinMode(X.dirPin, OUTPUT);
  pinMode(X.enPin, OUTPUT);
  digitalWrite(X.stepPin, LOW);
  enWrite(X.enPin, false);

  pinMode(Z.stepPin, OUTPUT);
  pinMode(Z.dirPin, OUTPUT);
  pinMode(Z.enPin, OUTPUT);
  digitalWrite(Z.stepPin, LOW);
  enWrite(Z.enPin, false);

  loadSysCfg();
  Serial.print("Microstep: 1/");
  Serial.println(gMicrostep);
  Serial.print("Resolution um/step: ");
  Serial.println(gUmPerStep, 6);

  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);
#if defined(ARDUINO_ARCH_ESP32)
  analogSetPinAttenuation(JOY_VRX, ADC_11db);
  analogSetPinAttenuation(JOY_VRY, ADC_11db);
#endif

  bool pwmOk = pwmAttachIR();
  if (!pwmOk) Serial.println("WARNING: IR PWM attach failed");
  irOff();

  Serial.println("Calibrating joystick. Do not touch joystick...");
  calibrateJoystickCenters(700);
  Serial.print("Joystick center X=");
  Serial.print(gJoyCenterX);
  Serial.print(" Y=");
  Serial.println(gJoyCenterY);

  if (connectWiFi()) {
    MDNS.begin(HOSTNAME);
    MDNS.addService("http", "tcp", 80);

    Serial.println("WiFi connected.");
    Serial.print("Open this address: http://");
    Serial.println(WiFi.localIP());
    Serial.print("mDNS address: http://");
    Serial.print(HOSTNAME);
    Serial.println(".local/");
  } else {
    Serial.println("WiFi FAILED. Check SSID/password and use 2.4GHz Wi-Fi.");
  }

  startServer();
  Serial.println("Web server started.");
  driversEnable();  // keep motors locked gently so the next movement does not snap into position
  Serial.println("Motors enabled and holding position.");
  Serial.println("===== SETUP COMPLETE =====");
}

void loop() {
  server.handleClient();

  if (gHasQueuedCmd && !gBusy) {
    MotionCmd c = gQueuedCmd;
    gHasQueuedCmd = false;
    executeCommand(c);
  }

  if (!gBusy) {
    processJoystick();
  }

  delay(1);
}

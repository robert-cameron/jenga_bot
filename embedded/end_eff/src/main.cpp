#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ----------------------------
//  Prongs Controller (Degree-based; cp + cf presets)
// ----------------------------

// ---- Pins ----
const int leftServoPin  = 9;
const int rightServoPin = 10;
const int FORCE_PIN     = A0;

// ---- Presets (can be redefined at runtime via "def ..." cmds) ----
int leftOpenDeg        = 150;
int rightOpenDeg       = 30;

int leftCloseBlockDeg  = 80;
int rightCloseBlockDeg = 100;

int leftCloseFullDeg   = 0;
int rightCloseFullDeg  = 180;

// ---- Motion behaviour ----
const float DEG_RATE_PER_SEC = 200.0f;  // deg/s
float leftTargetDeg, rightTargetDeg;
float leftCurrentDeg, rightCurrentDeg;
unsigned long lastUpdateMs = 0;

// ---- Force Sensor config ----
const float ADC_REF = 5.0f;
bool  g_invert = true;       // true if no-touch = 5V
float g_V0 = 4.90f;          // no-load voltage
float g_V1 = 2.00f;          // loaded voltage
float g_G1 = 500.0f;         // grams at V1

// ---- Comm ----
Servo leftServo, rightServo;
char cmdBuf[96];
uint8_t cmdLen = 0;
unsigned long lastRxMs = 0;
const unsigned long CMD_IDLE_FLUSH_MS = 100;

// ---- Plotting ----
bool g_plotCsv = false;  // CSV time,force for GUI plotters

// ----------------------------
//  Helpers
// ----------------------------
static inline float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

void moveServos(float lDeg, float rDeg) {
  lDeg = clampf(lDeg, 0, 180);
  rDeg = clampf(rDeg, 0, 180);
  leftServo.write(lDeg);
  rightServo.write(rDeg);
  leftCurrentDeg  = lDeg;
  rightCurrentDeg = rDeg;
}

void setTargetAngles(float lDeg, float rDeg) {
  leftTargetDeg  = clampf(lDeg, 0, 180);
  rightTargetDeg = clampf(rDeg, 0, 180);
  Serial.print(F("Target → L: ")); Serial.print(leftTargetDeg);
  Serial.print(F("  R: ")); Serial.println(rightTargetDeg);
}

// ----------------------------
//  Force Sensor
// ----------------------------
float readForceSensor(bool verbose = false) {
  const int N = 10;
  unsigned long sum = 0;
  for (int i = 0; i < N; i++) { sum += analogRead(FORCE_PIN); delay(2); }
  float adc   = sum / float(N);
  float volts = adc * (ADC_REF / 1023.0f);

  float vAdj = g_invert ? (ADC_REF - volts) : volts;
  float v0 = g_invert ? (ADC_REF - g_V0) : g_V0;
  float v1 = g_invert ? (ADC_REF - g_V1) : g_V1;

  float grams = 0.0f;
  float denom = (v1 - v0);
  if (fabs(denom) > 1e-3f) {
    grams = (vAdj - v0) * (g_G1 / denom);
    if (grams < 0) grams = 0;
  }

  if (verbose) {
    Serial.print(F("FS: ")); Serial.print(volts, 3);
    Serial.print(F("V -> ")); Serial.print(grams, 1); Serial.println(F(" g"));
  }
  return grams;
}

// ---- small token parser helpers ----
static bool parseLR(const char* s, int& L, int& R) {
  // accepts "... L<num> R<num>" in any order, spaces optional
  // examples: "L150 R30", "R115 L55"
  L = -1; R = -1;
  const char* p = s;
  while (*p) {
    while (*p == ' ') ++p;
    if (*p == 'L' || *p == 'l') {
      ++p; L = atoi(p);
      while (*p && *p != ' ') ++p;
    } else if (*p == 'R' || *p == 'r') {
      ++p; R = atoi(p);
      while (*p && *p != ' ') ++p;
    } else {
      ++p;
    }
  }
  return (L >= 0 && R >= 0);
}

// ----------------------------
//  Command Handler
// ----------------------------
void handleCommand(const char *cmd) {
  // --- basic motions from current presets ---
  if (!strcmp(cmd, "o")) {
    setTargetAngles(leftOpenDeg, rightOpenDeg);
    Serial.println(F("Opening (preset)."));
    return;
  }
  if (!strcmp(cmd, "cp")) {
    setTargetAngles(leftCloseBlockDeg, rightCloseBlockDeg);
    Serial.print(F("Closing (cp preset) → L: ")); Serial.print(leftCloseBlockDeg);
    Serial.print(F("  R: ")); Serial.println(rightCloseBlockDeg);
    return;
  }
  if (!strcmp(cmd, "cf")) {
    setTargetAngles(leftCloseFullDeg, rightCloseFullDeg);
    Serial.println(F("Closing fully (preset)."));
    return;
  }

  // --- absolute pose: "pose L<num> R<num>" (atomic both) ---
  if (!strncmp(cmd, "pose", 4)) {
    int L, R;
    if (parseLR(cmd + 4, L, R)) {
      setTargetAngles(L, R);
      Serial.print(F("Pose set → L: ")); Serial.print(L);
      Serial.print(F("  R: ")); Serial.println(R);
    } else {
      Serial.println(F("ERR: use 'pose L<deg> R<deg>'"));
    }
    return;
  }

  // --- redefine presets at runtime: "def <o|cp|cf> L<num> R<num>" ---
  if (!strncmp(cmd, "def ", 4)) {
    const char* p = cmd + 4;
    while (*p == ' ') ++p;
    if (!strncmp(p, "o", 1)) {
      int L, R; if (parseLR(p + 1, L, R)) {
        leftOpenDeg = (int)clampf(L, 0, 180);
        rightOpenDeg = (int)clampf(R, 0, 180);
        Serial.print(F("Defined o → L: ")); Serial.print(leftOpenDeg);
        Serial.print(F("  R: ")); Serial.println(rightOpenDeg);
      } else Serial.println(F("ERR: def o L<deg> R<deg>"));
      return;
    }
    if (!strncmp(p, "cp", 2)) {
      int L, R; if (parseLR(p + 2, L, R)) {
        leftCloseBlockDeg  = (int)clampf(L, 0, 180);
        rightCloseBlockDeg = (int)clampf(R, 0, 180);
        Serial.print(F("Defined cp → L: ")); Serial.print(leftCloseBlockDeg);
        Serial.print(F("  R: ")); Serial.println(rightCloseBlockDeg);
      } else Serial.println(F("ERR: def cp L<deg> R<deg>"));
      return;
    }
    if (!strncmp(p, "cf", 2)) {
      int L, R; if (parseLR(p + 2, L, R)) {
        leftCloseFullDeg  = (int)clampf(L, 0, 180);
        rightCloseFullDeg = (int)clampf(R, 0, 180);
        Serial.print(F("Defined cf → L: ")); Serial.print(leftCloseFullDeg);
        Serial.print(F("  R: ")); Serial.println(rightCloseFullDeg);
      } else Serial.println(F("ERR: def cf L<deg> R<deg>"));
      return;
    }
    Serial.println(F("ERR: def expects o|cp|cf"));
    return;
  }

  // --- legacy single-servo setters (still supported) ---
  if (!strncmp(cmd, "setL", 4)) { float v = atof(cmd + 4); setTargetAngles(v, rightTargetDeg); Serial.println(F("Left target set.")); return; }
  if (!strncmp(cmd, "setR", 4)) { float v = atof(cmd + 4); setTargetAngles(leftTargetDeg, v); Serial.println(F("Right target set.")); return; }

  // --- diagnostics & plot toggles ---
  if (!strcmp(cmd, "force")) { (void)readForceSensor(true); return; }
  if (!strcmp(cmd, "plot on"))  { g_plotCsv = true;  Serial.println(F("Plotting mode ON (CSV)."));  return; }
  if (!strcmp(cmd, "plot off")) { g_plotCsv = false; Serial.println(F("Plotting mode OFF."));       return; }

  if (!strcmp(cmd, "help")) {
    Serial.println(F("Commands:"));
    Serial.println(F("  o | cp | cf                  (move to presets)"));
    Serial.println(F("  def o  L<deg> R<deg>         (define open)"));
    Serial.println(F("  def cp L<deg> R<deg>         (define cp)"));
    Serial.println(F("  def cf L<deg> R<deg>         (define cf)"));
    Serial.println(F("  pose L<deg> R<deg>           (absolute, atomic)"));
    Serial.println(F("  setL<num> | setR<num>        (legacy single-servo)"));
    Serial.println(F("  plot on/off, force, help"));
    return;
  }

  Serial.println(F("ERR: unknown (try 'help')"));
}

// ----------------------------
//  Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // BOOT: closed-full
  moveServos(leftCloseFullDeg, rightCloseFullDeg);
  leftTargetDeg  = leftCloseFullDeg;
  rightTargetDeg = rightCloseFullDeg;

  pinMode(FORCE_PIN, INPUT);
  lastUpdateMs = millis();

  Serial.println(F("Prongs ready (boot: CF)."));
  Serial.println(F("Use 'def o/cp/cf L.. R..' from ROS to redefine presets."));
}

// ----------------------------
//  Main Loop
// ----------------------------
void loop() {
  // Command parser
  while (Serial.available() > 0) {
    char ch = Serial.read();
    lastRxMs = millis();
    if (isspace(ch) || ch == ',' || ch == '\n') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
  if (cmdLen > 0 && (millis() - lastRxMs) > CMD_IDLE_FLUSH_MS) {
    cmdBuf[cmdLen] = '\0';
    handleCommand(cmdBuf);
    cmdLen = 0;
  }

  // Smooth ramp toward target
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt > 0) {
    float maxStep = DEG_RATE_PER_SEC * dt;
    float lDelta = leftTargetDeg - leftCurrentDeg;
    float rDelta = rightTargetDeg - rightCurrentDeg;
    if (fabs(lDelta) > 1e-2f) leftCurrentDeg += clampf(lDelta, -maxStep, maxStep);
    if (fabs(rDelta) > 1e-2f) rightCurrentDeg += clampf(rDelta, -maxStep, maxStep);
    moveServos(leftCurrentDeg, rightCurrentDeg);
  }
  lastUpdateMs = now;

  // Force out (both CSV and ~g= for ROS parsing)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) { // ~20 Hz
    float f = readForceSensor(false);
    if (g_plotCsv) {
      Serial.print(millis()); Serial.print(','); Serial.println(f, 2);
    } else {
      Serial.print(F("~g=")); Serial.println(f, 2); // <- ROS bridge regex friendly
    }
    lastPrint = millis();
  }
}

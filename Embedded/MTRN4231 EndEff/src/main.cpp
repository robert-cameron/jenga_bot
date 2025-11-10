#include <Arduino.h>
#include <Servo.h>

// ----------------------------
//  Prongs Controller (Degree-based; cp + cf presets)
// ----------------------------

// ---- Pins ----
const int leftServoPin  = 9;
const int rightServoPin = 10;
const int FORCE_PIN     = A0;

// ---- Servo calibration (set these manually) ----
// Physical servo angles for each state
int leftOpenDeg        = 150;
int leftCloseBlockDeg  = 55;   // partial close (block grip)
int leftCloseFullDeg   = 0;   // full close

int rightOpenDeg       = 30;
int rightCloseBlockDeg = 115;    // partial close (block grip)
int rightCloseFullDeg  = 180;     // full close

// ---- Motion behaviour ----
const float DEG_RATE_PER_SEC = 200.0f;  // speed of servo movement (deg/s)
float leftTargetDeg  = leftOpenDeg;
float rightTargetDeg = rightOpenDeg;
float leftCurrentDeg = leftOpenDeg;
float rightCurrentDeg = rightOpenDeg;
unsigned long lastUpdateMs = 0;

// ---- Force Sensor config ----
const float ADC_REF = 5.0f;
bool  g_invert = true;       // true if no-touch = 5V
float g_V0 = 4.90f;          // no-load voltage
float g_V1 = 2.00f;          // loaded voltage
float g_G1 = 500.0f;         // grams at V1

// ---- Comm ----
Servo leftServo, rightServo;
char cmdBuf[32];
uint8_t cmdLen = 0;
unsigned long lastRxMs = 0;
const unsigned long CMD_IDLE_FLUSH_MS = 100;

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
  // Serial.print("L="); Serial.print(lDeg); Serial.print("  R="); Serial.println(rDeg);
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

// ----------------------------
//  Command Handler
// ----------------------------
void handleCommand(const char *cmd) {
  if (!strcmp(cmd, "o")) {               // Open
    setTargetAngles(leftOpenDeg, rightOpenDeg);
    Serial.println(F("Opening..."));
    return;
  }
  if (!strcmp(cmd, "cp")) {              // Close for block
    setTargetAngles(leftCloseBlockDeg, rightCloseBlockDeg);
    Serial.println(F("Closing (block grip)..."));
    return;
  }
  if (!strcmp(cmd, "cf")) {              // Close fully
    setTargetAngles(leftCloseFullDeg, rightCloseFullDeg);
    Serial.println(F("Closing fully..."));
    return;
  }
  if (!strncmp(cmd, "setL", 4)) {        // setL###
    float v = atof(cmd + 4);
    setTargetAngles(v, rightTargetDeg);
    Serial.println(F("Left servo target set."));
    return;
  }
  if (!strncmp(cmd, "setR", 4)) {        // setR###
    float v = atof(cmd + 4);
    setTargetAngles(leftTargetDeg, v);
    Serial.println(F("Right servo target set."));
    return;
  }
  if (!strcmp(cmd, "force")) {
    readForceSensor(true);
    return;
  }
  if (!strcmp(cmd, "help")) {
    Serial.println(F("Commands:"));
    Serial.println(F("  o      -> open"));
    Serial.println(F("  cp     -> close for block (partial close)"));
    Serial.println(F("  cf     -> close fully"));
    Serial.println(F("  setL<num> -> set left servo angle"));
    Serial.println(F("  setR<num> -> set right servo angle"));
    Serial.println(F("  force  -> print force"));
    return;
  }

  Serial.println(F("ERR: unknown (use o/cp/cf/setL#/setR#/force/help)"));
}

// ----------------------------
//  Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  moveServos(leftOpenDeg, rightOpenDeg);

  pinMode(FORCE_PIN, INPUT);
  lastUpdateMs = millis();

  Serial.println(F("Prongs ready."));
  Serial.println(F("Commands: o=open, cp=close_block, cf=close_full, setL#, setR#, force, help"));
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

  // Smoothly move current servos toward target
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt > 0) {
    float maxStep = DEG_RATE_PER_SEC * dt;
    float lDelta = leftTargetDeg - leftCurrentDeg;
    float rDelta = rightTargetDeg - rightCurrentDeg;

    if (fabs(lDelta) > 1e-2f)
      leftCurrentDeg += clampf(lDelta, -maxStep, maxStep);
    if (fabs(rDelta) > 1e-2f)
      rightCurrentDeg += clampf(rDelta, -maxStep, maxStep);

    moveServos(leftCurrentDeg, rightCurrentDeg);
  }
  lastUpdateMs = now;

  // Live force print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    float f = readForceSensor(false);
    Serial.print(F("Force ≈ ")); Serial.print(f, 1); Serial.println(F(" g"));
    lastPrint = millis();
  }
}

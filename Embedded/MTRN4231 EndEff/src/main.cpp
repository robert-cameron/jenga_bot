#include <Arduino.h>
#include <Servo.h>

// ----------------------------
//  Prongs Servo Controller (Degrees + Force Sensor + Opposite Rotation)
// ----------------------------

// ---- Pins ----
const int leftServoPin  = 9;
const int rightServoPin = 10;
const int FORCE_PIN     = A0;

// ---- Servo calibration ----
//  Tune these so that the claw fully opens/closes correctly.
float leftDeg   = 90.0f;   // starting position
float rightDeg  = 90.0f;

int leftOpenDeg   = 60;    // smaller angle for open
int leftCloseDeg  = 120;   // larger angle for close
int rightOpenDeg  = 120;   // larger angle for open
int rightCloseDeg = 60;    // smaller angle for close

// ---- Motion behaviour ----
const float RATE_DEG_PER_SEC = 200.0f;   // how fast to move continuously
int motion_dir = 0;                      // +1=open, -1=close, 0=stop
unsigned long lastUpdateMs = 0;

// ---- Force Sensor config ----
const float ADC_REF = 5.0f;  // reference voltage (5V)
bool  g_invert = true;       // true if no-touch = 5V
float g_V0 = 4.90f;          // no-load voltage
float g_V1 = 2.00f;          // loaded voltage
float g_G1 = 500.0f;         // grams at V1

// ---- Communication buffers ----
Servo leftServo, rightServo;
char cmdBuf[32];
uint8_t cmdLen = 0;
unsigned long lastRxMs = 0;
const unsigned long CMD_IDLE_FLUSH_MS = 100;

// ----------------------------
//  Force Sensor
// ----------------------------
float readForceSensor(bool verbose = false) {
  const int N = 10;
  unsigned long sum = 0;
  for (int i = 0; i < N; i++) { sum += analogRead(FORCE_PIN); delay(2); }
  float adc   = sum / float(N);
  float volts = adc * (ADC_REF / 1023.0f);

  // invert reading if no-touch is high
  float vAdj = g_invert ? (ADC_REF - volts) : volts;

  // linear mapping from (V0, 0g) → (V1, G1)
  float grams = 0.0f;
  float denom = (g_V1 - g_V0);
  if (fabs(denom) > 1e-3f) {
    grams = (vAdj - (g_invert ? (ADC_REF - g_V0) : g_V0)) *
            (g_G1 / ((g_invert ? (ADC_REF - g_V1) : g_V1) -
                     (g_invert ? (ADC_REF - g_V0) : g_V0)));
    if (grams < 0) grams = 0;
  }

  if (verbose) {
    Serial.print(F("FS: ")); Serial.print(volts, 3);
    Serial.print(F("V -> ")); Serial.print(grams, 1); Serial.println(F(" g"));
  }
  return grams;
}

// ----------------------------
//  Servo Motion
// ----------------------------
void moveServos(float lDeg, float rDeg) {
  // Opposite rotation mapping
  lDeg = constrain(lDeg, 0, 180);
  rDeg = constrain(rDeg, 0, 180);

  // The key: right servo should mirror left (opposite direction)
  leftServo.write(lDeg);
  rightServo.write(180 - rDeg);

  Serial.print(F("L=")); Serial.print(lDeg, 1);
  Serial.print(F("  R=")); Serial.println(180 - rDeg, 1);

  leftDeg = lDeg;
  rightDeg = rDeg;
}

// ----------------------------
//  Command Handler
// ----------------------------
void handleCommand(const char *cmd) {
  if (!strcmp(cmd, "o")) { 
    motion_dir = +1;
    Serial.println(F("Opening..."));
    return;
  }
  if (!strcmp(cmd, "c")) { 
    motion_dir = -1;
    Serial.println(F("Closing..."));
    return;
  }
  if (!strcmp(cmd, "s")) { 
    motion_dir = 0;
    Serial.println(F("Stopped."));
    return;
  }
  if (!strncmp(cmd, "setL", 4)) {
    float v = atof(cmd + 4);
    leftDeg = constrain(v, 0, 180);
    moveServos(leftDeg, rightDeg);
    Serial.println(F("Left servo set"));
    return;
  }
  if (!strncmp(cmd, "setR", 4)) {
    float v = atof(cmd + 4);
    rightDeg = constrain(v, 0, 180);
    moveServos(leftDeg, rightDeg);
    Serial.println(F("Right servo set"));
    return;
  }
  if (!strcmp(cmd, "force")) {
    readForceSensor(true);
    return;
  }
  if (!strcmp(cmd, "help")) {
    Serial.println(F("Commands:"));
    Serial.println(F("  o     -> start opening"));
    Serial.println(F("  c     -> start closing"));
    Serial.println(F("  s     -> stop motion"));
    Serial.println(F("  setL<num> / setR<num> -> set servo manually (0–180)"));
    Serial.println(F("  force -> print force"));
    return;
  }

  Serial.println(F("ERR: unknown (use o/c/s/setL/setR/force/help)"));
}

// ----------------------------
//  Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  moveServos(leftDeg, rightDeg);

  pinMode(FORCE_PIN, INPUT);
  lastUpdateMs = millis();

  Serial.println(F("Prongs ready."));
  Serial.println(F("Commands: o=open, c=close, s=stop, setL#, setR#, force, help"));
}

// ----------------------------
//  Main Loop
// ----------------------------
void loop() {
  // Command parser
  while (Serial.available() > 0) {
    char ch = Serial.read();
    lastRxMs = millis();
    if (isspace(ch) || ch == ',') {
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

  // Continuous motion
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt > 0 && motion_dir != 0) {
    leftDeg  += motion_dir * RATE_DEG_PER_SEC * dt;
    rightDeg += motion_dir * RATE_DEG_PER_SEC * dt;  // both increase, but right is mirrored when written
    moveServos(leftDeg, rightDeg);
  }
  lastUpdateMs = now;

  // Live force print every 250ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    float f = readForceSensor(false);
    Serial.print(F("Force ≈ ")); Serial.print(f, 1); Serial.println(F(" g"));
    lastPrint = millis();
  }
}

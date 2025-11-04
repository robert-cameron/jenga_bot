// src/main.cpp
#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>
#include <EEPROM.h>

// ---- Pins ----
const int leftServoPin  = 9;
const int rightServoPin = 10;

// --- Force Sensor ---
const int   FORCE_PIN = A0;     // Analog input from MH module A0 (or divider node)
const float ADC_REF   = 5.0f;   // 5V reference

// ---- Geometry/Calibration ----
const float MIN_GAP_MM = 0.0f;
const float MAX_GAP_MM = 40.0f;

// Servo angles at those endpoints:
const int LEFT_ANGLE_AT_MIN  = 40;
const int LEFT_ANGLE_AT_MAX  = 100;
const int RIGHT_ANGLE_AT_MIN = 100;
const int RIGHT_ANGLE_AT_MAX = 40;

// ---- Preset gaps (mm) ----
float openGapMM  = 30.0f;
float closeGapMM = 5.0f;

// ---- EEPROM layout ----
const int EE_MAGIC_ADDR   = 0;
const int EE_OPEN_ADDR    = EE_MAGIC_ADDR + 4;
const int EE_CLOSE_ADDR   = EE_OPEN_ADDR + sizeof(float);
const int EE_LAST_ADDR    = EE_CLOSE_ADDR + sizeof(float);
const uint32_t EE_MAGIC   = 0x504A4750UL;

// ---- Globals ----
Servo leftServo, rightServo;
char cmdBuf[48];
uint8_t cmdLen = 0;
const unsigned long CMD_IDLE_FLUSH_MS = 120;
unsigned long lastRxMs = 0;
const unsigned long STARTUP_CATCH_MS = 300;
float lastGapMM = -1.0f;

// --- Force reading state ---
bool  g_invert = true;      // << set true because your no-touch is ~5V
float g_V0 = 4.90f;         // no-touch voltage (0 g) – set via 'cal0'
float g_V1 = 2.00f;         // known-weight voltage – set via 'cal1 <g>'
float g_G1 = 500.0f;        // grams at V1

// --- Helpers ---
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
int lerpAngle(int a0, int a1, float t) {
  float val = a0 + (a1 - a0) * t;
  int iv = (int)(val + (val >= 0 ? 0.5f : -0.5f));
  return clampi(iv, 0, 180);
}

// --- EEPROM helpers ---
void eepromWriteFloat(int addr, float v) { EEPROM.put(addr, v); }
float eepromReadFloat(int addr, float fallback) {
  float v; EEPROM.get(addr, v);
  if (v < (MIN_GAP_MM - 1.0f) || v > (MAX_GAP_MM + 50.0f)) return fallback;
  return v;
}
void saveOpen()          { eepromWriteFloat(EE_OPEN_ADDR,  openGapMM); }
void saveClose()         { eepromWriteFloat(EE_CLOSE_ADDR, closeGapMM); }
void saveLast(float val) { eepromWriteFloat(EE_LAST_ADDR,  val); }

void loadPresetsAndLast() {
  uint32_t magic = 0; EEPROM.get(EE_MAGIC_ADDR, magic);
  if (magic == EE_MAGIC) {
    openGapMM  = eepromReadFloat(EE_OPEN_ADDR,  openGapMM);
    closeGapMM = eepromReadFloat(EE_CLOSE_ADDR, closeGapMM);
    lastGapMM  = eepromReadFloat(EE_LAST_ADDR,  closeGapMM);
  } else {
    EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
    saveOpen();
    saveClose();
    lastGapMM = closeGapMM;
    saveLast(lastGapMM);
  }
}

// --- Force Sensor read + simple 2-point calibration ---
float readForceSensor(bool verbose=true) {
  // Smooth analog reading
  const int N = 10;
  unsigned long sum = 0;
  for (int i = 0; i < N; i++) { sum += analogRead(FORCE_PIN); delay(2); }
  float adc   = sum / float(N);
  float volts = adc * (ADC_REF / 1023.0f);

  // Invert if no-touch is high
  float vAdj = g_invert ? (ADC_REF - volts) : volts;

  // Linear fit through (V0, 0 g) and (V1, G1)
  float grams = 0.0f;
  float denom = (g_V1 - g_V0);
  if (fabs(denom) > 1e-3f) {
    grams = (vAdj - (g_invert ? (ADC_REF - g_V0) : g_V0)) *
            (g_G1 / ((g_invert ? (ADC_REF - g_V1) : g_V1) - (g_invert ? (ADC_REF - g_V0) : g_V0)));
    if (grams < 0) grams = 0;
  }

  if (verbose) {
    Serial.print(F("FS: raw="));  Serial.print(volts, 3);
    Serial.print(F("V  adj="));   Serial.print(vAdj, 3);
    Serial.print(F("V  ~g="));    Serial.println(grams, 1);
  }
  return grams;
}

// ---- Motion ----
void moveToGapMM(float gapMM) {
  gapMM = clampf(gapMM, MIN_GAP_MM, MAX_GAP_MM);
  float t = (gapMM - MIN_GAP_MM) / (MAX_GAP_MM - MIN_GAP_MM);
  int leftAngle  = lerpAngle(LEFT_ANGLE_AT_MIN,  LEFT_ANGLE_AT_MAX,  t);
  int rightAngle = lerpAngle(RIGHT_ANGLE_AT_MIN, RIGHT_ANGLE_AT_MAX, t);
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
  lastGapMM = gapMM; saveLast(lastGapMM);
  Serial.print(F("OK: gap=")); Serial.println(gapMM, 2);
}

// ---- Commands ----
void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  open | close"));
  Serial.println(F("  gap <mm>           -> move to gap (0..40)"));
  Serial.println(F("  setopen <mm>       -> save open preset"));
  Serial.println(F("  setclose <mm>      -> save close preset"));
  Serial.println(F("  force              -> print force reading once"));
  Serial.println(F("  invert on|off      -> set force inversion"));
  Serial.println(F("  cal0               -> set current voltage as 0 g"));
  Serial.println(F("  cal1 <grams>       -> set current voltage as given grams"));
  Serial.println(F("  help"));
}

void handleCommand(const char *cmd) {
  char buf[64];
  strncpy(buf, cmd, sizeof(buf)); buf[sizeof(buf)-1] = '\0';
  char *tok = strtok(buf, " \t");
  if (!tok) { Serial.println(F("ERR: empty command")); return; }

  if (!strcmp(tok, "help")) { printHelp(); return; }

  if (!strcmp(tok, "open"))  { moveToGapMM(openGapMM);  return; }
  if (!strcmp(tok, "close")) { moveToGapMM(closeGapMM); return; }

  if (!strcmp(tok, "gap")) {
    char *v = strtok(NULL, " \t");
    if (!v) { Serial.println(F("ERR: gap <mm>")); return; }
    moveToGapMM(clampf(atof(v), MIN_GAP_MM, MAX_GAP_MM));
    return;
  }
  if (!strcmp(tok, "setopen")) {
    char *v = strtok(NULL, " \t"); if (!v) { Serial.println(F("ERR: setopen <mm>")); return; }
    openGapMM = clampf(atof(v), MIN_GAP_MM, MAX_GAP_MM); saveOpen(); Serial.println(F("OK"));
    return;
  }
  if (!strcmp(tok, "setclose")) {
    char *v = strtok(NULL, " \t"); if (!v) { Serial.println(F("ERR: setclose <mm>")); return; }
    closeGapMM = clampf(atof(v), MIN_GAP_MM, MAX_GAP_MM); saveClose(); Serial.println(F("OK"));
    return;
  }
  if (!strcmp(tok, "force")) { readForceSensor(true); return; }

  if (!strcmp(tok, "invert")) {
    char *v = strtok(NULL, " \t");
    if (!v) { Serial.println(F("invert on|off")); return; }
    if (!strcmp(v, "on"))  { g_invert = true;  Serial.println(F("invert=on")); }
    else if (!strcmp(v, "off")) { g_invert = false; Serial.println(F("invert=off")); }
    else Serial.println(F("invert on|off"));
    return;
  }

  if (!strcmp(tok, "cal0")) {
    // current voltage becomes zero-load
    float grams = readForceSensor(false);
    // reconstruct raw volts from last read
    int   raw = analogRead(FORCE_PIN);
    float v   = raw * (ADC_REF / 1023.0f);
    g_V0 = v;
    Serial.print(F("Cal0 set: V0=")); Serial.println(g_V0, 3);
    return;
  }

  if (!strcmp(tok, "cal1")) {
    char *g = strtok(NULL, " \t");
    if (!g) { Serial.println(F("ERR: cal1 <grams>")); return; }
    g_G1 = atof(g);
    // measure current volts as V1
    int   raw = analogRead(FORCE_PIN);
    float v   = raw * (ADC_REF / 1023.0f);
    g_V1 = v;
    Serial.print(F("Cal1 set: V1=")); Serial.print(g_V1, 3);
    Serial.print(F("  G1="));         Serial.println(g_G1, 1);
    return;
  }

  Serial.println(F("ERR: unknown command (try 'help')"));
}

// --- setup ---
void setup() {
  delay(50);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < STARTUP_CATCH_MS) {}

  loadPresetsAndLast();
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  moveToGapMM(lastGapMM);

  pinMode(FORCE_PIN, INPUT);

  Serial.println(F("Prongs ready with force sensor."));
  Serial.print(F("Presets: open="));  Serial.print(openGapMM);
  Serial.print(F(" mm, close="));     Serial.print(closeGapMM);
  Serial.print(F(" mm | last="));     Serial.println(lastGapMM);
  Serial.println(F("Type 'help' for commands."));
}

// --- loop ---
void loop() {
  // Command parser with idle-flush
  while (Serial.available() > 0) {
    char ch = Serial.read();
    lastRxMs = millis();
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t' || ch == ',') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; handleCommand(cmdBuf); cmdLen = 0; }
      continue;
    }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = ch;
    else { cmdLen = 0; Serial.println(F("ERR: command too long")); }
  }
  if (cmdLen > 0 && (millis() - lastRxMs) > CMD_IDLE_FLUSH_MS) {
    cmdBuf[cmdLen] = '\0'; handleCommand(cmdBuf); cmdLen = 0;
  }

  // Print force every 250 ms for live view
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    readForceSensor(true);
    lastPrint = millis();
  }
}

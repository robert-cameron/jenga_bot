// src/main.cpp
#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>    // strtod
#include <string.h>    // strtok, strcasecmp
#include <EEPROM.h>    // persist presets & last gap

// ---- Pins ----
const int leftServoPin  = 9;
const int rightServoPin = 10;

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
const int EE_LAST_ADDR    = EE_CLOSE_ADDR + sizeof(float);   // NEW: last gap
const uint32_t EE_MAGIC   = 0x504A4750UL; // "PJGP"

// ---- Globals ----
Servo leftServo, rightServo;
char cmdBuf[48];
uint8_t cmdLen = 0;

const unsigned long CMD_IDLE_FLUSH_MS = 120;
unsigned long lastRxMs = 0;
const unsigned long STARTUP_CATCH_MS = 300;

// NEW: remember current/last gap in RAM too
float lastGapMM = -1.0f;

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

// ---- EEPROM helpers ----
void eepromWriteFloat(int addr, float v) { EEPROM.put(addr, v); }
float eepromReadFloat(int addr, float fallback) {
  float v; EEPROM.get(addr, v);
  if (v < (MIN_GAP_MM - 1.0f) || v > (MAX_GAP_MM + 50.0f)) return fallback;
  return v;
}
void saveOpen()          { eepromWriteFloat(EE_OPEN_ADDR,  openGapMM); }
void saveClose()         { eepromWriteFloat(EE_CLOSE_ADDR, closeGapMM); }
void saveLast(float val) { eepromWriteFloat(EE_LAST_ADDR,  val); }     // NEW

void loadPresetsAndLast() {
  uint32_t magic = 0; EEPROM.get(EE_MAGIC_ADDR, magic);
  if (magic == EE_MAGIC) {
    openGapMM  = eepromReadFloat(EE_OPEN_ADDR,  openGapMM);
    closeGapMM = eepromReadFloat(EE_CLOSE_ADDR, closeGapMM);
    lastGapMM  = eepromReadFloat(EE_LAST_ADDR,  closeGapMM); // default to close
  } else {
    EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
    saveOpen();
    saveClose();
    lastGapMM = closeGapMM;
    saveLast(lastGapMM);
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  open            -> move to saved open gap (persists as last)"));
  Serial.println(F("  open <mm>       -> set & move to open gap (persists preset + last)"));
  Serial.println(F("  close           -> move to saved close gap (persists as last)"));
  Serial.println(F("  close <mm>      -> set & move to close gap (persists preset + last)"));
  Serial.println(F("  set <mm>        -> move to <mm> (persists as last)"));
  Serial.println(F("  <mm>            -> same as 'set <mm>'"));
  Serial.print(  F("Range: [")); Serial.print(MIN_GAP_MM, 1);
  Serial.print(  F(", "));       Serial.print(MAX_GAP_MM, 1);
  Serial.println(F("] mm"));
}

// Map desired gap (mm) to servo angles using linear interpolation
void moveToGapMM(float gapMM) {
  gapMM = clampf(gapMM, MIN_GAP_MM, MAX_GAP_MM);
  float denom = (MAX_GAP_MM - MIN_GAP_MM);
  float t = (denom > 1e-6f) ? (gapMM - MIN_GAP_MM) / denom : 0.0f;

  int leftAngle  = lerpAngle(LEFT_ANGLE_AT_MIN,  LEFT_ANGLE_AT_MAX,  t);
  int rightAngle = lerpAngle(RIGHT_ANGLE_AT_MIN, RIGHT_ANGLE_AT_MAX, t);

  leftServo.write(leftAngle);
  rightServo.write(rightAngle);

  // NEW: persist the commanded position so resets restore it
  lastGapMM = gapMM;
  saveLast(lastGapMM);

  Serial.print(F("OK: gap="));
  Serial.print(gapMM, 2);
  Serial.print(F("mm  (L="));
  Serial.print(leftAngle);
  Serial.print(F(", R="));
  Serial.print(rightAngle);
  Serial.println(F(")"));
}

bool parseNumber(const char* s, float &out) {
  if (!s || !*s) return false;
  char* endp = nullptr;
  out = (float)strtod(s, &endp);
  return (endp && endp != s);
}

void handleCommand(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (!*line) return;

  // Bare number → set & persist as last
  float vBare;
  if (parseNumber(line, vBare)) { moveToGapMM(vBare); return; }

  char* cmd = strtok(line, " ,\t\r\n");
  if (!cmd) return;
  char* arg = strtok(nullptr, " ,\t\r\n");

  char c0 = cmd[0];
  if (c0 == 'o' || c0 == 'O') {
    if (arg) {
      float v;
      if (parseNumber(arg, v)) {
        openGapMM = clampf(v, MIN_GAP_MM, MAX_GAP_MM);
        saveOpen();
        Serial.print(F("Set openGapMM=")); Serial.println(openGapMM, 2);
      } else { Serial.println(F("ERR: open <mm>")); return; }
    }
    moveToGapMM(openGapMM);   // persists as last
    return;
  }

  if (c0 == 'c' || c0 == 'C') {
    if (arg) {
      float v;
      if (parseNumber(arg, v)) {
        closeGapMM = clampf(v, MIN_GAP_MM, MAX_GAP_MM);
        saveClose();
        Serial.print(F("Set closeGapMM=")); Serial.println(closeGapMM, 2);
      } else { Serial.println(F("ERR: close <mm>")); return; }
    }
    moveToGapMM(closeGapMM);  // persists as last
    return;
  }

  if (strcasecmp(cmd, "set") == 0 || strcasecmp(cmd, "s") == 0) {
    if (!arg) { Serial.println(F("ERR: set <mm>")); return; }
    float v; if (!parseNumber(arg, v)) { Serial.println(F("ERR: set <mm>")); return; }
    moveToGapMM(v);           // persists as last
    return;
  }

  if (strcasecmp(cmd, "help") == 0 || strcasecmp(cmd, "?") == 0) {
    printHelp(); return;
  }

  Serial.println(F("ERR: unknown cmd. Try 'help'"));
}

void setup() {
  delay(50);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < STARTUP_CATCH_MS) { /* brief wait */ }

  loadPresetsAndLast();

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // NEW: Restore the last commanded gap (not forced-close)
  moveToGapMM(lastGapMM);

  Serial.println(F("Prongs ready."));
  Serial.println(F("Type 'help' for commands."));
  Serial.print(F("Presets: open="));  Serial.print(openGapMM, 2);
  Serial.print(F(" mm, close="));     Serial.print(closeGapMM, 2);
  Serial.print(F(" mm | last="));     Serial.print(lastGapMM, 2);
  Serial.println(F(" mm"));
}

void loop() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    lastRxMs = millis();

    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t' || ch == ',') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
      continue;
    }

    if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    } else {
      cmdLen = 0;
      Serial.println(F("ERR: command too long"));
    }
  }

  if (cmdLen > 0 && (millis() - lastRxMs) > CMD_IDLE_FLUSH_MS) {
    cmdBuf[cmdLen] = '\0';
    handleCommand(cmdBuf);
    cmdLen = 0;
  }
}

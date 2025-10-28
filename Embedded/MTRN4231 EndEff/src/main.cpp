// src/main.cpp
#include <Arduino.h>
#include <Servo.h>

// ---- Pins ----
const int leftServoPin  = 9;
const int rightServoPin = 10;

// ---- Angle calibration (tune these) ----
const int LEFT_OPEN_ANGLE   = 120;
const int LEFT_CLOSE_ANGLE  = 60;
const int RIGHT_OPEN_ANGLE  = 60;     // mirror of left
const int RIGHT_CLOSE_ANGLE = 120;

// ---- Globals ----
Servo leftServo, rightServo;
bool isOpen = false;

// Small input buffer to accept words like "open" or "close"
char cmdBuf[16];
uint8_t cmdLen = 0;

void openProngs() {
  leftServo.write(LEFT_OPEN_ANGLE);
  rightServo.write(RIGHT_OPEN_ANGLE);
  isOpen = true;
  Serial.println(F("OK: open"));
}

void closeProngs() {
  leftServo.write(LEFT_CLOSE_ANGLE);
  rightServo.write(RIGHT_CLOSE_ANGLE);
  isOpen = false;
  Serial.println(F("OK: close"));
}

void handleCommand(const char* s) {
  // Accept single chars or full words (case-insensitive)
  if (s[0] == 'o' || s[0] == 'O' || strcasecmp(s, "open") == 0) {
    openProngs();
  } else if (s[0] == 'c' || s[0] == 'C' || strcasecmp(s, "close") == 0) {
    closeProngs();
  } else {
    Serial.println(F("ERR: use 'o'/'open' or 'c'/'close'"));
  }
}

void setup() {
  Serial.begin(115200);
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // Start closed (safe)
  closeProngs();

  Serial.println(F("Two-servo prongs ready. Commands: 'o'/'open', 'c'/'close'"));
}

void loop() {
  // Read bytes as they arrive
  while (Serial.available() > 0) {
    char ch = Serial.read();

    // Treat CR/LF/space as delimiters
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;  // reset buffer
      }
      continue;
    }

    // Single-char immediate commands
    if (ch == 'o' || ch == 'O') {
      openProngs();
      cmdLen = 0;
      continue;
    } else if (ch == 'c' || ch == 'C') {
      closeProngs();
      cmdLen = 0;
      continue;
    }

    // Build word command (e.g., "open", "close")
    if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    } else {
      // Overflow protection: reset buffer
      cmdLen = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
}

#include <Arduino.h>
#define DEBUGSerial Serial

const int sensorPin = A0;

// 压力范围（单位：克）
// Pressure range (unit: grams)
#define PRESS_MIN 20
#define PRESS_MAX 6000

// 电压范围（单位：毫伏）
// Voltage range (unit: millivolts)
#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 5000

// 重力判断阈值（单位：克）
// Force thresholds (unit: grams)
#define PRESS_THRESHOLD_LOW 20    // 低于此值认为为空载 EMPTY
#define PRESS_THRESHOLD_HIGH 100  // 高于此值认为超载 NO

// 人工校准偏移量（单位：克）
// Manual calibration offset (unit: grams)
#define MANUAL_OFFSET 1967

// 自动校准参数（已更新）
// Auto-calibration parameters (updated)
#define AUTO_CALIBRATION_COUNT 5       // 连续多少次触发自动校准 / Number of consecutive low readings
#define AUTO_CALIBRATION_RANGE 30      // 低于此值认为是空重 / Threshold for auto-zero
#define AUTO_DISABLE_THRESHOLD 10      // 自动校准完成后低于此值将暂停自动校准 / Disable auto-calibration if below this

long autoOffset = 0;                    // 自动校准偏移量 / Auto-calibration offset
int lowCount = 0;                       // 连续低值计数器 / Consecutive low readings counter
long lowSum = 0;                        // 累积低值总和 / Sum of low readings
bool autoEnabled = true;               // 是否启用自动校准 / Auto-calibration enabled
int disableCount = 0;                  // 自动校准后低值计数 / Post-calibration low readings counter
int enableCount = 0;                   // 恢复自动校准的高值计数 / High readings counter to re-enable

void setup() {
  DEBUGSerial.begin(9600);
  DEBUGSerial.println("setup end!");  // 初始化完成 / Setup complete
}

void loop() {
  long Fdata = getPressValue(sensorPin);

  // 自动校准逻辑 / Auto-calibration logic
  if (autoEnabled) {
    if (Fdata >= 0 && Fdata < AUTO_CALIBRATION_RANGE) {
      lowCount++;
      lowSum += Fdata;

      if (lowCount >= AUTO_CALIBRATION_COUNT) {
        autoOffset = lowSum / lowCount;
        DEBUGSerial.println("Auto-calibration applied");
        lowCount = 0;
        lowSum = 0;
        autoEnabled = true;
        disableCount = 0;
      }
    } else {
      lowCount = 0;
      lowSum = 0;
    }
  }

  // 自动校准完成后监控是否需要关闭 / Monitor post-calibration behavior
  if (!autoEnabled) {
    if (Fdata > AUTO_DISABLE_THRESHOLD) {
      enableCount++;
      if (enableCount >= AUTO_CALIBRATION_COUNT) {
        autoEnabled = true;
        enableCount = 0;
      }
    } else {
      enableCount = 0;
    }
  } else {
    if (autoOffset > 0 && Fdata < AUTO_DISABLE_THRESHOLD) {
      disableCount++;
      if (disableCount >= AUTO_CALIBRATION_COUNT) {
        autoEnabled = false;
        disableCount = 0;
        DEBUGSerial.println("Auto-calibration disabled");
      }
    } else {
      disableCount = 0;
    }
  }

  DEBUGSerial.print("F = ");
  DEBUGSerial.print(Fdata);
  DEBUGSerial.print(" g, ");

  // 判断重量区间 / Evaluate weight range
  if (Fdata < PRESS_THRESHOLD_LOW) {
    DEBUGSerial.println("EMPTY");
  } else if (Fdata > PRESS_THRESHOLD_HIGH) {
    DEBUGSerial.println("NO");
  } else {
    DEBUGSerial.println("YES");
  }

  delay(300);  // 延迟 300 毫秒 / Delay for 300 milliseconds
}

long getPressValue(int pin) {
  int value = analogRead(pin);  // 读取模拟值 / Read analog value
  int voltage = map(value, 0, 1023, 5000, 0);  // 转换为毫伏 / Convert to millivolts

  long pressure = 0;

  // 判断电压是否在有效范围内 / Check if voltage is within valid range
  if (voltage < VOLTAGE_MIN) {
    pressure = 0;
  } else if (voltage > VOLTAGE_MAX) {
    pressure = PRESS_MAX;
  } else {
    pressure = map(voltage, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);  // 映射电压到压力 / Map voltage to pressure
  }

  long calibrated = pressure - MANUAL_OFFSET - autoOffset;  // 总校准 = 人工 + 自动 / Total offset = manual + auto

  // 如果重量在 -10g 到 0g 之间，则强制设为 0g / Clamp near-zero negative values to 0
  if (calibrated > -10 && calibrated < 0) {
    calibrated = 0;
  }

  return calibrated;
}

// 标准 C++ 主函数入口 / Standard C++ entry point
int main() {
  init();     // 初始化 Arduino 核心 / Initialize Arduino core
  setup();    // 调用用户定义的 setup() / Call user-defined setup()

  while (true) {
    loop();   // 持续调用 loop() / Continuously call loop()
  }

  return 0;
}

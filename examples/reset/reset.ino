/**
 * @~Chinese
 * @file reset.ino
 * @brief 示例：初始化和重置电机。
 * @example reset.ino
 * 初始化和重置电机。
 */
/**
 * @~English
 * @file reset.ino
 * @brief Example: Initialize and reset the motor.
 * @example reset.ino
 * Initialize and reset the motor.
 */

#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);
}  // namespace

void setup() {
  Serial.begin(115200);

  Wire.begin();

  g_md40.Init();

  Serial.print(F("Device ID: 0x"));
  Serial.println(g_md40.device_id(), HEX);
  Serial.print(F("Name: "));
  Serial.println(g_md40.name());
  Serial.print(F("Firmware Version: "));
  Serial.println(g_md40.firmware_version());

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    Serial.print(F("Motor "));
    Serial.print(i);
    Serial.print(F(" state: "));
    Serial.println(static_cast<uint8_t>(g_md40[i].state()));

    g_md40[i].Reset();

    Serial.print(F(" after reset: "));
    Serial.println(static_cast<uint8_t>(g_md40[i].state()));
  }
}

void loop() {
  delay(1000);
}
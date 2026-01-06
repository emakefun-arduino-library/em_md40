/**
 * @~Chinese
 * @file reset.ino
 * @brief 示例：初始化和重置电机。
 * @example reset.ino
 * 初始化和重置电机。
 *
 * @details platform_notes 平台兼容性说明
 * 本示例针对不同硬件平台的I2C引脚配置进行了兼容性处理：
 * - ESP32: 使用GPIO21(SDA)和GPIO22(SCL)，可通过Wire.begin(sda, scl)指定引脚。
 * - AVR平台: I2C引脚固定为A4(SDA)和A5(SCL)，使用Wire.begin()无参调用。
 * 用户可根据实际硬件修改kI2cPinSda和kI2cPinScl常量，或调整条件编译逻辑。
 */
/**
 * @~English
 * @file reset.ino
 * @brief Example: Initialize and reset the motor.
 * @example reset.ino
 * Initialize and reset the motor.
 *
 * @details platform_notes Platform Compatibility Notes
 * This example includes compatibility handling for I2C pin configuration across different hardware platforms:
 * - ESP32: Uses GPIO21(SDA) and GPIO22(SCL), pins can be specified via Wire.begin(sda, scl).
 * - AVR platform: I2C pins are fixed at A4(SDA) and A5(SCL), uses Wire.begin() without parameters.
 * Users can modify kI2cPinSda and kI2cPinScl constants or adjust conditional compilation logic according to actual hardware.
 */

#include <Wire.h>

#include "md40.h"

namespace {
#if defined(ESP32)
constexpr gpio_num_t kI2cPinSda = GPIO_NUM_21;
constexpr gpio_num_t kI2cPinScl = GPIO_NUM_22;
#endif

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);
}  // namespace

void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  Wire.begin(kI2cPinSda, kI2cPinScl);
#else
  Wire.begin();
#endif

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
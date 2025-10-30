/**
 * @~Chinese
 * @file dc_mode_run_pwm_duty.ino
 * @brief 示例：使用直流模式，以指定PWM占空比驱动电机，每2秒切换转向（PWM占空比值为±1023）。
 * @example dc_mode_run_pwm_duty.ino
 * 使用直流模式，以指定PWM占空比驱动电机，每2秒切换转向（PWM占空比值为±1023）。实时监控电机PWM占空比和状态。
 */
/**
 * @~English
 * @file dc_mode_run_pwm_duty.ino
 * @brief Example: Using DC mode, drive the motor with a specified PWM duty and switch the direction every 2 seconds (PWM duty is ± 1023).
 * @example dc_mode_run_pwm_duty.ino
 * Use DC mode to drive the motor with a specified PWM duty, switching the direction every 2 seconds (PWM duty is ± 1023). Real time
 * monitoring of motor PWM duty and status.
 */

#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int16_t g_pwm_duty = 1023;
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
    g_md40[i].SetDcMode();
  }
}

void loop() {
  if (g_trigger_time == 0 || millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();

    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" run pwm duty: "));
      Serial.println(g_pwm_duty);
      g_md40[i].RunPwmDuty(g_pwm_duty);
    }
    g_pwm_duty = -g_pwm_duty;
  }

  if (millis() - g_last_print_time > 200) {
    g_last_print_time = millis();

    Serial.print(F("pwm duties: "));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].pwm_duty());
      Serial.print(F(", "));
    }

    Serial.print(F("states: "));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(static_cast<uint8_t>(g_md40[i].state()));
      if (i < em::Md40::kMotorNum - 1) {
        Serial.print(F(", "));
      }
    }

    Serial.println();
  }
}
/**
 * @~Chinese
 * @file encoder_mode_run_speed.ino
 * @brief 示例：使用编码器模式，以指定的速度（单位RPM）驱动电机，每2秒切换方向（速度值为±100）。
 * @example encoder_mode_run_speed.ino
 * 使用编码器模式，以指定的速度（单位RPM）驱动电机，每2秒切换方向（速度值为±100）。实时监控电机速度、位置、PWM占空比等参数。
 */
/**
 * @~English
 * @file encoder_mode_run_speed.ino
 * @brief Example: Using encoder mode, drive the motor at a specified speed (in RPM) and switch direction every 2 seconds (with a speed value of ±
 * 100).
 * @example encoder_mode_run_speed.ino
 * Using encoder mode, drive the motor at a specified speed (in RPM) and switch direction every 2 seconds (with a speed value of ± 100). Real time
 * monitoring of motor speed, position, PWM duty and other parameters.
 */

#include <Wire.h>

#include "md40.h"

namespace {
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int32_t g_target_speed = 100;
}  // namespace

void setup() {
  Serial.begin(115200);

  Wire.begin();

  g_md40.Init();

  Serial.print("Device ID: 0x");
  Serial.println(g_md40.device_id(), HEX);
  Serial.print("Name: ");
  Serial.println(g_md40.name());
  Serial.print("Firmware Version: ");
  Serial.println(g_md40.firmware_version());

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    g_md40[i].SetEncoderMode(kEncoderPpr, kReductionRatio, em::Md40::Motor::PhaseRelation::kAPhaseLeads);
    g_md40[i].set_speed_pid_p(1.5);
    g_md40[i].set_speed_pid_i(1.5);
    g_md40[i].set_speed_pid_d(1.0);
    g_md40[i].set_position_pid_p(10.0);
    g_md40[i].set_position_pid_i(1.0);
    g_md40[i].set_position_pid_d(1.0);

    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" state:");
    Serial.print(static_cast<uint8_t>(g_md40[i].state()));
    Serial.print(", speed pid p:");
    Serial.print(g_md40[i].speed_pid_p());
    Serial.print(", speed pid i:");
    Serial.print(g_md40[i].speed_pid_i());
    Serial.print(", speed pid d:");
    Serial.print(g_md40[i].speed_pid_d());
    Serial.print(", position pid p:");
    Serial.print(g_md40[i].position_pid_p());
    Serial.print(", position pid i:");
    Serial.print(g_md40[i].position_pid_i());
    Serial.print(", position pid d:");
    Serial.println(g_md40[i].position_pid_d());
  }
}

void loop() {
  if (g_trigger_time == 0 || millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" run speed: ");
      Serial.println(g_target_speed);
      g_md40[i].RunSpeed(g_target_speed);
    }
    g_target_speed = -g_target_speed;
  }

  if (millis() - g_last_print_time > 200) {
    g_last_print_time = millis();

    Serial.print("speeds: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].speed());
      Serial.print(", ");
    }

    Serial.print("pwm duties: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].pwm_duty());
      Serial.print(", ");
    }

    Serial.print("positions: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].position());
      Serial.print(", ");
    }

    Serial.print("pulse counts: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].pulse_count());
      Serial.print(", ");
    }

    Serial.print("states: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(static_cast<uint8_t>(g_md40[i].state()));
      if (i < em::Md40::kMotorNum - 1) {
        Serial.print(", ");
      }
    }

    Serial.println();
  }
}
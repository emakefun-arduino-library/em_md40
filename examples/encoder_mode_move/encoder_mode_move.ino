/**
 * @~Chinese
 * @file encoder_mode_move.ino
 * @brief 示例：使用编码器模式，电机以60 RPM速度相对移动±720位置。
 * @example encoder_mode_move.ino
 * 使用编码器模式，电机以60 RPM速度相对移动±720位置。实时监控电机速度、位置、PWM占空比等参数。
 */
/**
 * @~English
 * @file encoder_mode_move.ino
 * @brief Example: Using encoder mode, the motor moves relative to ± 720 position at a speed of 60 RPM.
 * @example encoder_mode_move.ino
 * Using encoder mode, the motor moves relative to ± 720 position at a speed of 60 RPM. Real time monitoring of motor speed, position, PWM duty
 * and other parameters.
 */

#include <Wire.h>

#include "md40.h"

namespace {
constexpr int32_t kMotorSpeed = 60;
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int32_t g_offset = 720;
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
    g_md40[i].SetEncoderMode(kEncoderPpr, kReductionRatio, em::Md40::Motor::PhaseRelation::kAPhaseLeads);
    g_md40[i].set_speed_pid_p(1.5);
    g_md40[i].set_speed_pid_i(1.5);
    g_md40[i].set_speed_pid_d(1.0);
    g_md40[i].set_position_pid_p(10.0);
    g_md40[i].set_position_pid_i(1.0);
    g_md40[i].set_position_pid_d(1.0);

    Serial.print(F("Motor "));
    Serial.print(i);
    Serial.print(F(" state:"));
    Serial.print(static_cast<uint8_t>(g_md40[i].state()));
    Serial.print(F(", speed pid p:"));
    Serial.print(g_md40[i].speed_pid_p());
    Serial.print(F(", speed pid i:"));
    Serial.print(g_md40[i].speed_pid_i());
    Serial.print(F(", speed pid d:"));
    Serial.print(g_md40[i].speed_pid_d());
    Serial.print(F(", position pid p:"));
    Serial.print(g_md40[i].position_pid_p());
    Serial.print(F(", position pid i:"));
    Serial.print(g_md40[i].position_pid_i());
    Serial.print(F(", position pid d:"));
    Serial.println(g_md40[i].position_pid_d());
  }
}

void loop() {
  if (g_trigger_time == 0 || millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" move "));
      Serial.println(g_offset);
      g_md40[i].Move(g_offset, kMotorSpeed);
    }
    g_offset = -g_offset;
  }

  if (millis() - g_last_print_time > 200) {
    g_last_print_time = millis();

    Serial.print(F("speeds: "));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].speed());
      Serial.print(F(", "));
    }

    Serial.print(F("pwm duties: "));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].pwm_duty());
      Serial.print(F(", "));
    }

    Serial.print(F("positions: "));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].position());
      Serial.print(F(", "));
    }

    Serial.print(F("pulse counts: "));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(g_md40[i].pulse_count());
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
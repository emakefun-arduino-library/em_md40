/**
 * @~Chinese
 * @file encoder_mode_pulse_count_control.ino
 * @brief 示例：电机脉冲计数设置功能演示。
 * @example encoder_mode_pulse_count_control.ino
 * 电机脉冲计数设置功能演示。
 */
/**
 * @~English
 * @file encoder_mode_pulse_count_control.ino
 * @brief Example: Demonstration of motor pulse counting setting function.
 * @example encoder_mode_pulse_count_control.ino
 * Demonstration of motor pulse counting setting function.
 */

#include "md40.h"

namespace {
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;
constexpr int32_t kMotorSpeed = 60;
constexpr int32_t kMotorMoveOffset = 100;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

int32_t g_pulse_count = 200;
uint64_t g_trigger_time = 0;
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
    Serial.println(F("Initial state:"));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" pulse count: "));
      Serial.print(g_md40[i].pulse_count());
      Serial.print(F(", position: "));
      Serial.println(g_md40[i].position());
    }

    Serial.print(F("Set all motors pulse count to: "));
    Serial.println(g_pulse_count);
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].set_pulse_count(g_pulse_count);
    }

    Serial.println(F("After set:"));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" pulse count: "));
      Serial.print(g_md40[i].pulse_count());
      Serial.print(F(", position: "));
      Serial.println(g_md40[i].position());
    }
    g_pulse_count = -g_pulse_count;

    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" from position "));
      Serial.print(g_md40[i].position());
      Serial.println(F(" move 100 degrees"));

      g_md40[i].Move(kMotorMoveOffset, kMotorSpeed);
    }

    delay(1000);

    Serial.println(F("Final state:"));
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" pulse count: "));
      Serial.print(g_md40[i].pulse_count());
      Serial.print(F(", position: "));
      Serial.println(g_md40[i].position());
    }
    g_trigger_time = millis();
  }
}
/**
 * @~Chinese
 * @file encoder_mode_pulse_count_control.ino
 * @brief 示例：电机位置设置功能演示。
 * @example encoder_mode_pulse_count_control.ino
 * 演示如何使用位置设置功能校准电机位置。
 */
/**
 * @~English
 * @file encoder_mode_pulse_count_control.ino
 * @brief Example: Demonstration of motor position setting function.
 * @example encoder_mode_pulse_count_control.ino
 * Demonstrates how to use the position setting function to calibrate motor position.
 */

#include "md40.h"

namespace {
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;
constexpr int32_t kMotorSpeed = 100;
constexpr int32_t kMotorMoveOffset = 100;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

int32_t g_pulse_count = 200;
uint64_t g_trigger_time = 0;
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
    Serial.println("Initial state:");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" pulse count: ");
      Serial.print(g_md40[i].pulse_count());
      Serial.print(", position: ");
      Serial.println(g_md40[i].position());
    }

    Serial.print("Setting all motors pulse count to: ");
    Serial.println(g_pulse_count);
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].set_pulse_count(g_pulse_count);
    }

    Serial.println("After set:");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" pulse count: ");
      Serial.print(g_md40[i].pulse_count());
      Serial.print(", position: ");
      Serial.println(g_md40[i].position());
    }
    g_pulse_count = -g_pulse_count;

    Serial.println("Moving motors +100 degrees to show effect:");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" from position ");
      Serial.print(g_md40[i].position());
      Serial.println(" move +100 degrees");

      g_md40[i].Move(kMotorMoveOffset, kMotorSpeed);
    }

    delay(1000);

    Serial.println("Final state:");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" pulse count: ");
      Serial.print(g_md40[i].pulse_count());
      Serial.print(", position: ");
      Serial.println(g_md40[i].position());
    }
    g_trigger_time = millis();
  }
}
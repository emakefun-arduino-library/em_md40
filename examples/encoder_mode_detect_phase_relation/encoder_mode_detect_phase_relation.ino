/**
 * @~Chinese
 * @file encoder_mode_detect_phase_relation.ino
 * @brief 示例：根据电机正转时编码器AB相的实际相位关系，在串口打印输出是使用 @ref kAPhaseLeads 还是 @ref
 * kBPhaseLeads，以此帮助用户确定在设置编码器模式时phase_relation参数应设置的值。
 * @example encoder_mode_detect_phase_relation.ino
 * 根据电机正转时编码器AB相的实际相位关系，在串口打印输出是使用 @ref kAPhaseLeads 还是 @ref
 * kBPhaseLeads，以此帮助用户确定在设置编码器模式时phase_relation参数应设置的值。
 */
/**
 * @~English
 * @file encoder_mode_detect_phase_relation.ino
 * @brief Example: Based on the actual phase relationship between encoder A and B phases when the motor rotates forward, whether to use @ref
 * kAPhaseLeads or @ref kBPhaseLeads for serial port printing output can help users determine the value of the phase-relation parameter to be set
 * when setting the encoder mode.
 * @example encoder_mode_detect_phase_relation.ino
 * Based on the actual phase relationship between encoder phases A and B when the motor rotates forward, whether to use @ref kAPhaseLeads or @ref
 * kBPhaseLeads for serial port printing output can help users determine the value of the phase-relation parameter that should be set when setting the
 * encoder mode.
 */

#include "md40.h"

namespace {
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;
constexpr int16_t kMotorPwmDuty = 1023;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
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

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    Serial.print(F("Motor "));
    Serial.print(i);
    Serial.print(F(" run speed: "));
    Serial.println(kMotorPwmDuty);
    g_md40[i].RunPwmDuty(kMotorPwmDuty);
  }
}

void loop() {
  if (g_last_print_time == 0 || millis() - g_last_print_time > 200) {
    g_last_print_time = millis();
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      const auto speed = g_md40[i].speed();

      Serial.print(F("Motor "));
      Serial.print(i);
      Serial.print(F(" speed: "));
      Serial.print(speed);

      if (speed > 0) {
        Serial.println(F(". The phase of A leads B, constructed with the em::Md40::Motor::PhaseRelation::kAPhaseLeads enum."));
      } else if (speed < 0) {
        Serial.println(F(". The phase of B leads A, constructed with the em::Md40::Motor::PhaseRelation::kBPhaseLeads enum."));
      } else {
        Serial.println(F(". The motor is not running currently."));
      }
    }
    Serial.println();
  }
}

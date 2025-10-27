/**
 * @~Chinese
 * @file encoder_mode_pulse_count_control.ino
 * @brief 示例：使用编码器模式，以指定PWM占空比驱动电机，周期性切换转向（PWM值在+1023和-1023间切换），以及重置脉冲计数为±200。
 * @example encoder_mode_pulse_count_control.ino
 * 使用编码器模式，以指定PWM占空比驱动电机，周期性切换转向（PWM值在+1023和-1023间切换），以及重置脉冲计数为±200。实时监控电机速度、位置、PWM等参数。
 */
/**
 * @~English
 * @file encoder_mode_pulse_count_control.ino
 * @brief Example: Use encoder mode to drive the motor with a specified PWM duty cycle, periodically switch the steering (PWM values switch
 * between+1023 and -1023), and reset the pulse count to ± 200. between+1023 and -1023), and reset the position information to ± 180 degrees.
 * @example encoder_mode_pulse_count_control.ino
 * Use encoder mode to drive the motor with a specified PWM duty cycle, periodically switch the steering (PWM values switch between+1023 and -1023),
 * and reset the pulse count to ± 200. Real time monitoring of motor speed, position, PWM and other parameters.
 */

#include "md40.h"

namespace {
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

int32_t g_pwm_duty = 1023;
int32_t g_pulse_count = 200;

void PrintMotorInfo() {
  const auto start_time = millis();
  while ((millis() - start_time) < 3000) {
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

    delay(200);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();

  g_md40.Init();

  Serial.print("device id: 0x");
  Serial.println(g_md40.device_id(), HEX);
  Serial.print("name: ");
  Serial.println(g_md40.name());
  Serial.print("firmware version: ");
  Serial.println(g_md40.firmware_version());

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    g_md40[i].SetEncoderMode(kEncoderPpr, kReductionRatio, em::Md40::Motor::PhaseRelation::kAPhaseLeads);
    g_md40[i].set_speed_pid_p(1.5);
    g_md40[i].set_speed_pid_i(1.5);
    g_md40[i].set_speed_pid_d(1.0);
    g_md40[i].set_position_pid_p(10.0);
    g_md40[i].set_position_pid_i(1.0);
    g_md40[i].set_position_pid_d(1.0);

    Serial.print("motor ");
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
  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" run pwm duty: ");
    Serial.println(g_pwm_duty);
    g_md40[i].RunPwmDuty(g_pwm_duty);
  }
  g_pwm_duty = -g_pwm_duty;

  PrintMotorInfo();

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" pulse count: ");
    Serial.println(g_pulse_count);
    g_md40[i].set_pulse_count(g_pulse_count);
  }
  g_pulse_count = -g_pulse_count;

  PrintMotorInfo();

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" pulse count: ");
    Serial.println(g_pulse_count);
    g_md40[i].set_pulse_count(g_pulse_count);
  }
  g_pulse_count = -g_pulse_count;

  PrintMotorInfo();
}
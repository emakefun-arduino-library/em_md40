/**
 * @~Chinese
 * @file encoder_mode_run_speed_stop.ino
 * @brief 示例：使用编码器模式，周期性地启动和停止电机，每隔2秒切换运行状态。
 * @example encoder_mode_run_speed_stop.ino
 * 使用编码器模式，周期性地启动和停止电机，每隔2秒切换运行状态。实时监控电机速度、位置、PWM占空比等参数。
 *
 * @details platform_notes 平台兼容性说明
 * 本示例针对不同硬件平台的I2C引脚配置进行了兼容性处理：
 * - ESP32: 使用GPIO21(SDA)和GPIO22(SCL)，可通过Wire.begin(sda, scl)指定引脚。
 * - AVR平台: I2C引脚固定为A4(SDA)和A5(SCL)，使用Wire.begin()无参调用。
 * 用户可根据实际硬件修改kI2cPinSda和kI2cPinScl常量，或调整条件编译逻辑。
 */
/**
 * @~English
 * @file encoder_mode_run_speed_stop.ino
 * @brief Example: Using encoder mode to periodically start and stop the motor, switching the operating state every 2 seconds.
 * @example encoder_mode_run_speed_stop.ino
 * Use encoder mode to periodically start and stop the motor, switching operating states every 2 seconds. Real time monitoring of motor speed,
 * position, PWM duty and other parameters.
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

constexpr int32_t kMotorSpeed = 100;
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
bool g_motor_run = true;
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
  if (g_motor_run) {
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].RunSpeed(kMotorSpeed);
    }
  } else {
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].Stop();
    }
  }

  if (millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();
    g_motor_run = !g_motor_run;
    Serial.println(g_motor_run ? F("Motors Run") : F("Motors Stop"));
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
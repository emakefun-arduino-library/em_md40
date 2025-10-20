#include <Wire.h>

#include "md40.h"

namespace {
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int32_t g_pwm_duty = 1023;
}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();

  auto result = em::md40::ErrorCode::kOK;

  result = g_md40.Init();
  if (result != em::md40::ErrorCode::kOK) {
    Serial.print("module init failed: ");
    Serial.println(em::md40::ToString(result));
    while (true);
  }

  uint8_t device_id;
  String device_name;
  String firmware_version;

  g_md40.device_id(&device_id);
  g_md40.name(&device_name);
  g_md40.firmware_version(&firmware_version);

  Serial.print("device id: 0x");
  Serial.println(device_id, HEX);
  Serial.print("name: ");
  Serial.println(device_name);
  Serial.print("firmware version: ");
  Serial.println(firmware_version);

  for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
    g_md40[i].set_encoder_mode(kEncoderPpr, kReductionRatio, em::Md40::PhaseRelation::kAPhaseLeads);
    g_md40[i].set_speed_pid_p(1.5);
    g_md40[i].set_speed_pid_i(1.5);
    g_md40[i].set_speed_pid_d(1.0);
    g_md40[i].set_position_pid_p(10.0);
    g_md40[i].set_position_pid_i(1.0);
    g_md40[i].set_position_pid_d(1.0);
  }
  Serial.println("motors configured.");
}

void loop() {
  if (g_trigger_time == 0 || millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      Serial.print("motor ");
      Serial.print(i);
      Serial.print(" run pwm duty: ");
      Serial.println(g_pwm_duty);
      g_md40[i].RunPwmDuty(g_pwm_duty);
    }
    g_pwm_duty = -g_pwm_duty;
  }

  if (millis() - g_last_print_time > 200) {
    g_last_print_time = millis();

    int32_t speed_value;
    int16_t pwm_duty_value;
    int32_t position_value;
    int32_t pulse_count_value;
    uint8_t state_value;

    Serial.print("speeds: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].speed(&speed_value);
      Serial.print(speed_value);
      Serial.print(", ");
    }

    Serial.print("pwm duties: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].pwm_duty(&pwm_duty_value);
      Serial.print(pwm_duty_value);
      Serial.print(", ");
    }

    Serial.print("positions: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].position(&position_value);
      Serial.print(position_value);
      Serial.print(", ");
    }

    Serial.print("pulse counts: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].pulse_count(&pulse_count_value);
      Serial.print(pulse_count_value);
      Serial.print(", ");
    }

    Serial.print("states: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      g_md40[i].state(&state_value);
      Serial.print(state_value);
      if (i < em::Md40::kMotorNum - 1) {
        Serial.print(", ");
      }
    }

    Serial.println();
  }
}
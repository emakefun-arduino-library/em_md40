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
    uint8_t state_value;
    g_md40[i].state(&state_value);
    Serial.println(static_cast<uint8_t>(state_value));
    g_md40[i].set_dc_mode();
  }
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

    Serial.print("pwm duties: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      int16_t pwm_value;
      g_md40[i].pwm_duty(&pwm_value);
      Serial.print(pwm_value);
      if (i < em::Md40::kMotorNum - 1) {
        Serial.print(", ");
      }
    }

    Serial.print(", states: ");
    for (uint8_t i = 0; i < em::Md40::kMotorNum; i++) {
      uint8_t motor_state;
      g_md40[i].state(&motor_state);
      Serial.print(motor_state);
      if (i < em::Md40::kMotorNum - 1) {
        Serial.print(", ");
      }
    }

    Serial.println();
  }
}
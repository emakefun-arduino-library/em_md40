#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int16_t g_pwm_duty = 1023;
}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();

  g_md40.Init();

  Serial.print("device id: 0x");
  Serial.println(g_md40.DeviceId(), HEX);
  Serial.print("name: ");
  Serial.println(g_md40.Name());
  Serial.print("firmware version: ");
  Serial.println(g_md40.FirmwareVersion());

  for (uint8_t i = 0; i < 4; i++) {
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" state: ");
    Serial.println(static_cast<uint8_t>(g_md40[i].State()));
    g_md40[i].SetDcMode();
  }
}

void loop() {
  if (g_trigger_time == 0 || millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();
    for (uint8_t i = 0; i < 4; i++) {
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
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(g_md40[i].PwmDuty());
      if (i < 3) {
        Serial.print(", ");
      }
    }

    Serial.print(", states: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(static_cast<uint8_t>(g_md40[i].State()));
      if (i < 3) {
        Serial.print(", ");
      }
    }

    Serial.println();
  }
}
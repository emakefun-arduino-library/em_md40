#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int32_t g_target_speed = 100;
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
    g_md40[i].SetEncoderMode(12, 90, em::PhaseRelation::kAPhaseLeads);
    g_md40[i].SetSpeedPidP(1.5);
    g_md40[i].SetSpeedPidI(1.5);
    g_md40[i].SetSpeedPidD(1.0);
    g_md40[i].SetPositionPidP(10.0);
    g_md40[i].SetPositionPidI(1.0);
    g_md40[i].SetPositionPidD(1.0);
  }
  Serial.println("motors configured.");
}

void loop() {
  if (g_trigger_time == 0 || millis() - g_trigger_time > 2000) {
    g_trigger_time = millis();
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print("motor ");
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
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(g_md40[i].Speed());
      Serial.print(", ");
    }

    Serial.print("pwm duties: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(g_md40[i].PwmDuty());
      Serial.print(", ");
    }

    Serial.print("positions: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(g_md40[i].Position());
      Serial.print(", ");
    }

    Serial.print("pulse counts: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(g_md40[i].PulseCount());
      Serial.print(", ");
    }

    Serial.print("states: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(static_cast<uint8_t>(g_md40[i].State()));
      if (i < 3) {
        Serial.print(", ");
      }
    }

    Serial.println();
  }
}
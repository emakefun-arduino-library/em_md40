#include <Wire.h>

#include "md40.h"

namespace {
constexpr int32_t kMotorSpeed = 60;
constexpr uint16_t kEncoderPpr = 12;
constexpr uint16_t kReductionRatio = 90;

em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);

uint64_t g_last_print_time = 0;
uint64_t g_trigger_time = 0;
int32_t g_target_position = 720;
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
    g_md40[i].SetEncoderMode(kEncoderPpr, kReductionRatio, em::Md40::PhaseRelation::kAPhaseLeads);
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
      Serial.print(" move to ");
      Serial.println(g_target_position);
      g_md40[i].MoveTo(g_target_position, kMotorSpeed);
    }
    g_target_position = (g_target_position == 0) ? 720 : 0;
  }

  if (millis() - g_last_print_time > 200) {
    g_last_print_time = millis();

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
      Serial.print(em::md40::ToString(g_md40[i].state()));
      if (i < 3) {
        Serial.print(", ");
      }
    }

    Serial.println();
  }
}
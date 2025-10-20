#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);
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
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" state: ");

    uint8_t state_value;
    g_md40[i].state(&state_value);
    Serial.print(state_value);

    g_md40[i].Reset();

    Serial.print(" after reset: ");
    g_md40[i].state(&state_value);
    Serial.println(state_value);
  }
}

void loop() {
  delay(1000);
}
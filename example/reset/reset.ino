#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::Md40::kDefaultI2cAddress, Wire);
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
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" state: ");
    Serial.println(static_cast<uint8_t>(g_md40[i].state()));

    g_md40[i].Reset();

    Serial.print(" after reset: ");
    Serial.println(static_cast<uint8_t>(g_md40[i].state()));
  }
}

void loop() {
  delay(1000);
}
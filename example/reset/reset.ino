#include <Wire.h>

#include "md40.h"

namespace {
em::Md40 g_md40(em::kDefaultI2cAddress, Wire);
}

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
    Serial.print(static_cast<uint8_t>(g_md40[i].State()));

    g_md40[i].Reset();

    Serial.print("after reset: ");
    Serial.print(static_cast<uint8_t>(g_md40[i].State()));
  }
}

void loop() {
  delay(1000);
}
#include "Md40.h"
namespace em {

Md40::Md40(const uint8_t i2c_address, TwoWire &wire) : i2c_address_(i2c_address), wire_(wire) {
  for (uint8_t i = 0; i < kMotorNum; i++) {
    motors_[i] = new Motor(i, i2c_address, wire);
  }
}

Md40::Motor &Md40::operator[](const uint8_t index) {
  if (index >= kMotorNum) {
    printf("Error: Motor index %d out of bounds (max: %d)\n", index, kMotorNum - 1);
    abort();
  }
  return *motors_[index];
}

void Md40::Init() {
  for (auto motor : motors_) {
    motor->Reset();
  }
}

String Md40::FirmwareVersion() {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kMajorVersion);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read firmware version.\n");
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(3));

  uint8_t version[3] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 3) {
    if (wire_.available() > 0) {
      version[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }

  return String(version[0]) + "." + String(version[1]) + "." + String(version[2]);
}

uint8_t Md40::DeviceId() {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kDeviceId);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read device id.\n");
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(1));

  while (0 == wire_.available()) {
  }
  return wire_.read();
}

String Md40::Name() {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kName);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read name.\n");
    abort();
  }
  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(8));
  char name_bytes[9] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 8) {
    if (wire_.available() > 0) {
      name_bytes[bytes_received++] = wire_.read();
    }
  }
  return String(name_bytes);
}

Md40::Motor::Motor(const uint8_t index, const uint8_t i2c_address, TwoWire &wire) : index_(index), i2c_address_(i2c_address), wire_(wire) {
}

void Md40::Motor::ExecuteCommand() {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kCommandExecute);
  wire_.write(0x01);

  if (wire_.endTransmission() != 0) {
    printf("Error: ExecuteCommand function, endTransmission error.");
    abort();
  }

  WaitCommandEmptied();
}

void Md40::Motor::WaitCommandEmptied() {
  uint8_t ret = 0xFF;
  do {
    wire_.beginTransmission(i2c_address_);
    wire_.write(kCommandExecute);
    if (wire_.endTransmission() != 0) {
      printf("Error: WaitCommandEmptied function, endTransmission error.\n");
      abort();
    }

    wire_.requestFrom(i2c_address_, static_cast<uint8_t>(1));

    while (0 == wire_.available()) {
    }
    ret = wire_.read();
  } while (ret != 0);
}

bool Md40::Motor::WriteCommand(Command command, const uint8_t *data, const uint16_t length) {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kCommandType);
  wire_.write(command);
  wire_.write(index_);
  for (uint16_t i = 0; i < length; i++) {
    wire_.write(data[i]);
  }
  if (wire_.endTransmission() != 0) {
    return false;
  }
  return true;
}

void Md40::Motor::Reset() {
  WaitCommandEmptied();
  if (!WriteCommand(kReset, nullptr, 0)) {
    printf("Error: reset motor %d failed.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::SetEncoderMode(const uint16_t ppr, const uint16_t reduction_ratio, const PhaseRelation phase_relation) {
  WaitCommandEmptied();
  const uint8_t data[] = {ppr & 0xFF, (ppr >> 8) & 0xFF, reduction_ratio & 0xFF, (reduction_ratio >> 8) & 0xFF, static_cast<uint8_t>(phase_relation)};
  if (!WriteCommand(kSetup, data, sizeof(data))) {
    printf("Error: Set motor %d to encoder mode failed.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::SetDcMode() {
  WaitCommandEmptied();

  const uint8_t data[] = {0, 0, 0};
  if (!WriteCommand(kSetup, data, sizeof(data))) {
    printf("Error: Set motor %d to dc mode failed.\n", index_);
    abort();
  }
  ExecuteCommand();
}

float Md40::Motor::SpeedPidP() {
  const uint8_t address = kSpeedP + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read speed pid p for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t speed_pid_p[2] = {0};
  uint8_t bytes_received = 0;
  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      speed_pid_p[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return ((speed_pid_p[1] << 8) | speed_pid_p[0]) / 100.0;
}

void Md40::Motor::SetSpeedPidP(const float value) {
  WaitCommandEmptied();

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  const uint8_t data[] = {int_value & 0xFF, (int_value >> 8) & 0xFF};
  if (!WriteCommand(kSetSpeedPidP, data, sizeof(data))) {
    printf("Error: Failed to set the speed pid p of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

float Md40::Motor::SpeedPidI() {
  const uint8_t address = kSpeedI + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read speed pid i for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t speed_pid_i[2] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      speed_pid_i[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return ((speed_pid_i[1] << 8) | speed_pid_i[0]) / 100.0;
}

void Md40::Motor::SetSpeedPidI(const float value) {
  WaitCommandEmptied();

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  const uint8_t data[] = {int_value & 0xFF, (int_value >> 8) & 0xFF};
  if (!WriteCommand(kSetSpeedPidI, data, sizeof(data))) {
    printf("Error: Failed to set the speed pid i of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

float Md40::Motor::SpeedPidD() {
  const uint8_t address = kSpeedD + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read speed pid d for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t speed_pid_d[2] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      speed_pid_d[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }

  return ((speed_pid_d[1] << 8) | speed_pid_d[0]) / 100.0;
}

void Md40::Motor::SetSpeedPidD(const float value) {
  WaitCommandEmptied();

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  const uint8_t data[] = {int_value & 0xFF, (int_value >> 8) & 0xFF};
  if (!WriteCommand(kSetSpeedPidD, data, sizeof(data))) {
    printf("Error: Failed to set the speed pid d of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

float Md40::Motor::PositionPidP() {
  const uint8_t address = kPositionP + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read position pid p for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t position_pid_p[2] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      position_pid_p[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return ((position_pid_p[1] << 8) | position_pid_p[0]) / 100.0;
}

void Md40::Motor::SetPositionPidP(const float value) {
  WaitCommandEmptied();

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  const uint8_t data[] = {int_value & 0xFF, (int_value >> 8) & 0xFF};
  if (!WriteCommand(kSetPositionPidP, data, sizeof(data))) {
    printf("Error: Failed to set the position pid p of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

float Md40::Motor::PositionPidI() {
  const uint8_t address = kPositionI + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read position pid i for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t position_pid_i[2] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      position_pid_i[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return ((position_pid_i[1] << 8) | position_pid_i[0]) / 100.0;
}

void Md40::Motor::SetPositionPidI(const float value) {
  WaitCommandEmptied();

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  const uint8_t data[] = {int_value & 0xFF, (int_value >> 8) & 0xFF};
  if (!WriteCommand(kSetPositionPidI, data, sizeof(data))) {
    printf("Error: Failed to set the position pid i of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

float Md40::Motor::PositionPidD() {
  const uint8_t address = kPositionD + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read position pid d for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t position_pid_d[2] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      position_pid_d[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }

  return ((position_pid_d[1] << 8) | position_pid_d[0]) / 100.0;
}

void Md40::Motor::SetPositionPidD(const float value) {
  WaitCommandEmptied();

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  const uint8_t data[] = {int_value & 0xFF, (int_value >> 8) & 0xFF};
  if (!WriteCommand(kSetPositionPidD, data, sizeof(data))) {
    printf("Error: Failed to set the position pid D of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::SetCurrentPosition(const uint32_t position) {
  WaitCommandEmptied();

  const uint8_t data[] = {position & 0xFF, (position >> 8) & 0xFF, (position >> 16) & 0xFF, (position >> 24) & 0xFF};
  if (!WriteCommand(kSetPosition, data, sizeof(data))) {
    printf("Error: Failed to set the position of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::SetPulseCount(const uint32_t pulse_count) {
  WaitCommandEmptied();
  const uint8_t data[] = {pulse_count & 0xFF, (pulse_count >> 8) & 0xFF, (pulse_count >> 16) & 0xFF, (pulse_count >> 24) & 0xFF};
  if (!WriteCommand(kSetPulseCount, data, sizeof(data))) {
    printf("Error: Failed to set the pulse count for motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::Stop() {
  WaitCommandEmptied();
  if (!WriteCommand(kStop, nullptr, 0)) {
    printf("Error: Stop motor %d failed.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::RunSpeed(const int32_t rpm) {
  WaitCommandEmptied();

  const uint8_t data[] = {rpm & 0xFF, (rpm >> 8) & 0xFF, (rpm >> 16) & 0xFF, (rpm >> 24) & 0xFF};
  if (!WriteCommand(kRunSpeed, data, sizeof(data))) {
    printf("Error: Failed to set the speed of motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::RunPwmDuty(const int16_t pwm_duty) {
  WaitCommandEmptied();

  const uint8_t data[] = {pwm_duty & 0xFF, (pwm_duty >> 8) & 0xFF};
  if (!WriteCommand(kRunPwmDuty, data, sizeof(data))) {
    printf("Error: Failed to set pwm duty for motor %d.\n", index_);
    abort();
  }

  ExecuteCommand();
}

void Md40::Motor::MoveTo(const int32_t position, const int32_t speed) {
  WaitCommandEmptied();

  const uint8_t data[] = {position & 0xFF,
                          (position >> 8) & 0xFF,
                          (position >> 16) & 0xFF,
                          (position >> 24) & 0xFF,
                          speed & 0xFF,
                          (speed >> 8) & 0xFF,
                          (speed >> 16) & 0xFF,
                          (speed >> 24) & 0xFF};
  if (!WriteCommand(kMoveTo, data, sizeof(data))) {
    printf("Error: MoveTo command failed for motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

void Md40::Motor::Move(const int32_t offset, const int32_t speed) {
  WaitCommandEmptied();

  const uint8_t data[8] = {offset & 0xFF,
                           (offset >> 8) & 0xFF,
                           (offset >> 16) & 0xFF,
                           (offset >> 24) & 0xFF,
                           speed & 0xFF,
                           (speed >> 8) & 0xFF,
                           (speed >> 16) & 0xFF,
                           (speed >> 24) & 0xFF};
  if (!WriteCommand(kMove, data, sizeof(data))) {
    printf("Error: Move command failed for motor %d.\n", index_);
    abort();
  }
  ExecuteCommand();
}

StateCode Md40::Motor::State() {
  const uint8_t address = kState + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read state for motor %d.\n", index_);
    abort();
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read state for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(1));

  while (0 == wire_.available()) {
  }
  return static_cast<StateCode>(wire_.read());
}

int32_t Md40::Motor::Speed() {
  const uint8_t address = kSpeed + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read speed for motor %d.\n", index_);
    abort();
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read speed for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(4));
  uint8_t speed[4] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 4) {
    if (wire_.available() > 0) {
      speed[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return (int32_t)(speed[3] << 24 | speed[2] << 16 | speed[1] << 8 | speed[0]);
}

int32_t Md40::Motor::Position() {
  const uint8_t address = kPosition + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read position for motor %d.\n", index_);
    abort();
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read position for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(4));
  uint8_t position[4] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 4) {
    if (wire_.available() > 0) {
      position[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return (int32_t)(position[3] << 24 | position[2] << 16 | position[1] << 8 | position[0]);
}

int32_t Md40::Motor::PulseCount() {
  const uint8_t address = kPulseCount + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read pulse count for motor %d.\n", index_);
    abort();
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read pulse count for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(4));
  uint8_t pulse_count[4] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 4) {
    if (wire_.available() > 0) {
      pulse_count[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }
  return (int32_t)(pulse_count[3] << 24 | pulse_count[2] << 16 | pulse_count[1] << 8 | pulse_count[0]);
}

int16_t Md40::Motor::PwmDuty() {
  const uint8_t address = kPwmDuty + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read pwm duty for motor %d.\n", index_);
    abort();
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  if (wire_.endTransmission() != 0) {
    printf("Error: Failed to read pwm duty for motor %d.\n", index_);
    abort();
  }

  wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2));
  uint8_t pwm_duty[2] = {0};
  uint8_t bytes_received = 0;

  while (bytes_received < 2) {
    if (wire_.available() > 0) {
      pwm_duty[bytes_received++] = static_cast<uint8_t>(wire_.read());
    }
  }

  return (int16_t)((pwm_duty[1] << 8) | pwm_duty[0]);
}
}  // namespace em
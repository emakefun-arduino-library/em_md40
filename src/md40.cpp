#include "Md40.h"

namespace em {

namespace {
static constexpr uint8_t kMotorStateOffset = 0x20;

static constexpr uint8_t kCommandSetup = 1;
static constexpr uint8_t kCommandReset = 2;
static constexpr uint8_t kCommandSetSpeedPidP = 3;
static constexpr uint8_t kCommandSetSpeedPidI = 4;
static constexpr uint8_t kCommandSetSpeedPidD = 5;
static constexpr uint8_t kCommandSetPositionPidP = 6;
static constexpr uint8_t kCommandSetPositionPidI = 7;
static constexpr uint8_t kCommandSetPositionPidD = 8;
static constexpr uint8_t kCommandSetPosition = 9;
static constexpr uint8_t kCommandSetPulseCount = 10;
static constexpr uint8_t kCommandStop = 11;
static constexpr uint8_t kCommandRunPwmDuty = 12;
static constexpr uint8_t kCommandRunSpeed = 13;
static constexpr uint8_t kCommandMoveTo = 14;
static constexpr uint8_t kCommandMove = 15;

static constexpr uint8_t kMemoryAddressDeviceId = 0x00;
static constexpr uint8_t kMemoryAddressMajorVersion = 0x01;
static constexpr uint8_t kMemoryAddressMinorVersion = 0x02;
static constexpr uint8_t kMemoryAddressPatchVersion = 0x03;
static constexpr uint8_t kMemoryAddressName = 0x04;
static constexpr uint8_t kMemoryAddressCommandType = 0x11;
static constexpr uint8_t kMemoryAddressCommandIndex = 0x12;
static constexpr uint8_t kMemoryAddressCommandParam = 0x13;
static constexpr uint8_t kMemoryAddressCommandExecute = 0x23;
static constexpr uint8_t kMemoryAddressState = 0x24;
static constexpr uint8_t kMemoryAddressSpeedP = 0x26;
static constexpr uint8_t kMemoryAddressSpeedI = 0x28;
static constexpr uint8_t kMemoryAddressSpeedD = 0x2A;
static constexpr uint8_t kMemoryAddressPositionP = 0x2C;
static constexpr uint8_t kMemoryAddressPositionI = 0x2E;
static constexpr uint8_t kMemoryAddressPositionD = 0x30;
static constexpr uint8_t kMemoryAddressSpeed = 0x34;
static constexpr uint8_t kMemoryAddressPosition = 0x38;
static constexpr uint8_t kMemoryAddressPulseCount = 0x3C;
static constexpr uint8_t kMemoryAddressPwmDuty = 0x40;
}  // namespace

Md40::Md40(const uint8_t i2c_address, TwoWire &wire) : i2c_address_(i2c_address), wire_(wire) {
  for (uint8_t i = 0; i < kMotorNum; i++) {
    motors_[i] = new Motor(i, i2c_address, wire);
  }
}

Md40::Motor &Md40::operator[](const uint8_t index) {
  return *motors_[index];
}

md40::ErrorCode Md40::Init() {
  for (auto motor : motors_) {
    const auto error_code = motor->Reset();
    if (error_code != md40::ErrorCode::kOK) {
      return error_code;
    }
  }
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::firmware_version(String *const read_value) {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressMajorVersion);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(3)) != 3) {
    return md40::ErrorCode::kUnknownError;
  }

  uint8_t read_bytes[3] = {0};
  uint8_t offset = 0;

  while (offset < 3) {
    if (wire_.available() > 0) {
      read_bytes[offset++] = wire_.read();
    }
  }

  *read_value = String(read_bytes[0]) + "." + String(read_bytes[1]) + "." + String(read_bytes[2]);
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::device_id(uint8_t *const read_value) {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressDeviceId);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(1)) != 1) {
    return md40::ErrorCode::kUnknownError;
  }

  *read_value = wire_.read();
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::name(String *const read_value) {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressName);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(8)) != 8) {
    return md40::ErrorCode::kUnknownError;
  }

  char read_bytes[9] = {0};
  uint8_t offset = 0;

  while (offset < 8) {
    if (wire_.available() > 0) {
      read_bytes[offset++] = wire_.read();
    }
  }
  *read_value = String(read_bytes);
  return md40::ErrorCode::kOK;
}

Md40::Motor::Motor(const uint8_t index, const uint8_t i2c_address, TwoWire &wire) : index_(index), i2c_address_(i2c_address), wire_(wire) {
}

md40::ErrorCode Md40::Motor::ExecuteCommand() {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressCommandExecute);
  wire_.write(0x01);

  auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  return WaitCommandEmptied();
}

md40::ErrorCode Md40::Motor::WaitCommandEmptied() {
  uint8_t result = 0xFF;
  do {
    wire_.beginTransmission(i2c_address_);
    wire_.write(kMemoryAddressCommandExecute);

    const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
    if (error_code != md40::ErrorCode::kOK) {
      return error_code;
    }

    if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(1)) != 1) {
      return md40::ErrorCode::kUnknownError;
    }

    result = wire_.read();
  } while (result != 0);

  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::WriteCommand(const uint8_t command, const uint8_t *data, const uint16_t length) {
  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressCommandType);
  wire_.write(command);
  wire_.write(index_);
  wire_.write(data, length);
  return static_cast<md40::ErrorCode>(wire_.endTransmission());
}

md40::ErrorCode Md40::Motor::Reset() {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  error_code = WriteCommand(kCommandReset, nullptr, 0);
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::set_encoder_mode(const uint16_t ppr, const uint16_t reduction_ratio, const PhaseRelation phase_relation) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressCommandType);
  wire_.write(kCommandSetup);
  wire_.write(index_);
  wire_.write(ppr & 0xFF);
  wire_.write((ppr >> 8) & 0xFF);
  wire_.write(reduction_ratio & 0xFF);
  wire_.write((reduction_ratio >> 8) & 0xFF);
  wire_.write(static_cast<uint8_t>(phase_relation));

  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::set_dc_mode() {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(kMemoryAddressCommandType);
  wire_.write(kCommandSetup);
  wire_.write(index_);
  wire_.write(0);
  wire_.write(0);
  wire_.write(0);

  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::speed_pid_p(float *const read_value) {
  const uint8_t address = kMemoryAddressSpeedP + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  uint16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data / 100.0f;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::set_speed_pid_p(const float value) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  error_code = WriteCommand(kCommandSetSpeedPidP, reinterpret_cast<const uint8_t *>(&int_value), sizeof(int_value));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::speed_pid_i(float *const read_value) {
  const uint8_t address = kMemoryAddressSpeedI + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  uint16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data / 100.0f;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::set_speed_pid_i(const float value) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  error_code = WriteCommand(kCommandSetSpeedPidI, reinterpret_cast<const uint8_t *>(&int_value), sizeof(int_value));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::speed_pid_d(float *const read_value) {
  const uint8_t address = kMemoryAddressSpeedD + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  uint16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data / 100.0f;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::set_speed_pid_d(const float value) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  error_code = WriteCommand(kCommandSetSpeedPidD, reinterpret_cast<const uint8_t *>(&int_value), sizeof(int_value));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::position_pid_p(float *const read_value) {
  const uint8_t address = kMemoryAddressPositionP + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  uint16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data / 100.0f;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::set_position_pid_p(const float value) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  error_code = WriteCommand(kCommandSetPositionPidP, reinterpret_cast<const uint8_t *>(&int_value), sizeof(int_value));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::position_pid_i(float *const read_value) {
  const uint8_t address = kMemoryAddressPositionI + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  uint16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data / 100.0f;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::set_position_pid_i(const float value) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  error_code = WriteCommand(kCommandSetPositionPidI, reinterpret_cast<const uint8_t *>(&int_value), sizeof(int_value));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::position_pid_d(float *const read_value) {
  const uint8_t address = kMemoryAddressPositionD + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  const auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  uint16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data / 100.0f;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::set_position_pid_d(const float value) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const uint16_t int_value = static_cast<uint16_t>(value * 100);
  error_code = WriteCommand(kCommandSetPositionPidD, reinterpret_cast<const uint8_t *>(&int_value), sizeof(int_value));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::set_current_position(const uint32_t position) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  error_code = WriteCommand(kCommandSetPosition, reinterpret_cast<const uint8_t *>(&position), sizeof(position));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::set_pulse_count(const uint32_t pulse_count) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  error_code = WriteCommand(kCommandSetPulseCount, reinterpret_cast<const uint8_t *>(&pulse_count), sizeof(pulse_count));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::Stop() {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  error_code = WriteCommand(kCommandStop, nullptr, 0);
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::RunSpeed(const int32_t rpm) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  error_code = WriteCommand(kCommandRunSpeed, reinterpret_cast<const uint8_t *>(&rpm), sizeof(rpm));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::RunPwmDuty(const int16_t pwm_duty) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  error_code = WriteCommand(kCommandRunPwmDuty, reinterpret_cast<const uint8_t *>(&pwm_duty), sizeof(pwm_duty));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::MoveTo(const int32_t position, const int32_t speed) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const int32_t data[] = {position, speed};
  error_code = WriteCommand(kCommandMoveTo, reinterpret_cast<const uint8_t *>(data), sizeof(data));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::Move(const int32_t offset, const int32_t speed) {
  auto error_code = WaitCommandEmptied();
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  const int32_t data[] = {offset, speed};
  error_code = WriteCommand(kCommandMove, reinterpret_cast<const uint8_t *>(data), sizeof(data));
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }
  return ExecuteCommand();
}

md40::ErrorCode Md40::Motor::state(uint8_t *const read_value) {
  const uint8_t address = kMemoryAddressState + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(1)) != 1) {
    return md40::ErrorCode::kUnknownError;
  }

  *read_value = wire_.read();
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::speed(int32_t *const read_value) {
  const uint8_t address = kMemoryAddressSpeed + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(4)) != 4) {
    return md40::ErrorCode::kUnknownError;
  }

  int32_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::position(int32_t *const read_value) {
  const uint8_t address = kMemoryAddressPosition + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(4)) != 4) {
    return md40::ErrorCode::kUnknownError;
  }

  int32_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::pulse_count(int32_t *const read_value) {
  const uint8_t address = kMemoryAddressPulseCount + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(4)) != 4) {
    return md40::ErrorCode::kUnknownError;
  }

  int32_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data;
  return md40::ErrorCode::kOK;
}

md40::ErrorCode Md40::Motor::pwm_duty(int16_t *const read_value) {
  const uint8_t address = kMemoryAddressPwmDuty + index_ * kMotorStateOffset;

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  wire_.write(0);
  auto error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  wire_.beginTransmission(i2c_address_);
  wire_.write(address);
  error_code = static_cast<md40::ErrorCode>(wire_.endTransmission());
  if (error_code != md40::ErrorCode::kOK) {
    return error_code;
  }

  if (wire_.requestFrom(i2c_address_, static_cast<uint8_t>(2)) != 2) {
    return md40::ErrorCode::kUnknownError;
  }

  int16_t data = 0;
  uint8_t offset = 0;

  while (offset < sizeof(data)) {
    if (wire_.available()) {
      reinterpret_cast<uint8_t *>(&data)[offset++] = wire_.read();
    }
  }

  *read_value = data;
  return md40::ErrorCode::kOK;
}
}  // namespace em
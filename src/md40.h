#ifndef _EM_MD40_H_
#define _EM_MD40_H_

#include <Arduino.h>
#include <Wire.h>

namespace em {

static constexpr uint8_t kDefaultI2cAddress = 0x16;
static constexpr uint8_t kMotorNum = 4;
static constexpr uint8_t kMotorStateOffset = 0x20;

enum StateCode : uint8_t { kIdle = 0, kRunningWithPwmDuty = 1, kRunningWithSpeed = 2, kRunningToPosition = 3, kReachedPosition = 4 };

enum PhaseRelation : uint8_t { kAPhaseLeads = 0, kBPhaseLeads = 1 };

enum Command : uint8_t {
  kSetup = 1,
  kReset = 2,
  kSetSpeedPidP = 3,
  kSetSpeedPidI = 4,
  kSetSpeedPidD = 5,
  kSetPositionPidP = 6,
  kSetPositionPidI = 7,
  kSetPositionPidD = 8,
  kSetPosition = 9,
  kSetPulseCount = 10,
  kStop = 11,
  kRunPwmDuty = 12,
  kRunSpeed = 13,
  kMoveTo = 14,
  kMove = 15
};

enum MemoryAddress : uint8_t {
  kDeviceId = 0x00,
  kMajorVersion = 0x01,
  kMinorVersion = 0x02,
  kPatchVersion = 0x03,
  kName = 0x04,
  kCommandType = 0x11,
  kCommandIndex = 0x12,
  kCommandParam = 0x13,
  kCommandExecute = 0x23,

  kState = 0x24,
  kSpeedP = 0x26,
  kSpeedI = 0x28,
  kSpeedD = 0x2A,
  kPositionP = 0x2C,
  kPositionI = 0x2E,
  kPositionD = 0x30,
  kSpeed = 0x34,
  kPosition = 0x38,
  kPulseCount = 0x3C,
  kPwmDuty = 0x40
};

class Md40 {
 public:
  class Motor {
   public:
    Motor(const uint8_t index, const uint8_t i2c_address, TwoWire &wire);

    void Reset();
    void SetEncoderMode(const uint16_t ppr, const uint16_t reduction_ratio, const PhaseRelation phase_relation);
    void SetDcMode();

    float SpeedPidP();
    void SetSpeedPidP(const float value);
    float SpeedPidI();
    void SetSpeedPidI(const float value);
    float SpeedPidD();
    void SetSpeedPidD(const float value);

    float PositionPidP();
    void SetPositionPidP(const float value);
    float PositionPidI();
    void SetPositionPidI(const float value);
    float PositionPidD();
    void SetPositionPidD(const float value);

    void SetCurrentPosition(const uint32_t position);
    void SetPulseCount(const uint32_t pulse_count);
    void Stop();
    void RunSpeed(const int32_t rpm);
    void RunPwmDuty(const int16_t pwm_duty);
    void MoveTo(const int32_t position, const int32_t speed);
    void Move(const int32_t offset, const int32_t speed);

    StateCode State();
    int32_t Speed();
    int32_t Position();
    int32_t PulseCount();
    int16_t PwmDuty();

   private:
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;

    uint8_t index_;
    TwoWire &wire_;
    const uint8_t i2c_address_;

    void ExecuteCommand();
    void WaitCommandEmptied();
    bool WriteCommand(Command command, const uint8_t *data, const uint16_t length);
  };

  Md40(const uint8_t i2c_address, TwoWire &wire);
  Motor &operator[](uint8_t index);

  void Init();
  String FirmwareVersion();
  uint8_t DeviceId();
  String Name();

 private:
  Md40(const Md40 &) = delete;
  Md40 &operator=(const Md40 &) = delete;

  const uint8_t i2c_address_;
  TwoWire &wire_;
  Motor *motors_[kMotorNum] = {nullptr};
};
}  // namespace em
#endif
#ifndef _EM_MD40_H_
#define _EM_MD40_H_

#include <Arduino.h>
#include <Wire.h>

#include "error_code.h"

namespace em {

class Md40 {
 public:
  static constexpr uint8_t kDefaultI2cAddress = 0x16;
  static constexpr uint8_t kMotorNum = 4;

  enum class PhaseRelation : uint8_t { kAPhaseLeads = 0, kBPhaseLeads = 1 };

  class Motor {
   public:
    Motor(const uint8_t index, const uint8_t i2c_address, TwoWire &wire);

    md40::ErrorCode Reset();
    md40::ErrorCode set_encoder_mode(const uint16_t ppr, const uint16_t reduction_ratio, const PhaseRelation phase_relation);
    md40::ErrorCode set_dc_mode();

    md40::ErrorCode speed_pid_p(float *const read_value);
    md40::ErrorCode set_speed_pid_p(const float value);
    md40::ErrorCode speed_pid_i(float *const read_value);
    md40::ErrorCode set_speed_pid_i(const float value);
    md40::ErrorCode speed_pid_d(float *const read_value);
    md40::ErrorCode set_speed_pid_d(const float value);

    md40::ErrorCode position_pid_p(float *const read_value);
    md40::ErrorCode set_position_pid_p(const float value);
    md40::ErrorCode position_pid_i(float *const read_value);
    md40::ErrorCode set_position_pid_i(const float value);
    md40::ErrorCode position_pid_d(float *const read_value);
    md40::ErrorCode set_position_pid_d(const float value);

    md40::ErrorCode set_current_position(const uint32_t position);
    md40::ErrorCode set_pulse_count(const uint32_t pulse_count);
    md40::ErrorCode Stop();
    md40::ErrorCode RunSpeed(const int32_t rpm);
    md40::ErrorCode RunPwmDuty(const int16_t pwm_duty);
    md40::ErrorCode MoveTo(const int32_t position, const int32_t speed);
    md40::ErrorCode Move(const int32_t offset, const int32_t speed);

    md40::ErrorCode state(uint8_t *const read_value);
    md40::ErrorCode speed(int32_t *const read_value);
    md40::ErrorCode position(int32_t *const read_value);
    md40::ErrorCode pulse_count(int32_t *const read_value);
    md40::ErrorCode pwm_duty(int16_t *const read_value);

   private:
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;

    md40::ErrorCode ExecuteCommand();
    md40::ErrorCode WaitCommandEmptied();
    md40::ErrorCode WriteCommand(const uint8_t command, const uint8_t *data, const uint16_t length);
    const uint8_t index_;
    TwoWire &wire_;
    const uint8_t i2c_address_;
  };

  Md40(const uint8_t i2c_address, TwoWire &wire);
  Motor &operator[](uint8_t index);

  md40::ErrorCode Init();
  md40::ErrorCode firmware_version(String *const result);
  md40::ErrorCode device_id(uint8_t *const result);
  md40::ErrorCode name(String *const result);

 private:
  Md40(const Md40 &) = delete;
  Md40 &operator=(const Md40 &) = delete;

  const uint8_t i2c_address_;
  TwoWire &wire_;
  Motor *motors_[kMotorNum] = {nullptr};
};
}  // namespace em
#endif
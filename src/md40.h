#pragma once

#ifndef _EM_MD40_H_
#define _EM_MD40_H_

#include <Arduino.h>
#include <Wire.h>

#include "em_check.h"

/**
 * @file md40.h
 */

namespace em {

/**
 * @~Chinese
 * @class Md40
 * @brief Md40是一个用于控制MD40模块的驱动类，用来驱动电机。
 */
/**
 * @~English
 * @class Md40
 * @brief Md40 is a driver class used to control the MD40 module for motor driving.
 */
class Md40 {
 public:
  /**
   * @~Chinese
   * @brief 默认I2C地址。
   */
  /**
   * @~English
   * @brief Default I2C address.
   */
  static constexpr uint8_t kDefaultI2cAddress = 0x16;
  /**
   * @~Chinese
   * @brief 电机数量。
   */
  /**
   * @~English
   * @brief Number of motors.
   */
  static constexpr uint8_t kMotorNum = 4;

  /**
   * @~Chinese
   * @class Md40::Motor
   * @brief Motor类代表一个电机对象，提供对单个电机的控制功能，如速度、位置和PID参数设置。
   */
  /**
   * @~English
   * @class Md40::Motor
   * @brief The Motor class represents a motor object, providing control functions for a single motor, such as speed, position, and PID parameters
   * setting.
   */
  class Motor {
   public:
    /**
     * @~Chinese
     * @brief 用于明确电机正转时编码器AB相的相位关系，以便在脉冲计数及后续速度计算等操作中依据正确的相位关系进行处理。
     */
    /**
     * @~English
     * @brief Used to clarify the phase relationship between phase A and phase B of the encoder when the motor is rotating
     * forward, so that the correct phase relationship can be used in operations such as pulse counting and subsequent speed
     * calculation.
     */
    enum class PhaseRelation : uint8_t {
      /**
       * @~Chinese
       * @brief 表示电机正转时A相领先于B相。
       */
      /**
       * @~English
       * @brief Represents the situation where phase A leads phase B when the motor is rotating forward.
       */
      kAPhaseLeads = 0,

      /**
       * @~Chinese
       * @brief 表示电机正转时B相领先于A相。
       */
      /**
       * @~English
       * @brief Represents the situation where phase B leads phase A when the motor is rotating forward.
       */
      kBPhaseLeads = 1,
    };

    /**
     * @~Chinese
     * @brief 电机状态枚举。
     */
    /**
     * @~English
     * @brief Motor state enumeration.
     */
    enum class State : uint8_t {
      /**
       * @~Chinese
       * @brief 表示电机处于空闲状态。
       */
      /**
       * @~English
       * @brief Indicates that the motor is in idle state.
       */
      kIdle = 0,

      /**
       * @~Chinese
       * @brief 表示电机处于PWM占空比模式运行。
       */
      /**
       * @~English
       * @brief Indicates that the motor is running in PWM duty mode.
       */
      kRuningWithPwmDuty = 1,

      /**
       * @~Chinese
       * @brief 表示电机处于速度模式运行。
       */
      /**
       * @~English
       * @brief Indicates that the motor is running in speed mode.
       */
      kRuningWithSpeed = 2,

      /**
       * @~Chinese
       * @brief 表示电机正在执行位置闭环运动，向目标位置运行中。
       */
      /**
       * @~English
       * @brief Indicates that the motor is executing position closed-loop motion and moving towards the target position.
       */
      kRuningToPosition = 3,

      /**
       * @~Chinese
       * @brief 表示电机到达目标位置状态。
       */
      /**
       * @~English
       * @brief Indicates that the motor has reached the target position.
       */
      kReachedPosition = 4,
    };

    /**
     * @~Chinese
     * @brief 构造函数。
     * @param[in] index 电机索引。
     * @param[in] i2c_address I2C地址。
     * @param[in] wire TwoWire 对象引用。
     */
    /**
     * @~English
     * @brief Constructor.
     * @param[in] index Motor index.
     * @param[in] i2c_address I2C address.
     * @param[in] wire TwoWire object reference.
     */
    Motor(const uint8_t index, const uint8_t i2c_address, TwoWire &wire);

    /**
     * @~Chinese
     * @brief 重置电机。
     */
    /**
     * @~English
     * @brief Reset the motor.
     */
    void Reset();

    /**
     * @~Chinese
     * @brief 设置电机为编码器模式。
     * @param[in] ppr 每转脉冲数。
     * @param[in] reduction_ratio 减速比。
     * @param[in] phase_relation 相位关系（A相领先或B相领先，指电机正转时的情况），参数说明请查阅： @ref PhaseRelation 。
     */
    /**
     * @~English
     * @brief Set the motor to encoder mode.
     * @param[in] ppr Pulses per revolution.
     * @param[in] reduction_ratio Reduction ratio.
     * @param[in] phase_relation Phase relationship (A phase leads or B phase leads, referring to the situation when the motor is
     * rotating forward), for parameter descriptions, please refer to: @ref PhaseRelation.
     */
    void SetEncoderMode(const uint16_t ppr, const uint16_t reduction_ratio, const PhaseRelation phase_relation);

    /**
     * @~Chinese
     * @brief 设置电机为直流模式。
     */
    /**
     * @~English
     * @brief Set the motor to DC mode.
     */
    void SetDcMode();

    /**
     * @~Chinese
     * @brief 获取速度PID控制器的比例（P）值。
     * @return 速度PID控制器的比例（P）值。
     */
    /**
     * @~English
     * @brief Get the proportional (P) value of the speed PID controller.
     * @return The proportional (P) value of the speed PID controller.
     */
    float speed_pid_p();

    /**
     * @~Chinese
     * @brief 设置速度PID控制器的比例（P）值。
     * @param[in] value 速度PID控制器的比例（P）值。
     */
    /**
     * @~English
     * @brief Set the proportional (P) value of the speed PID controller.
     * @param[in] value The proportional (P) value of the speed PID controller.
     */
    void set_speed_pid_p(const float value);

    /**
     * @~Chinese
     * @brief 获取速度PID控制器的积分（I）值。
     * @return 速度PID控制器的积分（I）值。
     */
    /**
     * @~English
     * @brief Get the integral (I) value of the speed PID controller.
     * @return The integral (I) value of the speed PID controller.
     */
    float speed_pid_i();

    /**
     * @~Chinese
     * @brief 设置速度PID控制器的积分（I）值。
     * @param[in] value 速度PID控制器的积分（I）值。
     */
    /**
     * @~English
     * @brief Set the integral (I) value of the speed PID controller.
     * @param[in] value The integral (I) value of the speed PID controller.
     */
    void set_speed_pid_i(const float value);

    /**
     * @~Chinese
     * @brief 获取速度PID控制器的微分（D）值。
     * @return 速度PID控制器的微分（D）值。
     */
    /**
     * @~English
     * @brief Get the derivative (D) value of the speed PID controller.
     * @return The derivative (D) value of the speed PID controller.
     */
    float speed_pid_d();

    /**
     * @~Chinese
     * @brief 设置速度PID控制器的微分（D）值。
     * @param[in] value 速度PID控制器的微分（D）值。
     */
    /**
     * @~English
     * @brief Set the derivative (D) value of the speed PID controller.
     * @param[in] value The derivative (D) value of the speed PID controller.
     */
    void set_speed_pid_d(const float value);

    /**
     * @~Chinese
     * @brief 获取位置PID控制器的比例（P）值。
     * @return 位置PID控制器的比例（P）值。
     */
    /**
     * @~English
     * @brief Get the proportional (P) value of the position PID controller.
     * @return The proportional (P) value of the position PID controller.
     */
    float position_pid_p();

    /**
     * @~Chinese
     * @brief 设置位置PID控制器的比例（P）值。
     * @param[in] value 位置PID控制器的比例（P）值。
     */
    /**
     * @~English
     * @brief Set the proportional (P) value of the position PID controller.
     * @param[in] value The proportional (P) value of the position PID controller.
     */
    void set_position_pid_p(const float value);

    /**
     * @~Chinese
     * @brief 获取位置PID控制器的积分（I）值。
     * @return 位置PID控制器的积分（I）值。
     */
    /**
     * @~English
     * @brief Get the integral (I) value of the position PID controller.
     * @return The integral (I) value of the position PID controller.
     */
    float position_pid_i();

    /**
     * @~Chinese
     * @brief 设置位置PID控制器的积分（I）值。
     * @param[in] value 位置PID控制器的积分（I）值。
     */
    /**
     * @~English
     * @brief Set the integral (I) value of the position PID controller.
     * @param[in] value The integral (I) value of the position PID controller.
     */
    void set_position_pid_i(const float value);

    /**
     * @~Chinese
     * @brief 获取位置PID控制器的微分（D）值。
     * @return 位置PID控制器的微分（D）值。
     */
    /**
     * @~English
     * @brief Get the derivative (D) value of the position PID controller.
     * @return The derivative (D) value of the position PID controller.
     */
    float position_pid_d();

    /**
     * @~Chinese
     * @brief 设置位置PID控制器的微分（D）值。
     * @param[in] value 位置PID控制器的微分（D）值。
     */
    /**
     * @~English
     * @brief Set the derivative (D) value of the position PID controller.
     * @param[in] value The derivative (D) value of the position PID controller.
     */
    void set_position_pid_d(const float value);

    /**
     * @~Chinese
     * @brief 设定电机输出轴的位置值，单位为角度(°)。（电机输出轴累计角度值，例如：360度表示正转1整圈，-360度表示反转一整圈）
     * @param[in] position 位置设定值，单位为角度(°)，表示从零位开始的累计角度。
     */
    /**
     * @~English
     * @brief Set the position value of the motor output shaft unit degrees (°). (Accumulated angle value of motor output shaft, for example: 360
     * degrees represents 1 full circle of forward rotation, -360 degrees represents 1 full circle of reverse rotation)
     * @param[in] position Position setting value, unit degree (°), represents the cumulative angle from zero position.
     */
    void set_position(const int32_t position);

    /**
     * @~Chinese
     * @brief 设定电机的编码器脉冲计数。该计数值是在A相下降沿的时候计数，如果是正转会加一，反转则减一。
     * @param[in] pulse_count 编码器脉冲数。
     */
    /**
     * @~English
     * @brief Set the encoder pulse count for the motor. The count value is counted at the falling edge of phase A. If it is positive, add one; if it
     * is negative, subtract one.
     * @param[in] pulse_count Encoder pulse count.
     */
    void set_pulse_count(const int32_t pulse_count);

    /**
     * @~Chinese
     * @brief 停止电机运行。
     */
    /**
     * @~English
     * @brief Stop the motor.
     */
    void Stop();

    /**
     * @~Chinese
     * @brief 以设定的速度值（RPM）设置电机输出轴转速。正数代表正转，负数代表反转。
     * @param[in] rpm 速度设定值（RPM）。正数代表正转，负数代表反转。
     */
    /**
     * @~English
     * @brief Set the motor output shaft speed to the set speed value (RPM). Positive numbers represent forward rotation, while negative numbers
     * represent reverse rotation.
     * @param[in] rpm Speed setting value (RPM). Positive numbers represent forward rotation, while negative numbers represent reverse rotation.
     */
    void RunSpeed(const int32_t rpm);

    /**
     * @~Chinese
     * @brief 以设定的PWM占空比运行电机。正数代表正转，负数代表反转。
     * @param[in] pwm_duty PWM占空比（取值范围 -1023到1023）。正数代表正转，负数代表反转。
     */
    /**
     * @~English
     * @brief Run the motor with the set PWM duty. Positive numbers represent forward rotation, while negative numbers represent reverse
     * rotation.
     * @param[in] pwm_duty PWM duty (value range -1023 to 1023). Positive values represent forward rotation, negative values represent reverse
     * rotation.
     */
    void RunPwmDuty(const int16_t pwm_duty);

    /**
     * @~Chinese
     * @brief 将电机输出轴转动到指定位置，单位为角度(°)。
     * @param[in] position 目标位置设定值，单位为角度(°)，累计角度值。
     * @param[in] speed 电机输出轴运行速度设定值（RPM）。
     */
    /**
     * @~English
     * @brief Rotate the motor output shaft to the designated position, unit degrees (°).
     * @param[in] position Target position setting value, unit: angle (°), cumulative angle value.
     * @param[in] speed Motor output shaft operating speed set value (RPM).
     */
    void MoveTo(const int32_t position, const int32_t speed);

    /**
     * @~Chinese
     * @brief 电机输出轴相对转动指定角度，单位为角度(°)。
     * @param[in] offset 相对位移设定值，单位为角度(°)，基于当前位置的相对角度。
     * @param[in] speed 电机输出轴运行速度设定值（RPM）。
     */
    /**
     * @~English
     * @brief The motor output shaft rotates relative to the specified angle, unit degrees (°).
     * @param[in] offset Relative displacement setting value, unit degrees (°), based on the relative angle at the current position.
     * @param[in] speed Motor output shaft operating speed set value (RPM).
     */
    void Move(const int32_t offset, const int32_t speed);

    /**
     * @~Chinese
     * @brief 获取电机当前状态。
     * @return 电机当前状态。
     */
    /**
     * @~English
     * @brief Get the current state of the motor.
     * @return The current state of the motor.
     */
    State state();

    /**
     * @~Chinese
     * @brief 获取电机输出轴当前的转速（RPM）。正数代表正转，负数代表反转。
     * @return 电机输出轴当前的转速（RPM）。正数代表正转，负数代表反转。
     */
    /**
     * @~English
     * @brief Get the current RPM of the motor output shaft. Positive numbers represent forward rotation, while negative numbers represent reverse rotation.
     * @return The current speed (RPM) of the motor output shaft. Positive numbers represent forward rotation, while negative numbers represent
     * reverse rotation.
     */
    int32_t speed();

    /**
     * @~Chinese
     * @brief 获取电机输出轴的位置值，单位为角度(°)。（电机输出轴累计角度值，例如：360度表示正转1整圈，-360度表示反转一整圈）
     * @return 电机输出轴当前位置，单位为角度(°)，表示从零位开始的累计角度。
     */
    /**
     * @~English
     * @brief Get the position value of the motor output shaft, unit degrees (°). (Accumulated angle value of motor output shaft, for example: 360 degrees represents 1 full circle of forward rotation, -360 degrees represents 1 full circle of reverse rotation)
     * @return The current position of the motor output shaft, unit degrees (°), represents the cumulative angle from zero position.
     */
    int32_t position();

    /**
     * @~Chinese
     * @brief 获取电机当前的编码器脉冲计数。该计数值是在A相下降沿的时候计数，如果是正转会加一，反转则减一。
     * @return 电机当前的编码器脉冲数。
     */
    /**
     * @~English
     * @brief Get the current encoder pulse count of the motor. The count value is counted at the falling edge of phase A. If it is positive, add one;
     * if it is negative, subtract one.
     * @return The current encoder pulse count of the motor.
     */
    int32_t pulse_count();

    /**
     * @~Chinese
     * @brief 获取电机当前的PWM占空比。正数代表正转，负数代表反转。
     * @return 电机当前的PWM占空比（取值范围 -1023到1023）。正数代表正转，负数代表反转。
     */
    /**
     * @~English
     * @brief Get the current PWM duty of the motor. Positive numbers represent forward rotation, while negative numbers represent reverse
     * rotation.
     * @return The current PWM duty of the motor (value range -1023 to 1023). Positive values represent forward rotation, negative values
     * represent reverse rotation.
     */
    int16_t pwm_duty();

   private:
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;

    void ExecuteCommand();

    void WaitCommandEmptied();

    void WriteCommand(const uint8_t command, const uint8_t *data, const uint16_t length);

    const uint8_t index_ = 0;
    TwoWire &wire_ = Wire;
    const uint8_t i2c_address_ = kDefaultI2cAddress;
  };

  /**
   * @~Chinese
   * @brief 构造函数。
   * @param[in] i2c_address I2C地址。
   * @param[in] wire TwoWire 对象引用。
   */
  /**
   * @~English
   * @brief Constructor.
   * @param[in] i2c_address I2C address.
   * @param[in] wire TwoWire object reference.
   */
  Md40(const uint8_t i2c_address, TwoWire &wire);

  /**
   * @~Chinese
   * @brief 获取指定索引的电机对象。
   * @param[in] index 电机索引。
   * @return 电机对象引用。
   */
  /**
   * @~English
   * @brief Get the motor object with specified index.
   * @param[in] index Motor index.
   * @return Motor object reference.
   */
  Motor &operator[](uint8_t index);

  /**
   * @~Chinese
   * @brief 初始化。
   */
  /**
   * @~English
   * @brief Initialize.
   */
  void Init();

  /**
   * @~Chinese
   * @brief 获取固件版本。
   * @return 固件版本。
   */
  /**
   * @~English
   * @brief Get firmware version.
   * @return Firmware version.
   */
  String firmware_version();

  /**
   * @~Chinese
   * @brief 获取设备ID。
   * @return 设备ID。
   */
  /**
   * @~English
   * @brief Get device ID.
   * @return Device ID.
   */
  uint8_t device_id();

  /**
   * @~Chinese
   * @brief 获取设备名称。
   * @return 设备名称。
   */
  /**
   * @~English
   * @brief Get device name.
   * @return Device name.
   */
  String name();

 private:
  Md40(const Md40 &) = delete;
  Md40 &operator=(const Md40 &) = delete;

  const uint8_t i2c_address_ = kDefaultI2cAddress;
  TwoWire &wire_ = Wire;
  Motor *motors_[kMotorNum] = {nullptr};
};
}  // namespace em
#endif
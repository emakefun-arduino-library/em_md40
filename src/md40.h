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
       * @brief Indicates that the motor is running in PWM duty cycle mode.
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
     * @param index 电机索引。
     * @param i2c_address I2C地址。
     * @param wire TwoWire 对象引用。
     */
    /**
     * @~English
     * @brief Constructor.
     * @param index Motor index.
     * @param i2c_address I2C address.
     * @param wire TwoWire object reference.
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
     * @param ppr 每转脉冲数。
     * @param reduction_ratio 减速比。
     * @param phase_relation 相位关系（A相领先或B相领先，指电机正转时的情况），参数说明请查阅： @ref PhaseRelation。
     */
    /**
     * @~English
     * @brief Set the motor to encoder mode.
     * @param ppr Pulses per revolution.
     * @param reduction_ratio Reduction ratio.
     * @param phase_relation Phase relationship (A phase leads or B phase leads, referring to the situation when the motor is
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
     * @param value 速度PID控制器的比例（P）值。
     */
    /**
     * @~English
     * @brief Set the proportional (P) value of the speed PID controller.
     * @param value The proportional (P) value of the speed PID controller.
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
     * @param value 速度PID控制器的积分（I）值。
     */
    /**
     * @~English
     * @brief Set the integral (I) value of the speed PID controller.
     * @param value The integral (I) value of the speed PID controller.
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
     * @param value 速度PID控制器的微分（D）值。
     */
    /**
     * @~English
     * @brief Set the derivative (D) value of the speed PID controller.
     * @param value The derivative (D) value of the speed PID controller.
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
     * @param value 位置PID控制器的比例（P）值。
     */
    /**
     * @~English
     * @brief Set the proportional (P) value of the position PID controller.
     * @param value The proportional (P) value of the position PID controller.
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
     * @param value 位置PID控制器的积分（I）值。
     */
    /**
     * @~English
     * @brief Set the integral (I) value of the position PID controller.
     * @param value The integral (I) value of the position PID controller.
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
     * @param value 位置PID控制器的微分（D）值。
     */
    /**
     * @~English
     * @brief Set the derivative (D) value of the position PID controller.
     * @param value The derivative (D) value of the position PID controller.
     */
    void set_position_pid_d(const float value);

    /**
     * @~Chinese
     * @brief 重置电机的位置值。
     * @param position 位置设定值。
     */
    /**
     * @~English
     * @brief Reset the motor position value.
     * @param position Position setting value.
     */
    void set_position(const int32_t position);

    /**
     * @~Chinese
     * @brief 重置电机的编码器脉冲数。
     * @param pulse_count 编码器脉冲数。
     */
    /**
     * @~English
     * @brief Reset the encoder pulse count of the motor.
     * @param pulse_count Encoder pulse count.
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
     * @brief 以设定的速度值（RPM）运行电机。
     * @param rpm 速度设定值（RPM）。
     */
    /**
     * @~English
     * @brief Run the motor at the set speed value (RPM).
     * @param rpm Speed setting value (RPM).
     */
    void RunSpeed(const int32_t rpm);

    /**
     * @~Chinese
     * @brief 直接设置电机的PWM占空比。
     * @param pwm_duty PWM占空比（取值范围 -1023到1023）。正数代表正转，负数代表反转。
     */
    /**
     * @~English
     * @brief Directly set the PWM duty cycle of the motor.
     * @param pwm_duty PWM duty cycle (value range -1023 to 1023). Positive values represent forward rotation, negative values represent reverse
     * rotation.
     */
    void RunPwmDuty(const int16_t pwm_duty);

    /**
     * @~Chinese
     * @brief 将电机移动到指定位置。
     * @param position 目标位置。
     * @param speed 运行速度。
     */
    /**
     * @~English
     * @brief Move the motor to the specified position.
     * @param position Target position.
     * @param speed Running speed.
     */
    void MoveTo(const int32_t position, const int32_t speed);

    /**
     * @~Chinese
     * @brief 电机相对移动指定距离。
     * @param offset 相对位移。
     * @param speed 运行速度。
     */
    /**
     * @~English
     * @brief Move the motor relatively by a specified distance.
     * @param offset Relative displacement.
     * @param speed Running speed.
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
     * @brief 获取电机当前的转速（RPM）。
     * @return 电机当前的转速（RPM）。
     */
    /**
     * @~English
     * @brief Get the current speed of the motor (RPM).
     * @return The current speed of the motor (RPM).
     */
    int32_t speed();

    /**
     * @~Chinese
     * @brief 获取电机位置。
     * @return 电机位置。
     */
    /**
     * @~English
     * @brief Get the motor position.
     * @return The motor position.
     */
    int32_t position();

    /**
     * @~Chinese
     * @brief 获取电机当前的编码器脉冲数。
     * @return 电机当前的编码器脉冲数。
     */
    /**
     * @~English
     * @brief Get the current encoder pulse count of the motor.
     * @return The current encoder pulse count of the motor.
     */
    int32_t pulse_count();

    /**
     * @~Chinese
     * @brief 获取电机当前的PWM占空比。
     * @return 电机当前的PWM占空比（取值范围 -1023到1023）。正数代表正转，负数代表反转。
     */
    /**
     * @~English
     * @brief Get the current PWM duty cycle of the motor.
     * @return The current PWM duty cycle of the motor (value range -1023 to 1023). Positive values represent forward rotation, negative values
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
   * @param i2c_address I2C地址。
   * @param wire TwoWire 对象引用。
   */
  /**
   * @~English
   * @brief Constructor.
   * @param i2c_address I2C address.
   * @param wire TwoWire object reference.
   */
  Md40(const uint8_t i2c_address, TwoWire &wire);

  /**
   * @~Chinese
   * @brief 获取指定索引的电机对象。
   * @param index 电机索引。
   * @return 电机对象引用。
   */
  /**
   * @~English
   * @brief Get the motor object with specified index.
   * @param index Motor index.
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
#pragma once

#include <WString.h>
#include <stdint.h>

namespace em {
namespace md40 {

enum class MotorStateCode : uint8_t {
  kIdle = 0,
  kRuningWithPwmDuty = 1,
  kRuningWithSpeed = 2,
  kRuningToPosition = 3,
  kReachedPosition = 4,
};

String ToString(MotorStateCode state_code);

}  // namespace md40
}  // namespace em
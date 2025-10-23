#include "motor_state_code.h"

namespace em {
namespace md40 {

String ToString(MotorStateCode state_code) {
  switch (state_code) {
    case MotorStateCode::kIdle:
      return F("Idle");
    case MotorStateCode::kRuningWithPwmDuty:
      return F("RuningWithPwmDuty");
    case MotorStateCode::kRuningWithSpeed:
      return F("RuningWithSpeed");
    case MotorStateCode::kRuningToPosition:
      return F("RuningToPosition");
    case MotorStateCode::kReachedPosition:
      return F("ReachedPosition");
    default:
      return F("Unknown MotorState Code");
  }
}

}  // namespace md40
}  // namespace em
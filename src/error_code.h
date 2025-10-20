#pragma once

#include <WString.h>
#include <stdint.h>

namespace em {
namespace md40 {

enum class ErrorCode : uint8_t {
  kOK = 0,
  kI2cDataTooLongToFitInTransmitBuffer = 1,
  kI2cReceivedNackOnTransmitOfAddress = 2,
  kI2cReceivedNackOnTransmitOfData = 3,
  kI2cOtherError = 4,
  kI2cTimeout = 5,
  kInvalidParameter = 6,
  kUnknownError = 7,
};

String ToString(ErrorCode error_code);

}  // namespace md40
}  // namespace em
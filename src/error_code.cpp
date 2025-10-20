#include "error_code.h"

namespace em {
namespace md40 {

String ToString(ErrorCode error_code) {
  switch (error_code) {
    case ErrorCode::kOK:
      return F("OK");
    case ErrorCode::kI2cDataTooLongToFitInTransmitBuffer:
      return F("I2cDataTooLongToFitInTransmitBuffer");
    case ErrorCode::kI2cReceivedNackOnTransmitOfAddress:
      return F("I2cReceivedNackOnTransmitOfAddress");
    case ErrorCode::kI2cReceivedNackOnTransmitOfData:
      return F("I2cReceivedNackOnTransmitOfData");
    case ErrorCode::kI2cOtherError:
      return F("I2cOtherError");
    case ErrorCode::kI2cTimeout:
      return F("I2cTimeout");
    case ErrorCode::kInvalidParameter:
      return F("InvalidParameter");
    case ErrorCode::kUnknownError:
      return F("UnknownError");
    default:
      return F("Unknown ErrorCode Code");
  }
}

}  // namespace md40
}  // namespace em
#pragma once

#ifndef _EM_MD40_LIB_H_
#define _EM_MD40_LIB_H_

#include <WString.h>

namespace em {
namespace md40_lib {

constexpr uint8_t kVersionMajor = 1;

constexpr uint8_t kVersionMinor = 0;

constexpr uint8_t kVersionPatch = 0;

String Version() {
  return String(kVersionMajor) + '.' + kVersionMinor + '.' + kVersionPatch;
}
}  // namespace md40_lib
}  // namespace em
#endif
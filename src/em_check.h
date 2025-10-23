#pragma once
#ifndef _EM_CHECK_H_
#define _EM_CHECK_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
#include <stdio.h>
#endif

static inline void AssertFailHandle(const char* expr, const char* function, const char* file, const int line) {
#ifdef ARDUINO_ARCH_ESP32
  printf("\nassert failed: %s %s:%d (%s)\n", function, file, line, expr);
  abort();
#else
  Serial.print(F("\nassert failed: "));
  Serial.print(function);
  Serial.print(F(" "));
  Serial.print(file);
  Serial.print(F(":"));
  Serial.print(line);
  Serial.print(F(" ("));
  Serial.print(expr);
  Serial.println(F(")"));
  Serial.flush();
  noInterrupts();
  while (true) {
  }
#endif
}

#define EM_CHECK(expr) ((expr) ? (void)0 : AssertFailHandle(#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#define EM_CHECK_EQ(a, b) ((a) == (b) ? (void)0 : AssertFailHandle(#a " == " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#define EM_CHECK_NE(a, b) ((a) != (b) ? (void)0 : AssertFailHandle(#a " != " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#define EM_CHECK_GT(a, b) ((a) > (b) ? (void)0 : AssertFailHandle(#a " > " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#define EM_CHECK_LT(a, b) ((a) < (b) ? (void)0 : AssertFailHandle(#a " < " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#define EM_CHECK_GE(a, b) ((a) >= (b) ? (void)0 : AssertFailHandle(#a " >= " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#define EM_CHECK_LE(a, b) ((a) <= (b) ? (void)0 : AssertFailHandle(#a " <= " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#endif
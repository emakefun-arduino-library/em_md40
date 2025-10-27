#pragma once
#ifndef _EM_CHECK_H_
#define _EM_CHECK_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32
#include <stdio.h>
#endif

/**
 * @file em_check.h
 */

/**
 * @~Chinese
 * @brief 断言失败处理函数
 * @param expr 断言失败的表达式字符串
 * @param function 发生断言的函数名
 * @param file 发生断言的源文件名
 * @param line 发生断言的行号
 * @details 当断言失败时，此函数会输出错误信息并停止程序运行
 *          - 在ESP32平台上使用printf输出并调用abort()
 *          - 在其他Arduino平台上使用Serial输出并进入死循环
 */
/**
 * @~English
 * @brief Assertion failure handling function
 * @param expr The expression string that failed the assertion
 * @param function The function name where assertion occurred
 * @param file The source file name where assertion occurred
 * @param line The line number where assertion occurred
 * @details When an assertion fails, this function outputs error information and stops program execution
 *          - On ESP32 platform, uses printf output and calls abort()
 *          - On other Arduino platforms, uses Serial output and enters an infinite loop
 */
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

/**
 * @~Chinese
 * @brief 基本断言检查宏
 * @param expr 要检查的表达式
 * @details 如果表达式为假，则触发断言失败处理
 */
/**
 * @~English
 * @brief Basic assertion check macro
 * @param expr Expression to check
 * @details If the expression is false, triggers assertion failure handling
 */
#define EM_CHECK(expr) ((expr) ? (void)0 : AssertFailHandle(#expr, __PRETTY_FUNCTION__, __FILE__, __LINE__))

/**
 * @~Chinese
 * @brief 相等断言检查宏
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @details 如果a不等于b，则触发断言失败处理
 */
/**
 * @~English
 * @brief Equality assertion check macro
 * @param a First comparison value
 * @param b Second comparison value
 * @details If a is not equal to b, triggers assertion failure handling
 */
#define EM_CHECK_EQ(a, b) ((a) == (b) ? (void)0 : AssertFailHandle(#a " == " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

/**
 * @~Chinese
 * @brief 不相等断言检查宏
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @details 如果a等于b，则触发断言失败处理
 */
/**
 * @~English
 * @brief Inequality assertion check macro
 * @param a First comparison value
 * @param b Second comparison value
 * @details If a is equal to b, triggers assertion failure handling
 */
#define EM_CHECK_NE(a, b) ((a) != (b) ? (void)0 : AssertFailHandle(#a " != " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

/**
 * @~Chinese
 * @brief 大于断言检查宏
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @details 如果a不大于b，则触发断言失败处理
 */
/**
 * @~English
 * @brief Greater than assertion check macro
 * @param a First comparison value
 * @param b Second comparison value
 * @details If a is not greater than b, triggers assertion failure handling
 */
#define EM_CHECK_GT(a, b) ((a) > (b) ? (void)0 : AssertFailHandle(#a " > " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

/**
 * @~Chinese
 * @brief 小于断言检查宏
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @details 如果a不小于b，则触发断言失败处理
 */
/**
 * @~English
 * @brief Less than assertion check macro
 * @param a First comparison value
 * @param b Second comparison value
 * @details If a is not less than b, triggers assertion failure handling
 */
#define EM_CHECK_LT(a, b) ((a) < (b) ? (void)0 : AssertFailHandle(#a " < " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

/**
 * @~Chinese
 * @brief 大于等于断言检查宏
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @details 如果a不大于等于b，则触发断言失败处理
 */
/**
 * @~English
 * @brief Greater than or equal assertion check macro
 * @param a First comparison value
 * @param b Second comparison value
 * @details If a is not greater than or equal to b, triggers assertion failure handling
 */
#define EM_CHECK_GE(a, b) ((a) >= (b) ? (void)0 : AssertFailHandle(#a " >= " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

/**
 * @~Chinese
 * @brief 小于等于断言检查宏
 * @param a 第一个比较值
 * @param b 第二个比较值
 * @details 如果a不小于等于b，则触发断言失败处理
 */
/**
 * @~English
 * @brief Less than or equal assertion check macro
 * @param a First comparison value
 * @param b Second comparison value
 * @details If a is not less than or equal to b, triggers assertion failure handling
 */
#define EM_CHECK_LE(a, b) ((a) <= (b) ? (void)0 : AssertFailHandle(#a " <= " #b, __PRETTY_FUNCTION__, __FILE__, __LINE__))

#endif
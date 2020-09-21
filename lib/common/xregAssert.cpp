/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "xregAssert.h"

#include <cstdio>
#include <csignal>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

namespace
{

enum AssertFailHaltMethod
{
  kTRAP_DEBUG,
  kTHROW_EXCEPTION,
  kSPIN
};

// TODO: make this selectable programmatically (or via the PreProcessor)
static const AssertFailHaltMethod gAssertFailMethod = kTRAP_DEBUG;

}  // un-named

xreg::AssertFailedException::AssertFailedException(const char* expr_str, const char* file_str, const int line_num)
    // It's OK to do this, since the const char*'s are all literals created via
    // pre-processor macros and will therefore never dangle
  : expr_str_(expr_str), file_str_(file_str), line_num_(line_num)
{
  snprintf(what_buf_, sizeof(what_buf_), "%s  %s:%d", expr_str, file_str, line_num);
  what_buf_[sizeof(what_buf_) - 1] = 0;
}

void xreg::AssertCheck(const bool expr, const char* expr_str, const char* file_str, const int line_num)
{
  if (!expr)
  {
    fprintf(stderr, "Assertion failed: %s  (%s:%d)\n", expr_str, file_str, line_num);

    switch (gAssertFailMethod)
    {
    case kTHROW_EXCEPTION:
      throw AssertFailedException(expr_str, file_str, line_num);
      break;
    case kSPIN:
      do { } while (true);
      break;
    case kTRAP_DEBUG:
    default:
      std::raise(SIGABRT);  // this should trigger the debugger into action
      break;
    }
  }
}

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

#ifndef XREGASSERT_H_
#define XREGASSERT_H_

#include <exception>

#ifdef XREG_DISABLE_ASSERTS

#define xregASSERT(X) ((void) 0)
// useful when the only time a variable is used, is for an assertion statement.
#define XREG_NO_ASSERTS_UNUSED(X)

#else

#define xregASSERT(X) xreg::AssertCheck((X), #X, __FILE__, __LINE__)
#define XREG_NO_ASSERTS_UNUSED(X) X

#endif

namespace xreg
{

void AssertCheck(const bool expr, const char* expr_str, const char* file_str, const int line_num);

class AssertFailedException : public std::exception
{
public:
  AssertFailedException(const char* expr_str, const char* file_str, const int line_num);

  virtual ~AssertFailedException() throw() { }

  virtual const char* what() const throw()
  {
    return what_buf_;
  }

  const char* expr_str() const throw()
  {
    return expr_str_;
  }

  const char* file_str() const throw()
  {
    return file_str_;
  }

  int line_num() const throw()
  {
    return line_num_;
  }

private:

  const char* expr_str_;
  const char* file_str_;
  const int   line_num_;

  char what_buf_[1024];
};

}  // xreg

#endif

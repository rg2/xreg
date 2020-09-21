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

#ifndef XREGOBJWITHOSTREAM_H_
#define XREGOBJWITHOSTREAM_H_

#include <iostream>
#include <memory>

namespace xreg
{

class ObjWithOStream
{
public:
  ObjWithOStream();

  ObjWithOStream(const ObjWithOStream& other);

  ObjWithOStream& operator=(const ObjWithOStream& rhs);

  std::ostream& dout() const
  {
    return *cur_out_;
  }

  void enable_debug_output();

  void disable_debug_output();
  
  void set_debug_output_stream(std::ostream& dout, const bool and_enable = false);
  
  void set_debug_output_stream(const ObjWithOStream& alg);

private:
  void copy(const ObjWithOStream& other);

  mutable std::ostream* cur_out_;
  std::ostream* dout_;
  
  std::unique_ptr<std::ostream> nos_;
};

}  // xreg

#endif

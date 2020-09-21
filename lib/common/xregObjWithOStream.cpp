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

#include "xregObjWithOStream.h"

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>

xreg::ObjWithOStream::ObjWithOStream()
  : dout_(&std::cout)
{
  nos_.reset(new boost::iostreams::stream<boost::iostreams::null_sink>(boost::iostreams::null_sink()));

  disable_debug_output();
}

xreg::ObjWithOStream::ObjWithOStream(const ObjWithOStream& other)
{
  copy(other);
}

xreg::ObjWithOStream& xreg::ObjWithOStream::operator=(const ObjWithOStream& rhs)
{
  if (this != &rhs)
  {
    copy(rhs);
  }

  return *this;
}

void xreg::ObjWithOStream::set_debug_output_stream(std::ostream& dout, const bool and_enable)
{
  dout_ = &dout;

  if (and_enable)
  {
    enable_debug_output();
  }
}

void xreg::ObjWithOStream::set_debug_output_stream(const ObjWithOStream& alg)
{
  cur_out_ = alg.cur_out_;
  dout_    = alg.dout_;
}
  
void xreg::ObjWithOStream::enable_debug_output()
{
  cur_out_ = dout_;
}

void xreg::ObjWithOStream::disable_debug_output()
{
  cur_out_ = nos_.get();
}

void xreg::ObjWithOStream::copy(const ObjWithOStream& other)
{
  dout_ = other.dout_;
  if (other.cur_out_ == dout_)
  {
    cur_out_ = dout_;
  }
  else
  {
    cur_out_ = nos_.get();
  }
}


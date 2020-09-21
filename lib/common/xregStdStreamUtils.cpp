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

#include "xregStdStreamUtils.h"

#include <ostream>  // std::ostream
#include <istream>  // std::istream

xreg::StdInputStream::StdInputStream(std::istream& in)
  : in_(&in)
{ }

void xreg::StdInputStream::set_stream(std::istream& in)
{
  in_ = &in;
}

void xreg::StdInputStream::raw_read(void* ptr, const InputStream::size_type num_bytes)
{
  in_->read(static_cast<char*>(ptr), num_bytes);
}

xreg::StdOutputStream::StdOutputStream(std::ostream& out)
  : out_(&out)
{ }

void xreg::StdOutputStream::set_stream(std::ostream& out)
{
  out_ = &out;
}

void xreg::StdOutputStream::raw_write(const void* ptr, const OutputStream::size_type num_bytes)
{
  out_->write(static_cast<const char*>(ptr), num_bytes);
}


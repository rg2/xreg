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

#include "xregStreams.h"

void xreg::Stream::switch_byte_order()
{
  byte_order_ = (byte_order_ == kNATIVE_ENDIAN) ? ((GetNativeByteOrder() == kBIG_ENDIAN) ? kLITTLE_ENDIAN : kBIG_ENDIAN)
                                                : (byte_order_ == kBIG_ENDIAN) ? kLITTLE_ENDIAN : kBIG_ENDIAN;
}

bool xreg::Stream::need_to_swap() const
{
  static const ByteOrder host_bo = GetNativeByteOrder();
  return (byte_order_ != kNATIVE_ENDIAN) && (byte_order_ != host_bo);
}

void xreg::InputStream::read(Stream::int8* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::int16* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::int32* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::int64* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::uint8* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::uint16* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::uint32* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::uint64* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::float32* x, const size_type len)
{
  read_helper(x, len);
}

void xreg::InputStream::read(Stream::float64* x, const size_type len)
{
  read_helper(x, len);
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::int8& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::int16& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::int32& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::int64& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::uint8& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::uint16& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::uint32& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::uint64& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::float32& x)
{
  read(&x);
  return *this;
}

xreg::InputStream& xreg::InputStream::operator>>(Stream::float64& x)
{
  read(&x);
  return *this;
}

xreg::InputStream::size_type xreg::InputStream::num_bytes_read() const
{
  return num_bytes_;
}

void xreg::InputStream::skip(const size_type num_bytes)
{
  // This is next to the dumbest implementation possible, but I don't think it
  // will be used very frequently.

  const size_type kNUM_BYTES_TMP_BUF = 256;
  uint8 buf[kNUM_BYTES_TMP_BUF];

  // The number of calls to read in the entire buffer
  const size_type num_chunks = num_bytes / kNUM_BYTES_TMP_BUF;

  // The number of bytes left to read, after reading big chunks into the buffer
  const size_type num_leftover = num_bytes - (num_chunks * kNUM_BYTES_TMP_BUF);

  for (size_type chunk_idx = 0; chunk_idx < num_chunks; ++chunk_idx)
  {
    read(buf, kNUM_BYTES_TMP_BUF);
  }
  read(buf, num_leftover);
}

std::string xreg::InputStream::read_ascii_line()
{
  std::string line;

  bool looking = true;

  char tmp_char = 0;

  while (looking)
  {
    read(reinterpret_cast<uint8*>(&tmp_char));
    
    if ((tmp_char != '\n') && (tmp_char != '\0'))
    {
      line.push_back(tmp_char);
    }
    else
    {
      looking = false;
    }
  }

  return line;
}

std::string xreg::InputStream::read_ascii_token()
{
  std::string tok;

  const char white_space_chars[4] = { ' ', '\t', '\n', '\r' };
  const char* not_white_space_it = white_space_chars + 4;

  char tmp_char = 0;

  bool looking = true;

  // find first non-whitespace character
  while (looking)
  {
    read(reinterpret_cast<uint8*>(&tmp_char));
    
    if (std::find(white_space_chars, white_space_chars + 4, tmp_char)
                                                        == not_white_space_it)
    {
      // we've skipped through the white space, add this char to the token
      tok.push_back(tmp_char);
      looking = false;
    }
  }

  // now find the next whitespace character
  looking = true;

  while (looking)
  {
    read(reinterpret_cast<uint8*>(&tmp_char));
    
    if (std::find(white_space_chars, white_space_chars + 4, tmp_char)
                                                        == not_white_space_it)
    {
      // still non-whitespace
      tok.push_back(tmp_char);
    }
    else
    {
      // back to white space
      looking = false;
    }
  }

  return tok;
}

void xreg::OutputStream::write(const Stream::int8& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::int16& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::int32& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::int64& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::uint8& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::uint16& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::uint32& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::uint64& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::float32& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::float64& x)
{
  write(&x, 1);
}

void xreg::OutputStream::write(const Stream::int8* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::int16* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::int32* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::int64* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::uint8* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::uint16* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::uint32* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::uint64* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::float32* x, const size_type len)
{
  write_helper(x, len);
}

void xreg::OutputStream::write(const Stream::float64* x, const size_type len)
{
  write_helper(x, len);
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::int8& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::int16& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::int32& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::int64& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::uint8& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::uint16& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::uint32& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::uint64& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::float32& x)
{
  write(x);
  return *this;
}

xreg::OutputStream& xreg::OutputStream::operator<<(const Stream::float64& x)
{
  write(x);
  return *this;
}

void xreg::OutputStream::write_ascii(const std::string& s)
{
  write(reinterpret_cast<const uint8*>(s.data()), s.size());
}

void xreg::OutputStream::write_ascii_line(const std::string& s)
{
  write(reinterpret_cast<const int8*>(s.data()), s.size());
  write_ascii_new_line_char();
}

void xreg::OutputStream::write_ascii_new_line_char()
{
  const char new_line_char = '\n';
  write(reinterpret_cast<const uint8*>(&new_line_char), 1);
}

xreg::OutputStream::size_type xreg::OutputStream::num_bytes_written() const
{
  return num_bytes_;
}

#define XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(T) \
void xreg::Serialize(const T& x, OutputStream& out) \
{ \
  out << x; \
} \
\
void xreg::DeSerialize(T* x, InputStream& in) \
{ \
  in >> *x; \
}

XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::uint8);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::int8);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::uint16);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::int16);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::uint32);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::int32);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::uint64);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::int64);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::float32);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEF(Stream::float64);

#undef XREG_MAKE_BASIC_SERIALIZE_FNS_DEF



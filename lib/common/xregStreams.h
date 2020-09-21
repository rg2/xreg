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

/**
 * @file
 * @brief Utilities for handling streams of data.
 **/

#ifndef XREGSTREAMS_H_
#define XREGSTREAMS_H_

#include <cstddef>
#include <string>
#include <vector>
#include <type_traits>

#include "xregEndianUtils.h"
#include "xregSizedTypeUtils.h"
#include "xregObjectUtils.h"

namespace xreg
{

class Stream
{
public:
  using size_type = size_t;

  using int8   = sized_types::int8;
  using int16  = sized_types::int16;
  using int32  = sized_types::int32;
  using uint8  = sized_types::uint8;
  using uint16 = sized_types::uint16;
  using uint32 = sized_types::uint32;
  using int64  = sized_types::int64;
  using uint64 = sized_types::uint64;

  using float32 = sized_types::float32;
  using float64 = sized_types::float64;

  Stream() = default;
  
  Stream(const Stream&) = delete;
  Stream& operator=(const Stream&) = delete;

  ByteOrder byte_order() const
  {
    return byte_order_;
  }
  
  void set_byte_order(const ByteOrder bo)
  {
    byte_order_ = bo;
  }

  void switch_byte_order();

protected:
  bool need_to_swap() const;

  ByteOrder byte_order_ = kNATIVE_ENDIAN;
  size_type num_bytes_  = 0;

private:
  static_assert(sizeof(int8)    == 1, "8-bit signed int wrong size!");
  static_assert(sizeof(uint8)   == 1, "8-bit unsigned int wrong size!");
  static_assert(sizeof(int16)   == 2, "16-bit signed int wrong size!");
  static_assert(sizeof(uint16)  == 2, "16-bit unsigned int wrong size!");
  static_assert(sizeof(int32)   == 4, "32-bit signed int wrong size!");
  static_assert(sizeof(uint32)  == 4, "32-bit unsigned int wrong size!");
  static_assert(sizeof(int64)   == 8, "64-bit signed int wrong size!");
  static_assert(sizeof(uint64)  == 8, "64-bit unsigned int wrong size!");
  static_assert(sizeof(float32) == 4, "32 bit float wrong size!");
  static_assert(sizeof(float64) == 8, "64 bit float wrong size!");
};

template <class T>
struct IsBasicSerializableType
{
  enum { value = 0 };
};

#define XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(T) \
template <> \
struct IsBasicSerializableType<T> \
{ \
  enum { value = 1 }; \
}

XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::uint8);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::int8);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::uint16);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::int16);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::uint32);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::int32);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::uint64);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::int64);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::float32);
XREG_MAKE_BASIC_SERIALIZABLE_TRAIT(Stream::float64);

#undef XREG_MAKE_BASIC_SERIALIZABLE_TRAIT

template <class T>
struct IsSerializable
{
  enum { value = 0 };
};

#define XREG_MAKE_SERIALIZABLE_TRAIT(T) \
template <> \
struct IsSerializable<T> \
{ \
  enum { value = 1 }; \
}

XREG_MAKE_SERIALIZABLE_TRAIT(Stream::uint8);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::int8);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::uint16);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::int16);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::uint32);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::int32);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::uint64);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::int64);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::float32);
XREG_MAKE_SERIALIZABLE_TRAIT(Stream::float64);

namespace detail
{

template <class T>
void ByteSwapArray(T* arr, const Stream::size_type len)
{
  for (Stream::size_type i = 0; i < len; ++i)
  {
    SwapByteOrder(arr + i);
  }
}

// For 1-byte types this is a noop
inline void ByteSwapArray(Stream::int8*, const Stream::size_type)
{ }

inline void ByteSwapArray(Stream::uint8*, const Stream::size_type)
{ }

}  // detail

class InputStream : public Stream
{
public:
  virtual ~InputStream() { }

  void read(Stream::int8* x, const size_type len = 1);
  void read(Stream::int16* x, const size_type len = 1);
  void read(Stream::int32* x, const size_type len = 1);
  void read(Stream::int64* x, const size_type len = 1);
  void read(Stream::uint8* x, const size_type len = 1);
  void read(Stream::uint16* x, const size_type len = 1);
  void read(Stream::uint32* x, const size_type len = 1);
  void read(Stream::uint64* x, const size_type len = 1);
  void read(Stream::float32* x, const size_type len = 1);
  void read(Stream::float64* x, const size_type len = 1);

  InputStream& operator>>(Stream::int8& x);
  InputStream& operator>>(Stream::int16& x);
  InputStream& operator>>(Stream::int32& x);
  InputStream& operator>>(Stream::int64& x);
  InputStream& operator>>(Stream::uint8& x);
  InputStream& operator>>(Stream::uint16& x);
  InputStream& operator>>(Stream::uint32& x);
  InputStream& operator>>(Stream::uint64& x);
  InputStream& operator>>(Stream::float32& x);
  InputStream& operator>>(Stream::float64& x);

  /// \brief Reads bytes, and treats as ascii characters, until
  ///        a UNIX new line character is reached.
  ///
  /// This does not attempt to deal with Windows style new lines.
  /// This is not efficient, and should only be used when minimal
  /// text processing is needed on a stream.
  std::string read_ascii_line();

  /// \brief Reads bytes, and treats as ascii characters, returning
  ///        the next white-space delimited token.
  ///
  /// This is not efficient, and should only be used when minimal
  /// text processing is needed on a stream.
  std::string read_ascii_token();

  /**
   * @brief Reads a buffer and casts/copies it to a
   *        buffer of different type.
   *
   * This is meant to be called in the following manner:
   * <code>
     int x[10];
     in.read_and_cast<int8>(x, 10);
     </code>
   * In the above example, the stream has data written to
   * it in 8-bit integers, but the user wishes to store
   * the data in an array of int's (which is most likely
   * and array of int32's).
   * @param ptr The final buffer to be populated, does not
   *            need to be a serializable type, but a
   *            conversion must be well-defined.
   * @param len The length of the final buffer and number
   *            of items to be read from the stream
   **/
  template <class U, class T>
  void read_and_cast(T* ptr, const size_type len = 1)
  {
    static_assert(IsBasicSerializableType<U>::value, "type must be serializable");

    if (std::is_same<T,U>::value)
    {
      read(ptr, len);
    }
    else
    {
      // really inefficient for small lengths, but I'm anticipating
      // using this with large buffers, so let's roll with it for now
      // TODO: implement with 2 branches, one with a static buffer,
      //       the other with a dynamic buffer
      std::vector<U> tmp_buf(len);

      read(&tmp_buf[0], len);

      CopyObjectArray(ptr, &tmp_buf[0], len);
    }
  }

  size_type num_bytes_read() const;

  /// \brief Reads in bytes and discards them; to advance the stream.
  ///
  /// Stream objects with more knowledge about the stream source may implement
  /// this more efficiently (e.g. a file input stream, can seek through the file)
  virtual void skip(const size_type num_bytes);

private:
  template <class T>
  void read_helper(T* ptr, const size_type len)
  {
    const size_type nbytes = len * sizeof(T);
    raw_read(ptr, nbytes);
    num_bytes_ += nbytes;

    if (need_to_swap())
    {
      detail::ByteSwapArray(ptr, len);
    }
  }

  virtual void raw_read(void* ptr, const size_type num_bytes) = 0;
};

class OutputStream : public Stream
{
public:
  virtual ~OutputStream() { }

  void write(const Stream::int8& x);
  void write(const Stream::int16& x);
  void write(const Stream::int32& x);
  void write(const Stream::int64& x);
  void write(const Stream::uint8& x);
  void write(const Stream::uint16& x);
  void write(const Stream::uint32& x);
  void write(const Stream::uint64& x);
  void write(const Stream::float32& x);
  void write(const Stream::float64& x);

  void write(const Stream::int8* x, const size_type len);
  void write(const Stream::int16* x, const size_type len);
  void write(const Stream::int32* x, const size_type len);
  void write(const Stream::int64* x, const size_type len);
  void write(const Stream::uint8* x, const size_type len);
  void write(const Stream::uint16* x, const size_type len);
  void write(const Stream::uint32* x, const size_type len);
  void write(const Stream::uint64* x, const size_type len);
  void write(const Stream::float32* x, const size_type len);
  void write(const Stream::float64* x, const size_type len);

  OutputStream& operator<<(const Stream::int8& x);
  OutputStream& operator<<(const Stream::int16& x);
  OutputStream& operator<<(const Stream::int32& x);
  OutputStream& operator<<(const Stream::int64& x);
  OutputStream& operator<<(const Stream::uint8& x);
  OutputStream& operator<<(const Stream::uint16& x);
  OutputStream& operator<<(const Stream::uint32& x);
  OutputStream& operator<<(const Stream::uint64& x);
  OutputStream& operator<<(const Stream::float32& x);
  OutputStream& operator<<(const Stream::float64& x);

  /// \brief Writes an ascii string
  void write_ascii(const std::string& s);

  /// \brief Writes an ascii string AND a new line character.
  ///
  /// The input string is written, and a new line character is
  /// then separately written.
  void write_ascii_line(const std::string& s);

  /// Write a single UNIX new line character.
  void write_ascii_new_line_char();

  size_type num_bytes_written() const;

private:
  template <class T>
  void write_helper(const T* ptr, const size_type len)
  {
    if ((sizeof(T) == 1) || !need_to_swap())
    {
      const size_type nbytes = len * sizeof(T);
      raw_write(ptr, nbytes);
      num_bytes_ += nbytes;
    }
    else
    {
      for (size_type i = 0; i < len; ++i)
      {
        for (size_type j = 0; j < sizeof(T); ++j, ++num_bytes_)
        {
          raw_write(reinterpret_cast<const Stream::uint8*>(ptr + i) + sizeof(T) - 1 - j, 1);
        }
      }
    }
  }

  virtual void raw_write(const void* ptr, const size_type num_bytes) = 0;
};

#define XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(T)   \
void Serialize(const T& x, OutputStream& out); \
void DeSerialize(T* x, InputStream& in) 

XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::uint8);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::int8);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::uint16);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::int16);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::uint32);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::int32);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::uint64);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::int64);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::float32);
XREG_MAKE_BASIC_SERIALIZE_FNS_DEC(Stream::float64);

#undef XREG_MAKE_BASIC_SERIALIZE_FNS_DEC

} // xreg

#endif

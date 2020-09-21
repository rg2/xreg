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
 * @brief Utilities for handling byte order of data.
 **/

#ifndef XREGENDIANUTILS_H_
#define XREGENDIANUTILS_H_

#include <type_traits>

namespace xreg
{

/// @brief Byte Order enumeration
enum ByteOrder
{
  kLITTLE_ENDIAN,  ///< Little Endian (Intel)
  kBIG_ENDIAN,     ///< Big Endian (IBM/Motorola)
  kNATIVE_ENDIAN   ///< The byte order of the host machine executing the code
};

/**
 * @brief Finds the native byte order of the host executing this code.
 *
 * @return kLITTLE_ENDIAN or kBIG_ENDIAN, depending on the host
 **/
ByteOrder GetNativeByteOrder();

/**
 * @brief Given a byte order value that may be set to "native,"
 *        convert to the appropriate big or little values.
 * @param bo The byte order value to check
 * @return kLITTLE_ENDIAN if bo is kLITTLE_ENDIAN or bo is
 *         kNATIVE_ENDIAN and the native endianess is little.
 *         kBIG_ENDIAN if bo is kBIG_ENDIAN or bo is
 *         kNATIVE_ENDIAN and the native endianess is big.
 **/
inline ByteOrder GetBigOrLittleByteOrder(const ByteOrder bo)
{
  return ((bo == kLITTLE_ENDIAN) || (bo == kBIG_ENDIAN)) ? bo : GetNativeByteOrder();
}

/**
 * @brief Given a compile time value of ByteOrder type, obtain
 *        kLITTLE_ENDIAN or kBIG_ENDIAN; kNATIVE_ENDIAN will
 *        trigger a runtime check.
 **/
template <ByteOrder tBO>
struct LookupLittleOrBigByteOrder
{
  static ByteOrder value()
  {
    return tBO;
  };
};

/**
 * @brief Given a compile time value of ByteOrder type, obtain
 *        kLITTLE_ENDIAN or kBIG_ENDIAN; kNATIVE_ENDIAN will
 *        trigger a runtime check.
 **/
template <>
struct LookupLittleOrBigByteOrder<kNATIVE_ENDIAN>
{
  static ByteOrder value()
  {
    return GetNativeByteOrder();
  }
};

/**
 * @brief Performs byte swapping on a type of arbitray size
 *
 * @param x Data to swap
 **/
template <class T>
void SwapByteOrder(T* x)
{
  static_assert(std::is_fundamental<T>::value, "Cannot swap bytes on non-fundamental type!");
  char* arr = reinterpret_cast<char*>(x);
  char tmp = 0;
  for (unsigned i = 0; i < (sizeof(T) >> 1); ++i)
  {
    tmp = arr[i];
    arr[i] = arr[sizeof(T) - i - 1];
    arr[sizeof(T) - i - 1] = tmp;
  }
}

/**
 * @brief Performs byte swapping on a 16 byte type.
 *
 * @param x Data to swap
 **/
inline
void SwapByteOrder16(void* x)
{
  char* arr = static_cast<char*>(x);
  char tmp = arr[0];
  arr[0] = arr[1];
  arr[1] = tmp;
}

/**
 * @brief Performs byte swapping on a 32 byte type.
 *
 * @param x Data to swap
 **/
inline
void SwapByteOrder32(void* x)
{
  char* arr = static_cast<char*>(x);
  char tmp = arr[0];
  arr[0] = arr[3];
  arr[3] = tmp;
  tmp = arr[1];
  arr[1] = arr[2];
  arr[2] = tmp;
}

/**
 * @brief Performs byte swapping on a 64 byte type.
 *
 * @param x Data to swap
 **/
inline
void SwapByteOrder64(void* x)
{
  char* arr = static_cast<char*>(x);
  char tmp = arr[0];
  arr[0] = arr[7];
  arr[7] = tmp;
  tmp = arr[1];
  arr[1] = arr[6];
  arr[6] = tmp;
  tmp = arr[2];
  arr[2] = arr[5];
  arr[5] = tmp;
  tmp = arr[3];
  arr[3] = arr[4];
  arr[4] = tmp;
}

} // xreg

#endif

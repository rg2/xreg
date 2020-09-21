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
 * @brief Types and utilities for types of specific sizes.
 **/

#ifndef XREGSIZEDTYPEUTILS_H_
#define XREGSIZEDTYPEUTILS_H_

#if defined(_MSC_VER) && (_MSC_VER <= 1500)
// VS2008 and earlier do not have stdint.h :)
#define XREG_HAS_STDINT 0
#else
#define XREG_HAS_STDINT 1
#endif

#if XREG_HAS_STDINT
#include <stdint.h>
#endif

namespace xreg
{

/// Fixed-size typedefs
namespace sized_types
{

#if XREG_HAS_STDINT
using int8   = int8_t;
using uint8  = uint8_t;
using int16  = int16_t;
using uint16 = uint16_t;
using int32  = int32_t;
using uint32 = uint32_t;
using int64  = int64_t;
using uint64 = uint64_t;
#elif defined(_MSC_VER)
using int8  =  __int8;
using int16  = __int16;
using int32  = __int32;
using int64  = __int64;
using uint8  = unsigned __int8;
using uint16 = unsigned __int16;
using uint32 = unsigned __int32;
using uint64 = unsigned __int64;
#endif

using float32 = float;
using float64 = double;

}  // sized_types

}  // xreg

#undef XREG_HAS_STDINT

#endif

/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#ifndef XREGMETALSHADERSHARED_H_
#define XREGMETALSHADERSHARED_H_

// Definitions shared by host C++ code and the Metal shaders in lib/metal.
// This file is valid C++ and Metal Shading Language, it is also embedded in
// the shader source compiled at runtime.

namespace xreg
{

/// \brief Indices of the function constants used by the shader helpers.
///
/// Every function constant used in an xreg Metal library must have a unique
/// index, so all indices are listed here.
enum MetalFnConstIdx
{
  /// bool: true -> use hardware linear filtering when sampling volumes, false -> software trilinear
  kMETAL_FN_CONST_HW_LINEAR_INTERP = 0
};

/// \brief Buffer argument indices for the xregFillFloat kernel.
enum MetalFillFloatBufIdx
{
  kMETAL_FILL_FLOAT_BUF = 0,
  kMETAL_FILL_FLOAT_VAL,
  kMETAL_FILL_FLOAT_LEN
};

}  // xreg

#endif


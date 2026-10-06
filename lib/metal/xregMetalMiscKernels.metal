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

// Miscellaneous Metal kernels, the counterpart of xregOpenCLMiscKernels.

#ifndef XREG_METAL_MISC_KERNELS_METAL_
#define XREG_METAL_MISC_KERNELS_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregMetalShaderShared.h"
#endif

#include <metal_stdlib>

/// \brief Sets every element of a buffer to a value.
///
/// A blit encoder may only fill a buffer with a repeated byte, which is not
/// sufficient for arbitrary floating point values.
kernel void xregFillFloat(device float* buf    [[buffer(xreg::kMETAL_FILL_FLOAT_BUF)]],
                          constant float& val  [[buffer(xreg::kMETAL_FILL_FLOAT_VAL)]],
                          constant uint& len   [[buffer(xreg::kMETAL_FILL_FLOAT_LEN)]],
                          const uint idx       [[thread_position_in_grid]])
{
  if (idx < len)
  {
    buf[idx] = val;
  }
}

#endif


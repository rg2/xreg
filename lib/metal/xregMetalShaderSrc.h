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

#ifndef XREGMETALSHADERSRC_H_
#define XREGMETALSHADERSRC_H_

#include <string>

namespace xreg
{

// Sources of the Metal shader helpers in lib/metal, these are embedded at
// build time from the corresponding files (see xreg_embed_metal_src).

extern const char* const kMETAL_SHADER_SHARED_SRC;  ///< xregMetalShaderShared.h
extern const char* const kMETAL_MATH_SRC;           ///< xregMetalMath.metal
extern const char* const kMETAL_SPATIAL_SRC;        ///< xregMetalSpatial.metal
extern const char* const kMETAL_INTERP_SRC;         ///< xregMetalInterp.metal
extern const char* const kMETAL_MISC_KERNELS_SRC;   ///< xregMetalMiscKernels.metal

/// \brief All of the Metal shader helper sources, concatenated in dependency
///        order.
///
/// Shader sources that depend on the helpers should be appended to this
/// before compiling with MetalLibrary.
std::string MetalHelperShadersSrc();

}  // xreg

#endif


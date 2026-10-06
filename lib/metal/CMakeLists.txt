# MIT License
#
# Copyright (c) 2026 Robert Grupp
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

file(GLOB_RECURSE XREG_CUR_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

# Metal shader sources, embedded as strings and compiled at runtime
set(XREG_METAL_SHADER_SRCS)
xreg_embed_metal_src(XREG_METAL_SHADER_SRCS kMETAL_SHADER_SHARED_SRC xregMetalShaderShared.h)
xreg_embed_metal_src(XREG_METAL_SHADER_SRCS kMETAL_MATH_SRC          xregMetalMath.metal)
xreg_embed_metal_src(XREG_METAL_SHADER_SRCS kMETAL_SPATIAL_SRC       xregMetalSpatial.metal)
xreg_embed_metal_src(XREG_METAL_SHADER_SRCS kMETAL_INTERP_SRC        xregMetalInterp.metal)
xreg_embed_metal_src(XREG_METAL_SHADER_SRCS kMETAL_MISC_KERNELS_SRC  xregMetalMiscKernels.metal)

# The sources here are Objective-C++, however the headers are pure C++ and may
# be included by any other component.
add_library(xreg_metal OBJECT ${XREG_CUR_LIB_HEADERS}
                              ${XREG_METAL_SHADER_SRCS}
                              xregMetalSys.mm
                              xregMetalCompute.mm
                              xregMetalTexture.mm)


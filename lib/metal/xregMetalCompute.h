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

#ifndef XREGMETALCOMPUTE_H_
#define XREGMETALCOMPUTE_H_

// Pure C++ interface for compiling Metal shaders at runtime and creating compute
// pipelines. Objective-C++ code may access the underlying Metal objects through
// native_handle().

#include <map>
#include <memory>
#include <string>

#include "xregMetalSys.h"

namespace xreg
{

/// \brief A library of Metal shader functions compiled at runtime from source.
///
/// The macro XREG_METAL_RUNTIME_SRC is always defined, which xreg shader sources
/// use to omit their #include directives of other xreg shader sources (which
/// cannot be resolved at runtime). Dependent sources should be concatenated
/// prior to compilation, e.g. MetalHelperShadersSrc() + my_src.
///
/// Copies of this object refer to the same underlying library.
class MetalLibrary
{
public:
  using MacroMap = std::map<std::string,std::string>;

  MetalLibrary() = default;

  /// \brief Compile a library from source, throws with the compiler log on failure.
  ///
  /// Fast math is disabled by default, since some routines rely on IEEE
  /// semantics for infinities and NaNs (e.g. the intersection routines in
  /// xregMetalSpatial.metal).
  MetalLibrary(const MetalDevice& dev, const std::string& src,
               const MacroMap& macros = MacroMap(), const bool fast_math = false);

  bool valid() const;

  const MetalDevice& device() const;

  /// \brief The underlying id<MTLLibrary>, ownership is NOT transferred.
  void* native_handle() const;

private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
};

/// \brief A compute pipeline for a single kernel function in a library.
///
/// Copies of this object refer to the same underlying pipeline.
class MetalComputePipeline
{
public:
  /// \brief Map of function constant index to boolean value.
  using BoolFnConstMap = std::map<size_type,bool>;

  MetalComputePipeline() = default;

  /// \brief Create a pipeline for a kernel function, throws on failure.
  ///
  /// Every function constant used by the kernel must be assigned a value.
  MetalComputePipeline(const MetalLibrary& lib, const std::string& kernel_name,
                       const BoolFnConstMap& bool_fn_consts = BoolFnConstMap());

  bool valid() const;

  const std::string& kernel_name() const;

  /// \brief The number of threads executed in lock-step (SIMD group width).
  size_type thread_execution_width() const;

  /// \brief The maximum number of threads in a threadgroup for this pipeline.
  size_type max_total_threads_per_threadgroup() const;

  /// \brief The underlying id<MTLComputePipelineState>, ownership is NOT transferred.
  void* native_handle() const;

private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
};

}  // xreg

#endif


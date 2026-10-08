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

#include <array>
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

/// \brief Encodes compute kernel dispatches into a command buffer of a queue.
///
/// Dispatches are executed in the order they are encoded and each dispatch
/// observes the memory writes of previous dispatches, so dependent kernels may
/// be encoded together and waited on once with commit_and_wait(). Arguments
/// (the pipeline, buffers and bytes) persist between dispatches until they are
/// replaced.
///
/// Encoded work that has not been committed is discarded on destruction.
class MetalComputeEncoder
{
public:
  using Size3 = std::array<size_type,3>;

  /// \brief The maximum number of bytes that may be passed with set_bytes().
  static constexpr size_type kMAX_SET_BYTES_LEN = 4096;

  explicit MetalComputeEncoder(MetalCmdQueue& queue);

  MetalComputeEncoder(const MetalComputeEncoder&) = delete;
  MetalComputeEncoder& operator=(const MetalComputeEncoder&) = delete;

  ~MetalComputeEncoder();

  void set_pipeline(const MetalComputePipeline& pipeline);

  /// \brief Bind a buffer at an index of the [[buffer(idx)]] argument table,
  ///        starting at a byte offset.
  void set_buffer(const MetalBuffer& buf, const size_type idx, const size_type off_bytes = 0);

  /// \brief Bind a buffer at an index of the [[buffer(idx)]] argument table,
  ///        starting at an element offset.
  template <class T>
  void set_buffer(const MetalVector<T>& v, const size_type idx, const size_type elem_off = 0)
  {
    set_buffer(v.buffer(), idx, elem_off * sizeof(T));
  }

  /// \brief Copy bytes (at most kMAX_SET_BYTES_LEN) into the [[buffer(idx)]]
  ///        argument table, e.g. for a constant argument.
  void set_bytes(const void* src, const size_type num_bytes, const size_type idx);

  /// \brief Copy a value into the [[buffer(idx)]] argument table.
  template <class T>
  void set_value(const T& v, const size_type idx)
  {
    static_assert(std::is_trivially_copyable<T>::value,
                  "values passed to Metal kernels must be trivially copyable");
    set_bytes(&v, sizeof(T), idx);
  }

  /// \brief Dispatch a grid of threads using the current pipeline.
  ///
  /// The grid does not need to be a multiple of the threadgroup size. Nothing
  /// is dispatched when the grid is empty.
  void dispatch_threads(const Size3& grid, const Size3& threadgroup);

  /// \brief Commit the encoded work, wait for it to complete and throw when
  ///        the device reports an error.
  ///
  /// No more work may be encoded after this call.
  void commit_and_wait();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // xreg

#endif


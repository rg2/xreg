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

#ifndef XREGMETALSYS_H_
#define XREGMETALSYS_H_

// This header is pure C++ so that it may be included by any translation unit.
// The Objective-C++ implementation is in xregMetalSys.mm. Objective-C++ code
// may access the underlying Metal objects through native_handle(), e.g.:
//   id<MTLBuffer> b = (__bridge id<MTLBuffer>) buf.native_handle();

#include <memory>
#include <string>
#include <type_traits>

#include "xregCommon.h"

namespace xreg
{

/// \brief Handle to a Metal GPU device.
///
/// Copies of this object refer to the same underlying device.
class MetalDevice
{
public:
  MetalDevice() = default;

  /// \brief Returns the system default Metal device.
  ///
  /// Throws if Metal is not available.
  static MetalDevice Default();

  bool valid() const;

  std::string name() const;

  /// \brief true when the CPU and GPU share memory (e.g. Apple Silicon)
  bool has_unified_memory() const;

  /// \brief The underlying id<MTLDevice>, ownership is NOT transferred.
  void* native_handle() const;

private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
};

/// \brief Handle to a Metal command queue.
///
/// Command buffers submitted to a single queue are executed in order.
/// Copies of this object refer to the same underlying queue.
class MetalCmdQueue
{
public:
  MetalCmdQueue() = default;

  explicit MetalCmdQueue(const MetalDevice& dev);

  bool valid() const;

  const MetalDevice& device() const;

  /// \brief Blocks until all work previously submitted to this queue has completed.
  void finish();

  /// \brief The underlying id<MTLCommandQueue>, ownership is NOT transferred.
  void* native_handle() const;

private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
};

/// \brief Storage mode used for a MetalBuffer.
///
/// kAUTO selects shared storage on devices with unified memory and managed
/// storage on devices with discrete memory. Both modes are CPU accessible.
enum class MetalStorageMode
{
  kAUTO,
  kSHARED,
  kMANAGED
};

/// \brief Untyped buffer of bytes allocated on a Metal device.
///
/// This object owns its allocation and is therefore move-only.
class MetalBuffer
{
public:
  MetalBuffer();

  explicit MetalBuffer(const MetalDevice& dev, const size_type num_bytes = 0,
                       const MetalStorageMode mode = MetalStorageMode::kAUTO);

  MetalBuffer(const MetalBuffer&) = delete;
  MetalBuffer& operator=(const MetalBuffer&) = delete;

  MetalBuffer(MetalBuffer&&);
  MetalBuffer& operator=(MetalBuffer&&);

  ~MetalBuffer();

  size_type num_bytes() const;

  /// \brief Reallocates the buffer when the size changes.
  ///
  /// Unlike std::vector, existing contents are NOT preserved.
  /// Throws if this buffer was not constructed with a device.
  void resize_bytes(const size_type num_bytes);

  /// \brief true when the buffer uses managed storage, which requires explicit
  ///        CPU/GPU synchronization (handled by CopyHostToMetal/CopyMetalToHost).
  bool is_managed() const;

  const MetalDevice& device() const;

  /// \brief The CPU address of the buffer contents, nullptr when empty.
  ///
  /// Prefer CopyHostToMetal/CopyMetalToHost, which handle synchronization.
  void* contents() const;

  /// \brief The underlying id<MTLBuffer>, ownership is NOT transferred.
  ///
  /// This is nullptr for an empty buffer, since Metal does not allow zero
  /// length buffers.
  void* native_handle() const;

private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

/// \brief Typed buffer on a Metal device, analogous to boost::compute::vector.
template <class T>
class MetalVector
{
  static_assert(std::is_trivially_copyable<T>::value,
                "MetalVector elements must be trivially copyable");

public:
  using value_type = T;

  MetalVector() = default;

  explicit MetalVector(const MetalDevice& dev, const size_type n = 0,
                       const MetalStorageMode mode = MetalStorageMode::kAUTO)
    : buf_(dev, n * sizeof(T), mode)
  { }

  size_type size() const
  {
    return buf_.num_bytes() / sizeof(T);
  }

  bool empty() const
  {
    return size() == 0;
  }

  /// \brief Reallocates when the size changes, existing contents are NOT preserved.
  void resize(const size_type n)
  {
    buf_.resize_bytes(n * sizeof(T));
  }

  MetalBuffer& buffer()
  {
    return buf_;
  }

  const MetalBuffer& buffer() const
  {
    return buf_;
  }

private:
  MetalBuffer buf_;
};

/// \brief Copy bytes from the host into a Metal buffer.
///
/// Waits for all work previously submitted to queue to complete before
/// writing, so the buffer is not modified while the GPU is using it.
/// The copy has completed when this function returns.
void CopyHostToMetal(const void* src, const size_type num_bytes,
                     MetalBuffer& dst, const size_type dst_off_bytes,
                     MetalCmdQueue& queue);

/// \brief Copy bytes from a Metal buffer to the host.
///
/// Waits for all work previously submitted to queue to complete before
/// reading. The copy has completed when this function returns.
void CopyMetalToHost(const MetalBuffer& src, const size_type src_off_bytes,
                     const size_type num_bytes, void* dst,
                     MetalCmdQueue& queue);

/// \brief Copy the host elements [src_begin, src_end) into dst, starting at
///        element dst_off.
template <class T>
void CopyHostToMetal(const T* src_begin, const T* src_end,
                     MetalVector<T>& dst, const size_type dst_off,
                     MetalCmdQueue& queue)
{
  CopyHostToMetal(src_begin, static_cast<size_type>(src_end - src_begin) * sizeof(T),
                  dst.buffer(), dst_off * sizeof(T), queue);
}

/// \brief Copy the elements [src_start, src_end) of src to the host, writing
///        to dst[0], dst[1], ...
template <class T>
void CopyMetalToHost(const MetalVector<T>& src, const size_type src_start,
                     const size_type src_end, T* dst, MetalCmdQueue& queue)
{
  CopyMetalToHost(src.buffer(), src_start * sizeof(T),
                  (src_end - src_start) * sizeof(T), dst, queue);
}

}  // xreg

#endif


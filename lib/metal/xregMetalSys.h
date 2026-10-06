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

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "xregCommon.h"

namespace xreg
{

/// \brief Physical location of a Metal device.
enum class MetalDeviceLocation
{
  kBUILT_IN,
  kSLOT,
  kEXTERNAL,
  kUNSPECIFIED
};

/// \brief Metal GPU feature families, ordered from least to most capable.
///
/// Only the families relevant to MacOS are listed. kUNKNOWN indicates that a
/// device does not support any of the listed families.
enum class MetalGPUFamily
{
  kUNKNOWN,
  kMAC2,
  kMETAL3,
  kMETAL4
};

/// \brief Human readable name of a GPU family, e.g. "Metal 3"
std::string MetalGPUFamilyStr(const MetalGPUFamily fam);

/// \brief Version of the Metal framework installed on the system, e.g. "373.7"
///
/// Returns an empty string when the version is not available.
std::string MetalFrameworkVersion();

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

  /// \brief Wraps an existing id<MTLDevice>, a reference is retained.
  static MetalDevice FromNativeHandle(void* mtl_dev);

  bool valid() const;

  std::string name() const;

  /// \brief Identifier for this device which is unique and consistent across
  ///        processes.
  ///
  /// This is the device name with whitespace removed and the registry ID
  /// appended, e.g. "AMDRadeonProXXXX-10000abcd".
  std::string id_str() const;

  /// \brief The registry ID of this device, which is consistent across processes.
  std::uint64_t registry_id() const;

  /// \brief true when the CPU and GPU share memory (e.g. Apple Silicon)
  bool has_unified_memory() const;

  /// \brief true for a low power device, e.g. the integrated GPU on a Mac with
  ///        both integrated and discrete GPUs.
  bool is_low_power() const;

  /// \brief true when the device is not attached to a display
  bool is_headless() const;

  /// \brief true for a removable device, e.g. an eGPU
  bool is_removable() const;

  MetalDeviceLocation location() const;

  /// \brief The most capable GPU family supported by this device.
  MetalGPUFamily highest_gpu_family() const;

  /// \brief Approximate number of bytes that may be used by the device without
  ///        degrading performance.
  size_type recommended_max_working_set_size() const;

  /// \brief The maximum size of a single buffer in bytes.
  size_type max_buffer_length() const;

  /// \brief The underlying id<MTLDevice>, ownership is NOT transferred.
  void* native_handle() const;

private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
};

using MetalIDStrDevMap = std::map<std::string,MetalDevice>;

/// \brief All Metal devices on the system.
///
/// Returns an empty list when Metal is not available.
std::vector<MetalDevice> MetalAllDevices();

/// \brief Used to uniquely map devices between two processes.
///
/// Keys are MetalDevice::id_str().
MetalIDStrDevMap BuildMetalDevIDStrsToDevMap();

/// \brief Get a list of the unique IDs for each Metal device
///
/// \see BuildMetalDevIDStrsToDevMap
std::vector<std::string> MetalDevIDStrs();

/// \brief Find the device whose ID string matches, ignoring case, a full ID
///        string or a substring of exactly one ID string.
///
/// An exact match of a full ID string takes precedence over substring matches.
/// Throws when no ID matches or when the substring matches more than one ID.
MetalIDStrDevMap::const_iterator
FindMetalDevByIDSubstr(const MetalIDStrDevMap& id_str_to_devs, const std::string& id_substr);

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


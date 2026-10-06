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

#if !__has_feature(objc_arc)
#error "xregMetalSys.mm must be compiled with ARC (-fobjc-arc)"
#endif

#include "xregMetalSys.h"

#include <cstring>
#include <sstream>

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "xregExceptionUtils.h"
#include "xregStringUtils.h"

struct xreg::MetalDevice::Impl
{
  id<MTLDevice> dev = nil;
};

struct xreg::MetalCmdQueue::Impl
{
  MetalDevice dev;

  id<MTLCommandQueue> queue = nil;
};

struct xreg::MetalBuffer::Impl
{
  MetalDevice dev;

  MTLResourceOptions opts = MTLResourceStorageModeShared;

  size_type num_bytes = 0;

  // nil when num_bytes is zero
  id<MTLBuffer> buf = nil;
};

namespace
{

// Commits a command buffer, waits for it to complete and throws when the
// GPU reports an error.
void CommitAndWait(id<MTLCommandBuffer> cmd_buf)
{
  [cmd_buf commit];
  [cmd_buf waitUntilCompleted];

  if (cmd_buf.status == MTLCommandBufferStatusError)
  {
    xregThrow("Metal command buffer failed: %s",
              cmd_buf.error ? cmd_buf.error.localizedDescription.UTF8String : "unknown error");
  }
}

id<MTLCommandQueue> GetQueue(const xreg::MetalCmdQueue& queue)
{
  if (!queue.valid())
  {
    xregThrow("invalid Metal command queue!");
  }

  return (__bridge id<MTLCommandQueue>) queue.native_handle();
}

void CheckBufRange(const xreg::MetalBuffer& buf, const xreg::size_type off,
                   const xreg::size_type num_bytes)
{
  if ((off > buf.num_bytes()) || (num_bytes > (buf.num_bytes() - off)))
  {
    xregThrow("Metal buffer range out of bounds: offset=%lu, len=%lu, buf size=%lu",
              static_cast<unsigned long>(off), static_cast<unsigned long>(num_bytes),
              static_cast<unsigned long>(buf.num_bytes()));
  }
}

std::vector<std::string> MetalDevIDStrsFromMap(const xreg::MetalIDStrDevMap& id_dev_map)
{
  std::vector<std::string> id_strs;
  id_strs.reserve(id_dev_map.size());

  for (const auto& id_dev_kv : id_dev_map)
  {
    id_strs.push_back(id_dev_kv.first);
  }

  return id_strs;
}

}  // un-named

std::string xreg::MetalGPUFamilyStr(const MetalGPUFamily fam)
{
  switch (fam)
  {
  case MetalGPUFamily::kMAC2:
    return "Mac 2";
  case MetalGPUFamily::kMETAL3:
    return "Metal 3";
  case MetalGPUFamily::kMETAL4:
    return "Metal 4";
  default:
    return "Unknown";
  }
}

std::string xreg::MetalFrameworkVersion()
{
  @autoreleasepool
  {
    NSString* ver = [NSBundle bundleWithIdentifier:@"com.apple.Metal"]
                      .infoDictionary[@"CFBundleShortVersionString"];

    return ver ? std::string(ver.UTF8String) : std::string();
  }
}

xreg::MetalDevice xreg::MetalDevice::Default()
{
  @autoreleasepool
  {
    id<MTLDevice> dev = MTLCreateSystemDefaultDevice();

    if (!dev)
    {
      xregThrow("no Metal device available!");
    }

    return FromNativeHandle((__bridge void*) dev);
  }
}

xreg::MetalDevice xreg::MetalDevice::FromNativeHandle(void* mtl_dev)
{
  MetalDevice d;
  d.impl_ = std::make_shared<Impl>();
  d.impl_->dev = (__bridge id<MTLDevice>) mtl_dev;

  return d;
}

bool xreg::MetalDevice::valid() const
{
  return impl_ && impl_->dev;
}

std::string xreg::MetalDevice::name() const
{
  @autoreleasepool
  {
    return valid() ? std::string(impl_->dev.name.UTF8String) : std::string();
  }
}

std::string xreg::MetalDevice::id_str() const
{
  std::string s;

  if (valid())
  {
    std::ostringstream oss;
    oss << StringRemoveAll(name()) << '-' << std::hex << registry_id();

    s = oss.str();
  }

  return s;
}

std::uint64_t xreg::MetalDevice::registry_id() const
{
  return valid() ? impl_->dev.registryID : 0;
}

bool xreg::MetalDevice::has_unified_memory() const
{
  return valid() && impl_->dev.hasUnifiedMemory;
}

bool xreg::MetalDevice::is_low_power() const
{
  return valid() && impl_->dev.isLowPower;
}

bool xreg::MetalDevice::is_headless() const
{
  return valid() && impl_->dev.isHeadless;
}

bool xreg::MetalDevice::is_removable() const
{
  return valid() && impl_->dev.isRemovable;
}

xreg::MetalDeviceLocation xreg::MetalDevice::location() const
{
  MetalDeviceLocation loc = MetalDeviceLocation::kUNSPECIFIED;

  if (valid())
  {
    switch (impl_->dev.location)
    {
    case MTLDeviceLocationBuiltIn:
      loc = MetalDeviceLocation::kBUILT_IN;
      break;
    case MTLDeviceLocationSlot:
      loc = MetalDeviceLocation::kSLOT;
      break;
    case MTLDeviceLocationExternal:
      loc = MetalDeviceLocation::kEXTERNAL;
      break;
    default:
      loc = MetalDeviceLocation::kUNSPECIFIED;
      break;
    }
  }

  return loc;
}

xreg::MetalGPUFamily xreg::MetalDevice::highest_gpu_family() const
{
  MetalGPUFamily fam = MetalGPUFamily::kUNKNOWN;

  if (valid())
  {
    id<MTLDevice> dev = impl_->dev;

#if defined(__MAC_26_0)
    if (@available(macOS 26.0, *))
    {
      if ([dev supportsFamily:MTLGPUFamilyMetal4])
      {
        return MetalGPUFamily::kMETAL4;
      }
    }
#endif

    if (@available(macOS 13.0, *))
    {
      if ([dev supportsFamily:MTLGPUFamilyMetal3])
      {
        return MetalGPUFamily::kMETAL3;
      }
    }

    if ([dev supportsFamily:MTLGPUFamilyMac2])
    {
      fam = MetalGPUFamily::kMAC2;
    }
  }

  return fam;
}

xreg::size_type xreg::MetalDevice::recommended_max_working_set_size() const
{
  return valid() ? impl_->dev.recommendedMaxWorkingSetSize : 0;
}

xreg::size_type xreg::MetalDevice::max_buffer_length() const
{
  return valid() ? impl_->dev.maxBufferLength : 0;
}

bool xreg::MetalDevice::supports_32bit_float_filtering() const
{
  bool supported = false;

  if (valid())
  {
    if (@available(macOS 11.0, *))
    {
      supported = impl_->dev.supports32BitFloatFiltering;
    }
    else
    {
      // prior to this query being available, only Intel and AMD GPUs were
      // supported by MacOS, which support filtering of 32-bit floats
      supported = true;
    }
  }

  return supported;
}

void* xreg::MetalDevice::native_handle() const
{
  return valid() ? (__bridge void*) impl_->dev : nullptr;
}

std::vector<xreg::MetalDevice> xreg::MetalAllDevices()
{
  std::vector<MetalDevice> devs;

  @autoreleasepool
  {
    NSArray<id<MTLDevice>>* mtl_devs = MTLCopyAllDevices();

    devs.reserve(mtl_devs.count);

    for (id<MTLDevice> d in mtl_devs)
    {
      devs.push_back(MetalDevice::FromNativeHandle((__bridge void*) d));
    }
  }

  return devs;
}

xreg::MetalIDStrDevMap xreg::BuildMetalDevIDStrsToDevMap()
{
  MetalIDStrDevMap id_str_to_devs;

  for (const auto& d : MetalAllDevices())
  {
    id_str_to_devs.emplace(d.id_str(), d);
  }

  return id_str_to_devs;
}

std::vector<std::string> xreg::MetalDevIDStrs()
{
  return MetalDevIDStrsFromMap(BuildMetalDevIDStrsToDevMap());
}

xreg::MetalIDStrDevMap::const_iterator
xreg::FindMetalDevByIDSubstr(const MetalIDStrDevMap& id_str_to_devs, const std::string& id_substr)
{
  const std::string id_substr_lower = ToLowerCase(id_substr);

  std::vector<MetalIDStrDevMap::const_iterator> matches;

  for (auto it = id_str_to_devs.begin(); it != id_str_to_devs.end(); ++it)
  {
    const std::string cur_id_lower = ToLowerCase(it->first);

    if (cur_id_lower == id_substr_lower)
    {
      // an exact match takes precedence
      return it;
    }
    else if (cur_id_lower.find(id_substr_lower) != std::string::npos)
    {
      matches.push_back(it);
    }
  }

  if (matches.empty())
  {
    const auto all_ids = MetalDevIDStrsFromMap(id_str_to_devs);

    xregThrow("no Metal device ID matches \"%s\"; available IDs: %s", id_substr.c_str(),
              all_ids.empty() ? "<none>" : JoinTokens(all_ids.begin(), all_ids.end(), ", ").c_str());
  }
  else if (matches.size() > 1)
  {
    std::vector<std::string> match_ids;

    for (const auto& it : matches)
    {
      match_ids.push_back(it->first);
    }

    xregThrow("Metal device ID \"%s\" is ambiguous, it matches: %s", id_substr.c_str(),
              JoinTokens(match_ids.begin(), match_ids.end(), ", ").c_str());
  }

  return matches.front();
}

xreg::MetalCmdQueue::MetalCmdQueue(const MetalDevice& dev)
  : impl_(std::make_shared<Impl>())
{
  if (!dev.valid())
  {
    xregThrow("cannot create a Metal command queue with an invalid device!");
  }

  impl_->dev = dev;

  @autoreleasepool
  {
    impl_->queue = [(__bridge id<MTLDevice>) dev.native_handle() newCommandQueue];
  }

  if (!impl_->queue)
  {
    xregThrow("failed to create Metal command queue!");
  }
}

bool xreg::MetalCmdQueue::valid() const
{
  return impl_ && impl_->queue;
}

const xreg::MetalDevice& xreg::MetalCmdQueue::device() const
{
  if (!impl_)
  {
    xregThrow("invalid Metal command queue has no device!");
  }

  return impl_->dev;
}

void xreg::MetalCmdQueue::finish()
{
  @autoreleasepool
  {
    // command buffers on a queue are executed in order, so once this empty
    // command buffer completes all previously submitted work has completed
    CommitAndWait([GetQueue(*this) commandBuffer]);
  }
}

void* xreg::MetalCmdQueue::native_handle() const
{
  return valid() ? (__bridge void*) impl_->queue : nullptr;
}

xreg::MetalBuffer::MetalBuffer()
  : impl_(std::make_unique<Impl>())
{ }

xreg::MetalBuffer::MetalBuffer(const MetalDevice& dev, const size_type num_bytes,
                               const MetalStorageMode mode)
  : impl_(std::make_unique<Impl>())
{
  if (!dev.valid())
  {
    xregThrow("cannot create a Metal buffer with an invalid device!");
  }

  impl_->dev = dev;

  bool use_managed = false;

  switch (mode)
  {
  case MetalStorageMode::kAUTO:
    use_managed = !dev.has_unified_memory();
    break;
  case MetalStorageMode::kSHARED:
    use_managed = false;
    break;
  case MetalStorageMode::kMANAGED:
    use_managed = true;
    break;
  }

  impl_->opts = use_managed ? MTLResourceStorageModeManaged : MTLResourceStorageModeShared;

  resize_bytes(num_bytes);
}

xreg::MetalBuffer::MetalBuffer(MetalBuffer&&) = default;

xreg::MetalBuffer& xreg::MetalBuffer::operator=(MetalBuffer&&) = default;

xreg::MetalBuffer::~MetalBuffer() = default;

xreg::size_type xreg::MetalBuffer::num_bytes() const
{
  return impl_ ? impl_->num_bytes : 0;
}

void xreg::MetalBuffer::resize_bytes(const size_type num_bytes)
{
  if (!impl_ || !impl_->dev.valid())
  {
    xregThrow("cannot resize a Metal buffer that has no device!");
  }

  if (num_bytes != impl_->num_bytes)
  {
    // release the old allocation first to reduce peak memory usage
    impl_->buf       = nil;
    impl_->num_bytes = 0;

    if (num_bytes)
    {
      @autoreleasepool
      {
        impl_->buf = [(__bridge id<MTLDevice>) impl_->dev.native_handle()
                          newBufferWithLength:num_bytes options:impl_->opts];
      }

      if (!impl_->buf)
      {
        xregThrow("failed to allocate Metal buffer of %lu bytes!",
                  static_cast<unsigned long>(num_bytes));
      }

      impl_->num_bytes = num_bytes;
    }
  }
}

bool xreg::MetalBuffer::is_managed() const
{
  return impl_ && (impl_->opts == MTLResourceStorageModeManaged);
}

const xreg::MetalDevice& xreg::MetalBuffer::device() const
{
  if (!impl_)
  {
    xregThrow("moved-from Metal buffer has no device!");
  }

  return impl_->dev;
}

void* xreg::MetalBuffer::contents() const
{
  return (impl_ && impl_->buf) ? impl_->buf.contents : nullptr;
}

void* xreg::MetalBuffer::native_handle() const
{
  return (impl_ && impl_->buf) ? (__bridge void*) impl_->buf : nullptr;
}

void xreg::CopyHostToMetal(const void* src, const size_type num_bytes,
                           MetalBuffer& dst, const size_type dst_off_bytes,
                           MetalCmdQueue& queue)
{
  CheckBufRange(dst, dst_off_bytes, num_bytes);

  if (num_bytes)
  {
    // do not overwrite data that the GPU may still be using
    queue.finish();

    std::memcpy(static_cast<unsigned char*>(dst.contents()) + dst_off_bytes, src, num_bytes);

    if (dst.is_managed())
    {
      // notify Metal that the CPU copy has been modified so it is uploaded
      // to the GPU before the next use
      [(__bridge id<MTLBuffer>) dst.native_handle()
          didModifyRange:NSMakeRange(dst_off_bytes, num_bytes)];
    }
  }
}

void xreg::CopyMetalToHost(const MetalBuffer& src, const size_type src_off_bytes,
                           const size_type num_bytes, void* dst,
                           MetalCmdQueue& queue)
{
  CheckBufRange(src, src_off_bytes, num_bytes);

  if (num_bytes)
  {
    if (src.is_managed())
    {
      @autoreleasepool
      {
        // the CPU copy of a managed buffer must be explicitly updated with
        // any GPU modifications. This is executed after all previously
        // submitted work on the queue.
        id<MTLCommandBuffer> cmd_buf = [GetQueue(queue) commandBuffer];

        id<MTLBlitCommandEncoder> blit = [cmd_buf blitCommandEncoder];
        [blit synchronizeResource:(__bridge id<MTLBuffer>) src.native_handle()];
        [blit endEncoding];

        CommitAndWait(cmd_buf);
      }
    }
    else
    {
      queue.finish();
    }

    std::memcpy(dst, static_cast<const unsigned char*>(src.contents()) + src_off_bytes, num_bytes);
  }
}


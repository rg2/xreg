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

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "xregExceptionUtils.h"

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

}  // un-named

xreg::MetalDevice xreg::MetalDevice::Default()
{
  @autoreleasepool
  {
    id<MTLDevice> dev = MTLCreateSystemDefaultDevice();

    if (!dev)
    {
      xregThrow("no Metal device available!");
    }

    MetalDevice d;
    d.impl_ = std::make_shared<Impl>();
    d.impl_->dev = dev;

    return d;
  }
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

bool xreg::MetalDevice::has_unified_memory() const
{
  return valid() && impl_->dev.hasUnifiedMemory;
}

void* xreg::MetalDevice::native_handle() const
{
  return valid() ? (__bridge void*) impl_->dev : nullptr;
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


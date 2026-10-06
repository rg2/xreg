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
#error "xregMetalTexture.mm must be compiled with ARC (-fobjc-arc)"
#endif

#include "xregMetalTexture.h"

#include <algorithm>

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "xregExceptionUtils.h"

struct xreg::MetalTexture3D::Impl
{
  MetalDevice dev;

  id<MTLTexture> tex = nil;
};

xreg::MetalTexture3D::MetalTexture3D(const MetalDevice& dev, const size_type width,
                                     const size_type height, const size_type depth)
  : impl_(std::make_shared<Impl>())
{
  if (!dev.valid())
  {
    xregThrow("cannot create a Metal texture with an invalid device!");
  }

  if (!width || !height || !depth ||
      (width > kMAX_DIM) || (height > kMAX_DIM) || (depth > kMAX_DIM))
  {
    xregThrow("invalid Metal 3D texture size: %lu x %lu x %lu (each dimension must be in [1, %lu])",
              static_cast<unsigned long>(width), static_cast<unsigned long>(height),
              static_cast<unsigned long>(depth), static_cast<unsigned long>(kMAX_DIM));
  }

  impl_->dev = dev;

  @autoreleasepool
  {
    MTLTextureDescriptor* desc = [MTLTextureDescriptor new];

    desc.textureType = MTLTextureType3D;
    desc.pixelFormat = MTLPixelFormatR32Float;
    desc.width       = width;
    desc.height      = height;
    desc.depth       = depth;
    desc.usage       = MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModePrivate;

    impl_->tex = [(__bridge id<MTLDevice>) dev.native_handle() newTextureWithDescriptor:desc];
  }

  if (!impl_->tex)
  {
    xregThrow("failed to allocate Metal 3D texture of size %lu x %lu x %lu!",
              static_cast<unsigned long>(width), static_cast<unsigned long>(height),
              static_cast<unsigned long>(depth));
  }
}

void xreg::MetalTexture3D::upload(const float* src, MetalCmdQueue& queue)
{
  if (!valid())
  {
    xregThrow("cannot upload to an invalid Metal texture!");
  }

  const size_type w = width();
  const size_type h = height();
  const size_type d = depth();

  const size_type num_bytes_per_row   = w * sizeof(float);
  const size_type num_bytes_per_slice = h * num_bytes_per_row;

  // Copy several slices at a time to limit the size of the staging buffer
  constexpr size_type kMAX_STAGING_NUM_BYTES = 64 * 1024 * 1024;

  const size_type num_slices_per_copy = std::max<size_type>(1, kMAX_STAGING_NUM_BYTES / num_bytes_per_slice);

  id<MTLDevice> mtl_dev = (__bridge id<MTLDevice>) impl_->dev.native_handle();
  id<MTLCommandQueue> mtl_queue = (__bridge id<MTLCommandQueue>) queue.native_handle();

  for (size_type start_slice = 0; start_slice < d; start_slice += num_slices_per_copy)
  {
    const size_type num_slices = std::min(num_slices_per_copy, d - start_slice);

    @autoreleasepool
    {
      id<MTLBuffer> staging = [mtl_dev newBufferWithBytes:(src + (start_slice * w * h))
                                                   length:(num_slices * num_bytes_per_slice)
                                                  options:MTLResourceStorageModeShared];
      if (!staging)
      {
        xregThrow("failed to allocate Metal staging buffer for texture upload!");
      }

      id<MTLCommandBuffer> cmd_buf = [mtl_queue commandBuffer];

      id<MTLBlitCommandEncoder> blit = [cmd_buf blitCommandEncoder];

      [blit copyFromBuffer:staging
              sourceOffset:0
         sourceBytesPerRow:num_bytes_per_row
       sourceBytesPerImage:num_bytes_per_slice
                sourceSize:MTLSizeMake(w, h, num_slices)
                 toTexture:impl_->tex
          destinationSlice:0
          destinationLevel:0
         destinationOrigin:MTLOriginMake(0, 0, start_slice)];

      [blit endEncoding];

      [cmd_buf commit];
      [cmd_buf waitUntilCompleted];

      if (cmd_buf.status == MTLCommandBufferStatusError)
      {
        xregThrow("Metal texture upload failed: %s",
                  cmd_buf.error ? cmd_buf.error.localizedDescription.UTF8String : "unknown error");
      }
    }
  }
}

bool xreg::MetalTexture3D::valid() const
{
  return impl_ && impl_->tex;
}

xreg::size_type xreg::MetalTexture3D::width() const
{
  return valid() ? impl_->tex.width : 0;
}

xreg::size_type xreg::MetalTexture3D::height() const
{
  return valid() ? impl_->tex.height : 0;
}

xreg::size_type xreg::MetalTexture3D::depth() const
{
  return valid() ? impl_->tex.depth : 0;
}

void* xreg::MetalTexture3D::native_handle() const
{
  return valid() ? (__bridge void*) impl_->tex : nullptr;
}


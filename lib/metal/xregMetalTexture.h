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

#ifndef XREGMETALTEXTURE_H_
#define XREGMETALTEXTURE_H_

// Pure C++ interface for Metal textures. Objective-C++ code may access the
// underlying Metal objects through native_handle().

#include <memory>

#include "xregMetalSys.h"

namespace xreg
{

/// \brief A single channel, 32-bit float, 3D texture on a Metal device.
///
/// This is typically used to store a volume, which may be sampled in shaders
/// with interpolation (e.g. xreg::LinearVolumeSampler in xregMetalInterp.metal).
///
/// The texture uses private (GPU only) storage, which allows the device to use
/// a layout optimized for sampling, and is copied from host memory through a
/// staging buffer.
///
/// Copies of this object refer to the same underlying texture.
class MetalTexture3D
{
public:
  MetalTexture3D() = default;

  /// \brief Allocate a texture, the contents are undefined until upload() is called.
  ///
  /// Throws if a dimension exceeds the device limit for 3D textures (2048).
  MetalTexture3D(const MetalDevice& dev, const size_type width, const size_type height,
                 const size_type depth);

  /// \brief Copy the entire texture from host memory.
  ///
  /// The host buffer should have width * height * depth elements, with the
  /// width dimension changing fastest (e.g. the layout of an itk::Image).
  /// The copy is performed with the queue, through a staging buffer, and has
  /// completed when this function returns. Work previously submitted to the
  /// queue that uses this texture is completed prior to the copy.
  void upload(const float* src, MetalCmdQueue& queue);

  bool valid() const;

  size_type width() const;
  size_type height() const;
  size_type depth() const;

  /// \brief The maximum size of each 3D texture dimension supported by Metal devices.
  static constexpr size_type kMAX_DIM = 2048;

  /// \brief The underlying id<MTLTexture>, ownership is NOT transferred.
  void* native_handle() const;

private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
};

}  // xreg

#endif


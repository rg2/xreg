/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
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

#ifndef XREGIMGSIMMETRIC2DCPU_H_
#define XREGIMGSIMMETRIC2DCPU_H_

#include "xregImgSimMetric2D.h"

namespace xreg
{

// Forward Declarations
class RayCastSyncHostBuf;

/// \brief Base class for CPU based 2D/3D similarity metrics
class ImgSimMetric2DCPU : public ImgSimMetric2D
{
public:
  ImgSimMetric2DCPU() = default;
  
  void allocate_resources();

  /// \brief Sets the moving images buffer to use from a ray caster.
  ///
  /// This should always be called after calling allocate_resources() on the
  /// ray caster object and before calling allocate_resources() on the
  /// similarity object.
  void set_mov_imgs_buf_from_ray_caster(RayCaster* ray_caster,
                                        const size_type proj_offset = 0);

  /// \brief Sets the moving images buffer to use.
  void set_mov_imgs_host_buf(Scalar* mov_imgs_buf, const size_type proj_offset = 0);

protected:
  void pre_compute();

  RayCastSyncHostBuf* sync_host_buf_ = nullptr;

  Scalar* mov_imgs_buf_ = nullptr;

  size_type proj_off_ = 0;
};

}  // xreg

#endif


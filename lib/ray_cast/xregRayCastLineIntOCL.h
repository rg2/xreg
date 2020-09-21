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

#ifndef XREGRAYCASTLINEINTOCL_H_
#define XREGRAYCASTLINEINTOCL_H_

#include "xregRayCastBaseOCL.h"

namespace xreg
{

/// \brief Ray casting line integral using OpenCL.
///
/// All computations on the OpenCL device are with single precision floating point.
class RayCasterLineIntOCL : public RayCasterOCL, public RayCastLineIntParamInterface
{
public:
  /// \brief Default constructor, chooses a default device, creates a new
  ///        context and command queue.
  RayCasterLineIntOCL() = default;

  /// \brief Constructor specifying a device to use, but creates a new context
  ///        and command queue.
  explicit RayCasterLineIntOCL(const boost::compute::device& dev);

  /// \brief Constructor specifying a specific context and command queue to use
  RayCasterLineIntOCL(const boost::compute::context& ctx, const boost::compute::command_queue& queue);

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// The memory buffers are allocated by the parent class, this class only
  /// needs to create the OpenCL kernel.
  void allocate_resources() override;

  /// \brief Perform each ray cast.
  ///
  /// TODO: info
  void compute(const size_type vol_idx = 0) override;

private:
  boost::compute::kernel dev_kernel_;
};

}  // xreg

#endif


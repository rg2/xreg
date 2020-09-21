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

#ifndef XREGRAYCASTDEPTHOCL_H_
#define XREGRAYCASTDEPTHOCL_H_

#include "xregRayCastBaseOCL.h"

namespace xreg
{

class RayCasterDepthOCL : public RayCasterOCL, public RayCasterCollisionParamInterface
{
public:
  /// \brief Default constructor, chooses a default device, creates a new
  ///        context and command queue.
  RayCasterDepthOCL();

  /// \brief Constructor specifying a device to use, but creates a new context
  ///        and command queue.
  explicit RayCasterDepthOCL(const boost::compute::device& dev);

  /// \brief Constructor specifying a specific context and command queue to use
  RayCasterDepthOCL(const boost::compute::context& ctx,
                    const boost::compute::command_queue& queue);

  void compute(const size_type vol_idx = 0) override;
  
  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// The memory buffers are allocated by the parent class, this class only
  /// needs to create the device kernel.
  void allocate_resources() override;

private:
  boost::compute::kernel dev_kernel_;
};

}  // xreg

#endif


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

#ifndef XREGIMGSIMMETRIC2DSSDOCL_H_
#define XREGIMGSIMMETRIC2DSSDOCL_H_

#include "xregImgSimMetric2DOCL.h"

namespace xreg
{

class ImgSimMetric2DSSDOCL : public ImgSimMetric2DOCL
{
public:
  /// \brief Default constructor, chooses a default device, creates a new
  ///        context and command queue.
  ImgSimMetric2DSSDOCL() = default;

  /// \brief Constructor specifying a device to use, but creates a new context
  ///        and command queue.
  explicit ImgSimMetric2DSSDOCL(const boost::compute::device& dev);

  /// \brief Constructor specifying a specific context and command queue to use
  ImgSimMetric2DSSDOCL(const boost::compute::context& ctx,
                       const boost::compute::command_queue& queue);

  void allocate_resources() override;

  void compute() override;

protected:
  void process_mask() override;

private:
  std::unique_ptr<DevBuf> ssds_dev_;
  std::unique_ptr<DevBuf> one_over_n_dev_;

  boost::compute::kernel div_elems_krnl_;
  boost::compute::kernel sq_dist_krnl_;
};

}  // xreg

#endif


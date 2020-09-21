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

#ifndef XREGIMGSIMMETRIC2DGRADIMGOCL_H_
#define XREGIMGSIMMETRIC2DGRADIMGOCL_H_

#include <array>

#include "xregImgSimMetric2DOCL.h"
#include "xregImgSimMetric2DGradImgParamInterface.h"

namespace xreg
{

class ImgSimMetric2DGradImgOCL
  : public ImgSimMetric2DOCL,
    public ImgSimMetric2DGradImgParamInterface
{
public:
  ImgSimMetric2DGradImgOCL() = default;

  explicit ImgSimMetric2DGradImgOCL(const boost::compute::device& dev);

  ImgSimMetric2DGradImgOCL(const boost::compute::context& ctx,
                           const boost::compute::command_queue& queue);
  
  void allocate_resources() override;

  size_type smooth_img_before_sobel_kernel_radius() const override;

  void set_smooth_img_before_sobel_kernel_radius(const size_type r) override;

protected:
  
  void compute_sobel_grads();
  
  std::shared_ptr<DevBuf> fixed_grad_x_dev_buf_;
  std::shared_ptr<DevBuf> fixed_grad_y_dev_buf_;

  std::shared_ptr<DevBuf> mov_grad_x_dev_buf_;
  std::shared_ptr<DevBuf> mov_grad_y_dev_buf_;

  boost::compute::kernel sobel_krnl_;
  boost::compute::kernel smooth_krnl_;

private:

  std::array<std::size_t,3> sobel_ocl_kernel_global_size_;
  std::array<std::size_t,3> smooth_ocl_kernel_global_size_;

  std::unique_ptr<DevBuf> mov_smooth_dev_buf_;
  std::unique_ptr<DevBuf> smooth_kernel_dev_buf_;

  size_type smooth_img_kernel_rad_ = 5;
};

}  // xreg

#endif


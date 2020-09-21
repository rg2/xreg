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

#ifndef XREGIMGSIMMETRIC2DPATCHNCCOCL_H_
#define XREGIMGSIMMETRIC2DPATCHNCCOCL_H_

#include "xregImgSimMetric2DOCL.h"
#include "xregImgSimMetric2DPatchCommon.h"

namespace xreg
{

class ImgSimMetric2DPatchNCCOCL
  : public ImgSimMetric2DOCL,
    public ImgSimMetric2DPatchCommon
{
public:
  using Scalar = ImgSimMetric2DOCL::Scalar;

  /// \brief Default constructor, chooses a default device, creates a new
  ///        context and command queue.
  ImgSimMetric2DPatchNCCOCL() = default;
  
  /// \brief Constructor specifying a device to use, but creates a new context
  ///        and command queue.
  explicit ImgSimMetric2DPatchNCCOCL(const boost::compute::device& dev);

  /// \brief Constructor specifying a specific context and command queue to use
  ImgSimMetric2DPatchNCCOCL(const boost::compute::context& ctx, const boost::compute::command_queue& queue);

  void allocate_resources() override;

  void compute() override;

protected:
  void process_mask() override;

private:

  size_type img_num_rows_ = 0;
  size_type img_num_cols_ = 0;

  size_type fixed_img_proc_patches_max_len_ = 0;

  using DevBufUInt4  = boost::compute::vector<boost::compute::uint4_>;
  using DevBufFloat2 = boost::compute::vector<boost::compute::float2_>;
  using DevBufULong  = boost::compute::vector<boost::compute::ulong_>;

  std::unique_ptr<DevBufUInt4> patch_start_stops_dev_;
  std::unique_ptr<DevBufFloat2> fixed_img_patch_stats_dev_;
  std::unique_ptr<DevBuf> proc_fixed_img_patches_dev_;

  std::vector<boost::compute::ulong_> patch_inds_to_use_host_;
  std::unique_ptr<DevBufULong> patch_inds_to_use_dev_;

  std::vector<Scalar> wgts_to_use_host_;
  std::unique_ptr<DevBuf> wgts_to_use_dev_;

  std::unique_ptr<DevBuf> patch_nccs_dev_;

  std::unique_ptr<DevBuf> sim_vals_dev_;

  boost::compute::kernel fixed_img_proc_patches_krnl_;
  boost::compute::kernel fixed_img_stats_krnl_;
  boost::compute::kernel proc_mov_img_patches_krnl_;

  bool fixed_img_stats_proc_done_ = false;
};

}  // xreg

#endif


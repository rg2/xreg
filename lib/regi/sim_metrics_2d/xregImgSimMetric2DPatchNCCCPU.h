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

#ifndef XREGIMGSIMMETRIC2DPATCHNCCCPU_H_
#define XREGIMGSIMMETRIC2DPATCHNCCCPU_H_

#include "xregImgSimMetric2DCPU.h"
#include "xregImgSimMetric2DPatchCommon.h"
#include "xregHDF5ReadWriteInterface.h"

namespace xreg
{

class ImgSimMetric2DPatchNCCCPU : public ImgSimMetric2DCPU, public ImgSimMetric2DPatchCommon
{
public:
  // Need to redefine these as both parent classes have this alias
  // (they should be the same, but the ambiguity must be resolved)
  using Scalar     = ImgSimMetric2DCPU::Scalar;
  using MaskScalar = ImgSimMetric2DCPU::MaskScalar;

  /// \brief Constructor - trivial, no work performed.
  ImgSimMetric2DPatchNCCCPU() = default;

  /// \brief Allocation of other resources required and computation of
  ///        fixed image statistics.
  ///
  /// This will also allocate additional resources for patch computations.
  void allocate_resources() override;

  /// \brief Computation of the similarity metric
  void compute() override;
  
  std::shared_ptr<H5ReadWriteInterface> aux_info() override;
  
  struct SimAux : public H5ReadWriteInterface
  {
    std::vector<PatchInfoList>  patch_infos_per_compute_call;
    std::vector<PatchIndexList> patch_indices_per_compute_call;

    void write(H5::Group* h5) override;
  
    void read(const H5::Group& h5) override;
  };

  bool use_fixed_img_patch_variances_as_wgts() const;

  void set_use_fixed_img_patch_variances_as_wgts(const bool use_vars_as_wgts);

  bool use_mov_img_patch_variances_as_wgts() const;

  void set_use_mov_img_patch_variances_as_wgts(const bool use_vars_as_wgts);

  void set_other_mov_img_patch_vars(const std::vector<ScalarList>* other_vars);

protected:

  void process_mask() override;

private:

  size_type img_num_rows_ = 0;
  size_type img_num_cols_ = 0;

  std::vector<Scalar> fixed_scaled_buf_;

  std::vector<cv::Mat> fixed_scaled_patches_;

  ScalarList cur_mov_img_patch_ncc_vals_;

  bool use_fixed_img_patch_variances_as_wgts_ = false;

  bool use_mov_img_patch_variances_as_wgts_ = false;
  
  const std::vector<ScalarList>* other_mov_img_patch_vars_ = nullptr;

  std::shared_ptr<SimAux> sim_aux_;
    
  bool init_fixed_img_stats_computed_ = false;
    
  cv::Mat fixed_ocv_img_;
};

namespace detail
{

std::tuple<ImgSimMetric2DPatchNCCCPU::Scalar,
           ImgSimMetric2DPatchNCCCPU::Scalar,
           size_type>
ComputePatchMeanStdDev(const cv::Mat& p, const cv::Mat* m,
                       const bool use_mask_for_stats);

}  // detail

}  // xreg

#endif


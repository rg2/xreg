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

#ifndef XREGIMGSIMMETRIC2DPATCHGRADNCCCPU_H_
#define XREGIMGSIMMETRIC2DPATCHGRADNCCCPU_H_

#include "xregImgSimMetric2DGradImgCPU.h"
#include "xregImgSimMetric2DPatchNCCCPU.h"

namespace xreg
{

class ImgSimMetric2DPatchGradNCCCPU
  : public ImgSimMetric2DGradImgCPU, public ImgSimMetric2DPatchCommon
{
public:
  // Need to redefine these as both parent classes have this alias
  // (they should be the same, but the ambiguity must be resolved)
  using Scalar     = ImgSimMetric2DGradImgCPU::Scalar;
  using MaskScalar = ImgSimMetric2DGradImgCPU::MaskScalar;
  //using ScalarList = ImgSimMetric2DGradImgCPU::ScalarList;

  /// \brief Constructor - trivial, no work performed.
  ImgSimMetric2DPatchGradNCCCPU() = default;

  /// \brief Allocation of other resources required and computation of
  ///        fixed image gradients.
  ///
  /// \see GradImageSimMetric2DCPU for a description of gradient image
  /// reoursces allocated.
  /// This will also allocate additional resources for patch computations.
  void allocate_resources() override;

  /// \brief Computation of the similarity metric
  void compute() override;

  const ImgSimMetric2DPatchNCCCPU& patch_ncc_x() const;

  const ImgSimMetric2DPatchNCCCPU& patch_ncc_y() const;

  bool enforce_same_patches_in_both_x_and_y() const;

  void set_enforce_same_patches_in_both_x_and_y(const bool same_patches);
  
  bool use_fixed_img_patch_variances_as_wgts() const;

  void set_use_fixed_img_patch_variances_as_wgts(const bool use_vars_as_wgts);

  bool use_variances_in_grad_imgs_as_wgts() const;

  void set_use_variances_in_grad_imgs_as_wgts(const bool use_grad_vars_as_wgts);
  
  bool use_mov_img_patch_variances_as_wgts() const;

  void set_use_mov_img_patch_variances_as_wgts(const bool use_vars_as_wgts);

  std::shared_ptr<H5ReadWriteInterface> aux_info() override;
  
  struct SimAux : public H5ReadWriteInterface
  {
    using PatchNCCSimAux = ImgSimMetric2DPatchNCCCPU::SimAux;
    
    std::shared_ptr<H5ReadWriteInterface> sim_aux_x;
    std::shared_ptr<H5ReadWriteInterface> sim_aux_y;
    
    void write(H5::Group* h5) override;
    
    void read(const H5::Group& h5) override;
  };

protected:

  void process_mask() override;

private:
  ImgSimMetric2DPatchNCCCPU patch_ncc_x_;
  ImgSimMetric2DPatchNCCCPU patch_ncc_y_;

  bool enforce_same_patches_in_both_x_and_y_ = true;
  
  bool use_fixed_img_patch_variances_as_wgts_ = false;
  bool use_mov_img_patch_variances_as_wgts_   = false;
  bool use_variances_in_grad_imgs_as_wgts_    = false;

  std::shared_ptr<SimAux> sim_aux_;
      
  std::vector<ScalarList> mov_img_patch_vars_;
  std::vector<bool> do_not_use_scores_from_sub_objs_;
};

}  // xreg

#endif


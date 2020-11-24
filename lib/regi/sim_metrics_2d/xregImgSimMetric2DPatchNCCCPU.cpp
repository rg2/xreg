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

#include "xregImgSimMetric2DPatchNCCCPU.h"

#include "xregITKBasicImageUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"
#include "xregAssert.h"
#include "xregTBBUtils.h"
#include "xregHDF5Internal.h"

void xreg::ImgSimMetric2DPatchNCCCPU::allocate_resources()
{
  ImgSimMetric2DCPU::allocate_resources();

  xregASSERT(this->patch_radius_ > 0);

  // cannot use both moving and fixed image patch variances as weights
  xregASSERT((int(use_mov_img_patch_variances_as_wgts_) +
              int(use_fixed_img_patch_variances_as_wgts_)) < 2);
  
  auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  img_num_rows_ = itk_size[1];
  img_num_cols_ = itk_size[0];

  fixed_ocv_img_ = ShallowCopyItkToOpenCV(this->fixed_img_.GetPointer());
  
  const bool use_mask = this->mask_;

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }
  this->setup_patches(img_num_rows_, img_num_cols_,
                      use_mask ? &ocv_mask : nullptr,
                      this->num_mov_imgs_);
  
  const size_type num_patches = this->patch_infos_.size();
  
  cur_mov_img_patch_ncc_vals_.resize(num_patches);

  fixed_scaled_patches_ = AllocContiguousBufferForOpenCVImages<Scalar>(this->patch_diam_,
                                                                       this->patch_diam_,
                                                                       num_patches,
                                                                       &fixed_scaled_buf_);

  this->process_updated_mask();
}
  
void xreg::ImgSimMetric2DPatchNCCCPU::compute()
{
  this->pre_compute();

  const size_type img_num_pix = img_num_cols_ * img_num_rows_;
 
  // This will either be the total number of patches or the number of
  // randomly sampled patches 
  const size_type num_patches = this->num_patches();
    
  const bool use_mask = this->mask_;

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }
    
  // This uses some state to determine if the weights actually need to be recomputed
  this->compute_weights(this->mask_ ? &ocv_mask : nullptr);

  // NOTE: this can be moved into the loop below over moving images to
  //       sample different random patches for each moving image
  if (!this->do_not_update_patch_inds_to_use_)
  { 
    this->patch_inds_to_use_ = this->patch_indices_to_use();
  }
  xregASSERT(num_patches == this->patch_inds_to_use_.size());
   
  ScalarList mov_img_patch_vars;

  for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
  {
    cv::Mat mov_img(img_num_rows_, img_num_cols_, cv::DataType<Scalar>::type,
                    this->mov_imgs_buf_ + (mov_idx * img_num_pix));
  
    cur_mov_img_patch_ncc_vals_.assign(num_patches, 0);
      
    if (use_mov_img_patch_variances_as_wgts_)
    {
      if (other_mov_img_patch_vars_)
      {
        mov_img_patch_vars = other_mov_img_patch_vars_->at(mov_idx);
        xregASSERT(mov_img_patch_vars.size() == num_patches);
      }
      else
      {
        mov_img_patch_vars.resize(num_patches);
        
        auto mov_patch_vars_fn = [&] (const RangeType& r)
        {
          Scalar tmp_std_dev = 0;

          for (size_type local_patch_idx = r.begin(); local_patch_idx < r.end(); ++local_patch_idx)
          {
            const auto& patch_info = this->patch_infos_[this->patch_inds_to_use_[local_patch_idx]];
            
            const auto patch_row_col = patch_info.center_row_col();
            
            if (!use_mask || ocv_mask.at<MaskScalar>(patch_row_col[0], patch_row_col[1]))
            {
              cv::Mat mov_roi = mov_img(patch_info.ocv_roi());
              
              std::tie(std::ignore, tmp_std_dev, std::ignore) =
                                              detail::ComputePatchMeanStdDev(mov_roi, nullptr, false);
            
              mov_img_patch_vars[local_patch_idx] = tmp_std_dev * tmp_std_dev;
            }
            else
            {
              mov_img_patch_vars[local_patch_idx] = 0;
            }
          }
        };

        ParallelFor(mov_patch_vars_fn, RangeType(0, num_patches));
          
        if (this->normalize_weights_as_prob_)
        {
          Scalar var_sum = 0;

          for (const auto& v : mov_img_patch_vars)
          {
            var_sum += v;
          }

          if (var_sum > 1.0e-6)
          {
            for (auto& v : mov_img_patch_vars)
            {
              v /= var_sum;
            }
          }
          else
          {
            // Assign very large cost values if we have zero variance over all of the patches
            this->sim_vals_[mov_idx] = std::numeric_limits<Scalar>::max();
                
            if (this->save_all_per_patch_scores_)
            {
              for (size_type local_patch_idx = 0; local_patch_idx < num_patches; ++local_patch_idx)
              {    
                this->sim_vals_for_each_patch_[local_patch_idx][mov_idx] = 
                                                    std::numeric_limits<Scalar>::max();
              }
            }

            // do not compute NCC below - we've already set the similarity score to MAX
            continue;
          }
        }
      }
    }
      
    // use a patch as a unit of work to parallelize over

    auto patch_ncc_fn = [&] (const RangeType& r)
    {
      cv::Mat mask_roi;
      
      Scalar tmp_mean    = 0;
      Scalar tmp_std_dev = 0;

      const size_type local_patch_idx_begin = r.begin();
      const size_type local_patch_idx_end   = r.end();

      for (size_type local_patch_idx = local_patch_idx_begin;
           local_patch_idx < local_patch_idx_end;
           ++local_patch_idx)
      {
        const size_type global_patch_idx = this->patch_inds_to_use_[local_patch_idx];

        const PatchInfo& patch_info = this->patch_infos_[global_patch_idx];

        const Scalar cur_wgt = !use_mov_img_patch_variances_as_wgts_ ?
                                            patch_info.weight :
                                            mov_img_patch_vars[local_patch_idx - local_patch_idx_begin];
        
        if (!this->weight_patch_sims_in_combine_ || (std::abs(cur_wgt) > 1.0e-6))
        {
          const cv::Mat& fixed_patch = fixed_scaled_patches_[global_patch_idx];
      
          const auto roi = patch_info.ocv_roi();

          if (use_mask)
          {
            mask_roi = ocv_mask(roi);
          }
        
          cv::Mat mov_roi = mov_img(roi);
          std::tie(tmp_mean, tmp_std_dev, std::ignore) =
                      detail::ComputePatchMeanStdDev(mov_roi, use_mask ? &mask_roi : nullptr,
                                                     this->use_mask_for_patch_stats_);
       
          Scalar tmp_accum = 0;

          for (size_type pr = 0; pr < this->patch_diam_; ++pr)
          {
            const Scalar* fixed_row = &fixed_patch.at<Scalar>(pr,0);
            
            const Scalar* mov_row = &mov_roi.at<Scalar>(pr,0);

            for (size_type pc = 0; pc < this->patch_diam_; ++pc)
            {
              if (!use_mask || mask_roi.at<MaskScalar>(pr,pc))
              {
                tmp_accum += ((static_cast<Scalar>(mov_row[pc]) - tmp_mean) / tmp_std_dev) * fixed_row[pc];
              }
            } 
          }
          
          const Scalar cur_sim_val = 1 - tmp_accum;

          if (this->save_all_per_patch_scores_)
          {
            this->sim_vals_for_each_patch_[local_patch_idx][mov_idx] = cur_sim_val;
          }

          cur_mov_img_patch_ncc_vals_[local_patch_idx] =
            (this->weight_patch_sims_in_combine_ ? cur_wgt : Scalar(1)) * cur_sim_val;
        }
        // else cur_mov_img_patch_ncc_vals_[local_patch_idx]
        // has already been initialized to zero
      }
    };

    ParallelFor(patch_ncc_fn, RangeType(0, num_patches));
  
    Scalar patch_sims_sum = 0;
    for (const auto& s : cur_mov_img_patch_ncc_vals_)
    {
      patch_sims_sum += s;
    }
  
    if (this->compute_mean_of_patch_sims_)
    {
      patch_sims_sum /= num_patches;
    }
    else if (this->weight_patch_sims_in_combine_)
    { 
      // normalize by weightings of the patches - for example if we have randomly chosen
      // all unlikely (low weight) patches, then the weighted sum may artificially make
      // the objective function look small if we do not normalize
      Scalar tot_wgt = 0;

      for (const auto global_patch_idx : this->patch_inds_to_use_)
      {
        tot_wgt += this->patch_infos_[global_patch_idx].weight;
      }

      patch_sims_sum /= tot_wgt;
    }

    this->sim_vals_[mov_idx] = patch_sims_sum;
  }
   
  if (this->save_aux_info_)
  {
    if (!sim_aux_)
    {
      sim_aux_ = std::make_shared<SimAux>();
    }

    sim_aux_->patch_infos_per_compute_call.push_back(this->patch_infos_);
    sim_aux_->patch_indices_per_compute_call.push_back(this->patch_inds_to_use_);
  }
}
  
std::shared_ptr<xreg::H5ReadWriteInterface> xreg::ImgSimMetric2DPatchNCCCPU::aux_info()
{
  return sim_aux_;
}
  
bool xreg::ImgSimMetric2DPatchNCCCPU::use_fixed_img_patch_variances_as_wgts() const
{
  return use_fixed_img_patch_variances_as_wgts_;
}

void xreg::ImgSimMetric2DPatchNCCCPU::set_use_fixed_img_patch_variances_as_wgts(const bool use_vars_as_wgts)
{
  use_fixed_img_patch_variances_as_wgts_ = use_vars_as_wgts;
}

bool xreg::ImgSimMetric2DPatchNCCCPU::use_mov_img_patch_variances_as_wgts() const
{
  return use_mov_img_patch_variances_as_wgts_;
}

void xreg::ImgSimMetric2DPatchNCCCPU::set_use_mov_img_patch_variances_as_wgts(const bool use_vars_as_wgts)
{
  use_mov_img_patch_variances_as_wgts_ = use_vars_as_wgts;
}

void xreg::ImgSimMetric2DPatchNCCCPU::set_other_mov_img_patch_vars(const std::vector<ScalarList>* other_vars)
{
  other_mov_img_patch_vars_ = other_vars;
}
  
void xreg::ImgSimMetric2DPatchNCCCPU::process_mask()
{
  ImgSimMetric2DCPU::process_mask();

  // This is only called when the mask has been updated, or for the inital
  // processing
    
  const bool use_mask = this->mask_;

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  // the mask has changed, we need to make sure the weights are recomputed
  this->need_to_recompute_weights_ = true; 
  this->compute_weights(use_mask ? &ocv_mask : nullptr);
 
  // fixed image statistics:
  // we only need to compute these if we haven't done it yet - OR -
  // we use the mask when computing the stats, since the mask has changed,
  // the stats need to be recomputed
  if (!init_fixed_img_stats_computed_ || this->use_mask_for_patch_stats_)
  {
    WgtImgPtr vars_as_wgts_img;
    cv::Mat   vars_as_wgts_img_ocv;

    if (use_fixed_img_patch_variances_as_wgts_)
    {
      vars_as_wgts_img = MakeITK2DVol<Scalar>(img_num_cols_, img_num_rows_, Scalar(0));
      vars_as_wgts_img_ocv = ShallowCopyItkToOpenCV(vars_as_wgts_img.GetPointer());
    }

    // NOTE: we compute statistics over all possible patches, even when
    //       we will eventually choose random patches, e.g. we'll still
    //       precompute all information we need from the fixed image
    const size_type num_patches = this->patch_infos_.size();
    
    auto patch_preproc_fn = [&] (const RangeType& r)
    {
      Scalar    tmp_mean                      = 0;
      Scalar    tmp_std_dev                   = 0;
      size_type tmp_num_patch_elems_for_stats = 0;

      for (size_type patch_idx = r.begin(); patch_idx < r.end(); ++patch_idx)
      {
        PatchInfo& patch_info = this->patch_infos_[patch_idx];
        
        const auto roi = patch_info.ocv_roi();

        cv::Mat mask_roi;
        if (use_mask)
        {
          mask_roi = ocv_mask(roi);
        }

        cv::Mat& dst_roi = fixed_scaled_patches_[patch_idx];
      
        cv::Mat fixed_roi = fixed_ocv_img_(roi);
        std::tie(tmp_mean, tmp_std_dev,
                 tmp_num_patch_elems_for_stats) =
                       detail::ComputePatchMeanStdDev(fixed_roi,
                                                      use_mask ? &mask_roi : nullptr,
                                                      this->use_mask_for_patch_stats_);

        for (size_type pr = 0; pr < this->patch_diam_; ++pr)
        {
          Scalar* dst_roi_row = &dst_roi.at<Scalar>(pr,0);
          
          const Scalar* fixed_row = &fixed_roi.at<Scalar>(pr,0);

          for (size_type pc = 0; pc < this->patch_diam_; ++pc)
          {
            // Do not need to check for mask here... we'll check later for the moving images
            dst_roi_row[pc] = (static_cast<Scalar>(fixed_row[pc]) - tmp_mean) /
                                (tmp_std_dev * tmp_num_patch_elems_for_stats);
          } 
        }

        if (use_fixed_img_patch_variances_as_wgts_)
        {
          if (use_mask && !this->use_mask_for_patch_stats_)
          {
            // we previously did not use the mask when computing statistics,
            // use it this time
           
            std::tie(std::ignore, tmp_std_dev, std::ignore) =
               detail::ComputePatchMeanStdDev(fixed_roi, use_mask ? &mask_roi : nullptr, use_mask);
          }
  
          const auto patch_row_col = patch_info.center_row_col();

          vars_as_wgts_img_ocv.at<Scalar>(patch_row_col[0], patch_row_col[1]) = tmp_std_dev * tmp_std_dev;
        }
      }
    };
    
    // compute stats on fixed image
    ParallelFor(patch_preproc_fn, RangeType(0,num_patches));

    if (use_fixed_img_patch_variances_as_wgts_)
    {
      this->set_wgt_img(vars_as_wgts_img);
      this->compute_weights(this->mask_ ? &ocv_mask : nullptr);
    }
    
    init_fixed_img_stats_computed_ = true;
  }
}
    
void xreg::ImgSimMetric2DPatchNCCCPU::SimAux::write(H5::Group* h5)
{
  WriteStringH5("sim-aux-type", "patch-ncc-aux", h5, false);

  const size_type num_compute_calls = patch_infos_per_compute_call.size();
  xregASSERT(patch_indices_per_compute_call.size() == num_compute_calls);
  
  std::vector<size_type> tmp_patch_start_rows;
  std::vector<size_type> tmp_patch_start_cols;
  std::vector<size_type> tmp_patch_stop_rows;
  std::vector<size_type> tmp_patch_stop_cols;

  std::vector<Scalar> tmp_patch_wgts;

  for (size_type i = 0; i < num_compute_calls; ++i)
  {
    tmp_patch_start_rows.clear();
    tmp_patch_start_cols.clear();
    tmp_patch_stop_rows.clear();
    tmp_patch_stop_cols.clear();
    tmp_patch_wgts.clear();
    
    H5::Group cur_call_g = h5->createGroup(fmt::format("{:04d}", i));

    for (const auto& p : patch_infos_per_compute_call[i])
    {
      tmp_patch_start_rows.push_back(p.start_row);
      tmp_patch_start_cols.push_back(p.start_col);
      tmp_patch_stop_rows.push_back(p.stop_row);
      tmp_patch_stop_cols.push_back(p.stop_col);
      tmp_patch_wgts.push_back(p.weight);
    }
    
    WriteVectorH5("start-rows", tmp_patch_start_rows, &cur_call_g);
    WriteVectorH5("start-cols", tmp_patch_start_cols, &cur_call_g);
    WriteVectorH5("stop-rows",  tmp_patch_stop_rows,  &cur_call_g);
    WriteVectorH5("stop-cols",  tmp_patch_stop_cols,  &cur_call_g);
    
    WriteVectorH5("weights", tmp_patch_wgts, &cur_call_g);

    WriteVectorH5("patch-inds", patch_indices_per_compute_call[i], &cur_call_g);
  }
}

void xreg::ImgSimMetric2DPatchNCCCPU::SimAux::read(const H5::Group& h5)
{
  patch_infos_per_compute_call.clear();
  patch_indices_per_compute_call.clear();

  std::vector<size_type> start_rows;
  std::vector<size_type> start_cols;
  std::vector<size_type> stop_rows;
  std::vector<size_type> stop_cols;

  std::vector<Scalar> wgts;

  PatchInfoList patch_infos;

  size_type num_compute_calls_found = 0;

  bool should_stop = false;
  while (!should_stop)
  {
    H5::Group call_g;

    {
      HideH5ExceptionPrints suppress_exception_prints;

      try
      {
        call_g = h5.openGroup(fmt::format("{:04d}", num_compute_calls_found));
        ++num_compute_calls_found;
      }
      catch (H5::Exception&)
      {
        should_stop = true;
      }
    }

    if (!should_stop)
    {
      start_rows = detail::ReadVectorH5Helper<size_type>("start-rows", call_g);
      start_cols = detail::ReadVectorH5Helper<size_type>("start-cols", call_g);
      stop_rows  = detail::ReadVectorH5Helper<size_type>("stop-rows",  call_g);
      stop_cols  = detail::ReadVectorH5Helper<size_type>("stop-cols",  call_g);
    
      wgts = detail::ReadVectorH5Helper<Scalar>("weights", call_g);

      const size_type num_patches = start_rows.size();

      xregASSERT(start_cols.size() == num_patches);
      xregASSERT(stop_rows.size()  == num_patches);
      xregASSERT(stop_cols.size()  == num_patches);
      xregASSERT(wgts.size()       == num_patches);

      patch_infos.resize(num_patches);

      for (size_type pi = 0; pi < num_patches; ++pi)
      {
        PatchInfo& p = patch_infos[pi];

        p.start_row = start_rows[pi];
        p.start_col = start_cols[pi];
        p.stop_row  = stop_rows[pi];
        p.stop_col  = stop_cols[pi];
        p.weight    = wgts[pi];
      }
      
      patch_infos_per_compute_call.push_back(patch_infos);

      patch_indices_per_compute_call.push_back(detail::ReadVectorH5Helper<size_type>("patch-inds", call_g));
    }
  }
}

std::tuple<xreg::ImgSimMetric2DPatchNCCCPU::Scalar,
           xreg::ImgSimMetric2DPatchNCCCPU::Scalar,
           xreg::size_type>
xreg::detail::ComputePatchMeanStdDev(const cv::Mat& p, const cv::Mat* m,
                                     const bool use_mask_for_stats)
{
  using Scalar     = xreg::ImgSimMetric2DPatchNCCCPU::Scalar;
  using MaskScalar = xreg::ImgSimMetric2DPatchNCCCPU::MaskScalar;

  const int patch_nr = p.rows;
  const int patch_nc = p.cols;

  size_type num_pix_for_stats = 0;

  Scalar mean = 0;

  for (int r = 0; r < patch_nr; ++r)
  {
    const auto* p_row = &p.at<Scalar>(r,0);

    for (int c = 0; c < patch_nc; ++c)
    {
      // if we are not using a mask for stats OR the pixel is not even
      // masked, then use this pixel value in the statistics
      if (!use_mask_for_stats || (!m || m->at<MaskScalar>(r,c)))
      {
        mean += static_cast<Scalar>(p_row[c]);
        ++num_pix_for_stats;
      }
    }
  }
  
  Scalar var = 0;

  if (num_pix_for_stats > 1)
  {
    mean /= num_pix_for_stats;
  
    Scalar tmp = 0;

    for (int r = 0; r < patch_nr; ++r)
    {
      const auto* p_row = &p.at<Scalar>(r,0);

      for (int c = 0; c < patch_nc; ++c)
      {
        // if we are not using a mask for stats OR the pixel is not even
        // masked, then use this pixel value in the statistics
        if (!use_mask_for_stats || (!m || m->at<MaskScalar>(r,c)))
        {
          tmp = static_cast<Scalar>(p_row[c]) - mean;
          var += tmp * tmp;
        }
      }
    }
    var /= num_pix_for_stats - Scalar(1);
  }

  return std::make_tuple(mean,
                         std::max(Scalar(1.0e-6), std::sqrt(var)),
                         num_pix_for_stats);
}


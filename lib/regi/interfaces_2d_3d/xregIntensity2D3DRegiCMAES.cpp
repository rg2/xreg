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

#include "xregIntensity2D3DRegiCMAES.h"

#include <fmt/format.h>

#include <opencv2/imgcodecs.hpp>

#include <cmaes_interface.h>

#include "xregIntensity2D3DRegiDebug.h"
#include "xregSE3OptVars.h"
#include "xregHDF5.h"
#include "xregImgSimMetric2D.h"
#include "xregFilesystemUtils.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"

xreg::Intensity2D3DRegiCMAES::Intensity2D3DRegiCMAES()
{
  this->num_projs_per_view_ = kDEFAULT_LAMBDA;

  set_opt_obj_fn_tol(1.0e-3);
  set_opt_x_tol(1.0e-6);
}

void xreg::Intensity2D3DRegiCMAES::set_pop_size(const size_type pop_size)
{
  this->num_projs_per_view_ = pop_size;
}

void xreg::Intensity2D3DRegiCMAES::set_bounds(const ScalarList& bounds)
{
  bounds_ = bounds;
}

void xreg::Intensity2D3DRegiCMAES::remove_bounds()
{
  bounds_.clear();
}

void xreg::Intensity2D3DRegiCMAES::set_sigma(const ScalarList& sigma)
{
  sigma_ = sigma;
}

void xreg::Intensity2D3DRegiCMAES::reset_sigma()
{
  sigma_.clear();
}

void xreg::Intensity2D3DRegiCMAES::run()
{
  // TODO: support optimizing over multiple sources

  constexpr Scalar kDEFAULT_SIGMA = 0.3;  ///< Default sigma value to be used in all directions
  
  const auto& opt_vars = *this->opt_vars_;

  // number of volumes we're trying to estimate the pose of.
  const size_type nv = this->num_vols();

  const size_type num_params_per_xform = opt_vars.num_params();

  const size_type tot_num_params = nv * num_params_per_xform;

  const size_type pop_size = this->num_projs_per_view_;

  xregASSERT(sigma_.empty() || (sigma_.size() == tot_num_params));
  xregASSERT(bounds_.empty() || (bounds_.size() == tot_num_params));

  if (sigma_.empty())
  {
    sigma_.assign(tot_num_params, kDEFAULT_SIGMA);
  }

  const bool run_unc = bounds_.empty();

  // setup the intermediate structure to store projection parameterizations:
  // pop_params[i][j][k] is the kth parameter of the jth SE(3) (projection) element for the ith object/volume
  ListOfListsOfScalarLists pop_params(nv, ListOfScalarLists(pop_size, ScalarList(num_params_per_xform, 0)));

  // intermediate structure to store the similarity values
  ScalarList sim_vals(pop_size, 0);

  std::vector<double> init_delta_guess(tot_num_params, 0);
  std::vector<double> tmp_sigma_double(sigma_.begin(), sigma_.end());

  evo_ = std::make_shared<cmaes_t>();

  auto* evo = evo_.get();

  memset(evo, 0, sizeof(cmaes_t));

  // The return value will store the objective function values at each of the
  // lambda search points
  // The "non" string parameter indicates that no parameter file should be read
  // or written to.
  double* obj_fn_vals = cmaes_init(evo, static_cast<int>(tot_num_params), &init_delta_guess[0],
                                   &tmp_sigma_double[0], 0, static_cast<int>(pop_size), "non");

  evo->sp.stopTolFun = obj_fn_tol_;

  evo->sp.stopTolX = x_tol_;

  this->before_first_iteration();
  
  OptAux* opt_aux = nullptr;
  
  // Turning aux output off, since it takes up a significant amount of space in HDF5
  // for the PAO simulation studies
  if (false && this->debug_save_iter_debug_info_)
  {
    this->debug_info_->opt_aux = std::make_shared<OptAux>();
    opt_aux = static_cast<OptAux*>(this->debug_info_->opt_aux.get());

    opt_aux->se3_param_dim = num_params_per_xform;
    opt_aux->pop_size = pop_size;

    const size_type init_capacity = this->debug_info_->sims.capacity();
    opt_aux->cov_mats.reserve(init_capacity);
    opt_aux->num_rejects.reserve(init_capacity);
    opt_aux->sim_vals.reserve(init_capacity);
    opt_aux->pop_params.reserve(init_capacity);
  }

  debug_sim_val_ = std::numeric_limits<Scalar>::max();

  size_type iter = 0;

  while (!cmaes_TestForTermination(evo) && (iter < this->max_num_iters_))
  {
    this->begin_of_iteration(get_cmaes_cur_params_list());

    // generate lambda new search points, sample population
    // This is an array of double arrays, e.g. pop[i] has tot_num_params doubles.
    double* const* pop = cmaes_SamplePopulation(evo);  // do not change content of pop

    if (!run_unc)
    {
      size_type num_rejects = 0;

      // enforce constraints
      for (size_type pop_ind = 0; pop_ind < pop_size; ++pop_ind)
      {
        bool cur_params_in_bounds = true;

        do
        {
          cur_params_in_bounds = true;

          const double* cur_params = pop[pop_ind];

          for (size_type param_idx = 0; param_idx < tot_num_params; ++param_idx)
          {
            const double cur_bound = bounds_[param_idx];

            // if out of bounds, then resample and re-check the newly sampled version
            if ((cur_params[param_idx] < -cur_bound) || (cur_params[param_idx] > cur_bound))
            {
              cmaes_ReSampleSingle(evo, pop_ind);
              cur_params_in_bounds = false;
              ++num_rejects;
              break;
            }
          }
        }
        while (!cur_params_in_bounds);
      }

      if (opt_aux)
      {
        opt_aux->num_rejects.push_back(num_rejects);
      }
    }

    // transfer to intermediate storage:
    for (size_type pop_ind = 0; pop_ind < pop_size; ++pop_ind)
    {
      size_type param_off = 0;
      for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx, param_off += num_params_per_xform)
      {
        pop_params[vol_idx][pop_ind].assign(pop[pop_ind] + param_off,
                                            pop[pop_ind] + param_off + num_params_per_xform);
      }
    }

    // compute the DRRs and similarities
    this->obj_fn(pop_params, &sim_vals);

    // copy similarities from intermediate storage
    std::copy(sim_vals.begin(), sim_vals.end(), obj_fn_vals);

    if (this->write_combined_sim_scores_to_stream_ || this->debug_save_iter_debug_info_)
    {
      debug_sim_val_ = std::min(debug_sim_val_,
                                      *std::min_element(sim_vals.begin(), sim_vals.end()));
    }

    // update the search distribution used for cmaes_SamplePopulation()
    cmaes_UpdateDistribution(evo, obj_fn_vals);
    
    if (opt_aux)
    {
      MatMxN cur_cov(tot_num_params,tot_num_params);

      for (size_type r = 0; r < tot_num_params; ++r)
      {
        for (size_type c = 0; c < r; ++c)
        {
          cur_cov(r,c) = evo->C[r][c];
          cur_cov(c,r) = evo->C[r][c];
        }
        cur_cov(r,r) = evo->C[r][r];
      }

      opt_aux->cov_mats.push_back(cur_cov);
      opt_aux->sim_vals.push_back(sim_vals);
      opt_aux->pop_params.push_back(pop_params);
    }

    this->end_of_iteration();
    
    ++iter;
  }

  this->after_last_iteration();

  this->update_regi_xforms(xforms_at_cur_mean());

  if (this->src_and_obj_pose_opt_vars_)
  {
    this->regi_cam_models_.assign(1, cam_model_at_cur_mean());
  }

  // clean up any memory allocated interally to CMAES
  cmaes_exit(evo);
}

void xreg::Intensity2D3DRegiCMAES::set_opt_obj_fn_tol(const Scalar& tol)
{
  obj_fn_tol_ = tol;
}

void xreg::Intensity2D3DRegiCMAES::set_opt_x_tol(const Scalar& tol)
{
  x_tol_ = tol;
}

xreg::size_type xreg::Intensity2D3DRegiCMAES::max_num_projs_per_view_per_iter() const
{
  return this->num_projs_per_view_;
}

void xreg::Intensity2D3DRegiCMAES::init_opt()
{
  // we'll init during the run call
}

void xreg::Intensity2D3DRegiCMAES::debug_write_remapped_drrs()
{
  const size_type num_sim_metrics = this->sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    Path dst_path = this->debug_output_dir_path_;
    dst_path += fmt::format("drr_remap_{:03d}_{:03d}.png", this->num_obj_fn_evals_, view_idx);

    WriteITKImageRemap8bpp(debug_cur_mean_drrs_[view_idx].GetPointer(), dst_path.string());
  }
}

void xreg::Intensity2D3DRegiCMAES::debug_write_raw_drrs()
{
  const size_type num_sim_metrics = this->sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    Path dst_path = this->debug_output_dir_path_;
    dst_path += fmt::format("drr_raw_{:03d}_{:03}.nii.gz", this->num_obj_fn_evals_, view_idx);

    WriteITKImageToDisk(debug_cur_mean_drrs_[view_idx].GetPointer(), dst_path.string());
  }
}

void xreg::Intensity2D3DRegiCMAES::debug_write_fixed_img_edge_overlays()
{
  const size_type num_sim_metrics = this->sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    const auto img_2d_size = debug_cur_mean_drrs_[view_idx]->GetLargestPossibleRegion().GetSize();

    cv::Mat edge_img(img_2d_size[1], img_2d_size[0], cv::DataType<unsigned char>::type);
    RemapAndComputeEdges(debug_cur_mean_drrs_[view_idx].GetPointer(), &edge_img, 12, 75);

    auto fixed_u8 = ITKImageRemap8bpp(this->sim_metrics_[view_idx]->fixed_image().GetPointer());

    cv::Mat edge_overlay = OverlayEdges(ShallowCopyItkToOpenCV(fixed_u8.GetPointer()), edge_img, 1 /* 1 -> green edges */);

    Path dst_path = this->debug_output_dir_path_;
    dst_path += fmt::format("edges_{:03d}_{:03d}.png", this->num_obj_fn_evals_, view_idx);
    cv::imwrite(dst_path.string(), edge_overlay);
  }
}

void xreg::Intensity2D3DRegiCMAES::write_debug()
{
  if (this->write_debug_requires_drr())
  {
    compute_drrs_at_mean();
  }

  Intensity2D3DRegi::write_debug();
}

void xreg::Intensity2D3DRegiCMAES::debug_write_comb_sim_score()
{
  if (this->write_combined_sim_scores_to_stream_)
  {
    this->dout() << fmt::format("{:+20.6f}\n", debug_sim_val_);
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    this->debug_info_->sims.push_back(debug_sim_val_);
  }
}

void xreg::Intensity2D3DRegiCMAES::debug_write_opt_pose_vars()
{
  const size_type nv = this->num_vols();
  const size_type num_params_per_xform = this->opt_vars_->num_params();

  const double* cur_params = get_cmaes_cur_params_buf();

  if (this->write_opt_vars_to_stream_)
  {
    size_type global_param_idx = 0;

    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      for (size_type param_idx = 0; param_idx < num_params_per_xform; ++param_idx, ++global_param_idx)
      {
        this->dout() << fmt::format("{:+14.6f}, ", cur_params[global_param_idx]);
      }
      this->dout() << "| ";
    }
    this->dout() << '\n';
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      this->debug_info_->iter_vars[vol_idx].push_back(
            Eigen::Map<PtN_d>(const_cast<double*>(&cur_params[vol_idx * num_params_per_xform]),
                              num_params_per_xform).cast<Scalar>());
    }
  }
}

const double* xreg::Intensity2D3DRegiCMAES::get_cmaes_cur_params_buf()
{
  const char* param_name = "xmean";
  //const char* param_name = "xbestever";
  
  return cmaes_GetPtr(evo_.get(), param_name);
}

xreg::Intensity2D3DRegiCMAES::ScalarList
xreg::Intensity2D3DRegiCMAES::get_cmaes_cur_params_list()
{
  const double* cmaes_buf = get_cmaes_cur_params_buf();
  
  return ScalarList(cmaes_buf, cmaes_buf + (this->opt_vars_->num_params() * this->num_vols()));
}

xreg::FrameTransformList xreg::Intensity2D3DRegiCMAES::xforms_at_cur_mean()
{
  return this->opt_vec_to_frame_transforms(get_cmaes_cur_params_list());
}

xreg::CameraModel xreg::Intensity2D3DRegiCMAES::cam_model_at_cur_mean()
{
  xregASSERT(this->src_and_obj_pose_opt_vars_);

  return this->src_and_obj_pose_opt_vars_->cam(
                    Eigen::Map<PtN_d>(const_cast<double*>(get_cmaes_cur_params_buf()),
                                            this->src_and_obj_pose_opt_vars_->num_params()).cast<Scalar>());
}

void xreg::Intensity2D3DRegiCMAES::compute_drrs_at_mean()
{
  FrameTransformList xform_list = xforms_at_cur_mean();

  CamModelList* cams = nullptr;
  CamModelList tmp_cams; 

  if (this->src_and_obj_pose_opt_vars_)
  {
    cams = &tmp_cams;
    tmp_cams.assign(1, cam_model_at_cur_mean()); 
  }

  this->debug_compute_drrs_single_proj_per_view(xform_list, &debug_cur_mean_drrs_, true, cams);
}

void xreg::Intensity2D3DRegiCMAES::OptAux::read(const H5::Group& h5)
{
  xregThrow("Intens. 2D/3D Regi. CMA-ES OptAux::read() unsuported!");
}

void xreg::Intensity2D3DRegiCMAES::OptAux::write(H5::Group* h5)
{
  H5::Group aux_g = h5->createGroup("opt-aux");

  WriteStringH5("name", "CMA-ES", &aux_g, false);
    
  const size_type num_its = cov_mats.size();
  WriteSingleScalarH5("num-iterations", num_its, &aux_g);
  
  WriteSingleScalarH5("pop-size", pop_size, &aux_g);
   
  xregASSERT(pop_params.size() == num_its);

  // write every population parameterization
  {
    H5::Group all_pop_params_g = aux_g.createGroup("all-pop-params");

    MatMxN single_obj_pop_per_iter(pop_size, se3_param_dim);

    for (size_type i = 0; i < num_its; ++i)
    {
      H5::Group cur_it_pop_params_g = all_pop_params_g.createGroup(fmt::format("iter-{:03d}", i));

      const auto& pop_params_cur_it = pop_params[i];

      const size_type num_objs = pop_params_cur_it.size();

      for (size_type oi = 0; oi < num_objs; ++oi)
      {
        const auto& pop_params_cur_obj = pop_params_cur_it[oi];
        xregASSERT(pop_params_cur_obj.size() == pop_size);

        for (size_type p = 0; p < pop_size; ++p)
        {
          for (size_type k = 0; k < se3_param_dim; ++k)
          {
            single_obj_pop_per_iter(p,k) = pop_params_cur_obj[p][k];
          }
        }
        WriteMatrixH5(fmt::format("obj-{:03d}-pop", oi), single_obj_pop_per_iter, &cur_it_pop_params_g);  
      }
    }
  }

  // write all sim values for every iteration and every population sample
  {
    MatMxN all_sim_vals(num_its, pop_size);
    for (size_type i = 0; i < num_its; ++i)
    {
      const auto& cur_it_sim_vals = sim_vals[i];

      for (size_type p = 0; p < pop_size; ++p)
      {
        all_sim_vals(i,p) = cur_it_sim_vals[p];
      }
    }

    WriteMatrixH5("all-sim-vals", all_sim_vals, &aux_g);
  }

  // write covariances
  {
    H5::Group cov_g = aux_g.createGroup("covariances");

    for (size_type i = 0; i < num_its; ++i)
    {
      WriteMatrixH5(fmt::format("iter-{:03d}", i), cov_mats[i], &cov_g);
    }
  }
  
  // write number of rejections per iteration
  if (!num_rejects.empty())
  {
    xregASSERT(num_rejects.size() == num_its);
    WriteVectorH5("all-num-rejects", num_rejects, &aux_g); 
  }
}


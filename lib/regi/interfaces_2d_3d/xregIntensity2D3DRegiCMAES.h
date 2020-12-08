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

#ifndef XREGINTENSITY2D3DREGICMAES_H_
#define XREGINTENSITY2D3DREGICMAES_H_

#include "xregIntensity2D3DRegi.h"
#include "xregHDF5ReadWriteInterface.h"
#include "xregPerspectiveXform.h"

// Forward declaration
struct cmaes_struct;

namespace xreg
{

/// \brief 2D/3D Intensity-Based registration using the CMA-ES method for
///        optimization.
///
/// This can operate in contrained and unconstrained modes.
class Intensity2D3DRegiCMAES : public Intensity2D3DRegi
{
public:
  /// \brief Constructor; performs no work.
  Intensity2D3DRegiCMAES();

  /// \brief Sets the population size; the number of projections per camera
  ///
  /// This effects the number of DRRs computed.
  void set_pop_size(const size_type pop_size);

  /// \brief Set custom values of bounds; triggers constrained optimization.
  ///
  /// These are box constraints with the valid range in component k equal to
  /// [-bounds[k], bounds[k]]
  void set_bounds(const ScalarList& bounds);

  /// \brief Remove user specified bounds; default behavior will be to run
  ///        unconstrained.
  void remove_bounds();

  /// \brief Sets custom values of sigma
  void set_sigma(const ScalarList& sigma);

  /// \brief Reset user specified values of sigma; the default will be used.
  void reset_sigma();

  /// \brief Performs the registration; blocks until completion.
  void run() override;

  void set_opt_obj_fn_tol(const Scalar& tol) override;

  void set_opt_x_tol(const Scalar& tol) override;

  size_type max_num_projs_per_view_per_iter() const override;

protected:
  void init_opt() override;

  /// \brief Writes remapped (8 bpp) DRRs to disk.
  ///
  /// This CMA-ES implementation will write the image corresponding to the
  /// current mean, for every view, for every objective function
  /// invocation with the following file name
  /// pattern: "drr_remap_<obj fn #>_<view index>.png"
  void debug_write_remapped_drrs() override;

  /// \brief Writes raw DRRs to disk.
  ///
  /// This CMA-ES implementation will write the image corresponding to the
  /// current mean, for every view, for every objective function
  /// invocation with the following file name
  /// pattern: "drr_raw_<obj fn #>_<view index>.nii.gz"
  void debug_write_raw_drrs() override;

  /// \brief Writes remapped fixed images with edges from the DRRs overlaid to disk.
  ///
  /// This CMA-ES implementation will write the image corresponding to the
  /// current mean, for every view, for every objective function
  /// invocation with the following file name
  /// pattern: "edges_<obj fn #>_<view index>.png"
  void debug_write_fixed_img_edge_overlays() override;

  /// \brief Will write out debug images if request by the user.
  ///
  /// If any images will be written, then this override will compute the DRR at
  /// the current mean of the CMA-ES distribution and then call the parent
  /// implementation of write_debug()
  void write_debug() override;

  void debug_write_comb_sim_score() override;

  void debug_write_opt_pose_vars() override;

private:

  const double* get_cmaes_cur_params_buf();

  ScalarList get_cmaes_cur_params_list();

  /// \brief Retrieves the frame transformations for each object at the current CMA-ES mean
  FrameTransformList xforms_at_cur_mean();

  CameraModel cam_model_at_cur_mean();

  /// \brief Computes DRRs for each camera model using the frame transform represented
  ///        by the CMA-ES mean.
  void compute_drrs_at_mean();

  struct OptAux : H5ReadWriteInterface
  {
    size_type se3_param_dim = 0;

    size_type pop_size;

    /// \brief cov_mats[i] is the covariance matrix at the jth iteration
    ///
    /// It stores covariances amongst objects as well.
    MatMxNList cov_mats;

    /// \brief num_rejects[i] is the number of times resampling was performed in iteration
    ///        i to get a population satisfying the bounds
    std::vector<size_type> num_rejects;

    /// \brief sim_vals[i][j] is the similarity value for the jth population element in the ith iteration
    ListOfScalarLists sim_vals;
    
    /// \brief pop_params[i][j][k][l] is the lth parameter of the kth SE(3) (projection)
    ///        element for the jth object/volume in the ith iteration
    std::vector<ListOfListsOfScalarLists> pop_params;

    void read(const H5::Group& h5) override;

    void write(H5::Group* h5) override;
  };

  ProjList debug_cur_mean_drrs_;

  ScalarList bounds_;
  ScalarList sigma_;

  // a unique_ptr cannot use a forwarded type, so we use shared_ptr
  std::shared_ptr<cmaes_struct> evo_;

  Scalar debug_sim_val_;

  Scalar obj_fn_tol_;
  Scalar x_tol_;

  /// \brief Default population size to use.
  enum { kDEFAULT_LAMBDA = 50 };
};

}  // xreg

#endif

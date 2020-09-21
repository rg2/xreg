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

#ifndef XREGINTENSITY2D3DREGIHILLCLIMB_H_
#define XREGINTENSITY2D3DREGIHILLCLIMB_H_

#include "xregIntensity2D3DRegi.h"

namespace xreg
{

/// \brief 2D/3D Intensity-Based registration using the hill climbing method
///        described in Penney TMI 1998.
class Intensity2D3DRegiHillClimb : public Intensity2D3DRegi
{
public:
  /// \brief Constructor; performs no work.
  Intensity2D3DRegiHillClimb();

  void setup() override;

  void set_num_step_levels(const size_type num_step_levels);

  void set_init_steps(const ScalarList& init_steps);

  /// \brief Performs the registration; blocks until completion.
  void run() override;

  size_type max_num_projs_per_view_per_iter() const override;

protected:
  void init_opt() override;

  /// \brief Writes remapped (8 bpp) DRRs to disk.
  ///
  /// This implementation will write the image corresponding to the
  /// current mean, for every view, for every objective function
  /// invocation with the following file name
  /// pattern: "drr_remap_<obj fn #>_<view index>.png"
  void debug_write_remapped_drrs() override;

  /// \brief Writes raw DRRs to disk.
  ///
  /// This implementation will write the image corresponding to the
  /// current mean, for every view, for every objective function
  /// invocation with the following file name
  /// pattern: "drr_raw_<obj fn #>_<view index>.nii.gz"
  void debug_write_raw_drrs() override;

  /// \brief Writes remapped fixed images with edges from the DRRs overlaid to disk.
  ///
  /// This implementation will write the image corresponding to the
  /// current estimate, for every view, for every objective function
  /// invocation with the following file name
  /// pattern: "edges_<obj fn #>_<view index>.png"
  void debug_write_fixed_img_edge_overlays() override;

  void debug_write_comb_sim_score() override;

  void debug_write_opt_pose_vars() override;

private:

  ScalarList init_steps_;

  size_type num_step_levels_;

  ScalarList cur_param_vec_;

  Scalar cur_param_sim_val_;
};

}  // xreg

#endif


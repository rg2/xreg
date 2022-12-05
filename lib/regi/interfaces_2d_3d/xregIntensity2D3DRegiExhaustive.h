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

#ifndef XREGINTENSITY2D3DREGIEXHAUSTIVE_H_
#define XREGINTENSITY2D3DREGIEXHAUSTIVE_H_

#include "xregNDRange.h"
#include "xregIntensity2D3DRegi.h"
#include "xregHDF5ReadWriteInterface.h"

namespace xreg
{

// Forward declarations:

/// \brief Exhaustively search a list of candidate rigid transformations
///        and select the set of transformations that yields the smallest
///        similarity metric.
class Intensity2D3DRegiExhaustive : public Intensity2D3DRegi
{
public:
  Intensity2D3DRegiExhaustive() = default;

  // This should be called prior to calling setup()
  // TODO: also allow passing optimization space params
  void set_cam_wrt_vols(const ListOfFrameTransformLists& xforms,
                        const size_type batch_size = 0);

  void set_cam_wrt_vols(const ConstSpacedMeshGrid& mesh_grid, const size_type batch_size = 0);

  void run() override;

  size_type max_num_projs_per_view_per_iter() const override;

  bool save_all_sim_vals() const;

  void set_save_all_sim_vals(const bool s);

  const ScalarList& all_sim_vals() const;

  const ScalarList& all_penalty_vals() const;

  const ScalarList& all_penalty_log_probs() const;

  bool save_all_penalty_vals() const;

  void set_save_all_penalty_vals(const bool s);

  void set_save_all_cam_wrt_vols_in_aux(const bool s);

  bool save_all_cam_wrt_vols_in_aux() const;

  size_type tot_num_xforms() const;

  void set_print_status_inc(const double inc);

protected:
  void write_debug() override;

  void init_opt() override;

  using MeshGridIt = ConstSpacedMeshGrid::const_iterator;
  
  ListOfFrameTransformLists cam_wrt_vols_;
  
  size_type tot_num_xforms_ = 0;

  ConstSpacedMeshGrid mesh_grid_;
  MeshGridIt cur_mesh_grid_start_it_;
  MeshGridIt cur_mesh_grid_stop_it_;

  bool use_mesh_grid_ = false;

  size_type cur_start_xform_idx_ = 0;
  size_type cur_num_xforms_      = 0;
  
  ScalarList sim_vals_;
  
  bool compute_min_ = true;
 
  Scalar min_sim_val_;

  bool save_all_sim_vals_ = false;

  bool save_all_penalty_vals_ = false;

  size_type best_all_sim_vals_idx_ = std::numeric_limits<size_type>::max();

  ScalarList all_sim_vals_;

  ScalarList all_pen_vals_;

  ScalarList all_pen_log_prob_vals_;

private:
  // Interval to print the percentage of transforms evaluated.
  // 0.1 --> print in approximately 10% intervals
  // 1.1 --> do not print (e.g. print in 110% intervals)
  double print_status_inc_ = 0.1;

  bool save_all_cam_wrt_vols_in_aux_ = false;

  struct OptAux : H5ReadWriteInterface
  {
    ListOfFrameTransformLists cam_wrt_vols;
  
    bool has_min;
    Scalar min_sim_val;
  
    bool has_all_sim_vals;

    size_type best_all_sim_vals_idx;

    ScalarList all_sim_vals;

    void read(const H5::Group& h5) override;

    void write(H5::Group* h5) override;
  };
};

}  // xreg

#endif


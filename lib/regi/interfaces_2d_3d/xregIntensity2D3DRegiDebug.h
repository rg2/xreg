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

#ifndef XREGINTENSITYREGI2D3DDEBUG_H_
#define XREGINTENSITYREGI2D3DDEBUG_H_

#include <boost/optional.hpp>

#include "xregIntensity2D3DRegi.h"

// Forward Declarations
namespace H5
{

class Group;

}  // H5

namespace xreg
{

struct H5ReadWriteInterface;

struct SingleRegiDebugResults
{
  using SE3OptVarsPtr = Intensity2D3DRegi::SE3OptVarsPtr;

  using ScalarList = Intensity2D3DRegi::ScalarList;

  using ListOfSE3Params       = std::vector<PtN>;
  using ListOfListOfSE3Params = std::vector<ListOfSE3Params>;

  using BoolList = std::vector<bool>;

  using IndexList = Intensity2D3DRegi::IndexList;

  /// \brief Parameterization of SE3
  SE3OptVarsPtr se3_params;

  /// \brief Mapping from object indices used in this registration to global
  ///        volume indices.
  ///
  /// For example, global indices of 0, 1, and 2 may represent the pelvis volume,
  /// femur volume, and fragment volume, respectively, however in the registration
  /// of only the femur, the registration index is 0, but will map to the global
  /// index of 1.
  IndexList vols_used;

  /// \brief Indices of the projection views used by this registration. Empty --> use all views.
  ///
  /// These are global indices from the original list of projection datas.
  IndexList projs_used;

  /// \brief Initial poses of the objects, before registration
  ///
  /// init_poses[i] is the initial pose of the camera wrt the ith volume
  FrameTransformList init_poses;

  /// \brief Final poses of the objects, after registration
  ///
  /// final_poses[i] is the final pose of the camera wrt the ith volume
  FrameTransformList final_poses;

  /// \brief The information encoded in static_vols and static_vol_poses
  ///        was previously computed with an imprecise heuristic, setting
  ///        this flag to true tells users that the heuristic is not
  ///        necessary, since the information is available in the
  ///        static_vols and static_vol_poses members.
  bool do_not_use_static_vol_heuristic = false;

  /// \brief List of global volume indices that should be statically rendered
  ///        into the background
  IndexList static_vols;

  /// \brief List of poses for each static volume
  FrameTransformList static_vol_poses;

  /// \brief Intermediate frames used during the registration.
  ///
  /// inter_frames[i] is the intermediate frame used for the ith volume.
  FrameTransformList inter_frames;

  /// \brief Determines if the intermediate frame is with respect to the camera
  ///        or the volume.
  ///
  /// \see Regi2D3DIntensity::set_intermediate_frame
  BoolList inter_frames_wrt_vol;

  /// \brief Pose parameters of each object, at each registration iteration.
  ///
  /// iter_vars[i][j][k] is the kth pose parameter of the ith object on the jth iteration
  /// e.g. (*se3_params)(iter_vars[i][j]) returns a frame transform.
  ListOfListOfSE3Params iter_vars;

  /// \brief Similarity scores at each iteration.
  ///
  /// sims[j] is the similarity score at the jth iteration
  ScalarList sims;

  /// \brief Elapsed time to only run the registration
  ///
  /// This does not include pre-processing, etc.
  boost::optional<double> regi_time_secs;

  /// \brief Auxiliary info saved by the optimizer
  std::shared_ptr<H5ReadWriteInterface> opt_aux;

  /// \brief Auxiliary info saved by the similarity metrics
  std::vector<std::shared_ptr<H5ReadWriteInterface>> sims_aux;

  std::shared_ptr<H5ReadWriteInterface> pen_fn_debug;

  /// \brief basic initialization
  void init(SE3OptVarsPtr se3, const IndexList& global_vol_inds,
            const size_type init_num_iters_capacity = 1000);
};

void WriteSingleRegiDebugResultsH5(const SingleRegiDebugResults& results, H5::Group* h5);

SingleRegiDebugResults ReadSingleRegiDebugResultsH5(const H5::Group& h5);

}  // xreg

#endif

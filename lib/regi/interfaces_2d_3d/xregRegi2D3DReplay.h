/*
 * MIT License
 *
 * Copyright (c) 2020-2026 Robert Grupp
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

#ifndef XREGREGI2D3DREPLAY_H_
#define XREGREGI2D3DREPLAY_H_

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "xregCommon.h"
#include "xregObjWithOStream.h"

namespace xreg
{

// Forward declarations
class RayCaster;
struct DebugRegiResultsMultiLevel;

/// \brief Replays the iterations of a 2D/3D registration, using the debug
///        information saved during the registration, into movie frames.
///
/// Two sets of frames are created: DRRs of the objects at the pose estimates
/// of each iteration, and the fixed images overlaid with edges computed from
/// these DRRs. Each frame is a tiling of all views.
///
/// This implements the xreg-regi2d3d-replay tool. The default values of the
/// options match the defaults of the tool's flags.
struct Regi2D3DReplay : ObjWithOStream
{
  using RayCasterFactoryFn = std::function<std::shared_ptr<RayCaster>()>;

  /// \brief Thrown when the debug data may not be replayed, e.g. when the
  ///        registrations use different subsets of the views.
  class UnsupportedDataException : public std::runtime_error
  {
  public:
    using std::runtime_error::runtime_error;
  };

  /// Creates a line integral ray caster; this must be set prior to calling run().
  /// A new ray caster is created for each registration level.
  RayCasterFactoryFn line_int_ray_caster_fn;

  /// Creates a depth ray caster; this must be set prior to calling run() when
  /// compute_boundary_edges is true.
  RayCasterFactoryFn depth_ray_caster_fn;

  /// Compute boundary/silhouette contours of the objects.
  bool compute_boundary_edges = true;

  /// Threshold, in HU, used to determine the boundary of the objects.
  double boundary_edge_hu_thresh = 150.0;

  /// Compute Canny edges on the DRRs.
  bool compute_canny_edges = true;

  /// The Gaussian kernel width/height used for smoothing an image prior to
  /// Canny edge detection. <= 1 will do no smoothing.
  int smooth_kernel_width = 3;

  /// Lower threshold used for Canny edge detection.
  int canny_lo_thresh = 0;

  /// Upper threshold used for Canny edge detection.
  int canny_hi_thresh = 48;

  /// Amount of dilation applied to the edge map to make it easier to see.
  /// <= 1 will not enlarge.
  int edge_dilate_width = 3;

  /// Overlay landmark information from penalty functions (single view only).
  bool show_lands = false;

  /// Use the metadata of the projection data (when present) indicating the
  /// rotation needed for a patient "up" orientation to flip each view;
  /// this overrides flip_rows and flip_cols.
  bool use_pat_rot_up = true;

  /// Flip each image up/down.
  bool flip_rows = false;

  /// Flip each image left/right.
  bool flip_cols = false;

  /// When information about projection pre-processing is stored in the debug
  /// data, use it to apply pre-processing to the projections.
  bool do_preproc = false;

  /// 2D projection downsampling factor from the input fixed image dimensions.
  /// 1.0 -> no downsampling. 0.5 -> reduce by half in each dimension.
  double proj_ds_factor = 1.0;

  /// The maximum number of columns in the output tiling; once exceeded, a new
  /// row is made.
  int max_tile_cols = 7;

  /// When not empty, debug PNG images are written with paths of the format
  /// <prefix>_{tiled|view index}_{edges|mov}_<frame #>.png. The images of the
  /// individual views are written by run() and the tiled images by write_videos().
  std::string debug_img_prefix;

  /// Include the text burn-in (e.g. with iteration info, regi. name, etc.) in
  /// the tiled debug images.
  bool debug_img_include_text = false;

  /// When not empty, the raw DRRs of each view at the final iteration are
  /// written to <prefix>_<view index>.nii.gz by run().
  std::string debug_raw_final_drr_prefix;

  /// \brief Computes all of the frames, populating drr_frames, edge_frames,
  ///        and frame_titles.
  void run(const DebugRegiResultsMultiLevel& regi_results);

  /// \brief Writes the frames into two movies, the title of each frame is
  ///        burned into the movie frames (the stored frames are not modified).
  ///
  /// A movie is not written when its path is empty.
  /// run() must be called prior to this.
  void write_videos(const std::string& drr_path, const std::string& edges_path, const double fps);

  /// \brief The frames per second needed for movies with a specific length in seconds.
  ///
  /// run() must be called prior to this.
  double fps_for_video_len(const double video_len_secs) const;

  // OUTPUTS:

  /// Tiled DRRs (BGR, 8 bits per channel), there is one frame for the initial
  /// pose, each iteration, and the final pose of each registration at each level.
  std::vector<cv::Mat> drr_frames;

  /// Tiled fixed images overlaid with edges from the DRRs (BGR, 8 bits per channel).
  std::vector<cv::Mat> edge_frames;

  /// Title of each frame, e.g. with the level, registration name, and iteration.
  StrList frame_titles;

private:
  size_type num_views_ = 0;

  // number of columns of a single view in the (downsampled) frames
  size_type num_view_cols_ = 0;
};

}  // xreg

#endif


/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#include "xreg_py_backend.h"

#include <opencv2/imgproc.hpp>

#include "xregExceptionUtils.h"
#include "xregRayCastInterface.h"
#include "xregMultiObjMultiLevel2D3DRegiDebug.h"
#include "xregRegi2D3DReplay.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

/// \brief Python facing version of Regi2D3DReplay: the ray casters are created
///        using a compute backend.
struct PyRegi2D3DReplay : Regi2D3DReplay
{
  BackendPtr backend;
};

void SetBackend(PyRegi2D3DReplay& r, const BackendArg& backend_arg)
{
  // Strings are resolved here, while the GIL is held; the factory functions
  // are called without the GIL and only use the backend object.
  BackendPtr b = ResolveBackend(backend_arg);

  r.backend = b;

  r.line_int_ray_caster_fn = [b] () { return MakeRayCaster("line_int", BackendArg(b)); };
  r.depth_ray_caster_fn    = [b] () { return MakeRayCaster("depth", BackendArg(b)); };
}

void Run(PyRegi2D3DReplay& r, const DebugRegiResultsMultiLevel& regi_results)
{
  nb::gil_scoped_release no_gil;

  r.run(regi_results);
}

void RunFromFile(PyRegi2D3DReplay& r, const std::string& path)
{
  nb::gil_scoped_release no_gil;

  const auto regi_results = ReadMultiLevel2D3DRegiDebugFromDisk(path);

  r.run(regi_results);
}

// Copies BGR frames into a list of RGB numpy arrays of shape (rows, cols, 3)
nb::list FramesToArrays(const std::vector<cv::Mat>& frames)
{
  nb::list arrs;

  cv::Mat rgb;

  for (const auto& f : frames)
  {
    if (f.type() == CV_8UC3)
    {
      cv::cvtColor(f, rgb, cv::COLOR_BGR2RGB);
    }
    else if (f.type() == CV_8UC1)
    {
      cv::cvtColor(f, rgb, cv::COLOR_GRAY2RGB);
    }
    else
    {
      xregThrow("unexpected pixel type of replay frame!");
    }

    // (the output of cvtColor is continuous)
    arrs.append(CopyBufToArray<unsigned char,3>(rgb.ptr<unsigned char>(),
                                                { static_cast<size_t>(rgb.rows),
                                                  static_cast<size_t>(rgb.cols),
                                                  size_t(3) }));
  }

  return arrs;
}

}  // un-named

void xreg::py::BindReplay(nb::module_& m)
{
  using Replay = PyRegi2D3DReplay;

  // used for the default values of the constructor arguments
  const Regi2D3DReplay defs;

  nb::class_<Replay>(m, "Regi2D3DReplay",
      "Replays the iterations of a 2D/3D registration into movie frames, using the debug "
      "information collected during the registration (see "
      "MultiLevelMultiObjRegi.set_save_debug_info()). Two sets of frames are created: DRRs of "
      "the objects at the pose estimate of each iteration, and the fixed images overlaid with "
      "edges computed from these DRRs. Each frame is a tiling of all views. This is the same "
      "implementation as the xreg-regi2d3d-replay tool, and the options have the same defaults "
      "as the tool's flags. Call run(), then use the frames and/or write_videos().")
    .def("__init__",
        [](Replay* r, const BackendArg& backend,
           const bool compute_boundary_edges, const double boundary_edge_hu_thresh,
           const bool compute_canny_edges, const int smooth_kernel_width,
           const int canny_lo_thresh, const int canny_hi_thresh, const int edge_dilate_width,
           const bool show_lands, const bool use_pat_rot_up, const bool flip_rows,
           const bool flip_cols, const bool do_preproc, const double proj_ds_factor,
           const int max_tile_cols, const std::string& debug_img_prefix,
           const bool debug_img_include_text, const std::string& debug_raw_final_drr_prefix)
        {
          new (r) Replay();

          try
          {
            SetBackend(*r, backend);
          }
          catch (...)
          {
            r->~Replay();
            throw;
          }

          r->compute_boundary_edges     = compute_boundary_edges;
          r->boundary_edge_hu_thresh    = boundary_edge_hu_thresh;
          r->compute_canny_edges        = compute_canny_edges;
          r->smooth_kernel_width        = smooth_kernel_width;
          r->canny_lo_thresh            = canny_lo_thresh;
          r->canny_hi_thresh            = canny_hi_thresh;
          r->edge_dilate_width          = edge_dilate_width;
          r->show_lands                 = show_lands;
          r->use_pat_rot_up             = use_pat_rot_up;
          r->flip_rows                  = flip_rows;
          r->flip_cols                  = flip_cols;
          r->do_preproc                 = do_preproc;
          r->proj_ds_factor             = proj_ds_factor;
          r->max_tile_cols              = max_tile_cols;
          r->debug_img_prefix           = debug_img_prefix;
          r->debug_img_include_text     = debug_img_include_text;
          r->debug_raw_final_drr_prefix = debug_raw_final_drr_prefix;
        },
        "backend"_a = "",
        nb::kw_only(),
        "compute_boundary_edges"_a = defs.compute_boundary_edges,
        "boundary_edge_hu_thresh"_a = defs.boundary_edge_hu_thresh,
        "compute_canny_edges"_a = defs.compute_canny_edges,
        "smooth_kernel_width"_a = defs.smooth_kernel_width,
        "canny_lo_thresh"_a = defs.canny_lo_thresh,
        "canny_hi_thresh"_a = defs.canny_hi_thresh,
        "edge_dilate_width"_a = defs.edge_dilate_width,
        "show_lands"_a = defs.show_lands,
        "use_pat_rot_up"_a = defs.use_pat_rot_up,
        "flip_rows"_a = defs.flip_rows,
        "flip_cols"_a = defs.flip_cols,
        "do_preproc"_a = defs.do_preproc,
        "proj_ds_factor"_a = defs.proj_ds_factor,
        "max_tile_cols"_a = defs.max_tile_cols,
        "debug_img_prefix"_a = defs.debug_img_prefix,
        "debug_img_include_text"_a = defs.debug_img_include_text,
        "debug_raw_final_drr_prefix"_a = defs.debug_raw_final_drr_prefix,
        "backend: the compute backend (a Backend or a backend string, \"\" -> default backend) "
        "used to create the ray casters. All other arguments are also available as attributes.")
    .def_prop_rw("backend",
        [](const Replay& r) { return r.backend; },
        &SetBackend,
        "The compute backend used to create the ray casters (a Backend or backend string may be assigned).")
    .def_rw("compute_boundary_edges", &Replay::compute_boundary_edges,
            "Compute boundary/silhouette contours of the objects.")
    .def_rw("boundary_edge_hu_thresh", &Replay::boundary_edge_hu_thresh,
            "Threshold, in HU, used to determine the boundary of the objects.")
    .def_rw("compute_canny_edges", &Replay::compute_canny_edges,
            "Compute Canny edges on the DRRs.")
    .def_rw("smooth_kernel_width", &Replay::smooth_kernel_width,
            "The Gaussian kernel width/height used for smoothing an image prior to Canny edge "
            "detection. <= 1 will do no smoothing.")
    .def_rw("canny_lo_thresh", &Replay::canny_lo_thresh,
            "Lower threshold used for Canny edge detection.")
    .def_rw("canny_hi_thresh", &Replay::canny_hi_thresh,
            "Upper threshold used for Canny edge detection.")
    .def_rw("edge_dilate_width", &Replay::edge_dilate_width,
            "Amount of dilation applied to the edge map to make it easier to see. <= 1 will not enlarge.")
    .def_rw("show_lands", &Replay::show_lands,
            "Overlay landmark information from penalty functions (single view only).")
    .def_rw("use_pat_rot_up", &Replay::use_pat_rot_up,
            "Use the metadata of the projection data (when present) indicating the rotation needed "
            "for a patient \"up\" orientation to flip each view; this overrides flip_rows and flip_cols.")
    .def_rw("flip_rows", &Replay::flip_rows, "Flip each image up/down.")
    .def_rw("flip_cols", &Replay::flip_cols, "Flip each image left/right.")
    .def_rw("do_preproc", &Replay::do_preproc,
            "When information about projection pre-processing is stored in the debug data, use it "
            "to apply pre-processing to the projections.")
    .def_rw("proj_ds_factor", &Replay::proj_ds_factor,
            "2D projection downsampling factor from the input fixed image dimensions. "
            "1.0 -> no downsampling. 0.5 -> reduce by half in each dimension.")
    .def_rw("max_tile_cols", &Replay::max_tile_cols,
            "The maximum number of columns in the tiling of views; once exceeded, a new row is made.")
    .def_rw("debug_img_prefix", &Replay::debug_img_prefix,
            "When not empty, debug PNG images are written with paths of the format "
            "<prefix>_{tiled|view index}_{edges|mov}_<frame #>.png. The images of the individual "
            "views are written by run() and the tiled images by write_videos().")
    .def_rw("debug_img_include_text", &Replay::debug_img_include_text,
            "Include the title text in the tiled debug images.")
    .def_rw("debug_raw_final_drr_prefix", &Replay::debug_raw_final_drr_prefix,
            "When not empty, the raw DRRs of each view at the final iteration are written to "
            "<prefix>_<view index>.nii.gz by run().")
    .def("set_verbose", [](Replay& r, const bool v) { SetVerbose(r, v); }, "verbose"_a = true,
         "Print verbose status information to stdout.")
    .def("run", &Run, "debug_info"_a,
         "Computes all of the frames from debug information that is in memory, e.g. the "
         "debug_info of a MultiLevelMultiObjRegi after it has run. Volumes and projections "
         "referenced by path in the debug information are read from disk; relative paths are "
         "relative to the current working directory. The GIL is released while running.")
    .def("run", &RunFromFile, "path"_a,
         "Computes all of the frames from a debug file (HDF5) written by a registration.")
    .def_prop_ro("num_frames", [](const Replay& r) { return r.drr_frames.size(); },
                 "Number of frames computed by run(): one for the initial pose, each iteration, "
                 "and the final pose of each registration at each level.")
    .def_prop_ro("drr_frames", [](const Replay& r) { return FramesToArrays(r.drr_frames); },
                 "The tiled DRR frames: a list of uint8 numpy arrays of shape (rows, cols, 3) in "
                 "RGB order. NOTE: all frames are copied each time this is accessed.")
    .def_prop_ro("edge_frames", [](const Replay& r) { return FramesToArrays(r.edge_frames); },
                 "The tiled fixed images overlaid with edges from the DRRs: a list of uint8 numpy "
                 "arrays of shape (rows, cols, 3) in RGB order. NOTE: all frames are copied each "
                 "time this is accessed.")
    .def_ro("frame_titles", &Replay::frame_titles,
            "Title of each frame, with the level, registration name, and iteration.")
    .def("write_videos",
        [](Replay& r, const std::optional<std::string>& drr_path,
           const std::optional<std::string>& edges_path,
           const std::optional<double>& fps, const double video_len)
        {
          if (r.drr_frames.empty())
          {
            throw nb::value_error("no frames are available, call run() before writing videos");
          }

          const std::string drr_path_str   = drr_path   ? *drr_path   : std::string();
          const std::string edges_path_str = edges_path ? *edges_path : std::string();

          if (drr_path_str.empty() && edges_path_str.empty())
          {
            throw nb::value_error("at least one of drr_path and edges_path must be specified");
          }

          if (fps && !(*fps > 0))
          {
            throw nb::value_error("fps must be positive");
          }

          if (!fps && !(video_len > 0))
          {
            throw nb::value_error("video_len must be positive");
          }

          nb::gil_scoped_release no_gil;

          r.write_videos(drr_path_str, edges_path_str, fps ? *fps : r.fps_for_video_len(video_len));
        },
        "drr_path"_a = nb::none(), "edges_path"_a = nb::none(),
        nb::kw_only(), "fps"_a = nb::none(), "video_len"_a = 30.0,
        "Writes the frames into movies (e.g. .mp4 files), with the title of each frame burned "
        "in. A movie is not written when its path is None. fps: frames per second of the "
        "movies; when None, the frame rate is chosen so that the movies are video_len seconds "
        "long. The ffmpeg program is used for encoding when it is found on the system path, "
        "otherwise a platform specific writer is used (AVFoundation on MacOS, OpenCV elsewhere).");
}

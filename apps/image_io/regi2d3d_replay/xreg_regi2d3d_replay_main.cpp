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

#include <fmt/format.h>

// xreg
#include "xregProgOptUtils.h"
#include "xregStringUtils.h"
#include "xregRayCastProgOpts.h"
#include "xregMultiObjMultiLevel2D3DRegiDebug.h"
#include "xregRegi2D3DReplay.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  const int kEXIT_VAL_SUCCESS          = 0;
  const int kEXIT_VAL_BAD_USE          = 1;
  const int kEXIT_VAL_UNSUPPORTED_DATA = 2;

  // file extension to fourcc for output movies - I have had difficulty using mp4 on Windows 10,
  // however wmv output works nicely
  std::unordered_map<std::string, std::string> ext_to_cc;
  ext_to_cc["mp4"] = "AVC1";  // This works on Mac, I think it's quicktime
#ifdef _WIN32
  ext_to_cc["wmv"] = "WMV2";
#endif

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Replays registration iterations into a movie, or collection of movies.");
  po.set_arg_usage("<Regi Debug> [<edges movie prefix> [<drr movie prefix>]]");
  po.set_min_num_pos_args(1);

  po.add("no-boundary-contours", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-boundary-contours",
         "Do not compute boundary/silhouette contours.")
    << false;

  po.add("boundary-edge-thresh-hu", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "boundary-edge-thresh-hu",
         "Threshold, in HU, for collisions.")
    << 150.0;

  po.add("no-canny-edges", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-canny-edges",
         "Do not compute canny edges on the DRRs.")
    << false;

  po.add("smooth-kernel-width", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "smooth-kernel-width",
         "The Gaussian kernel width/height to be used for smoothing an image prior to edge detection.")
    << ProgOpts::int32(3);

  po.add("canny-lo", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "canny-lo",
         "Lower threshold to be used for Canny edge detection. <= 1 will do no smoothing.")
    << ProgOpts::int32(0);

  po.add("canny-hi", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "canny-hi",
         "Upper threshold to be used for Canny edge detection.")
    << ProgOpts::int32(48);

  po.add("lands", 'l', ProgOpts::kSTORE_TRUE, "lands",
         "Overlay landmark information from penalty functions.")
    << false;

  po.add("no-pat-rot-up", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-pat-rot-up",
         "Ignore any flags for rotating the image to achieve patient \"up\" orientation. ")
    << false;

  po.add("flip-rows", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "flip-rows",
         "Flip each image up/down. "
         "Unless the \"no-pat-rot-up\" flag is passed, this flag is automatically "
         "overriden based on metadata within the projection file indicating patient "
         "up orientation (if the metadata is present).")
    << false;

  po.add("flip-cols", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "flip-cols",
         "Flip each image left/right. "
         "Unless the \"no-pat-rot-up\" flag is passed, this flag is automatically "
         "overriden based on metadata within the projection file indicating patient "
         "up orientation (if the metadata is present).")
    << false;

  po.add("pre-proc", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "pre-proc",
         "If any information about projection pre-processing is stored in the debug data, "
         "use it to apply pre-processing to the projections.")
    << false;

  po.add("video-len", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "video-len",
         "The length of the generative videos in seconds.")
    << 30.0;

  po.add("video-fps", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "video-fps",
         "Frames per second of the output movies - when specified to non-zero this "
         "overrides the video length flag.");

  po.add("edge-dilate-width", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "edge-dilate-width",
         "Amount of dilation to apply to the edge map to make it easier to see. <= 1 will not enlarge.")
    << ProgOpts::int32(3);

  po.add("proj-ds", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "proj-ds",
         "2D projection downsampling factor from the input fixed image dimensions. "
         "1.0 -> no downsampling. 0.5 -> reduce by half in each dimension.")
    << 1.0;

  po.add("max-tile-cols", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "max-tile-cols",
         "The maximum number of columns in the output tiling, once the maximum number of tiles "
         "is exceeded, a new row is made.")
    << ProgOpts::int32(7);

  {
    std::vector<std::string> exts;
    for (const auto& ext_cc_pair : ext_to_cc)
    {
      exts.push_back(ext_cc_pair.first);
    }

    std::stringstream ss;
    ss << "Filename extension (movie format) to use when writing movies. "
          "Supported extensions are: "
       << JoinTokens(exts, ",");

    po.add("mov-ext", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "mov-ext",
           ss.str())
      << "mp4";
  }

  po.add_backend_flags();

  po.add("debug-img-prefix", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "debug-img-prefix",
         "Prefix to write debug PNG images out, the format will be <prefix>_{tiled|proj index}_{edge|mov}_<movie frame #>.png")
    << "";
  
  po.add("debug-img-include-text", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "debug-img-include-text",
         "Include the text burn-in (e.g. with iteration info, regi. name, etc.) for the tiled debug images.")
    << false;

  po.add("debug-raw-final-drr-prefix", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "debug-raw-final-drr-prefix",
         "Prefix to write out the raw DRRs for each view at the final iteration. "
         "Useful for debugging. File path with be <prefix>_<view index>.nii.gz")
    << "";

  try
  {
    po.parse(argc, argv);
  }
  catch (const ProgOpts::Exception& e)
  {
    std::cerr << "Error parsing command line arguments: " << e.what() << std::endl;
    po.print_usage(std::cerr);
    return kEXIT_VAL_BAD_USE;
  }

  if (po.help_set())
  {
    po.print_usage(std::cout);
    po.print_help(std::cout);
    return kEXIT_VAL_SUCCESS;
  }

  std::ostream& vout = po.vout();
  
  const bool verbose = po.get("verbose");

  const size_type num_cmd_args = po.pos_args().size();

  const std::string edges_mov_prefix = (num_cmd_args > 1) ? po.pos_args()[1] : std::string("edges");
  const std::string drr_mov_prefix   = (num_cmd_args > 2) ? po.pos_args()[2] : std::string("mov");

  const std::string regi_results_path = po.pos_args()[0];
  
  const bool no_pat_rot_up = po.get("no-pat-rot-up");

  const bool flip_rows_flag = po.get("flip-rows");
  const bool flip_cols_flag = po.get("flip-cols");

  const bool do_preproc = po.get("pre-proc");

  const bool compute_boundary_edges = !po.get("no-boundary-contours");
  const double boundary_edge_hu_thresh = po.get("boundary-edge-thresh-hu");

  const bool compute_canny_edges = !po.get("no-canny-edges");

  const int smooth_kernel_width = po.get("smooth-kernel-width").as_int32();
  const int canny_lo_thresh = po.get("canny-lo").as_uint32();
  const int canny_hi_thresh = po.get("canny-hi").as_uint32();

  const bool show_lands = po.get("lands");

  const double video_len_secs = po.get("video-len");

  const bool has_fps = po.has("video-fps");
  const double video_fps_arg = has_fps ? po.get("video-fps").as_uint32() : ProgOpts::uint32(0);

  const double ds_factor = po.get("proj-ds");

  const int edge_dilate_width = po.get("edge-dilate-width").as_uint32();

  if (!compute_canny_edges && !compute_boundary_edges)
  {
    std::cerr << "WARNING: NO EDGE OVERLAYS WILL BE ADDED!" << std::endl;
  }

  const int max_tile_cols = po.get("max-tile-cols").as_int32();

  const std::string mov_ext_str = po.get("mov-ext").as_string();

  auto ext_to_cc_it = ext_to_cc.find(mov_ext_str);
  if (ext_to_cc_it == ext_to_cc.end())
  {
    std::cerr << "ERROR: invalid movie extension provided!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const std::string fourcc_str = ext_to_cc_it->second;

  const std::string debug_frame_prefix       = po.get("debug-img-prefix");
  const bool        write_txt_in_debug_tiles = po.get("debug-img-include-text");

  const std::string debug_raw_final_drr_prefix = po.get("debug-raw-final-drr-prefix");

  vout << "reading registration results from file..." << std::endl;
  const auto regi_results = ReadMultiLevel2D3DRegiDebugFromDisk(regi_results_path);

  // The replay logic is implemented in the library, so that it may also be
  // used by other programs (e.g. the Python bindings).
  Regi2D3DReplay replay;
  replay.set_debug_output_stream(vout, verbose);

  replay.line_int_ray_caster_fn = [&po] () { return LineIntRayCasterFromProgOpts(po); };
  replay.depth_ray_caster_fn    = [&po] () { return DepthRayCasterFromProgOpts(po); };

  replay.compute_boundary_edges  = compute_boundary_edges;
  replay.boundary_edge_hu_thresh = boundary_edge_hu_thresh;
  replay.compute_canny_edges     = compute_canny_edges;
  replay.smooth_kernel_width     = smooth_kernel_width;
  replay.canny_lo_thresh         = canny_lo_thresh;
  replay.canny_hi_thresh         = canny_hi_thresh;
  replay.edge_dilate_width       = edge_dilate_width;
  replay.show_lands              = show_lands;
  replay.use_pat_rot_up          = !no_pat_rot_up;
  replay.flip_rows               = flip_rows_flag;
  replay.flip_cols               = flip_cols_flag;
  replay.do_preproc              = do_preproc;
  replay.proj_ds_factor          = ds_factor;
  replay.max_tile_cols           = max_tile_cols;

  replay.debug_img_prefix           = debug_frame_prefix;
  replay.debug_img_include_text     = write_txt_in_debug_tiles;
  replay.debug_raw_final_drr_prefix = debug_raw_final_drr_prefix;

  try
  {
    replay.run(regi_results);
  }
  catch (const Regi2D3DReplay::UnsupportedDataException& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return kEXIT_VAL_UNSUPPORTED_DATA;
  }

  const double video_fps = has_fps ? video_fps_arg : replay.fps_for_video_len(video_len_secs);

  const std::string mov_path   = fmt::format("{}.{}", drr_mov_prefix, mov_ext_str);
  const std::string edges_path = fmt::format("{}.{}", edges_mov_prefix, mov_ext_str);

  replay.write_videos(mov_path, edges_path, video_fps);

  vout << "exiting..." << std::endl;

  return kEXIT_VAL_SUCCESS;
}

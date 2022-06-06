/*
 * MIT License
 *
 * Copyright (c) 2020-2022 Robert Grupp
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

#include <vtkCamera.h>

#include <opencv2/imgcodecs.hpp>

#include "xregProgOptUtils.h"
#include "xregFilesystemUtils.h"
#include "xregPerspectiveXform.h"
#include "xregStringUtils.h"
#include "xregCSVUtils.h"
#include "xregAssert.h"
#include "xregITKRemapUtils.h"
#include "xregITKIOUtils.h"
#include "xregVTK3DPlotter.h"
#include "xregH5ProjDataIO.h"
#include "xregMeshIO.h"
#include "xregLandmarkFiles.h"
#include "xregAnatCoordFrames.h"
#include "xregRigidUtils.h"
#include "xregSampleUtils.h"
#include "xregMeshIO.h"
#include "xregPointCloudUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;

  const int kEXIT_VAL_SUCCESS  = 0;
  const int kEXIT_VAL_BAD_USE  = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Visualizes an X-Ray scene. Displays the X-Ray images with "
              "camera geometries in 3D space, along with an optional surface "
              "meshes or point clouds. The pose of a provided surface should "
              "map X-Ray/C-Arm world coordinates to the surface coordinates. "
              "If \"-\" is passed as the pose path, then the identity transform is used. "
              "The following hot keys are available: "
              "q - quit, "
              "s - save screen shot to current directory, "
              "a - toggle axes display, "
              "f - show FPS, "
              "h - toggle hiding/showing all overlays, and "
              "c - print the current view geometry to stdout "
              "(useful for starting back up with the same view).");
  po.set_arg_usage("<Proj. Data File> "
                   "[<surface mesh or points #1> <surface mesh or points #1 pose>, ... "
                   "[<surface mesh or points #N> <surface mesh or points #N pose>]]");
  po.set_min_num_pos_args(1);

  po.add("show-image", 'i', ProgOpts::kSTORE_TRUE, "show-image",
         "Shows each 2D image overlaid on the detector.")
     << false;

  po.add("projs", 'p', ProgOpts::kSTORE_STRING, "projs",
         "Comma delimited list of zero-based projection indices and ranges to "
         "visualize. \"\" --> use all projections.")
    << "";

  po.add("axes", 'a', ProgOpts::kSTORE_TRUE, "axes", "Show axes.") << false;

  po.add("geom-colors-csv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "geom-colors-csv",
         "Path to a CSV file that defines the colors to use for the geometry of each projection "
         "specified on the command line. The first row corresponds to the RGBA "
         "used to color the first mesh file, second row is for "
         "the second mesh, and so on. When less colors than mesh files have been "
         "specified, then the colors cycle.")
    << "";

  po.add("color-grad", 'c', ProgOpts::kSTORE_TRUE, "color-grad",
         "Change the geometry color from red to green, for increasing projection index.")
     << false;

  po.add("det-corners", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "det-corners",
         "Draw the detector corners (easier to debug). "
         "Yellow sphere -> (image index 0,0), "
         "Orange sphere -> (image index nc-1,0), "
         "Purple sphere -> (image index 0, nr-1), "
         "Cyan sphere -> (image index nc-1,nr-1)")
     << false;

  po.add("title", 't', ProgOpts::kSTORE_STRING, "title",
         "Sets the window title string, default is to use the proj. data file name.")
     << "";

  po.add("mesh-color-bone", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "mesh-color-bone",
         "Use a whitish/bone-ish color for surfaces.")
    << false;

  po.add("rand-mesh-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "rand-mesh-color",
         "Assigns random colors to each mesh that is drawn.")
    << false;

  po.add("mesh-alpha", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "mesh-alpha",
         "Alpha value used for all surface meshes, only one alpha surface mesh should be "
         "supplied per invocation. -1 --> use default value or one of the other alpha flags.")
    << -1.0;

  po.add("dist-mesh-alpha", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "dist-mesh-alpha",
         "Set the alpha component of each mesh to 1 / N, where N is the number of surface "
         "meshes. Only one alpha surface mesh should be supplied per invocation.")
    << false;
  
  po.add("mesh-colors-csv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "mesh-colors-csv",
         "Path to a CSV file that defines the colors to use for each mesh file "
         "specified on the command line. The first row corresponds to the RGBA "
         "used to color the first mesh file, second row is for "
         "the second mesh, and so on. When less colors than mesh files have been "
         "specified, then the colors cycle.")
    << "";

  po.add("landmarks", 'l', ProgOpts::kSTORE_TRUE, "landmarks",
         "Draw 2D landmarks")
    << false;

  po.add("single-land", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "single-land",
         "Only draw landmarks with the specified name/string.")
    << "";

  po.add("draw-landmark-lines", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "draw-landmark-lines",
         "Draw lines from the source to detector intersecting each landmark.")
    << false;

  po.add("no-src-det-line", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-src-det-line",
         "Do NOT draw the source to detector principal point line (useful when there are many other lines).")
    << false;;

  po.add("draw-origin", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "draw-origin",
         "Draw a sphere at the origin of the extrinsic coordinate frame.")
    << false;

  po.add("save-img", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "save-img",
         "Path to save to image file, instead of showing interactive GUI.")
    << "";

  po.add("azimuth", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "azimuth",
         "Custom azimuth value for initial view of scene (degrees).");

  po.add("elevation", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "elevation",
         "Custom elevation value for initial view of scene (degrees).");

  auto& view_up_arg = po.add("view-up", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "view-up",
                             "View-up vector to use for initial view.");
  view_up_arg.num_vals = 3;
  view_up_arg.default_vals.assign(3, ProgOpts::ArgVal(0.0));

  auto& pos_arg = po.add("position", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "position",
                         "The position to use for initial view.");
  pos_arg.num_vals = 3;
  pos_arg.default_vals.assign(3, ProgOpts::ArgVal(0.0));

  po.add("app-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "app-lands",
         "Path to a landmarks (e.g. FCSV or Slicer JSON) file with landmarks used to compute the anterior "
         "pelvic plane (APP). This is required if some variation of an AP, lateral, or oblique pelvis view "
         "is specified.")
    << "";

  po.add("pelvis-view", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pelvis-view",
         "Render from an anatomical view of the pelvis, valid options are: "
         "\"ap,\" \"lat-r,\" \"lat-l,\" \"oblique-r,\" \"oblique-l,\" \"is-<rot deg>.\"")
    << "";

  po.add("no-cam", 'n', ProgOpts::kSTORE_TRUE, "no-cam",
         "Do not draw ANY of the camera geometry - the detectors, the sources, "
         "source-detector lines, etc.. This is useful for show registered surfaces, "
         "without the clutter of the camera/c-arm objects.")
    << false;

  po.add("pts-as-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "pts-as-ras",
         "Any 3D point clouds supplied as positional arguments should be populated in RAS coordinates, "
         "otherwise they are populated in LPS coordinates.")
    << false;

  po.add("pts-rad", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "pts-rad",
         "Radius of the spheres to draw for any 3D point clouds. "
         "Non-positive values skip drawing the spheres.")
    << 3.0;
  
  po.add("pts-rad-txt", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pts-rad-txt",
         "Text file where each line defines the sphere radius to draw for a specific landmarks file. "
         "Line 1 is associated with file 1, line 2 with file 2, and so on. "
         "Overrides \"pts-rad\" when supplied.")
    << "";

  {
    auto& fcsv_color_arg = po.add("pts-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "pts-color",
                                  "Color used for all 3D point spheres (no alpha component, only RGB).");
    fcsv_color_arg.nvals(3);
    fcsv_color_arg << 1.0 << 0.0 << 0.0;
  }

  po.add("pts-colors-csv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pts-colors-csv",
         "Path to a CSV file that defines the colors to use for each landmarks file "
         "specified on the command line. The first row corresponds to the RGBA "
         "used to color all 3D points in the first landmarks file, second row is for "
         "the second landmarks file, and so on. When less colors than landmark files have been "
         "specified, then the colors cycle.")
    << "";

  po.add("pts-line-colors-csv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pts-line-colors-csv",
         "Path to a CSV file defining the colors to use for source-to-detector "
         "lines intersecting landmark 3D points. Same format as \"pts-colors-csv.\" "
         "Overrides \"land-line-color.\"")
    << "";

  po.add("pts-line-projs-txt", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pts-line-projs-txt",
         "Path to a text file that defines which projections source-to-detector lines should be drawn "
         "for each landmarks file. Each line specifies a set of projection indices; first line for the first "
         "landmarks file, second line for the second landmarks file, and so forth. If not provided, all "
         "projections that are visualized are used. The formatting on each line is identical to the "
         "\"projs\" (\'p\') flag. Similar to the \"pts-\" color flags, the projection associations cycle "
         "when there are more landmark files than the number of lines in this file.")
    << "";

  {
    auto& land_line_color_arg = po.add("land-line-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
                                       "land-line-color", "Color used for landmark lines.");
    land_line_color_arg.nvals(3);
    land_line_color_arg << 0.0 << 1.0 << 0.0;
  }

  po.add("rand-land-line-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "rand-land-line-color",
         "Randomize landmark line colors; overrides any specified color")
    << false;

  po.add("line-width", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "line-width",
         "Width/thickness to use when drawing lines.")
    << 1.0;

  po.add("xray-src-rad", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "xray-src-rad",
         "Radius of the sphere drawn at the X-Ray source positions.")
    << 5.0;
  
  {
    auto& bg_color_arg = po.add("bg-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "bg-color",
                                "Color of the rendered background.");
    bg_color_arg.default_vals.resize(3);
    bg_color_arg.num_vals = 3;
  }

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
  
  const size_type num_surfaces = (po.pos_args().size() - 1) / 2;
  vout << "number of surface/transform pairs specified on command line: " << num_surfaces << std::endl;

  const bool show_imgs = po.get("show-image");

  const std::string projs_str = po.get("projs");

  const bool show_axes = po.get("axes");

  const std::string geom_color_csv_path = po.get("geom-colors-csv");

  const bool use_geom_color_csv = !geom_color_csv_path.empty();

  const bool do_color_grad = po.get("color-grad");

  if (use_geom_color_csv && do_color_grad)
  {
    std::cerr << "ERROR: cannot use both geometry color CSV and color gradient!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const bool show_det_corners = po.get("det-corners");

  const std::string custom_title_str = po.get("title").as_string();

  const bool mesh_color_bone = po.get("mesh-color-bone");

  const bool rand_mesh_color = po.get("rand-mesh-color");
  
  const std::string mesh_color_csv_path = po.get("mesh-colors-csv");
  
  const bool use_mesh_color_csv = !mesh_color_csv_path.empty();

  if ((static_cast<int>(mesh_color_bone) +
       static_cast<int>(rand_mesh_color) +
       static_cast<int>(use_mesh_color_csv)) > 1)
  {
    std::cerr << "WARNING: More than one mesh color option passed;"
                 " order of precedence is: CSV, bone color, random." << std::endl;
  }

  const bool dist_mesh_alpha = po.get("dist-mesh-alpha");
  const double specified_mesh_alpha = po.get("mesh-alpha");
  
  const bool mesh_alpha_provided = specified_mesh_alpha >= 0; 

  if (dist_mesh_alpha && mesh_alpha_provided)
  {
    std::cerr << "ERROR: CANNOT SPECIFY BOTH MESH ALPHA FLAGS!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const bool draw_landmarks = po.get("landmarks");

  const std::string single_land_str = po.get("single-land");

  const bool draw_landmark_lines = po.get("draw-landmark-lines");

  const bool no_src_det_line = po.get("no-src-det-line");

  const bool draw_origin = po.get("draw-origin");

  const std::string dst_img_path = po.get("save-img");

  const bool has_azimuth = po.has("azimuth");
  const bool has_elev    = po.has("elevation");

  const bool has_az_or_el = has_azimuth || has_elev;

  const bool has_view_up = po.has("view-up");
  const bool has_pos     = po.has("position");

  const bool has_view_up_or_pos = has_view_up || has_pos;

  const std::string pelvis_view_str = ToLowerCase(po.get("pelvis-view").as_string());
  
  const bool has_pelvis_view_arg = !pelvis_view_str.empty();

  const std::string app_lands_fcsv_path = po.get("app-lands");

  if ((static_cast<int>(has_az_or_el) + static_cast<int>(has_view_up_or_pos) +
       static_cast<int>(has_pelvis_view_arg)) > 1)
  {
    std::cerr << "Can only specify one combination of azimuth/elevation, or view-up/position, or pelvis-view!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  if (has_pelvis_view_arg && app_lands_fcsv_path.empty())
  {
    std::cerr << "Must specify landmarks file for pelvis view!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const bool no_cam = po.get("no-cam");

  const bool fcsv_as_ras = po.get("pts-as-ras");

  std::vector<double> fcsv_rads;

  const std::string fcsv_rad_txt_path = po.get("pts-rad-txt");
  
  if (fcsv_rad_txt_path.empty())
  {
    fcsv_rads.assign(1, po.get("pts-rad").as_double());
  }
  else
  {
    std::ifstream fin(fcsv_rad_txt_path);
    
    const auto lines = GetNonEmptyLinesFromStream(fin);

    for (const auto& l : lines)
    {
      fcsv_rads.push_back(StringCast<double>(l));
    }
  }

  const size_type num_fcsv_rads = fcsv_rads.size();
  vout << "number of points radii: " << num_fcsv_rads << std::endl;

  const std::string fcsv_color_csv_path = po.get("pts-colors-csv");
  
  const std::string fcsv_line_colors_csv_path = po.get("pts-line-colors-csv");

  std::vector<VTK3DPlotter::RGBAVec> geom_colors;
  std::vector<VTK3DPlotter::RGBAVec> mesh_colors;
  std::vector<VTK3DPlotter::RGBAVec> fcsv_colors;
  std::vector<VTK3DPlotter::RGBAVec> fcsv_line_colors;
  
  VTK3DPlotter::RGBVec landmark_line_color;

  {
    auto read_csv_colors = [] (const std::string& csv_path)
    {
      const CSVFileFloatValued csv_rows = ReadCSVFileFloat(csv_path, false);
     
      std::vector<VTK3DPlotter::RGBAVec> colors;
      colors.reserve(csv_rows.size());

      for (const auto& csv_row : csv_rows)
      {
        colors.push_back(VTK3DPlotter::RGBAVec{ csv_row[0], csv_row[1], csv_row[2], csv_row[3] });
      }
      
      return colors;
    };
    
    std::vector<double> tmp_color(3);

    if (use_geom_color_csv)
    {
      geom_colors = read_csv_colors(geom_color_csv_path);
    }
    else
    {
      geom_colors = { VTK3DPlotter::RGBAVec{0, 1, 0, 1} };
    }

    if (use_mesh_color_csv)
    {
      mesh_colors = read_csv_colors(mesh_color_csv_path);
    }
    else if (mesh_color_bone)
    {
      mesh_colors = { { 1.0, 0.9922, 0.8980, 1.0 } };
    }
    else
    {
      mesh_colors = { VTK3DPlotter::DefaultMeshColorAlpha() } ;
    }

    if (mesh_alpha_provided || dist_mesh_alpha)
    {
      if (mesh_alpha_provided)
      {
        for (auto& c : mesh_colors)
        {
          c[3] = specified_mesh_alpha;
        }
      }
      else
      {
        for (auto& c : mesh_colors)
        {
          c[3] = 1.0 / num_surfaces;
        }
      }
    }

    if (fcsv_color_csv_path.empty())
    {
      po.get_vals("pts-color", &tmp_color);
      
      fcsv_colors.resize(1);
      fcsv_colors[0][0] = tmp_color[0];
      fcsv_colors[0][1] = tmp_color[1];
      fcsv_colors[0][2] = tmp_color[2];
      fcsv_colors[0][3] = 1;
    }
    else
    {
      fcsv_colors = read_csv_colors(fcsv_color_csv_path);
    }
  
    po.get_vals("land-line-color", &tmp_color);
    landmark_line_color[0] = tmp_color[0];
    landmark_line_color[1] = tmp_color[1];
    landmark_line_color[2] = tmp_color[2];

    if (fcsv_line_colors_csv_path.empty())
    {
      fcsv_line_colors.resize(1);
      fcsv_line_colors[0].head(3) = landmark_line_color;
      fcsv_line_colors[0][3] = 1;
    }
    else
    {
      fcsv_line_colors = read_csv_colors(fcsv_line_colors_csv_path);
    }
  }

  const size_type num_fcsv_colors = fcsv_colors.size();
  vout << "number of FCSV colors specified: " << num_fcsv_colors << std::endl;

  const size_type num_fcsv_line_colors = fcsv_line_colors.size();
  vout << "number of FCSV line colors specified: " << num_fcsv_colors << std::endl;

  const bool rand_land_line_color = po.get("rand-land-line-color");

  const double line_width = po.get("line-width");

  const double xray_src_rad = po.get("xray-src-rad");
  
  std::vector<double> bg_color;
  if (po.has("bg-color"))
  {
    bg_color.resize(3);
    po.get_vals("bg-color", &bg_color);
  }

  FrameTransform pelvis_view_pose = FrameTransform::Identity();

  if (has_pelvis_view_arg)
  {
    vout << "will use a custom pelvis view" << std::endl;

    vout << "reading landmarks to compute APP (wrt LPS)..." << std::endl;
    xregASSERT(!app_lands_fcsv_path.empty());
    auto fcsv_lands = ReadLandmarksFileNamePtMap(app_lands_fcsv_path, true);
    
    vout << "compute app --> pelvis vol..." << std::endl;
    const FrameTransform app_to_vol = AnteriorPelvicPlaneFromLandmarksMap(fcsv_lands);

    vout << "deriving AP orientation..." << std::endl;
    CameraModel tmp_cam;
    tmp_cam.setup(1200, 768, 768, 0.388, 0.388);

    const auto tmp_cam_ap_to_app = CreateAPViewOfAPP(tmp_cam, CoordScalar(0.5), false);

    if (pelvis_view_str == "ap")
    {
      vout << "AP view selected" << std::endl;
      pelvis_view_pose = app_to_vol * tmp_cam_ap_to_app;
    }
    else
    {
      const auto toks = StringSplit(pelvis_view_str, "-");
      
      if (toks.size() == 2)
      {
        const bool is_lat = toks[0] == "lat";
        const bool is_obl = toks[0] == "oblique";
       
        if (is_lat || is_obl)
        {
          bool is_left = toks[1] == "l";

          xregASSERT(is_left || (toks[1] == "r"));

          CoordScalar rot_ang = 0;

          if (is_lat)
          {
            rot_ang = 90 * kDEG2RAD;
          }
          else if (is_obl)
          {
            rot_ang = 45 * kDEG2RAD;
          }
          
          pelvis_view_pose = app_to_vol * 
                             EulerRotYFrame((is_left ? 1.0 : -1.0) * rot_ang) *
                             tmp_cam_ap_to_app;
        }
        else if (toks[0] == "is")
        {
          pelvis_view_pose = app_to_vol * 
                             EulerRotYFrame(StringCast<CoordScalar>(toks[1]) * kDEG2RAD) *
                             tmp_cam_ap_to_app;
        }
        else
        {
          xregThrow("Invalid pelvis pose string: %s", pelvis_view_str.c_str());
        }

      }
      else
      {
        xregThrow("bad pelvis view string: %s", pelvis_view_str.c_str());
      }
    }
  }

  const std::string fcsv_line_projs_txt_path = po.get("pts-line-projs-txt");

  const std::string pd_path = po.pos_args()[0];

  DeferredProjReader proj_reader(pd_path, false);  // false --> no caching of read projs

  auto& proj_datas = proj_reader.proj_data_F32();

  const size_type num_src_projs = proj_datas.size();
  vout << "number of cameras/projections in file: " << num_src_projs << std::endl;
  
  std::vector<long> projs_to_use;
  
  if (projs_str.empty())
  {
    projs_to_use.resize(num_src_projs);
    std::iota(projs_to_use.begin(), projs_to_use.end(), 0);
  }
  else
  {
    projs_to_use = ParseCSVRangeOfInts(projs_str);
  }

  const size_type num_projs_to_draw = projs_to_use.size();
  vout << "number of cameras/projections to draw: " << num_projs_to_draw << std::endl;

  std::vector<std::vector<long>> fcsv_line_projs;
  if (fcsv_line_projs_txt_path.empty())
  {
    fcsv_line_projs.assign(1, projs_to_use);
  }
  else
  {
    std::ifstream fin(fcsv_line_projs_txt_path);

    const auto lines = GetNonEmptyLinesFromStream(fin);
    
    for (const auto& l : lines)
    {
      fcsv_line_projs.push_back(ParseCSVRangeOfInts(l));
    }
  }

  const size_type num_fcsv_line_proj_assocs = fcsv_line_projs.size();
  vout << "number of points line/projection associations: " << num_fcsv_line_proj_assocs << std::endl;

  std::mt19937 rng_eng;
  SeedRNGEngWithRandDev(&rng_eng);

  std::uniform_real_distribution<VTK3DPlotter::Scalar> uni_dist_01(0,1);

  VTK3DPlotter plotter;

  if (!bg_color.empty())
  {
    plotter.set_bg_color(bg_color[0], bg_color[1], bg_color[2]);
  }

  const size_type num_geom_colors = geom_colors.size();

  if (!no_cam && num_projs_to_draw)
  {
    for (size_type proj_idx = 0; proj_idx < num_projs_to_draw; ++proj_idx)
    {
      const size_type src_proj_idx = static_cast<size_type>(projs_to_use[proj_idx]);
      xregASSERT(src_proj_idx < num_src_projs);

      auto cur_geom_color = geom_colors[proj_idx % num_geom_colors];

      if (do_color_grad)
      {
        cur_geom_color[0] = 1.0 - (static_cast<double>(src_proj_idx) / num_src_projs);
        cur_geom_color[1] = static_cast<double>(src_proj_idx) / num_src_projs;
        cur_geom_color[2] = 0;
      }

      auto& cam = proj_datas[src_proj_idx].cam;

      // Draw a sphere at the source
      if (xray_src_rad > 1.0e-3)
      {
        plotter.add_sphere(cam.pinhole_pt, xray_src_rad, cur_geom_color);
      }

      if (!no_src_det_line)
      {
        // Draw a line connecting the source to the principle point of the image/detector
        const Pt3 principle_pt_img_ind = cam.intrins.block(0,2,3,1);
        
        plotter.add_line(cam.pinhole_pt, cam.ind_pt_to_phys_det_pt(principle_pt_img_ind),
                         cur_geom_color, line_width);
      }

      // draw the detector

      const size_type num_rows = cam.num_det_rows;
      const size_type num_cols = cam.num_det_cols;

      const Pt3 det_r0_c0 = cam.ind_pt_to_phys_det_pt(Pt2{0,            0});
      const Pt3 det_rn_c0 = cam.ind_pt_to_phys_det_pt(Pt2{0,            num_rows - 1});
      const Pt3 det_r0_cm = cam.ind_pt_to_phys_det_pt(Pt2{num_cols - 1, 0});
      const Pt3 det_rn_cm = cam.ind_pt_to_phys_det_pt(Pt2{num_cols - 1, num_rows - 1});

      if (show_det_corners)
      {
        plotter.add_sphere(det_r0_c0, 5, 1, 1, 0);                      // yellow
        plotter.add_sphere(det_r0_cm, 5, 1, 146.0/255.0, 20.0/255.0);   // orange
        plotter.add_sphere(det_rn_c0, 5, 116.0/255.0, 0, 189.0/255.0);  // purple
        plotter.add_sphere(det_rn_cm, 5, 46.0/255.0, 238.0/255.0, 1);   // cyan
      }

      if (show_imgs)
      {
        vout << "reading image and remapping to 8bpp..." << std::endl;
        auto uint8_img = ITKImageRemap8bpp(proj_reader.read_proj_F32(src_proj_idx).GetPointer());

        // The VTK flipping causes a new buffer to be allocated internally,
        // do not have to worry about uint8_img destructing
        vout << "  plotting remapped image as detector..." << std::endl;
        plotter.add_2D_image(uint8_img.GetPointer(), cam);
      }
      else
      {
        plotter.add_plane(det_r0_c0, det_r0_cm, det_rn_c0, cur_geom_color);
      }

      if (draw_landmarks)
      {
        const bool single_land = !single_land_str.empty();

        for (const auto& landmark_name_pt_pair : proj_datas[src_proj_idx].landmarks)
        {
          if (!single_land || (single_land_str == landmark_name_pt_pair.first))
          {
            const auto landmark_wrt_cam_ext = cam.ind_pt_to_phys_det_pt(landmark_name_pt_pair.second);

            plotter.add_sphere(landmark_wrt_cam_ext, 3, 0, 1, 0);

            if (draw_landmark_lines)
            {
              VTK3DPlotter::RGBVec cur_line_color = landmark_line_color;
              if (rand_land_line_color)
              {
                cur_line_color[0] = uni_dist_01(rng_eng);
                cur_line_color[1] = uni_dist_01(rng_eng);
                cur_line_color[2] = uni_dist_01(rng_eng);
              }

              plotter.add_line(cam.pinhole_pt, landmark_wrt_cam_ext, cur_line_color, line_width); 
            }
          }
        }
      }
    }
  }  // end if num_projs

  if (draw_origin)
  {
    vout << "drawing extrinsic origin..." << std::endl;
    plotter.add_sphere(Pt3::Zero(), 5, 1, 1, 0);
  }

  // draw surfaces and points
    
  const size_type num_mesh_colors = mesh_colors.size();
  
  size_type sur_count  = 0;
  size_type fcsv_count = 0;

  for (size_type sur_idx = 0; sur_idx < num_surfaces; ++sur_idx)
  {
    const size_type off = 1 + (2 * sur_idx);

    const std::string next_sur_or_pts_path = po.pos_args()[off];

    vout << "reading transform " << sur_idx << "..." << std::endl;
    FrameTransform xform_cam_wrt_ct;

    const std::string xform_path = po.pos_args()[off + 1];
    if (xform_path == "-")
    {
      xform_cam_wrt_ct.setIdentity();
    }
    else
    {
      xform_cam_wrt_ct = ReadITKAffineTransformFromFile(xform_path);
    }

    // compute color
    VTK3DPlotter::RGBAVec mesh_color = mesh_colors[sur_count % num_mesh_colors];
    
    if (rand_mesh_color)
    {
      mesh_color[0] = uni_dist_01(rng_eng);
      mesh_color[1] = uni_dist_01(rng_eng);
      mesh_color[2] = uni_dist_01(rng_eng);
    }

    if (!IsSupportedLandmarksFilePts(next_sur_or_pts_path))
    {
      vout << "reading surface " << sur_idx << "..." << std::endl;
      auto m = ReadMeshFromDisk(next_sur_or_pts_path);

      vout << "transforming surface..." << std::endl;
      ApplyTransform(xform_cam_wrt_ct.inverse(), m.vertices, &m.vertices);
      m.normals_valid = false;

      vout << "plotting surface..." << std::endl;
      
      plotter.add_mesh(m, mesh_color);
    
      ++sur_count;
    }
    else
    {
      auto pts_3d = ReadLandmarksFilePts(next_sur_or_pts_path, !fcsv_as_ras);

      ApplyTransform(xform_cam_wrt_ct.inverse(), pts_3d, &pts_3d);
    
      const double fcsv_rad = fcsv_rads[fcsv_count % num_fcsv_rads];
      
      const auto& cur_proj_inds_to_use = fcsv_line_projs[fcsv_count % num_fcsv_line_proj_assocs];
      
      VTK3DPlotter::RGBAVec cur_line_color = fcsv_line_colors[fcsv_count % num_fcsv_line_colors];

      for (const auto& p : pts_3d)
      {
        if (fcsv_rad > 1.0e-3)
        {
          plotter.add_sphere(p, fcsv_rad, fcsv_colors[fcsv_count % num_fcsv_colors]);
        }
      
        if (draw_landmark_lines)
        {
          for (const auto& src_proj_idx : cur_proj_inds_to_use)
          {
            xregASSERT(static_cast<size_type>(src_proj_idx) < num_src_projs);
            
            if (rand_land_line_color)
            {
              cur_line_color[0] = uni_dist_01(rng_eng);
              cur_line_color[1] = uni_dist_01(rng_eng);
              cur_line_color[2] = uni_dist_01(rng_eng);
              cur_line_color[3] = 1;
            }
            
            plotter.add_line(proj_datas[src_proj_idx].cam.pinhole_pt,
                             proj_datas[src_proj_idx].cam.proj_pt_to_det_pt(p),
                             cur_line_color, line_width);
          }
        }
      }
      
      ++fcsv_count;
    }
  }


  plotter.set_show_axes(show_axes);

  if (custom_title_str.empty())
  {
    plotter.set_title(Path(pd_path).filename().string());
  }
  else
  {
    plotter.set_title(custom_title_str);
  }

  if (has_pelvis_view_arg)
  {
    // NOTE: this will mess up the axes if they're plotted
    plotter.set_cam_pose(pelvis_view_pose);
  }
  else if (has_az_or_el)
  {
    if (has_azimuth)
    {
      plotter.vtk_cam()->Azimuth(po.get("azimuth"));
    }

    if (has_elev)
    {
      plotter.vtk_cam()->Elevation(po.get("elevation"));
    }

    plotter.vtk_cam()->OrthogonalizeViewUp();
  }
  else if (has_view_up_or_pos)
  {
    if (has_view_up)
    {
      std::vector<double> view_up(3);
      po.get_vals("view-up", &view_up);
      plotter.vtk_cam()->SetViewUp(&view_up[0]);
    }

    if (has_pos)
    {
      std::vector<double> pos(3);
      po.get_vals("position", &pos);
      plotter.vtk_cam()->SetPosition(&pos[0]);
    }
  }

  if (dst_img_path.empty())
  {
    vout << "showing interactive GUI" << std::endl;
    plotter.show();
  }
  else
  {
    vout << "rendering to image and writing to disk..." << std::endl;
    cv::imwrite(dst_img_path, plotter.draw_to_ocv());
  }

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}

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

#include "xregProgOptUtils.h"
#include "xregPAODrawBones.h"
#include "xregITKIOUtils.h"
#include "xregLandmarkFiles.h"
#include "xregAnatCoordFrames.h"
#include "xregPAOIO.h"
#include "xregMeshIO.h"
#include "xregStringUtils.h"

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS       = 0;
  constexpr int kEXIT_VAL_BAD_USE       = 1;
  constexpr int kEXIT_VAL_BAD_CUTS_FILE = 2;

  using namespace xreg;

  DrawPAOBones draw_bones;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Visualizes PAO bone surfaces. If a PNG path is provided, then "
              "the user is not shown the rendering, it is written directly to "
              "file.");
  po.set_arg_usage("<Label Map> <APP Landmarks> <side> [<PNG Path>]");
  po.set_min_num_pos_args(3);

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Landmark and point clouds should be populated in RAS coordinates, otherwise they are populated in LPS coordinates.")
    << false;

  po.add("pelvis-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32,
         "pelvis-label",
         "The value used to indicate the pelvis in the input segmentation; if not "
         "provided, then the smallest positive value in the segmentation is used.");

  po.add("frag-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32,
         "frag-label",
         "The value used to indicate the fragment in the output segmentation; if not "
         "provided, then the N+1 is used, where N is the largest positive value in"
         " the input segmentation.");

  po.add("femur-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32,
         "femur-label",
         "The label of the the operative side femur. If not provided - it will be "
         "estimated by looking at the label of the femural head on the operative"
         " side.");

  po.add("contra-femur-label", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32,
         "contra-femur-label",
         "The label of the the non-operative side (contra-lateral) femur. "
         "If not provided, it will be estimated by looking at the label of the "
         "femural head on the non-operative side.");

  po.add("femur-frag-xform", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING,
         "femur-frag-xform",
         "The transformation, in the APP, that is applied to the combined fragment"
         " and femur.")
   << "";

  po.add("femur-only-xform", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING,
         "femur-only-xform",
         "The transformation, in the APP, that is applied to the femur only, "
         "prior to the combined fragment and femur transformation.")
   << "";

  po.add("femur-not-rel-to-frag", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "femur-not-rel-to-frag",
         "Indicates that the femur pose is not relative to the fragment - e.g. the "
         "combined fragment/femur pose will not be used to visualize the femur.")
    << false;

  po.add("hide-contra-femur", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "hide-contra-femur",
         "Do not render/display the contra-lateral femur.")
    << false;

  po.add("femur-frag-xform2", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING,
         "femur-frag-xform2",
         "(Draw another fragment) The transformation, in the APP, that is "
         "applied to the combined fragment and femur.")
   << "";

  //po.add("femur-only-xform2", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING,
  //       "femur-only-xform2",
  //       "(Draw another fragment) The transformation, in the APP, that is "
  //       "applied to the femur only, prior to the combined fragment and "
  //       "femur transformation.")
  // << "";

  po.add("frag-alpha", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "frag-alpha",
         "When drawing two fragments, use an alpha component of 0.5 for each; "
         "otherwise the component is 1.0")
    << false;

  po.add("axes", 'a', ProgOpts::kSTORE_TRUE, "axes", "Show axes.") << false;

  po.add("title", 't', ProgOpts::kSTORE_STRING, "title",
         "Sets the window title string") << draw_bones.win_title;

  po.add("cam-view", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "cam-view",
         "The camera view to use, valid inputs are:"
     " \"ap,\" \"oblique,\" \"lat,\" and \"vtk.\" \"vtk\" is the default VTK view.")
    << "ap";

  po.add("vol-frame", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "vol-frame",
         "Use the volume frame for determining views, instead of an APP frame computed "
         "from landmarks.")
    << false;

  po.add("cut-defs", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "cut-defs",
         "Path to cutting planes file - will also display these when provided.")
    << "";

  po.add("other-pts", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "other-pts",
         "A set of 3D landmarks to draw; these should be in the label map space."
         " These points will be loaded in LPS or RAS coordinates according to the \"lands-ras\" flag.")
    << "";

  po.add("no-frag", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-frag",
         "Do not draw the fragment surface.")
    << false;

  po.add("no-femurs", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-femurs",
         "Do not draw the femur surfaces.")
    << false;

  po.add("pelvis-mesh", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pelvis-mesh",
         "Path to a mesh representing the pelvis; this is useful when this utility is "
         "invoked many times, so that the mesh does not repeatedly need to be generated "
         "from the label map.")
    << "";

  po.add("frag-mesh", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "frag-mesh",
         "Path to a mesh representing the fragment; this is useful when this utility is "
         "invoked many times, so that the mesh does not repeatedly need to be generated "
         "from the label map.")
    << "";

  po.add("femur-mesh", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "femur-mesh",
         "Path to a mesh representing the ipsilateral femur; this is useful when this utility is "
         "invoked many times, so that the mesh does not repeatedly need to be generated "
         "from the label map.")
    << "";

  po.add("contra-femur-mesh", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "contra-femur-mesh",
         "Path to a mesh representing the contralateral femur; this is useful when this utility is "
         "invoked many times, so that the mesh does not repeatedly need to be generated "
         "from the label map.")
    << "";

  {
    auto& pelvis_color_arg = po.add("pelvis-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
                                    "pelvis-color", "Color of the pelvis shape.");
    pelvis_color_arg.default_vals.resize(3);
    pelvis_color_arg.num_vals = 3;
    pelvis_color_arg << draw_bones.pelvis_color[0] << draw_bones.pelvis_color[1] << draw_bones.pelvis_color[2];
  }

  {
    auto& frag_color_arg = po.add("frag-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
                                  "frag-color", "Color of the fragment shape.");
    frag_color_arg.default_vals.resize(3);
    frag_color_arg.num_vals = 3;
    frag_color_arg << draw_bones.frag_color[0] << draw_bones.frag_color[1] << draw_bones.frag_color[2];
  }

  {
    auto& sec_frag_color_arg = po.add("sec-frag-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
                                      "sec-frag-color", "Color of the second fragment shape.");
    sec_frag_color_arg.default_vals.resize(3);
    sec_frag_color_arg.num_vals = 3;
    sec_frag_color_arg << draw_bones.sec_frag_color[0] << draw_bones.sec_frag_color[1]
                       << draw_bones.sec_frag_color[2];
  }

  {
    auto& femur_color_arg = po.add("femur-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
                                   "femur-color", "Color of the femur shape.");
    femur_color_arg.default_vals.resize(3);
    femur_color_arg.num_vals = 3;
    femur_color_arg << draw_bones.femur_color[0] << draw_bones.femur_color[1] << draw_bones.femur_color[2];
  }

  {
    auto& contra_femur_color_arg = po.add("contra-femur-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
                                          "contra-femur-color", "Color of the contra-lateral femur shape.");
    contra_femur_color_arg.default_vals.resize(3);
    contra_femur_color_arg.num_vals = 3;
    contra_femur_color_arg << draw_bones.contra_femur_color[0] << draw_bones.contra_femur_color[1]
                           << draw_bones.contra_femur_color[2];
  }
  
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

  const bool verbose = po.get("verbose");
  std::ostream& vout = po.vout();

  draw_bones.set_debug_output_stream(vout, verbose);

  const bool lands_in_ras = po.get("lands-ras");

  const std::string src_labels_path = po.pos_args()[0];

  const std::string app_fcsv_path = po.pos_args()[1];

  const std::string side_str = po.pos_args()[2];

  const bool write_png = po.pos_args().size() > 3;
  draw_bones.png_path = write_png ? po.pos_args()[3] : std::string();

  draw_bones.show_axes = po.get("axes");

  const std::string cam_view_str = po.get("cam-view");
  
  const bool cam_ap_view          = cam_view_str == "ap";
  const bool cam_oblique_view     = cam_view_str == "oblique";
  const bool cam_lat_view         = cam_view_str == "lat";
  const bool cam_vtk_default_view = cam_view_str == "vtk";

  if (!(cam_ap_view || cam_oblique_view || cam_lat_view || cam_vtk_default_view))
  {
    std::cerr << "ERROR: Invalid camera view string: " << cam_view_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  if (cam_ap_view)
  {
    draw_bones.cam_view = DrawPAOBones::kAP;
  }
  else if (cam_oblique_view)
  {
    draw_bones.cam_view = DrawPAOBones::kOBLIQUE;
  }
  else if (cam_lat_view)
  {
    draw_bones.cam_view = DrawPAOBones::kLAT;
  }
  else
  {
    draw_bones.cam_view = DrawPAOBones::kVTK;
  }

  draw_bones.use_vol_frame = po.get("vol-frame");

  draw_bones.win_title = po.get("title").as_string();

  const std::string femur_and_frag_xform_path = po.get("femur-frag-xform").as_string();
  const std::string femur_only_xform_path     = po.get("femur-only-xform").as_string();

  const std::string femur_and_frag_xform2_path = po.get("femur-frag-xform2").as_string();
  //const std::string femur_only_xform2_path     = po.get("femur-only-xform2").as_string();

  draw_bones.do_frag_alpha = po.get("frag-alpha");

  auto copy_sur_color = [](const ProgOpts::DoubleList& src)
  {
    return Pt3{ static_cast<CoordScalar>(src[0]),
                static_cast<CoordScalar>(src[1]),
                static_cast<CoordScalar>(src[2]) };
  };

  ProgOpts::DoubleList pelvis_color(3);
  po.get_vals("pelvis-color", &pelvis_color);
  draw_bones.pelvis_color = copy_sur_color(pelvis_color);

  ProgOpts::DoubleList frag_color(3);
  po.get_vals("frag-color", &frag_color);
  draw_bones.frag_color = copy_sur_color(frag_color);

  ProgOpts::DoubleList sec_frag_color(3);
  po.get_vals("sec-frag-color", &sec_frag_color);
  draw_bones.sec_frag_color = copy_sur_color(sec_frag_color);

  ProgOpts::DoubleList femur_color(3);
  po.get_vals("femur-color", &femur_color);
  draw_bones.femur_color = copy_sur_color(femur_color);

  ProgOpts::DoubleList contra_femur_color(3);
  po.get_vals("contra-femur-color", &contra_femur_color);
  draw_bones.contra_femur_color = copy_sur_color(contra_femur_color);

  if (po.has("bg-color"))
  {
    ProgOpts::DoubleList bg_color(3);
    po.get_vals("bg-color", &bg_color);
    
    Pt3 bg = { static_cast<CoordScalar>(bg_color[0]),
               static_cast<CoordScalar>(bg_color[1]),
               static_cast<CoordScalar>(bg_color[2]) };
    draw_bones.bg_color_arg = bg;
  }

  const bool is_left = side_str == "left";

  if (!is_left && (side_str != "right"))
  {
    std::cerr << "ERROR: Invalid side string: " << side_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "Fragment specified on the " << side_str << " side." << std::endl;

  draw_bones.side = is_left ? DrawPAOBones::kLEFT : DrawPAOBones::kRIGHT;

  const std::string cut_defs_path = po.get("cut-defs");
  const bool draw_cut_planes = !cut_defs_path.empty();

  const std::string other_fcsv_path = po.get("other-pts");
  const bool draw_other_fcsv = !other_fcsv_path.empty();

  draw_bones.no_draw_frag = po.get("no-frag");

  draw_bones.no_draw_femurs = po.get("no-femurs");

  const std::string pelvis_mesh_path        = po.get("pelvis-mesh");
  const std::string frag_mesh_path          = po.get("frag-mesh");
  const std::string femur_mesh_path         = po.get("femur-mesh");
  const std::string contra_femur_mesh_path  = po.get("contra-femur-mesh");

  //////////////////////////////////////////////////////////////////////////////
  // Get the landmarks

  draw_bones.app_pts = ReadLandmarksFileNamePtMap(app_fcsv_path, !lands_in_ras);

  // Read other FCSV Landmarks
  if (draw_other_fcsv)
  {
    vout << "reading other landmark points..." << std::endl;
    draw_bones.other_pts = ReadLandmarksFilePts(other_fcsv_path, !lands_in_ras);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Read in input label map

  vout << "reading in source labels..." << std::endl;
  draw_bones.labels = ReadITKImageFromDisk<DrawPAOBones::LabelImage>(src_labels_path);

  //////////////////////////////////////////////////////////////////////////////
  // Determine labels of pelvis, fragment, femurs, and cut

  using LabelType = DrawPAOBones::LabelType;

  if (po.has("pelvis-label"))
  {
    draw_bones.pelvis_label_arg = static_cast<LabelType>(po.get("pelvis-label").as_uint32());
  }

  if (po.has("frag-label"))
  {
    draw_bones.frag_label_arg = static_cast<LabelType>(po.get("frag-label").as_uint32());
  }

  if (po.has("femur-label"))
  {
    draw_bones.femur_label_arg = static_cast<LabelType>(po.get("femur-label").as_uint32());
  }

  if (po.has("contra-femur-label"))
  {
    draw_bones.contra_femur_label_arg = static_cast<LabelType>(po.get("contra-femur-label").as_uint32());
  }

  if (!femur_and_frag_xform_path.empty())
  {
    vout << "reading frag+femur transform..." << std::endl;
    draw_bones.delta_Frag_and_Femur = ReadITKAffineTransformFromFile(femur_and_frag_xform_path);
    vout << draw_bones.delta_Frag_and_Femur.matrix() << std::endl;
  }
  else
  {
    vout << "no frag+femur transform specified, using identity..." << std::endl;
  }

  if (!femur_only_xform_path.empty())
  {
    vout << "reading femur only transform..." << std::endl;
    draw_bones.delta_Femur_only = ReadITKAffineTransformFromFile(femur_only_xform_path);
    vout << draw_bones.delta_Femur_only.matrix() << std::endl;

    draw_bones.femur_pose_rel_to_frag = !po.get("femur-not-rel-to-frag").as_bool();
    vout << "  is rel. to frag. pose? " << BoolToYesNo(draw_bones.femur_pose_rel_to_frag) << std::endl;
  }
  else
  {
    vout << "no femur only transform specified, using identity..." << std::endl;
  }

  draw_bones.show_contra_femur = !po.get("hide-contra-femur").as_bool();
  vout << "  show contra. femur? " << BoolToYesNo(draw_bones.show_contra_femur) << std::endl;

  if (!femur_and_frag_xform2_path.empty())
  {
    vout << "reading second fragment transform..." << std::endl;
    FrameTransform delta_sec_frag = ReadITKAffineTransformFromFile(femur_and_frag_xform2_path);
    vout << delta_sec_frag.matrix() << std::endl;
    draw_bones.delta_sec_frag = delta_sec_frag;
  }

  if (draw_cut_planes)
  {
    vout << "Reading cutting planes file..." << std::endl;
    const auto cuts_info = ReadPAOCutPlanesFile(cut_defs_path);
    
    if (!std::get<1>(cuts_info))
    {
      std::cerr << "ERROR: visualization info missing form PAO cuts file!" << std::endl;
      return kEXIT_VAL_BAD_CUTS_FILE;
    }

    draw_bones.cuts = std::make_tuple(std::get<0>(cuts_info), *std::get<1>(cuts_info));
  }

  if (!pelvis_mesh_path.empty())
  {
    vout << "reading pelvis mesh from disk: " << pelvis_mesh_path << std::endl;
    draw_bones.pelvis_mesh_arg = ReadMeshFromDisk(pelvis_mesh_path);
  }
  
  if (!draw_bones.no_draw_femurs)
  {
    if (!femur_mesh_path.empty())
    {
      vout << "reading ipsilateral femur mesh from disk: " << femur_mesh_path << std::endl;
      draw_bones.femur_mesh_arg = ReadMeshFromDisk(femur_mesh_path);
    }

    if (!contra_femur_mesh_path.empty())
    {
      vout << "reading contralateral femur mesh from disk: " << contra_femur_mesh_path << std::endl;
      draw_bones.contra_femur_mesh_arg = ReadMeshFromDisk(contra_femur_mesh_path);
    }
  }

  if (!draw_bones.no_draw_frag)
  {
    if (!frag_mesh_path.empty())
    {
      vout << "reading fragment mesh from disk: " << frag_mesh_path << std::endl;
      draw_bones.frag_mesh_arg = ReadMeshFromDisk(frag_mesh_path);
    }
  }
  else
  {
    vout << "Not drawing the fragment!" << std::endl;
  }
  
  draw_bones();

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}


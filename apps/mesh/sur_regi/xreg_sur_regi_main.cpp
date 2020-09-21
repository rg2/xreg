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

#include "xregProgOptUtils.h"
#include "xregMeshIO.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"
#include "xregFCSVUtils.h"
#include "xregCSVUtils.h"
#include "xregICP3D3D.h"
#include "xregAnatCoordFrames.h"
#include "xregLandmarkMapUtils.h"
#include "xregPairedPointRegi3D3D.h"
#include "xregITKIOUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS    = 0;
  constexpr int kEXIT_VAL_PARSE_FAIL = 1;
  constexpr int kEXIT_VAL_BAD_USE    = 2;
  
  PointToSurRegiICP icp;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Performs point cloud to surface registration using ICP. The output transformation "
              "maps points in the point cloud coordinate frame to the surface coordinate frame. "
              "The point cloud (second positional argument) is assumed to be in CSV format "
              "(x,y,z) with no header, unless an FCSV file is passed (determined by file extension).");

  po.add("no-ras2lps", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-ras2lps",
         "Do NOT convert RAS/LPS landmark and point cloud points to LPS/RAS")
    << false;

  po.add("mesh-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "mesh-lands",
         "Landmarks on the mesh to be used to compute an initialization. "
         "Landmark names are used to establish correspondences. FCSV format.")
    << "";

  po.add("pts-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pts-lands",
         "Landmarks in the point cloud to be used to compute an initialization. "
         "Landmark names are used to establish correspondences. FCSV format.")
    << "";

  po.add("max-its", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "max-its",
         "The maximum number of iterations to run. < 0 indicates no limit.")
    << ProgOpts::int32(icp.max_its);

  po.add("stop-ratio", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "stop-ratio",
         "Stopping tolerance with respect to the change in mean surface distance between iterations.")
    << icp.stop_ratio;

  po.add("std-dev-outlier", 'o', ProgOpts::kSTORE_DOUBLE, "std-dev-outlier",
         "Number of standard deviations of the current iteration's mean surface distance to use for "
         "classifying outlier points. <= 0 indicates no outlier detection.")
    << icp.std_dev_outlier_coeff;

  po.add("similarity", 's', ProgOpts::kSTORE_TRUE, "similarity",
         "Compute similarity transform - e.g. compute scale in addition to rigid pose.")
    << false;

  po.set_arg_usage("<Mesh Path> <Point Cloud Path> <output transform>");

  po.set_min_num_pos_args(3);

  try
  {
    po.parse(argc, argv);
  }
  catch (const ProgOpts::Exception& e)
  {
    std::cerr << "Error parsing command line arguments: " << e.what() << std::endl;
    po.print_usage(std::cerr);
    return kEXIT_VAL_PARSE_FAIL;
  }

  if (po.help_set())
  {
    po.print_usage(std::cout);
    po.print_help(std::cout);
    return kEXIT_VAL_SUCCESS;
  }

  const bool verbose = po.get("verbose");
  std::ostream& vout = po.vout();

  icp.set_debug_output_stream(vout, verbose);

  const std::string mesh_path     = po.pos_args()[0];
  const std::string pt_cloud_path = po.pos_args()[1];
  const std::string dst_xform     = po.pos_args()[2];

  const bool ras2lps = !po.get("no-ras2lps").as_bool();

  icp.max_its = po.get("max-its").as_int32();

  icp.stop_ratio = po.get("stop-ratio");

  icp.std_dev_outlier_coeff = po.get("std-dev-outlier");

  icp.allow_similarity = po.get("similarity");

  const std::string mesh_lands_path = po.get("mesh-lands");
  const std::string pts_lands_path = po.get("pts-lands");
  
  if (mesh_lands_path.empty() ^ pts_lands_path.empty())
  {
    std::cerr << "ERROR: when supplying landmark FCSV, both mesh and point cloud must be provided!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "reading mesh from disk..." << std::endl;
  const auto mesh = ReadMeshFromDisk(mesh_path);
  vout << "  complete." << std::endl;

  icp.sur = &mesh;

  const bool pt_cloud_is_fcsv = ToLowerCase(Path(pt_cloud_path).file_extension()) == ".fcsv";

  vout << "Point Cloud is FCSV: " << BoolToYesNo(pt_cloud_is_fcsv) << std::endl;

  vout << "reading point cloud from disk..." << std::endl;
  auto pt_cloud = pt_cloud_is_fcsv ? ReadFCSVFilePts(pt_cloud_path) : Read3DPtCloudCSV(pt_cloud_path, false);
  vout << "  complete." << std::endl;
  
  if (ras2lps && pt_cloud_is_fcsv)
  {
    vout << "converting FCSV point cloud RAS --> LPS..." << std::endl;
    ConvertRASToLPS(&pt_cloud);
  }

  if (!mesh_lands_path.empty())
  {
    vout << "reading mesh landmarks..." << std::endl;
    auto mesh_lands_map = ReadFCSVFileNamePtMap(mesh_lands_path);
    PrintLandmarkMap(mesh_lands_map, vout);

    vout << "reading point cloud landmarks..." << std::endl;
    auto pt_cloud_lands_map = ReadFCSVFileNamePtMap(pts_lands_path);
    PrintLandmarkMap(pt_cloud_lands_map, vout);

    if (ras2lps)
    {
      vout << "landmarks RAS --> LPS..." << std::endl;
      ConvertRASToLPS(&mesh_lands_map);
      ConvertRASToLPS(&pt_cloud_lands_map);
    }

    vout << "estimating initial transformation using corresponding paired points..." << std::endl;
    icp.init_pts_to_sur_xform = PairedPointRegi3D3D(pt_cloud_lands_map, mesh_lands_map,
                                                    icp.allow_similarity ? -1 : 1);
  }
  
  icp.pts = &pt_cloud;

  vout << "running ICP..." << std::endl;
  const FrameTransform regi_xform = icp.run();

  vout << "writing regi transform to disk..." << std::endl;
  WriteITKAffineTransform(dst_xform, regi_xform);

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}


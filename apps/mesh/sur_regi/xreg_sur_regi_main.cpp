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
#include "xregMeshIO.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"
#include "xregLandmarkFiles.h"
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
              "maps points in the point cloud coordinate frame to the surface coordinate frame.");

  po.add("lands-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "lands-ras",
         "Landmark and point cloud points should be populated in RAS coordinates, otherwise they are populated in LPS coordinates")
    << false;

  po.add("mesh-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "mesh-lands",
         "Landmarks on the mesh to be used to compute an initialization. "
         "Landmark names are used to establish correspondences.")
    << "";

  po.add("pts-lands", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pts-lands",
         "Landmarks in the point cloud to be used to compute an initialization. "
         "Landmark names are used to establish correspondences.")
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

  const bool lands_as_ras = po.get("lands-ras").as_bool();

  icp.max_its = po.get("max-its").as_int32();

  icp.stop_ratio = po.get("stop-ratio");

  icp.std_dev_outlier_coeff = po.get("std-dev-outlier");

  icp.allow_similarity = po.get("similarity");

  const std::string mesh_lands_path = po.get("mesh-lands");
  const std::string pts_lands_path = po.get("pts-lands");
  
  if (mesh_lands_path.empty() ^ pts_lands_path.empty())
  {
    std::cerr << "ERROR: when supplying landmark files for initial transform estimation, "
                 "both mesh and point cloud must be provided!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "reading mesh from disk..." << std::endl;
  const auto mesh = ReadMeshFromDisk(mesh_path);
  vout << "  complete." << std::endl;

  icp.sur = &mesh;

  vout << "reading point cloud from disk..." << std::endl;
  auto pt_cloud = ReadLandmarksFilePts(pt_cloud_path, !lands_as_ras);
  vout << "  complete." << std::endl;

  if (!mesh_lands_path.empty())
  {
    vout << "reading mesh landmarks..." << std::endl;
    auto mesh_lands_map = ReadLandmarksFileNamePtMap(mesh_lands_path, !lands_as_ras);
    PrintLandmarkMap(mesh_lands_map, vout);

    vout << "reading point cloud landmarks..." << std::endl;
    auto pt_cloud_lands_map = ReadLandmarksFileNamePtMap(pts_lands_path, !lands_as_ras);
    PrintLandmarkMap(pt_cloud_lands_map, vout);

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


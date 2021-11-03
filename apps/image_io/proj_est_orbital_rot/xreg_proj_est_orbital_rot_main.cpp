/*
 * MIT License
 *
 * Copyright (c) 2021 Robert Grupp
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
#include "xregHDF5.h"
#include "xregH5ProjDataIO.h"
#include "xregH5CamModelIO.h"
#include "xregPerspectiveXform.h"
#include "xregRotUtils.h"
#include "xregFitCylinder.h"
#include "xregVTK3DPlotter.h"

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;

  using namespace xreg;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  // The method used here is derived from the following paper:
  // Navab N, Bani-Hashemi A, Nadar MS, Wiesent K, Durlak P, Brunner T, Barth K, Graumann R.
  // 3D reconstruction from projection matrices in a C-arm based 3D-angiography system.
  // In International Conference on Medical Image Computing and Computer-Assisted
  // Intervention 1998 Oct 11 (pp. 119-129). Springer, Berlin, Heidelberg.

  po.set_help("Given a collection of projections collected as part of an orbital "
      "rotation, estimate the axis of rotation and the center of rotation. "
      "The updated extrinsic matrix for a reference view are printed to standard out. "
      "This optionally updates the extrinsic frames of the input projection file. "
      "Uses the method described by the 1998 MICCAI by Navab, et al..");
  
  po.set_arg_usage("<Proj. Data File>");
  
  po.set_min_num_pos_args(1);

  po.add("ref-proj", 'p', ProgOpts::kSTORE_UINT64, "ref-proj",
         "The index of the reference projection used to estimate the new y-axis and "
         "print out the new extrinsic frame. When not supplied, either the "
         "first (when not reversing projection order) or last projection (when reversing "
         "projection order) is used.");

  po.add("update", 'u', ProgOpts::kSTORE_TRUE, "update",
         "Update the input projection data file to use the new extrinsic frame.")
    << false;

  po.add("reverse", 'r', ProgOpts::kSTORE_TRUE, "reverse",
         "reverse the order of projections when calculating the orbit rotation axis.")
    << false;;

  po.add("debug", 'd', ProgOpts::kSTORE_TRUE, "debug",
         "Display a debug visualization of the newly computed coordinate frame.")
    << false;

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

  const bool has_ref_proj_idx = po.has("ref-proj");

  const bool update_proj_data = po.get("update");

  const bool reverse_order = po.get("reverse");

  const bool debug_disp = po.get("debug");

  const std::string proj_path = po.pos_args()[0];

  vout << "opening H5 file for "
       << (update_proj_data ? "modification" : "reading") << ": "
       << proj_path << std::endl;
  H5::H5File h5(proj_path, update_proj_data ? H5F_ACC_RDWR : H5F_ACC_RDONLY);
    
  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", h5);

  if (num_projs < 3)
  {
    std::cerr << "need at least three projections, this file has: " << num_projs << std::endl;
    return kEXIT_VAL_BAD_USE;
  }
 
  vout << "num. projs. in file: " << num_projs << std::endl;

  const size_type ref_proj_idx = has_ref_proj_idx ?
    static_cast<size_type>(po.get("ref-proj").as_uint64()) :
    (reverse_order ? (num_projs - 1) : size_type(0));
  
  vout << "ref. proj. index: " << ref_proj_idx << std::endl;
  xregASSERT(ref_proj_idx < num_projs);

  vout << "extracting existing camera models..." << std::endl;
  auto cams = ReadCamModelsFromProjData(h5);
  xregASSERT(cams.size() == num_projs);

  vout << "estimating orbital rotation axis..." << std::endl;
  
  const size_type num_projs_minus_1 = num_projs - 1;

  Mat3x3List delta_rots;
  delta_rots.reserve(num_projs_minus_1);
  
  vout << "  calc. rotation deltas between adjacent projections..." << std::endl;
  vout << "    reverse order? " << (reverse_order ? "YES" : "NO") << std::endl;

  for (size_type i = 0; i < num_projs_minus_1; ++i)
  {
    const size_type idx_1 = reverse_order ? (num_projs_minus_1 - i) : i;
    const size_type idx_2 = reverse_order ? (num_projs_minus_1 - 1 - i) : (i + 1);
    
    const Mat3x3 delta_R = (cams[idx_2].extrins_inv * cams[idx_1].extrins).matrix().block(0,0,3,3);

    delta_rots.push_back(delta_R);
    
    vout << fmt::format("    {:2d} --> {:2d} : {:4.1f} (deg.)", idx_1, idx_2, LogSO3ToPt(delta_R).norm() * kRAD2DEG) << std::endl;
  }

  const Mat3x3 mean_delta_R = SO3FrechetMean(delta_rots);

  // This will be the x-axis in the updated extrinsic frame
  Pt3 mean_rot_axis = LogSO3ToPt(mean_delta_R);
  mean_rot_axis.normalize();

  // The y-axis will be approximately source-to-detector direction of reference view
  const auto& ref_cam = cams[ref_proj_idx];

  Pt3 ref_cam_src_to_det = ref_cam.ind_pt_to_phys_det_pt(
                              Pt2(ref_cam.intrins.block(0,2,2,1)))
                           - ref_cam.pinhole_pt;
  ref_cam_src_to_det.normalize();

  Pt3 y_axis = ref_cam_src_to_det -
                  (ref_cam_src_to_det.dot(mean_rot_axis) * mean_rot_axis);
  y_axis.normalize();

  Mat3x3 new_to_old_R;
  new_to_old_R.col(0) = mean_rot_axis;
  new_to_old_R.col(1) = y_axis;
  new_to_old_R.col(2) = mean_rot_axis.cross(y_axis);

  vout << "new to old, R^T * R:\n" << new_to_old_R.transpose() * new_to_old_R << std::endl;
  
  vout << "det(new to old R): " << new_to_old_R.determinant() << std::endl;

  xregASSERT(std::abs(new_to_old_R.col(0).dot(new_to_old_R.col(1))) < 1.0e-6f);
  xregASSERT(std::abs(new_to_old_R.col(0).dot(new_to_old_R.col(2))) < 1.0e-6f);
  xregASSERT(std::abs(new_to_old_R.col(1).dot(new_to_old_R.col(2))) < 1.0e-6f);

  vout << "estimating center of rotation from x-ray source positions..." << std::endl;

  Pt3List src_pts;
  src_pts.reserve(num_projs);

  for (size_type i = 0; i < num_projs; ++i)
  {
    src_pts.push_back(cams[i].pinhole_pt);
  }

  Pt3 center_of_rot;
  CoordScalar orbit_radius = 0;
  std::tie(center_of_rot, orbit_radius) = FitOrbitToPts(src_pts);

  if (debug_disp)
  {
    vout << "creating debug visualization..." << std::endl;

    VTK3DPlotter plotter;

    plotter.add_pts(src_pts, 0, 1, 0);
    
    plotter.add_sphere(center_of_rot, 5, 1, 1, 0);
    
    plotter.add_circle(center_of_rot, orbit_radius, mean_rot_axis, 0, 0, 0);

    constexpr CoordScalar AXIS_LEN = 20;

    plotter.add_arrow(center_of_rot,
                      center_of_rot + (new_to_old_R * Pt3(AXIS_LEN,0,0)),
                      1, 0, 0);

    plotter.add_arrow(center_of_rot,
                      center_of_rot + (new_to_old_R * Pt3(0,AXIS_LEN,0)),
                      0, 1, 0);
    
    plotter.add_arrow(center_of_rot,
                      center_of_rot + (new_to_old_R * Pt3(0,0,AXIS_LEN)),
                      0, 0, 1);

    plotter.show();
  }

  FrameTransform new_extrins_to_old = FrameTransform::Identity();
  new_extrins_to_old.matrix().block(0,0,3,3) = new_to_old_R;
  new_extrins_to_old.matrix().block(0,3,3,1) = center_of_rot;

  Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
  std::cout << "Updated extrinsic matrix:\n"
            << (ref_cam.extrins * new_extrins_to_old).matrix().format(HeavyFmt)
            << std::endl;

  if (update_proj_data)
  {
    vout << "updating camera models in HDF5..." << std::endl;
    for (size_type i = 0; i < num_projs; ++i)
    {
      vout << "  proj " << i << std::endl;

      H5::Group proj_g = h5.openGroup(fmt::format("proj-{:03d}", i));

      proj_g.unlink("cam");

      H5::Group cam_g = proj_g.createGroup("cam");

      auto& cam = cams[i];

      cam.extrins = cam.extrins * new_extrins_to_old;
      cam.extrins_inv = cam.extrins.inverse(Eigen::Isometry);

      WriteCamModelH5(cam, &cam_g);
    }

    h5.flush(H5F_SCOPE_GLOBAL);
  }

  h5.close();

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}

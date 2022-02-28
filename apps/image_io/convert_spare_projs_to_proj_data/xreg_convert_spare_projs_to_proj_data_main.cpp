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

#include <fmt/format.h>

#include "xregFilesystemUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregITKBasicImageUtils.h"
#include "xregProgOptUtils.h"
#include "xregRTKGeom.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS   = 0;
  constexpr int kEXIT_VAL_BAD_USE   = 1;
  constexpr int kEXIT_VAL_BAD_DATA  = 2;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Converts a collection of 2D projection images the SPARE dataset/challenge "
              "into an HDF5 projection data file for use by xReg. "
              "SPARE homepage: https://image-x.sydney.edu.au/spare-challenge .");
  po.set_arg_usage("<Input SPARE projs directory> <Output xReg HDF5 file>");
  po.set_min_num_pos_args(2);

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

  const std::string& src_spare_dir = po.pos_args()[0];
  const std::string& dst_pd_path   = po.pos_args()[1];

  const Path src_dir_path(src_spare_dir);

  const Path geom_xml_path = src_dir_path + "Geometry.xml";

  if (!geom_xml_path.exists())
  {
    std::cerr << "ERROR: Geometry.xml file not located in SPARE directory!" << std::endl;
    return kEXIT_VAL_BAD_DATA;
  }

  vout << "reading RTK geometry XML..." << std::endl;
  const auto rtk_geom_info = ReadRTKGeomInfoFromXMLFile(geom_xml_path.string());

  const size_type num_projs = rtk_geom_info.proj_infos.size();
  
  vout << "num. projs.: " << num_projs << std::endl;

  size_type proj_file_size = 0;

  {
    const Path first_proj_path = src_dir_path + "Proj_00001.bin";

    vout << "checking file size of first proj: " << first_proj_path.string() << std::endl;

    if (!first_proj_path.exists())
    {
      std::cerr << "ERROR: first projection file does not existing: "
                << first_proj_path.string() << std::endl;
      return kEXIT_VAL_BAD_DATA;
    }

    proj_file_size = FileInputStream(first_proj_path.string()).num_bytes_left();
  }

  vout << "proj. file size: " << proj_file_size << std::endl;

  size_type proj_num_rows = 0;
  size_type proj_num_cols = 0;
  CoordScalar ps = 0;
  CoordScalar expected_src_to_det = 0;
  CoordScalar expected_src_to_iso = 0;

  switch (proj_file_size)
  {
  case size_type(512 * 512 * 4):
    vout << "detected Elekta dataset" << std::endl;
    proj_num_rows = 512;
    proj_num_cols = 512;
    ps = 0.8f;
    expected_src_to_det = 1536;
    expected_src_to_iso = 1000;
    break;
  case size_type(1024 * 768 * 4):
    vout << "detected Varian P1/P2 dataset" << std::endl;
    proj_num_rows = 1024;
    proj_num_cols = 768;
    ps = 0.388f;
    expected_src_to_det = 1500;
    expected_src_to_iso = 1000;
    break;
  case size_type(1008 * 752 * 4):
    vout << "detected Varian P3/P4/P5 dataset" << std::endl;
    proj_num_rows = 1008;
    proj_num_cols = 752;
    ps = 0.388f;
    expected_src_to_det = 1500;
    expected_src_to_iso = 1000;
    break;
  case size_type(512 * 384 * 4):
    vout << "detected Monte Carlo simulated dataset" << std::endl;
    proj_num_rows = 512;
    proj_num_cols = 384;
    ps = 0.776f;
    expected_src_to_det = 1500;
    expected_src_to_iso = 1000;
    break;
  default:
    std::cerr << "Unable to determine data source from file size: "
              << proj_file_size << std::endl;
    return kEXIT_VAL_BAD_DATA;
  }

  if (std::abs(rtk_geom_info.src_to_iso_center_dist_mm - expected_src_to_iso) > 1.0e-6f)
  {
    std::cerr << fmt::format("ERROR: source-to-isocenter from XML ({:.3f}) does not match expected ({:.3f})") << std::endl;
    return kEXIT_VAL_BAD_DATA;
  }

  if (std::abs(rtk_geom_info.src_to_det_dist_mm - expected_src_to_det) > 1.0e-6f)
  {
    std::cerr << fmt::format("ERROR: source-to-detector from XML ({:.3f}) does not match expected ({:.3f})") << std::endl;
    return kEXIT_VAL_BAD_DATA;
  }
  
  ProjDataF32List pd(num_projs);

  // The RTK geometry projects to physical points (e.g. in mm with origin at the center of the image),
  // this converts those into pixel indices (e.g. in pixels with origin at the corner pixel (0,0). 
  Mat3x3 intrins_phys_to_pixels = Mat3x3::Identity();
  intrins_phys_to_pixels(0,0) = CoordScalar(1) / ps;
  intrins_phys_to_pixels(1,1) = CoordScalar(1) / ps;
  intrins_phys_to_pixels(0,2) = proj_num_cols / CoordScalar(2);
  intrins_phys_to_pixels(1,2) = proj_num_rows / CoordScalar(2);
  
  Mat3x3 K;
  Mat4x4 H;
  CoordScalar rho;
 
  vout << "creating camera models for each proj..." << std::endl; 
  for (size_type i = 0; i < num_projs; ++i)
  {
    vout << fmt::format("  proj.: {:4d}", i) << std::endl;

    std::tie(K,H,rho) = DecompProjMat(rtk_geom_info.proj_infos[i].cam_mat, true);
    
    auto& cam = pd[i].cam;

    pd[i].cam.setup(intrins_phys_to_pixels * K, H, proj_num_rows, proj_num_cols, ps, ps);
    xregASSERT(std::abs(pd[i].cam.focal_len - rtk_geom_info.src_to_det_dist_mm) < 1.0e-3f);
  }
  vout << "---------------------------------\n\n";

  const size_type tot_num_pix = proj_num_cols * proj_num_rows;

  // The projection pixels are stored on disk in column-major, but we eventually need them in
  // row-major order. We will use a column-major representation when reading from disk and
  // then copy it into a row-major overlay on the ITK image buffer.
  using MatColMaj = Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor|Eigen::DontAlign>;
  using MatRowMaj = Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor|Eigen::DontAlign>;

  MatColMaj tmp_proj_buf(proj_num_rows, proj_num_cols);

  std::array<double,2> tmp_spacing = { static_cast<double>(ps), static_cast<double>(ps) };

  vout << "reading projection pixels files..." << std::endl;

  for (size_type i = 0; i < num_projs; ++i)
  {
    vout << fmt::format("  proj.: {:4d}", i) << std::endl;

    const Path cur_proj_path = src_dir_path + fmt::format("Proj_{:05d}.bin", i+1);
    
    if (!cur_proj_path.exists())
    {
      std::cerr << "ERROR: projection file does not exist: " << cur_proj_path.string()
                << std::endl;
      return kEXIT_VAL_BAD_DATA;
    }

    FileInputStream fin(cur_proj_path.string());

    if (fin.num_bytes_left() != proj_file_size)
    {
      std::cerr << "ERROR: proj. file size (" << fin.num_bytes_left()
                << ") does not match expected (" << proj_file_size  << "): "
                << cur_proj_path.string() << std::endl;
      return kEXIT_VAL_BAD_DATA;
    }
    
    auto img = MakeITK2DVol<float>(proj_num_cols, proj_num_rows);
    
    img->SetSpacing(tmp_spacing.data());

    fin.read(&tmp_proj_buf(0,0), tot_num_pix);
    
    Eigen::Map<MatRowMaj> dst_proj_buf(img->GetBufferPointer(), proj_num_rows, proj_num_cols);
    dst_proj_buf = tmp_proj_buf;

    pd[i].img = img;
  }

  vout << "saving to proj data HDF5..." << std::endl;
  WriteProjDataH5ToDisk(pd, dst_pd_path);

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}


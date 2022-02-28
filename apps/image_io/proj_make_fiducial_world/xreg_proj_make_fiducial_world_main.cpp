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
#include "xregITKIOUtils.h"
#include "xregPerspectiveXform.h"

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;

  using namespace xreg;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Updates the extrinsic transformations of each projection using previously "
      "computed transformations (e.g. via registration) from the existing projection "
      "extrinsic frames to a fiducial coordinate frame. The input projection file is "
      "modified. A transformation must be specified for each projection or pass a \"-\" "
      "value to indicate the extrinsic transformation should not be updated.");
  
  po.set_arg_usage("<Proj. Data File> <Proj. #1 cam wrt. fid.> "
      "[... <Proj. #N cam wrt. fid.>]");
  
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

  const auto pos_args = po.pos_args();

  const size_type num_xform_files = pos_args.size() - 1;

  const std::string proj_path = pos_args[0];

  vout << "opening H5 file for modification: " << proj_path << std::endl;
  H5::H5File h5(proj_path, H5F_ACC_RDWR);
    
  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", h5);
 
  vout << "num. projs. in file: " << num_projs << std::endl;

  if (num_xform_files != num_projs)
  {
    std::cerr << "ERROR: number of passed transforms (" << num_xform_files
      << ") must match number of projections in file (" << num_projs << ")!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  std::vector<FrameTransform> xforms;
  xforms.reserve(num_projs);

  for (size_type i = 0; i < num_projs; ++i)
  {
    const auto& cur_xform_path = pos_args[1 + i];
   
    if (cur_xform_path != "-")
    {
      vout << "  reading transform for projection " << i << ": "
           << cur_xform_path << std::endl;

      xforms.push_back(ReadITKAffineTransformFromFile(cur_xform_path));
    }
    else
    {
      vout << "  using identity for projection " << i << std::endl;
      xforms.push_back(FrameTransform::Identity());
    }
  }

  vout << "extracting existing camera models..." << std::endl;
  auto cams = ReadCamModelsFromProjData(h5);

  vout << "calculating new fiducial world coords..." << std::endl;
  cams = CreateCameraWorldUsingFiducial(cams, xforms);
  
  vout << "updating camera models in HDF5..." << std::endl;
  for (size_type i = 0; i < num_projs; ++i)
  {
    vout << "  proj " << i << std::endl;

    H5::Group proj_g = h5.openGroup(fmt::format("proj-{:03d}", i));

    proj_g.unlink("cam");

    H5::Group cam_g = proj_g.createGroup("cam");

    WriteCamModelH5(cams[i], &cam_g);
  }

  h5.flush(H5F_SCOPE_GLOBAL);
  h5.close();

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}

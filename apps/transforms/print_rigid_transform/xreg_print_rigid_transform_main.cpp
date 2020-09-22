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

#include "xregProgOptUtils.h"
#include "xregITKIOUtils.h"
#include "xregRotUtils.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  const int kEXIT_VAL_SUCCESS   = 0;
  const int kEXIT_VAL_BAD_USE   = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Prints an ITK Rigid Transform and related metadata.");
  po.set_arg_usage("<path to transform file>");
  po.set_min_num_pos_args(1);

  po.add("invert", 'i', ProgOpts::kSTORE_TRUE, "invert",
         "Invert the transformation (useful for output from 3D-Slicer).")
      << false;

  po.add("axis-angle", 'a', ProgOpts::kSTORE_TRUE, "axis-angle",
         "Print rotation axis and angle.")
    << false;

  po.add("euler", 'e', ProgOpts::kSTORE_TRUE, "euler",
         "Print out Euler ZYX angles.")
    << false;

  po.add("rad", 'r', ProgOpts::kSTORE_TRUE, "rad",
         "Print rotation angles in radians")
    << false;

  po.add("trans", 't', ProgOpts::kSTORE_TRUE, "trans",
         "Print the translation magnitude")
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

  const std::string xform_path = po.pos_args()[0];

  const bool invert_xform  = po.get("invert");

  const bool axis_ang = po.get("axis-angle");

  const bool eulerZYX = po.get("euler");

  const bool use_rad = po.get("rad");

  const bool trans_mag = po.get("trans");

  FrameTransform xform = ReadITKAffineTransformFromFile(xform_path);

  if (invert_xform)
  {
    xform = xform.inverse();
  }

  std::cout << xform.matrix().format(Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]")) << std::endl;

  const Mat3x3 rot_mat = xform.matrix().block(0,0,3,3);

  const std::string deg_rad_str = use_rad ? "rad." : "deg.";

  if (axis_ang)
  {
    Pt3 x = LogSO3ToPt(rot_mat);
  
    CoordScalar theta = x.norm();

    x /= theta;

    if (!use_rad)
    {
      theta *= kRAD2DEG;
    }

    std::cout << fmt::format("Rotation Angle ({}):   {:.4f}", deg_rad_str, theta) << std::endl;
    std::cout << fmt::format("        Rotation Axis: [ {:+.4f}, {:+.4f}, {:+.4f} ]",
                              x[0], x[1], x[2]) << std::endl;
  }

  if (trans_mag)
  {
    std::cout << fmt::format("Translation Magnitude: {:.4f}", xform.matrix().block(0,3,3,1).norm()) << std::endl;
  }

  if (eulerZYX)
  {
    CoordScalar theta_x = 0;
    CoordScalar theta_y = 0;
    CoordScalar theta_z = 0;

    std::tie(theta_x,theta_y,theta_z) = RotMatrixToEulerXYZ(rot_mat);

    if (!use_rad)
    {
      theta_x *= kRAD2DEG;
      theta_y *= kRAD2DEG;
      theta_z *= kRAD2DEG;
    }

    std::cout << "Euler ZYX Angles (" << deg_rad_str << "):\n";
    std::cout << fmt::format("  Rot X: {:+9.4f}", theta_x) << std::endl;
    std::cout << fmt::format("  Rot Y: {:+9.4f}", theta_y) << std::endl;
    std::cout << fmt::format("  Rot Z: {:+9.4f}", theta_z) << std::endl;
  }

  return kEXIT_VAL_SUCCESS;
}

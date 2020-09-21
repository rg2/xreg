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

#include <itkBinaryThresholdImageFilter.h>
#include <itkFlipImageFilter.h>

// xreg
#include "xregStringUtils.h"
#include "xregMeshIO.h"
#include "xregProgOptUtils.h"
#include "xregVTKMeshUtils.h"
#include "xregITKIOUtils.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  const int kEXIT_VAL_SUCCESS = 0;
  const int kEXIT_VAL_BAD_USE = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Creates a 3D surface mesh from a 3D volume. The default behavior "
              "assumes the input image is a label map; however the user may "
              "specify the appropriate options to use an intensity image as "
              "input and internally create a labelmap via basic thresholding. "
              "After isosurface creation, the surface is smoothed "
              "with a sinc filter, then triangles are decimated/reduced via a "
              "quadric approach, finally normals are (optionally) computed.");
  po.set_arg_usage("<label/intensity image> <output mesh> [<label 1> [<label 2> ... [<label N>]]]");

  po.add("intensity", 'i', ProgOpts::kSTORE_TRUE, "intensity-image",
         "Indicates that the input image is an intensity image; at least one "
         "threshold bound must be provided.") << false;

  po.add("lower", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "lower_bound",
         "The lower bound on the threshold interval; e.g. keep voxels with "
         "intensities above this value.");

  po.add("upper", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "upper_bound",
         "The upper bound on the threshold interval; e.g. keep voxels with "
         "intensities below this value.");

  po.add("no-smooth", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "no-smooth",
         "Do not perform smoothing.")
      << false;

  po.add("smooth-its", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32,
         "smooth-its", "The number of smoothing iterations.")
      << ProgOpts::uint32(25);

  po.add("smooth-passband", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "smooth-passband", "Used to adjust the sinc in the frequency domain. "
         "Smaller -> more smoothing")
      << 0.1;

  po.add("smooth-boundary", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "smooth-boundary", "Smooth vertices on the boundary of the mesh.")
      << false;

  po.add("smooth-feature-edges", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "smooth-feature-edges",
         "Do not exclude feature edges (sharp interior) from smoothing")
      << false;

  po.add("smooth-feature-angle", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "smooth-feature-angle",
         "The angle (degrees) threshold for sharp interior edge identification.")
      << 120.0;

  po.add("smooth-only-manifold", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "smooth-only-manifold", "Only smooth manifold vertices.")
      << false;

  po.add("no-reduce", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "no-reduce",
         "Do not perform reduction.")
      << false;

  po.add("reduce-amount", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "reduce-amount",
         "The desired number of triangles to remove; specified as the "
         "percentage of the original number of triangles.")
      << 0.25;

  po.add("normals", 'n', ProgOpts::kSTORE_TRUE, "compute-normals",
         "Computes normals for each triangle face and stores in the output mesh.")
      << false;

  po.add("no-flip-vtk-normals", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-flip-vtk-normals",
         "Do not flip the VTK normal vectors to face inward.")
      << false;

  po.add("flip-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "flip-x",
         "Flips the label map about the x-axis prior to running marching cubes.")
      << false;

  po.add("flip-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "flip-y",
         "Flips the label map about the y-axis prior to running marching cubes.")
      << false;

  po.add("flip-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "flip-z",
         "Flips the label map about the z-axis prior to running marching cubes.")
      << false;

  po.add("swap-lps-ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "swap-lps-ras",
         "Converts mesh vertices from LPS to RAS (or vice-versa)")
      << false;

  po.add("reverse-vertex-order", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "reverse-vertex-order",
         "Reverses the ordering of vertices for each triangle and flips the associated normal; done after the mesh is created.")
      << false;

  po.add("no-phys-coords", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-phys-coords",
         "Output a mesh with vertices in continuous image indices - not physical coordinates associated with the image.")
      << false;

  po.add("ascii", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "ascii",
         "Write into an ASCII compatible format when possible/supported.")
    << false;

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

  const bool prefer_ascii = po.get("ascii");

  VTKCreateMesh create_mesh_fn;

  create_mesh_fn.do_smoothing         = !po.get("no-smooth");
  create_mesh_fn.num_smooth_its       =  po.get("smooth-its").as_uint32();
  create_mesh_fn.smooth_passband      =  po.get("smooth-passband");
  create_mesh_fn.smooth_boundary      =  po.get("smooth-boundary");
  create_mesh_fn.smooth_feature_edges =  po.get("smooth-feature-edges");
  create_mesh_fn.smooth_feature_angle =  po.get("smooth-feature-angle");
  create_mesh_fn.smooth_non_manifold  = !po.get("smooth-only-manifold");

  create_mesh_fn.do_reduction     = !po.get("no-reduce");
  create_mesh_fn.reduction_amount =  po.get("reduce-amount");

  create_mesh_fn.compute_normals     = po.get("compute-normals");
  create_mesh_fn.no_flip_vtk_normals = po.get("no-flip-vtk-normals");

  const bool flip_x = po.get("flip-x");
  const bool flip_y = po.get("flip-y");
  const bool flip_z = po.get("flip-z");

  const bool swap_lps_ras = po.get("swap-lps-ras");

  const bool intensity_input = po.get("intensity-image");

  create_mesh_fn.reverse_vertex_order = po.get("reverse-vertex-order");

  create_mesh_fn.no_phys_coords = po.get("no-phys-coords");

  const std::string input_img_path = po.pos_args()[0];

  const size_type num_labels_cmd_line = po.pos_args().size() - 2;  // first 2 arguments are input image and output mesh

  // the number of labels is equal to the number of labels provided by the user,
  // or when no labels are provided, then we default to 1 label.
  const size_type num_labels = num_labels_cmd_line ? num_labels_cmd_line : 1;

  vout << "number of labels specified by user: " << num_labels_cmd_line << std::endl;

  if (num_labels_cmd_line == 0)
  {
    vout << "using default label of 1" << std::endl;
    create_mesh_fn.labels = { 1.0 };
  }
  else
  {
    vout << "parsing label values from command line..." << std::endl;

    create_mesh_fn.labels.clear();
    create_mesh_fn.labels.reserve(num_labels);

    for (size_type label_idx = 0; label_idx < num_labels; ++label_idx)
    {
      create_mesh_fn.labels.push_back(StringCast<double>(po.pos_args()[2 + label_idx]));
    }
  }

  using LabelImageType = itk::Image<unsigned char,3>;
  LabelImageType::Pointer label_img;

  if (!intensity_input)
  {
    vout << "reading label map..." << std::endl;
    label_img = ReadITKImageFromDisk<LabelImageType>(input_img_path);
  }
  else
  {
    const bool has_lower = po.has("lower_bound");
    const bool has_upper = po.has("upper_bound");

    if (!has_lower && !has_lower)
    {
      std::cerr << "ERROR: at least one threshold limit must be provided!" << std::endl;
      return kEXIT_VAL_BAD_USE;
    }

    vout << "reading intensity volume..." << std::endl;
    // Read the intensity image from disk
    using IntensityImageType = itk::Image<float,3>;

    auto intens_vol = ReadITKImageFromDisk<IntensityImageType>(input_img_path);

    vout << "thresholding... " << std::endl;
    using ThresholdType = itk::BinaryThresholdImageFilter<IntensityImageType, LabelImageType>;
    ThresholdType::Pointer thresholder = ThresholdType::New();
    thresholder->SetInput(intens_vol);

    if (has_lower)
    {
      thresholder->SetLowerThreshold(po.get("lower_bound").as_double());
    }
    if (has_upper)
    {
      thresholder->SetUpperThreshold(po.get("upper_bound").as_double());
    }

    thresholder->SetInsideValue(create_mesh_fn.labels[0]);
    thresholder->SetOutsideValue(0);
    thresholder->Update();

    label_img = thresholder->GetOutput();
  }

  vout << "ITK label info:" << std::endl;
  vout << "  origin: " << label_img->GetOrigin() << std::endl;
  vout << "  direction:\n" << label_img->GetDirection() << std::endl;
  vout << "  spacing: " << label_img->GetSpacing() << std::endl;

  LabelImageType::Pointer cur_label_img = label_img;

  if (flip_x || flip_y || flip_z)
  {
    vout << "flipping label image..." << std::endl;

    using ImageFlipper = itk::FlipImageFilter<LabelImageType>;
    ImageFlipper::Pointer flipper = ImageFlipper::New();
    flipper->SetInput(label_img);
    ImageFlipper::FlipAxesArrayType axes_to_flip;
    axes_to_flip[0] = flip_x;
    axes_to_flip[1] = flip_y;
    axes_to_flip[2] = flip_z;
    flipper->SetFlipAxes(axes_to_flip);
    flipper->Update();
    cur_label_img = flipper->GetOutput();
  }

  vout << "creating mesh..." << std::endl;
  auto mesh = create_mesh_fn(cur_label_img.GetPointer());

  if (swap_lps_ras)
  {
    vout << "LPS -> RAS..." << std::endl;
    FrameTransform lps2ras = FrameTransform::Identity();
    lps2ras.matrix()(0,0) = -1;
    lps2ras.matrix()(1,1) = -1;
    mesh.transform(lps2ras);
  }

  // Write mesh to disk
  vout << "writing mesh to disk..." << std::endl;
  WriteMeshToDisk(mesh, po.pos_args()[1], prefer_ascii);

  return kEXIT_VAL_SUCCESS;
}

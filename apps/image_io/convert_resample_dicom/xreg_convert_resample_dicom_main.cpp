/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Robert Grupp
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

#include <memory>

#include <fmt/format.h>

#include <itkImageFileReader.h>
#include <itkGDCMImageIO.h>
#include <itkResampleImageFilter.h>
#include <itkIdentityTransform.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkBSplineInterpolateImageFunction.h>
#include <itkConstantBoundaryCondition.h>
#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkDiscreteGaussianImageFilter.h>

#include "xregProgOptUtils.h"
#include "xregDICOMUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregITKBackgroundImageWriter.h"
#include "xregITKResampleUtils.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"
#include "xregVariableSpacedSlices.h"
#include "xregCSVUtils.h"

using namespace xreg;

template <class T>
void ReadDICOMPixelsResampleAndWriteVolume(const DICOMFIleBasicFieldsList& dcm_infos,
                                           const double dst_rs,
                                           const double dst_cs,
                                           const double dst_ss,
                                           const size_type out_of_plane_axis_dim,
                                           const std::string& img_path,
                                           const std::string& interp_name,
                                           ITKBackgroundImageWriter<itk::Image<T,3>>* img_writer,
                                           const bool const_out_of_plane_spacing,
                                           const bool skip_non_const_out_of_plane_spacing,
                                           const T default_pixel_val,
                                           const bool smooth_on_ds,
                                           const double smooth_sigma_x,
                                           const double smooth_sigma_y,
                                           const double smooth_sigma_z,
                                           const bool force_no_compression,
                                           std::ostream& vout)
{
  using VolumeType = itk::Image<T,3>;
  using VolumePointer = typename VolumeType::Pointer;

  using SliceType = itk::Image<T,2>;
  using SlicePointer = typename SliceType::Pointer;

  if (dcm_infos[0].sec_cap_dev_software_versions &&
      (dcm_infos[0].sec_cap_dev_software_versions->find("Amira") != std::string::npos))
  {
    std::cerr << "WARNING: THIS SERIES WAS EXPORTED FROM AMIRA - "
                 "LPS DIRECTIONS MAY NOT BE CORRECT IF RESAMPLING/WARPING WAS "
                 "PERFORMED!\n  File: "
              << dcm_infos[0].file_path << std::endl;
  }

  const CoordScalar src_row_spacing = dcm_infos[0].row_spacing;
  const CoordScalar src_col_spacing = dcm_infos[0].col_spacing;
  
  const size_type src_num_rows = dcm_infos[0].num_rows;
  const size_type src_num_cols = dcm_infos[0].num_cols;

  const size_type num_in_plane_pixels = src_num_rows * src_num_cols;

  Pt3 in_plane_col_dir = dcm_infos[0].col_dir;
  Pt3 in_plane_row_dir = dcm_infos[0].row_dir;
  Pt3 out_of_plane_dir = in_plane_col_dir.cross(in_plane_row_dir);

  {
    const bool has_pat_orient_strs = dcm_infos[0].pat_orient.has_value();

    const std::string pat_orient_x = has_pat_orient_strs ?
                                        StringStrip((*dcm_infos[0].pat_orient)[0]) : std::string();
    const std::string pat_orient_y = has_pat_orient_strs ?
                                        StringStrip((*dcm_infos[0].pat_orient)[1]) : std::string();
    
    if (dcm_infos[0].pat_pos)
    {
      const std::string pat_pos_str  = StringStrip(*dcm_infos[0].pat_pos);

      if (pat_pos_str == "HFS")  // head first supine, most common
      {
        // This should have pixels already in LPS
        if (has_pat_orient_strs && ((pat_orient_x != "L") || (pat_orient_y != "P")))
        {
          std::cerr << "WARNING: Patient position (" << pat_pos_str
                    << ") is inconsistent with patient orientation ("
                    << pat_orient_x << " , " << pat_orient_y << ')' << std::endl;
        }
      }
      else if(pat_pos_str == "HFP")  // head first prone, have seen a few of these
      {
        // This should have pixels in RAS
        if (has_pat_orient_strs && ((pat_orient_x != "R") || (pat_orient_y != "A")))
        {
          std::cerr << "WARNING: Patient position (" << pat_pos_str
                    << ") is inconsistent with patient orientation ("
                    << pat_orient_x << " , " << pat_orient_y << ')' << std::endl;
        }
      }
      //else if (pat_pos_str == "FFS")  // Feet first supine
      else
      {
        std::cerr << "WARNING: UNCOMMON PATIENT POSITON STRING: "
                  << pat_pos_str << std::endl;
      }
    }

    if (has_pat_orient_strs)
    {
      Pt3 std_x_axis;
      std_x_axis[0] = 1;
      std_x_axis[1] = 0;
      std_x_axis[2] = 0;

      Pt3 std_y_axis;
      std_y_axis[0] = 0;
      std_y_axis[1] = 1;
      std_y_axis[2] = 0;

      if (pat_orient_x == "L")
      {
        if ((in_plane_col_dir - std_x_axis).norm() > 1.0e-6)
        {
          std::cerr << "WARNING: PATIENT ORIENTATION OF X-AXIS (" << pat_orient_x
                    << ") IS NOT CONSISTENT WITH DIRECTION VECTOR ("
                    << in_plane_col_dir[0] << ',' << in_plane_col_dir[1]
                    << ',' << in_plane_col_dir[2] << ')' << std::endl;
        }
      }
      else if (pat_orient_x == "R")
      {
        if ((in_plane_col_dir + std_x_axis).norm() > 1.0e-6)
        {
          std::cerr << "WARNING: PATIENT ORIENTATION OF X-AXIS (" << pat_orient_x
                    << ") IS NOT CONSISTENT WITH DIRECTION VECTOR ("
                    << in_plane_col_dir[0] << ',' << in_plane_col_dir[1]
                    << ',' << in_plane_col_dir[2] << ')' << std::endl;
        }
      }
      else
      {
        std::cerr << "WARNING: UNCOMMON PATIENT ORIENTATION OF X-AXIS ("
                  << pat_orient_x << ')' << std::endl;
      }

      if (pat_orient_y == "P")
      {
        if ((in_plane_row_dir - std_y_axis).norm() > 1.0e-6)
        {
          std::cerr << "WARNING: PATIENT ORIENTATION OF Y-AXIS (" << pat_orient_y
                    << ") IS NOT CONSISTENT WITH DIRECTION VECTOR ("
                    << in_plane_row_dir[0] << ',' << in_plane_row_dir[1]
                    << ',' << in_plane_row_dir[2] << ')' << std::endl;
        }
      }
      else if (pat_orient_y == "A")
      {
        if ((in_plane_row_dir + std_y_axis).norm() > 1.0e-6)
        {
          std::cerr << "WARNING: PATIENT ORIENTATION OF Y-AXIS (" << pat_orient_y
                    << ") IS NOT CONSISTENT WITH DIRECTION VECTOR ("
                    << in_plane_row_dir[0] << ',' << in_plane_row_dir[1]
                    << ',' << in_plane_row_dir[2] << ')' << std::endl;
        }
      }
      else
      {
        std::cerr << "WARNING: UNCOMMON PATIENT ORIENTATION OF Y-AXIS ("
                  << pat_orient_y << ')' << std::endl;
      }
    }
  }

  if (out_of_plane_dir[out_of_plane_axis_dim] < 0)
  {
    std::cerr << "WARNING: Out-of-plane direction computed from DICOM is negative; "
                 "flipping an in-plane-direction sign!\n---> For: "
              << img_path << std::endl;

    // the slices have been sorted in ascending order in the out-of-plane direction,
    // but the current metadata indicates otherwise, find the in plane direction with
    // non-negative component and flip it.
    // Why the direction with non-negative component? This usually happens when the
    // patient is in some non-standard position (e.g. prone), and one direction is
    // already correctly flipped, but not the other

    out_of_plane_dir *= -1;

    if ((in_plane_col_dir[0] >= -1.0e-8) && (in_plane_col_dir[1] >= -1.0e-8) &&
        (in_plane_col_dir[2] >= 1.0e-8))
    {
      in_plane_col_dir *= -1;
    }
    else
    {
      in_plane_row_dir *= -1;
    }
  }

  // direction matrix used later when creating ITK image volumes
  Mat3x3 img_dir_mat;
  img_dir_mat.block(0,0,3,1) = in_plane_col_dir;
  img_dir_mat.block(0,1,3,1) = in_plane_row_dir;
  img_dir_mat.block(0,2,3,1) = out_of_plane_dir;

  VolumePointer src_vol;

  VolumePointer dst_vol;

  const size_type src_num_slices = dcm_infos.size();

  std::vector<SlicePointer> slices;
  std::vector<CoordScalar> slice_pos(src_num_slices);

  if (const_out_of_plane_spacing)
  {
    vout << "setting up the, constant slice-spaced, input volume size and geometry..."
         << std::endl;
    src_vol = VolumeType::New();

    // set the input size
    typename VolumeType::RegionType itk_img_region;
    itk_img_region.SetSize(0, src_num_cols);
    itk_img_region.SetSize(1, src_num_rows);
    itk_img_region.SetSize(2, src_num_slices);
    itk_img_region.SetIndex(0, 0);
    itk_img_region.SetIndex(1, 0);
    itk_img_region.SetIndex(2, 0);
    src_vol->SetRegions(itk_img_region);

    // set the output spacing
    typename VolumeType::SpacingType itk_img_spacing;
    itk_img_spacing[0] = src_col_spacing;
    itk_img_spacing[1] = src_row_spacing;
    itk_img_spacing[2] = (src_num_slices > 1) ?
                            (dcm_infos[1].img_pos_wrt_pat -
                             dcm_infos[0].img_pos_wrt_pat)[out_of_plane_axis_dim]
                                              : 0.0;
    src_vol->SetSpacing(itk_img_spacing);
    
    // set the output physical origin
    SetITKOriginPoint(src_vol.GetPointer(), dcm_infos[0].img_pos_wrt_pat);
    vout << "origin point:\n" << dcm_infos[0].img_pos_wrt_pat << std::endl;

    // set the direction matrix
    SetITKDirectionMatrix(src_vol.GetPointer(), img_dir_mat);

    vout << "allocating input volume buffer..." << std::endl;
    src_vol->Allocate();
  }
  else if (skip_non_const_out_of_plane_spacing)
  {
    vout << "skipping series with non-uniform slice spacing..." << std::endl;
    return;
  }
  else
  {
    slices.resize(src_num_slices);
  }

  vout << "reading slice pixel datas..." << std::endl;
  for (size_type slice_idx = 0; slice_idx < src_num_slices; ++slice_idx)
  {
    slice_pos[slice_idx] =
                  dcm_infos[slice_idx].img_pos_wrt_pat[out_of_plane_axis_dim];

    using SliceReader = itk::ImageFileReader<SliceType>;
    typename SliceReader::Pointer slice_reader = SliceReader::New();
    slice_reader->SetFileName(dcm_infos[slice_idx].file_path);

    itk::GDCMImageIO::Pointer gdcm_io = itk::GDCMImageIO::New();
    slice_reader->SetImageIO(gdcm_io);

    slice_reader->Update();

    SlicePointer cur_slice = slice_reader->GetOutput();

    if (const_out_of_plane_spacing)
    {
      // The out of plane spacing for this input is constant, therefore just
      // copy this slice directly into the corresponding location into the
      // allocated 3D volume.
      
      const size_type off = num_in_plane_pixels * slice_idx;

      memcpy(src_vol->GetBufferPointer() + off, cur_slice->GetBufferPointer(),
             sizeof(T) * num_in_plane_pixels);
    }
    else
    {
      // reset the directions according to the DICOM metadata -
      // it seems ITK doesn't set this properly, maybe because it's
      // 2D and DICOM specifies 3D directions?
      Mat2x2 slice_dir_mat;
      slice_dir_mat.block(0,0,2,1) = GetInPlanePt3D2D(
                                            in_plane_col_dir, out_of_plane_axis_dim);
      slice_dir_mat.block(0,1,2,1) = GetInPlanePt3D2D(
                                            in_plane_row_dir, out_of_plane_axis_dim);

      SetITKDirectionMatrix(cur_slice.GetPointer(), slice_dir_mat);

      // Also need to set the origin of each slice and the spacings;
      // I beleive this is a regression in ITK or the GDCM library, I really feel
      // like this information used to be correctly...

      SetITKOriginPoint(cur_slice.GetPointer(),
          GetInPlanePt3D2D(dcm_infos[slice_idx].img_pos_wrt_pat,
                           out_of_plane_axis_dim));

      typename SliceType::SpacingType in_slice_spacing;
      in_slice_spacing[0] = dcm_infos[slice_idx].col_spacing;
      in_slice_spacing[1] = dcm_infos[slice_idx].row_spacing;
      cur_slice->SetSpacing(in_slice_spacing);

      slices[slice_idx] = cur_slice;
    }
  }

  vout << "calculating destination image dimensions and geometry..." << std::endl;

  const bool keep_src_row_spacing   = dst_rs < 1.0e-6;
  const bool keep_src_col_spacing   = dst_cs < 1.0e-6;
  const bool keep_src_slice_spacing = dst_ss < 1.0e-6;

  const bool keep_src_all_spacing = keep_src_slice_spacing &&
                                    keep_src_row_spacing   &&
                                    keep_src_col_spacing;

  const CoordScalar src_first_slice_spacing = 
                              (dcm_infos[1].img_pos_wrt_pat[out_of_plane_axis_dim] -
                               dcm_infos[0].img_pos_wrt_pat[out_of_plane_axis_dim]);

  const CoordScalar dst_row_spacing   = keep_src_row_spacing ? src_row_spacing : dst_rs;
  const CoordScalar dst_col_spacing   = keep_src_col_spacing ? src_col_spacing : dst_cs;
  const CoordScalar dst_slice_spacing = keep_src_slice_spacing ?
                                              src_first_slice_spacing : dst_ss;

  const size_type dst_num_rows = static_cast<size_type>(
                           (src_num_rows * src_row_spacing / dst_row_spacing) + 0.5);
  
  const size_type dst_num_cols = static_cast<size_type>(
                           (src_num_cols * src_col_spacing / dst_col_spacing) + 0.5);
  
  const size_type dst_num_slices = 1 + static_cast<size_type>(
((dcm_infos[src_num_slices - 1].img_pos_wrt_pat[out_of_plane_axis_dim] -
  dcm_infos[0].img_pos_wrt_pat[out_of_plane_axis_dim]) / dst_slice_spacing) + 0.5);

  vout << "Output Size:"
       << "\n       Num Rows: " << dst_num_rows
       << "\n       Num Cols: " << dst_num_cols
       << "\n     Num Slices: " << dst_num_slices
       << "\n    Row Spacing: " << dst_row_spacing
       << "\n    Col Spacing: " << dst_col_spacing
       << "\n  Slice Spacing: " << dst_slice_spacing
       << std::endl;

   // set the output size
   typename VolumeType::RegionType itk_img_region;
   itk_img_region.SetSize(0, dst_num_cols);
   itk_img_region.SetSize(1, dst_num_rows);
   itk_img_region.SetSize(2, dst_num_slices);
   itk_img_region.SetIndex(0, 0);
   itk_img_region.SetIndex(1, 0);
   itk_img_region.SetIndex(2, 0);

  // set the output spacing
  typename VolumeType::SpacingType itk_img_spacing;
  itk_img_spacing[0] = dst_col_spacing;
  itk_img_spacing[1] = dst_row_spacing;
  itk_img_spacing[2] = dst_slice_spacing;


  if (const_out_of_plane_spacing)
  {
    if (keep_src_all_spacing)
    {
      vout << "slice spacings are constant and all destination spacings are "
              "equal to input, no interpolation required..." << std::endl;
      dst_vol = src_vol;
    }
    else
    {
      if (smooth_on_ds)
      {
        bool is_ds = false;

        double sigma[3] = { 0, 0, 0 };

        const double ds_factors[3] = { src_col_spacing / dst_col_spacing,
                                       src_row_spacing / dst_row_spacing,
                               src_first_slice_spacing / dst_slice_spacing };                   
        if ((ds_factors[0] - 1.0) < -1.0e-6)
        {
          vout << "downsampling in the X dimension, factor: " << ds_factors[0]
               << std::endl;
          
          is_ds = true;

          if (smooth_sigma_x > 1.0e-6)
          {
            sigma[0] = smooth_sigma_x;
          }
          else
          {
            sigma[0] = 0.5 / ds_factors[0];
            vout << "auto sigma_x = " << sigma[0] << std::endl;
          }
        }
        
        if ((ds_factors[1] - 1.0) < -1.0e-6)
        {
          vout << "downsampling in the Y dimension, factor: " << ds_factors[1]
               << std::endl;
          
          is_ds = true;

          if (smooth_sigma_y > 1.0e-6)
          {
            sigma[1] = smooth_sigma_y;
          }
          else
          {
            sigma[1] = 0.5 / ds_factors[1];
            vout << "auto sigma_y = " << sigma[1] << std::endl;
          }
        }
        
        if ((ds_factors[2] - 1.0) < -1.0e-6)
        {
          vout << "downsampling in the Z dimension, factor: " << ds_factors[2]
               << std::endl;
          
          is_ds = true;

          if (smooth_sigma_z > 1.0e-6)
          {
            sigma[2] = smooth_sigma_z;
          }
          else
          {
            sigma[2] = 0.5 / ds_factors[2];
            vout << "auto sigma_z = " << sigma[2] << std::endl;
          }
        }

        if (is_ds)
        {
          // std. dev. -> variance
          sigma[0] *= sigma[0];
          sigma[1] *= sigma[1];
          sigma[2] *= sigma[2];

          vout << "smoothing..." << std::endl;

          using SmoothFilter = itk::DiscreteGaussianImageFilter<VolumeType,VolumeType>;
          
          typename SmoothFilter::Pointer smoother = SmoothFilter::New();
          smoother->SetUseImageSpacing(false);
          smoother->SetVariance(sigma);
          smoother->SetInput(src_vol);
          smoother->Update();

          src_vol = smoother->GetOutput();
        }
      }

      vout << "setting up UNIFORM slice spacing interpolation..." << std::endl;

      using IdentityTransformType = itk::IdentityTransform<double, 3>;
      using ResampleFilterType    = itk::ResampleImageFilter<VolumeType, VolumeType>;
      using NNInterpFn            = itk::NearestNeighborInterpolateImageFunction<VolumeType>;
      using LinearInterpFn        = itk::LinearInterpolateImageFunction<VolumeType>;
      using BSplineInterpFn       = itk::BSplineInterpolateImageFunction<VolumeType>;

      IdentityTransformType::Pointer id_xform = IdentityTransformType::New();

      typename ResampleFilterType::Pointer resampler = ResampleFilterType::New();
      resampler->SetInput(src_vol);
      resampler->SetTransform(id_xform);
      resampler->SetOutputOrigin(src_vol->GetOrigin());
      resampler->SetOutputSpacing(itk_img_spacing);
      resampler->SetOutputDirection(src_vol->GetDirection());
      resampler->SetSize(itk_img_region.GetSize());
      resampler->SetDefaultPixelValue(default_pixel_val);

      if (interp_name == "nn")
      {
        typename NNInterpFn::Pointer nn_interp = NNInterpFn::New();
        resampler->SetInterpolator(nn_interp);
      }
      else if (interp_name == "linear")
      {
        typename LinearInterpFn::Pointer linear_interp = LinearInterpFn::New();
        resampler->SetInterpolator(linear_interp);
      }
      else
      {
        typename BSplineInterpFn::Pointer spline_interp = BSplineInterpFn::New();
        spline_interp->SetSplineOrder(3);

        resampler->SetInterpolator(spline_interp);
      }

      vout << "resampling..." << std::endl;
      resampler->Update();

      dst_vol = resampler->GetOutput();
    }
  }
  else
  {
    dst_vol = VolumeType::New();

    dst_vol->SetRegions(itk_img_region);

    dst_vol->SetSpacing(itk_img_spacing);

    // set the output physical origin
    SetITKOriginPoint(dst_vol.GetPointer(), dcm_infos[0].img_pos_wrt_pat);

    // set the direction matrix
    SetITKDirectionMatrix(dst_vol.GetPointer(), img_dir_mat);

    vout << "allocating destination volume buffer..." << std::endl;
    dst_vol->Allocate();

    vout << "setting up NON-uniform slice spacing interpolator object..."
         << std::endl;
    
    using SpacedSlices = VariableSpacedSlices<T>;
    using InterpMethod =  typename SpacedSlices::InterpMethodType;

    SpacedSlices spaced_slices(out_of_plane_axis_dim,
                               slices.begin(), slices.end(),
                               slice_pos.begin(), slice_pos.end(),
                               default_pixel_val);

    const InterpMethod interp_method = (interp_name == "nn") ?
                                          SpacedSlices::kNEAREST_NEIGHBOR :
                                          ((interp_name == "linear") ?
                                              SpacedSlices::kLINEAR :
                                              SpacedSlices::kCUBIC_BSPLINE);

    vout << "resampling..." << std::endl;
    spaced_slices.resample(dst_vol.GetPointer(), interp_method);

    slices.clear();  // free memory for the 2D slices
  }

  vout << "queuing writing output to disk..." << std::endl;
  img_writer->add(dst_vol, img_path, force_no_compression);
}

int main( int argc, char* argv[] )
{
  const int kEXIT_VAL_SUCCESS = 0;
  const int kEXIT_VAL_BAD_USE = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Given a root directory containing an aribtrarily orgainized "
              "collection of DICOM files, resample (optional), and convert "
              "each volume from a unique image series to another volume format "
              " (e.g. MHD or NIFTI). Output volumes have the following naming "
              "convention: <PatientID>_XX.<extension>, where XX is just a "
              "non-negative integer to distinguish between different scans of "
              "the same patient. When the user knows that the input directory "
              "only contains a single series, and therefore a single output, "
              "the second argument may specify a single file output path. "
              "See the help for the \"one\" flag.");
  po.set_arg_usage("<Input Root Directory> <Output Directory/File>");

  po.set_min_num_pos_args(2);

  po.add("rs", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "rs",
         "The desired in-plane row spacing; when not provided the value defined "
         "in the DICOM is used. This is typically spacing in the Y direction "
         "for LPS coordinates.");

  po.add("cs", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "cs",
         "The desired in-plane column spacing; when not provided the value defined "
         "in the DICOM is used. This is typically spacing in the X direction "
         "for LPS coordinates.");

  po.add("ss", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "ss",
         "The desired out-of-plane slice spacing; when not provided the spacing "
         "between the first and second slices is used. It is very common for "
         "clinical data sets to have variable slice spacings, so the default "
         "value is not always the best choice. This is typically the Z direction "
         "for LPS coordinates.");

  po.add("iso", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "iso",
         "The desired isotropic spacing; disjoint from \"rs\" , \"cs\" , \"ss\"");

  po.add("interp", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "interp",
         "The interpolation algorithm to use. \"nn\" -> Nearest Neighbor, "
         "\"linear\" -> Tri-Linear, \"spline\" -> Cubic B-Spline.")
    << "linear";

  po.add("default-val", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "default-val",
         "Override the default pixel value to use when interpolating outside of the set of "
         "support of the input DICOM. When not set, the default pixel value is -1000 for CT, "
         "0 for MR, and 0 for all other modalities.");

  po.add("no-smooth-on-ds", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-smooth-on-ds",
         "Do NOT smooth the input volume prior to downsampling; skipping smoothing may induce aliasing. "
         "If upsampling, or there is no change in resolution, then no smoothing is "
         "performed. Smoothing is currently only supported for series with a constant "
         "slice spacing.")
    << false;

  po.add("smooth-sigma-x", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "smooth-sigma-x",
         "The smoothing std. dev. to use in the x direction. A value less than, "
         "or equal to, zero will autocompute sigma to be 0.5 / ds factor.")
    << 0.0;

  po.add("smooth-sigma-y", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "smooth-sigma-y",
         "The smoothing std. dev. to use in the y direction. A value less than, "
         "or equal to, zero will autocompute sigma to be 0.5 / ds factor.")
    << 0.0;

  po.add("smooth-sigma-z", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "smooth-sigma-z",
         "The smoothing std. dev. to use in the z direction. A value less than, "
         "or equal to, zero will autocompute sigma to be 0.5 / ds factor.")
    << 0.0;

  po.add("double", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "double",
         "Use double precision floating point pixel values (default is single"
         "precision).")
    << false;

  po.add("out-format", 'o', ProgOpts::kSTORE_STRING, "out-format",
         "The output file format, this should be a file extension (without the "
         "leading period), that is recognized by the ITK Image Writer.")
    << "nii.gz";

  po.add("max-GB", 'm', ProgOpts::kSTORE_DOUBLE, "max-GB",
         "The maximum number of GB of raw image buffers to queue for writing "
         "in the background. When this limit is reached, image writes become "
         "blocking, so that system memory is not exhausted. 0 -> no limit.")
    << 0.0;

  po.add("writer-threads", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32, "writer-threads",
         "The number of background threads to use when writing volume files to disk. "
         "A value of 0 uses a number of threads equal to the number of virtual cores "
         "detected on the system.")
    << ProgOpts::uint32(0);

  po.add("one", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "one",
         "Indicates that the input directory contains a single series and thus "
         "the output argument is a path to a file. If the file is specified without "
         "an extension, then the extension is inferred from the \"out-format\" "
         "flag and appended to the path. When an extension is present the "
         "\"out-format\" flag is ignored. With this flag, only DICOM files directly "
         "in the input directory are used; no sub-directories are traversed.")
      << false;

  po.add("slice-space-tol", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "slice-space-tol",
         "The tolerance used when comparing slice spacings to determine if the "
         "spacings remain constant a series.")
      << 1.0e-3;

  po.add("force-uniform", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "force-uniform",
         "Forces uniform slice spacing - be careful!")
      << false;

  po.add("force-non-uniform", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "force-non-uniform", "Forces the non-uniform interpolation code to be run."
         " This should produce valid output no matter what, but this flag is "
         "intended to be used as a debugging tool.")
      << false;
  
  po.add("skip-non-uniform", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "skip-non-uniform",
         "Skip converting scans which have non-uniform slice spacing.")
    << false;

  po.add("force-no-compress", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "force-no-compress",
         "Forces the output volumes to be written in uncompressed format, when "
         "allowed by the format in use. This could be used to write un-compressed "
         ".raw files to match with .mhd (the default is to write compressed .zraw).")
      << false;

  po.add("out-of-plane-upper", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "out-of-plane-upper",
         "Maximum physical value allowed in the out of plane direction; useful for "
         "discarding slices that are outside the region of interest on a full body "
         "CT.");

  po.add("out-of-plane-lower", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE,
         "out-of-plane-lower",
         "Minimum physical value allowed in the out of plane direction; useful for "
         "discarding slices that are outside the region of interest on a full body "
         "CT.");

  po.add("summary-only", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "summary-only",
         "Print a summary of the organized DICOM files to standard output, do NOT perform "
         "any resampling or conversion.")
    << false;

  po.add("pat-id", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pat-id",
         "Limit processing to a specific patient ID.")
    << "";

  po.add("study-uid", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "study-uid",
         "Limit processing to a specific study UID.")
    << "";

  po.add("series-uid", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "series-uid",
         "Limit processing to a specific series UID.")
    << "";

  po.add("no-name-w-pat-id", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-name-w-pat-id",
         "Do NOT include the patient ID for naming output files. Use care with this option as "
         "datasets may have different patient IDs with the same patient name, which could "
         "possibily result in several series mapping to the same output file. This could result "
         "in not all series being written out to disk WITHOUT WARNING.")
    << false;

  po.add("name-w-pat-name", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "name-w-pat-name",
         "Include patient name for naming output files.")
    << false;

  po.add("name-w-desc", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "name-w-desc",
         "Include study and series descriptions for naming output files.")
    << false;

  po.add("name-w-conv", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "name-w-conv",
         "Include convolution kernel for naming output files (useful for distinguishing between "
         "soft-tissue and bone reconstructions).")
    << false;

  po.add("include-localizer", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "include-localizer",
         "Include localizer images, by default they are detected and ignored. "
         "This flag is ignored when the \"--one\" flag is passed.")
    << false;

  po.add("include-multiframe-files", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "include-multiframe-files",
         "Include multi-frame DICOM files "
         "(e.g. the file contains a 3D volume compared to a 2D slice of a volume "
         "or a sequence of 2D images.)")
    << false;

  po.add("exclude-derived", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "exclude-derived",
         "Do NOT include DERIVED images (e.g. images with pixel values derived from other images).")
    << false;
  
  po.add("exclude-secondary", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "exclude-secondary",
         "Do NOT include SECONDARY images (e.g. images created after initial exam).")
    << false;

  po.add("modalities", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "modalities",
         "A comma delimited string indicating which modalities should be considered. "
         "An empty string (the default) considers all modalities. "
         "This is case-sensitive - upper case strings are standard. "
         "Examples: \"CT\" will only convert CT datasets, "
         "\"CT,MR\" will convert CT and MR datasets.")
    << "";

  po.add("pat-lut", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pat-lut",
         "Path to a CSV file which serves as a LUT between DICOM patient IDs and strings "
         "used to prefix output file names. The first column are the DICOM patient IDs "
         "and the second column are the new strings to be used.")
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

  const bool verbose = po.get("verbose");
  std::ostream& vout = po.vout();

  const std::string input_root_dir = po.pos_args()[0];
  const std::string output_dir     = po.pos_args()[1];

  const bool has_dst_row_spacing   = po.has("rs");
  const bool has_dst_col_spacing   = po.has("cs");
  const bool has_dst_slice_spacing = po.has("ss");

  const bool has_iso_spacing = po.has("iso");

  if (has_iso_spacing && (has_dst_row_spacing || has_dst_col_spacing ||
      has_dst_slice_spacing))
  {
    std::cerr << "Cannot specify both an isotropic spacing and a spacing "
                 "in one or more dimensions!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  double dst_row_spacing   = has_dst_row_spacing ? po.get("rs").as_double() : 0.0;
  double dst_col_spacing   = has_dst_col_spacing ? po.get("cs").as_double() : 0.0;
  double dst_slice_spacing = has_dst_slice_spacing ? po.get("ss").as_double() : 0.0;

  if (has_iso_spacing)
  {
    const double iso_spacing = po.get("iso");
    dst_row_spacing   = iso_spacing;
    dst_col_spacing   = iso_spacing;
    dst_slice_spacing = iso_spacing;
  }
  else
  {
    dst_row_spacing   = has_dst_row_spacing ? po.get("rs").as_double() : 0.0;
    dst_col_spacing   = has_dst_col_spacing ? po.get("cs").as_double() : 0.0;
    dst_slice_spacing = has_dst_slice_spacing ? po.get("ss").as_double() : 0.0;
  }

  const std::string interp_name = po.get("interp").as_string();
  if ((interp_name != "nn") && (interp_name != "linear") && (interp_name != "spline"))
  {
    std::cerr << "Unsupported interpolation method: " << interp_name << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const std::string out_file_ext = po.get("out-format").as_string();

  const bool use_double = po.get("double");

  const double max_img_queue_GB = po.get("max-GB");
  
  const size_type img_writer_num_threads = po.get("writer-threads").as_uint32();

  const bool override_default_pixel_val = po.has("default-val");
  const double passed_default_pixel_val = override_default_pixel_val ? po.get("default-val").as_double() : 0.0;

  const bool one_image_input = po.get("one");

  const double slice_space_tol = po.get("slice-space-tol");

  const bool force_uni_slice_spacing = po.get("force-uniform");
  const bool force_non_uni_interp = po.get("force-non-uniform");

  if ((int(force_non_uni_interp) + int(force_uni_slice_spacing)) > 1)
  {
    std::cerr << "Cannot specify both force uniform and force non-uniform flags!"
              << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  const bool skip_non_uniform = po.get("skip-non-uniform");

  const bool smooth_on_ds = !po.get("no-smooth-on-ds").as_bool();

  const double smooth_sigma_x = po.get("smooth-sigma-x");
  const double smooth_sigma_y = po.get("smooth-sigma-y");
  const double smooth_sigma_z = po.get("smooth-sigma-z");

  const bool force_no_compress = po.get("force-no-compress");

  const bool has_out_of_plane_upper = po.has("out-of-plane-upper");
  const bool has_out_of_plane_lower = po.has("out-of-plane-lower");

  const double out_of_plane_upper = has_out_of_plane_upper ?
                                      po.get("out-of-plane-upper").as_double() : 0.0;
  const double out_of_plane_lower = has_out_of_plane_lower ?
                                      po.get("out-of-plane-lower").as_double() : 0.0;

  const bool summary_only = po.get("summary-only");

  const std::string single_pat_id       = po.get("pat-id");
  const bool        limit_single_pat    = !single_pat_id.empty();
  const std::string single_study_uid    = po.get("study-uid");
  const bool        limit_single_study  = !single_study_uid.empty();
  const std::string single_series_uid   = po.get("series-uid");
  const bool        limit_single_series = !single_series_uid.empty();

  const bool use_pat_id_for_name = !po.get("no-name-w-pat-id").as_bool();

  const bool use_pat_name_for_name = po.get("name-w-pat-name");

  const bool use_desc_for_name = po.get("name-w-desc");
  const bool use_conv_for_name = po.get("name-w-conv");
  
  const bool inc_localizers = po.get("include-localizer");

  const bool inc_multiframe_files = po.get("include-multiframe-files");

  const bool exclude_derived = po.get("exclude-derived");
  
  const bool exclude_secondary = po.get("exclude-secondary");

  const std::string pat_id_lut_path = po.get("pat-lut");

  const std::string modalities_to_consider_str = po.get("modalities");

  const auto modalities_to_consider = StringSplit(modalities_to_consider_str, ",");

  const bool use_pat_id_lut = !pat_id_lut_path.empty();

  vout << "Inputs: "
       << "\n         Input Root Directory: " << input_root_dir
       << "\n        Output Directory/File: " << output_dir
       << "\n        Isotropic Flag Passed: " << has_iso_spacing
       << "\n           Output Row Spacing: "
       << ((has_iso_spacing || has_dst_row_spacing) ?
           fmt::sprintf("%f", dst_row_spacing) : std::string("From DICOM"))
       << "\n           Output Col Spacing: "
       << ((has_iso_spacing || has_dst_col_spacing) ?
           fmt::sprintf("%f", dst_col_spacing) : std::string("From DICOM"))
       << "\n         Output Slice Spacing: "
       << ((has_iso_spacing || has_dst_slice_spacing) ?
           fmt::sprintf("%f", dst_slice_spacing) : std::string("From DICOM"))
       << "\n                Interpolation: " << interp_name
       << "\n Override Default Pixel Value: " << (override_default_pixel_val ?
                                                    fmt::format("{}", passed_default_pixel_val) :
                                                    std::string("No"))
       << "\n           Output File Format: " << out_file_ext
       << "\n          Double Float Pixels: " << use_double
       << "\n     Image Write Queue Max GB: " << max_img_queue_GB
       << "\n              One Image Input: " << one_image_input
       << "\n    Image Writer Num. Threads: " << img_writer_num_threads
       << "\n      Slice Spacing Tolerance: " << slice_space_tol
       << "\n     Force Uni. Slice Spacing: " << force_uni_slice_spacing
       << "\n  Force No Uni. Slice Spacing: " << force_uni_slice_spacing
       << "\n  Skip Non Uni. Slice Spacing: " << skip_non_uniform
       << "\n         Smooth on Downsample: " << smooth_on_ds
       << "\n               Smooth Sigma X: " << ((smooth_sigma_x > 1.0e-6) ?
                       fmt::sprintf("%f", smooth_sigma_x) : std::string("auto"))
       << "\n               Smooth Sigma Y: " << ((smooth_sigma_y > 1.0e-6) ?
                       fmt::sprintf("%f", smooth_sigma_y) : std::string("auto"))
       << "\n               Smooth Sigma Z: " << ((smooth_sigma_z > 1.0e-6) ?
                       fmt::sprintf("%f", smooth_sigma_z) : std::string("auto"))
       << "\n         Force No Compression: " << force_no_compress
       << "\n                Summary Only?: " << summary_only
       << "\n            Single Patient ID: " << (limit_single_pat    ? single_pat_id.c_str()     : "(All Patients)")
       << "\n             Single Study UID: " << (limit_single_study  ? single_study_uid.c_str()  : "(All Studies)")
       << "\n            Single Series UID: " << (limit_single_series ? single_series_uid.c_str() : "(All Series)")
       << "\n      Use Patient ID for Name: " << use_pat_id_for_name
       << "\n    Use Patient Name for Name: " << use_pat_name_for_name
       << "\n  Use Study/Series Desc. Name: " << use_desc_for_name
       << "\n         Use Conv. Kern. Name: " << use_conv_for_name
       << "\n           Include Localizers: " << inc_localizers
       << "\n    Include Multi-Frame Files: " << inc_multiframe_files
       << "\n        Exclude Derived Files: " << exclude_derived
       << "\n      Exclude Secondary Files: " << exclude_secondary
       << "\n        Modalities Considered: " << (modalities_to_consider_str.empty() ? "<ALL>" : modalities_to_consider_str.c_str())
       << "\n           Use Patient ID LUT: " << use_pat_id_lut
       << "\n-----------------------------------------------------\n" << std::endl;

  std::unordered_map<std::string,std::string> pat_id_lut;
  
  if (use_pat_id_lut)
  {
    vout << "reading patient ID LUT from CSV..." << std::endl;
    
    // false --> no header
    const auto csv_file = ReadCSVFileStr(pat_id_lut_path, false);
    
    for (const auto& csv_line : csv_file)
    {
      xregASSERT(csv_line.size() == 2);
      
      vout << "  " << csv_line[0] << "  --> " << csv_line[1] << std::endl;

      pat_id_lut.emplace(csv_line[0], csv_line[1]);
    }
  }

  OrganizedDICOMFiles org_dcm;

  if (!one_image_input)
  {
    vout << "reading directory tree and organizing..." << std::endl;
    GetOrgainizedDICOMInfos(input_root_dir, &org_dcm,
                            inc_localizers, inc_multiframe_files,
                            !exclude_secondary, !exclude_derived,
                            modalities_to_consider);
  }
  else
  {
    vout << "getting DICOMs in single directory..." << std::endl;

    // get the files and shoe-horn them into the organized structure
    PathList dcm_paths;
    GetDICOMFilePathObjsInDir(input_root_dir, &dcm_paths);

    const size_type num_dcm = dcm_paths.size();
    vout << "found " << num_dcm << " DICOM files" << std::endl;

    if (num_dcm)
    {
      PathStringList dcm_str_paths(num_dcm);
      for (size_type dcm_idx = 0; dcm_idx < num_dcm; ++dcm_idx)
      {
        dcm_str_paths[dcm_idx] = dcm_paths[dcm_idx].string();
      }

      org_dcm.root_dir = input_root_dir;

      // manually assign all DICOM files to a single patient and study
      DICOMFIleBasicFields dcm_info = ReadDICOMFileBasicFields(dcm_str_paths[0]);

      org_dcm.patient_infos[dcm_info.patient_id]
                        [dcm_info.study_uid][dcm_info.series_uid] = dcm_str_paths;
    }
  }

  if (summary_only)
  {
    PrintOrganizedDICOMFiles(org_dcm, std::cout, true);
    
    return kEXIT_VAL_SUCCESS;
  }

  using ImageWriterBgSingle = ITKBackgroundImageWriter<itk::Image<float,3>>;
  using ImageWriterBgDouble = ITKBackgroundImageWriter<itk::Image<double,3>>;

  std::unique_ptr<ImageWriterBgSingle> img_writer_single;
  std::unique_ptr<ImageWriterBgDouble> img_writer_double;

  const size_type max_img_queue_bytes =
    static_cast<size_type>((max_img_queue_GB * 1024 * 1024 * 1024) + 0.5);

  if (use_double)
  {
    img_writer_double.reset(new ImageWriterBgDouble(img_writer_num_threads));
    img_writer_double->set_memory_limit(max_img_queue_bytes);
  }
  else
  {
    img_writer_single.reset(new ImageWriterBgSingle(img_writer_num_threads));
    img_writer_single->set_memory_limit(max_img_queue_bytes);
  }

  // Setup a mapping to replace characters which are invalid file name tokens
  auto file_name_char_remap = IdentityCharMap();
  file_name_char_remap[' ']  = '_';
  file_name_char_remap['.']  = '_';
  file_name_char_remap[',']  = '_';
  file_name_char_remap['?']  = '_';
  file_name_char_remap[':']  = '_';
  file_name_char_remap[';']  = '_';
  file_name_char_remap['\''] = '_';
  file_name_char_remap['\"'] = '_';
  file_name_char_remap['\t'] = '_';
  file_name_char_remap['\r'] = '_';
  file_name_char_remap['\n'] = '_';
  file_name_char_remap['\\'] = '_';
  file_name_char_remap['/']  = '_';
  file_name_char_remap['|']  = '_';
  file_name_char_remap['~']  = '_';
  file_name_char_remap['`']  = '_';
  file_name_char_remap['!']  = '_';
  file_name_char_remap['@']  = '_';
  file_name_char_remap['#']  = '_';
  file_name_char_remap['$']  = '_';
  file_name_char_remap['%']  = '_';
  file_name_char_remap['^']  = '_';
  file_name_char_remap['&']  = '_';
  file_name_char_remap['*']  = '_';
  file_name_char_remap['+']  = '_';
  file_name_char_remap['=']  = '_';
  file_name_char_remap['(']  = '_';
  file_name_char_remap[')']  = '_';
  file_name_char_remap['[']  = '_';
  file_name_char_remap[']']  = '_';
  file_name_char_remap['{']  = '_';
  file_name_char_remap['}']  = '_';
  file_name_char_remap['<']  = '_';
  file_name_char_remap['>']  = '_';

  if (false)
  {
    vout << "Filename char remap:\n";
    for (int i = 0; i < 256; ++i)
    {
      vout << '\'' << static_cast<char>(i) << '\'' << " --> " << file_name_char_remap[i] << '\n';
    }
    vout << std::endl;
  }

  // Replace invalid/undesired characters in a path string using the file_name_char_remap LUT.
  // When the optional variable for the input string indicates the value is not available, then
  // and empty string is returned
  auto get_valid_name_for_path = [file_name_char_remap] (const boost::optional<std::string>& s)
  {
    std::string ss;
    
    if (s)
    {
      ss = MapChars(*s, file_name_char_remap);
    }

    return ss;
  };

  // for each patient ID
  for (const auto& patid2studyuid : org_dcm.patient_infos)
  {
    const std::string pat_id_str = StringStrip(patid2studyuid.first);

    std::string pat_id_str_remap;
    
    if (use_pat_id_lut)
    {
      auto it = pat_id_lut.find(pat_id_str);

      if (it != pat_id_lut.end())
      {
        pat_id_str_remap = it->second;
      }
      else
      {
        std::cerr << "WARNING: patient ID (" << pat_id_str << ") not found in LUT, not remapping..." << std::endl;
        pat_id_str_remap = pat_id_str;
      }
    }
    else
    {
      pat_id_str_remap = pat_id_str;
    }

    if (limit_single_pat && (single_pat_id != pat_id_str))
    {
      vout << "skipping patient ID: " << pat_id_str << std::endl;
      continue;
    }

    size_type cur_img_idx = 0;

    const OrganizedDICOMFiles::StudyUIDToSeriesUIDInfosMap& studyuid_map =
                                                          patid2studyuid.second;

    // for each study uid
    for (const auto& studyuid2seriesuid : studyuid_map)
    {
      const std::string& cur_study_uid = studyuid2seriesuid.first;

      if (limit_single_study && (single_study_uid != cur_study_uid))
      {
        vout << "  skipping study UID: " << cur_study_uid << std::endl;
        continue;
      }
      
      const OrganizedDICOMFiles::SeriesUIDToFilePathsMap& seriesuid_map =
                                                        studyuid2seriesuid.second;

      // for each series uid
      for (auto seriesuid2files_it = seriesuid_map.begin();
           seriesuid2files_it != seriesuid_map.end();
           ++seriesuid2files_it, ++cur_img_idx)
      {
        const std::string& cur_series_uid = seriesuid2files_it->first;

        if (limit_single_series && (single_series_uid != cur_series_uid))
        {
          vout << "    skipping series UID: " << cur_series_uid << std::endl;
          continue;
        }
        
        const PathStringList& paths = seriesuid2files_it->second;
        const size_type num_dcm = paths.size();

        vout << "Starting processing for Patient ID: " << pat_id_str
             << (use_pat_id_lut ? fmt::format(" (Remapped: {}) ", pat_id_str_remap) : std::string())
             << ", image index: " << cur_img_idx << ", num dcm files: "
             << num_dcm << std::endl;

        DICOMFIleBasicFieldsList dcm_infos(num_dcm);

        for (size_type dcm_idx = 0; dcm_idx < num_dcm; ++dcm_idx)
        {
          dcm_infos[dcm_idx] = ReadDICOMFileBasicFields(paths[dcm_idx]);
        }

        ReorderAndCheckDICOMInfos dcm_check_reorder;
        dcm_check_reorder.out_of_plane_spacing_tol = slice_space_tol;
        dcm_check_reorder.set_debug_output_stream(vout, verbose);

        DICOMFIleBasicFieldsList dcm_infos_sorted;
        if (dcm_check_reorder(dcm_infos, &dcm_infos_sorted))
        {
          // discard slices outside of ROI

          if (has_out_of_plane_lower)
          {
            auto begin_lower_it = dcm_infos_sorted.begin();
            
            vout << "checking for slices below the lower boundary..." << std::endl;
            for (; begin_lower_it != dcm_infos_sorted.end(); ++begin_lower_it)
            {
              if (begin_lower_it->img_pos_wrt_pat[
                      dcm_check_reorder.out_of_plane_axis_dim] > out_of_plane_lower)
              {
                break;
              }
            }

            if (begin_lower_it != dcm_infos_sorted.begin())
            {
              dcm_infos_sorted.erase(dcm_infos_sorted.begin(), begin_lower_it);
            }
          }

          if (has_out_of_plane_upper)
          {
            auto begin_upper_rit  = dcm_infos_sorted.rbegin();
            
            vout << "checking for slices above the upper boundary..." << std::endl;
            for (; begin_upper_rit != dcm_infos_sorted.rend(); ++begin_upper_rit)
            {
              if (begin_upper_rit->img_pos_wrt_pat[
                      dcm_check_reorder.out_of_plane_axis_dim] < out_of_plane_upper)
              {
                break;
              }
            }

            if (begin_upper_rit != dcm_infos_sorted.rbegin())
            {
              auto end_it = dcm_infos_sorted.end();

              dcm_infos_sorted.erase(end_it -
                                       (begin_upper_rit - dcm_infos_sorted.rbegin()),
                                     end_it);
            }
          }

          if (dcm_infos_sorted.size() > 1)
          {
            const auto& first_dcm = dcm_infos_sorted[0];
            
            vout << "DICOM values:\n";
            PrintDICOMFileBasicFields(first_dcm, vout, " ");
            vout << "     Slice Spacing: "
                 << (dcm_check_reorder.const_out_of_plane_spacing ?
              fmt::sprintf("%f", dcm_check_reorder.out_of_plane_spacing_const_val)
                                                                  :
                                                             std::string("Variable"))
                 << "\n        Num Slices: " << dcm_infos_sorted.size()
                 << std::endl;

            if ((!has_dst_slice_spacing && !has_iso_spacing) &&
                !dcm_check_reorder.const_out_of_plane_spacing)
            {
              std::cerr << "WARNING: OUTPUT SLICE SPACING HAS NOT BEEN SPECIFIED "
                           "AND DICOM HAS VARIABLE SLICE SPACING, THE SLICING "
                           "BETWEEN THE FIRST TWO SLICES WILL BE USED!"
                        << std::endl;
            }

            Path dst_file_path(output_dir);

            if (!one_image_input)
            {
              std::stringstream dst_file_name_ss;
            
              if (use_pat_id_for_name)
              {
                dst_file_name_ss << get_valid_name_for_path(pat_id_str_remap);
                dst_file_name_ss << '_';
              }
              
              if (use_pat_name_for_name)
              {
                dst_file_name_ss << get_valid_name_for_path(first_dcm.patient_name)
                                 << '_';
              }

              dst_file_name_ss << fmt::format("{:02d}", cur_img_idx);

              if (use_desc_for_name)
              {
                dst_file_name_ss << '_'
                                 << get_valid_name_for_path(first_dcm.study_desc)
                                 << get_valid_name_for_path(first_dcm.series_desc);
              }
              
              if (use_conv_for_name)
              {
                dst_file_name_ss << '_'
                                 << get_valid_name_for_path(first_dcm.conv_kernel);
              }
              
              dst_file_name_ss << '.' << out_file_ext;

              dst_file_path += dst_file_name_ss.str();
            }
            else if (dst_file_path.file_extension().empty())
            {
              vout << "appending file extension for single image output..."
                   << std::endl;
              dst_file_path += fmt::sprintf(".%s", out_file_ext);
            }
          
            double default_pixel_val = 0;
            
            if (override_default_pixel_val)
            {
              default_pixel_val = passed_default_pixel_val;
            }
            else if (first_dcm.modality == "CT")
            {
              default_pixel_val = -1000;
            }
            else if (first_dcm.modality == "MR")
            {
              default_pixel_val = 0;
            }
            
            vout << "Using default pixel value of " << default_pixel_val << " for boundary cases." << std::endl;

            // use constant out of plane spacing if we are forcing it
            //   OR
            // we are not forcing non uniform and the spacing actually is constant
            const bool use_const_out_of_plane_spacing = force_uni_slice_spacing ||
                                       (!force_non_uni_interp && 
                                        dcm_check_reorder.const_out_of_plane_spacing);

            if (use_double)
            {
              ReadDICOMPixelsResampleAndWriteVolume<double>(dcm_infos_sorted,
                                                            dst_row_spacing,
                                                            dst_col_spacing,
                                                            dst_slice_spacing,
                                                            dcm_check_reorder.out_of_plane_axis_dim,
                                                            dst_file_path.string(),
                                                            interp_name,
                                                            img_writer_double.get(),
                                                            use_const_out_of_plane_spacing,
                                                            skip_non_uniform,
                                                            default_pixel_val,
                                                            smooth_on_ds,
                                                            smooth_sigma_x,
                                                            smooth_sigma_y,
                                                            smooth_sigma_z,
                                                            force_no_compress,
                                                            vout);
            }
            else
            {
              ReadDICOMPixelsResampleAndWriteVolume<float>(dcm_infos_sorted,
                                                           dst_row_spacing,
                                                           dst_col_spacing,
                                                           dst_slice_spacing,
                                                           dcm_check_reorder.out_of_plane_axis_dim,
                                                           dst_file_path.string(),
                                                           interp_name,
                                                           img_writer_single.get(),
                                                           use_const_out_of_plane_spacing,
                                                           skip_non_uniform,
                                                           default_pixel_val,
                                                           smooth_on_ds,
                                                           smooth_sigma_x,
                                                           smooth_sigma_y,
                                                           smooth_sigma_z,
                                                           force_no_compress,
                                                           vout);
            }
          }
          else
          {
            std::cerr << "only one slice available... skipping!" << std::endl;
          }
        }
      }
    }
  }

  vout << "Finished resampling... may have to wait for images to finish writing!"
       << std::endl;

  return kEXIT_VAL_SUCCESS;
}

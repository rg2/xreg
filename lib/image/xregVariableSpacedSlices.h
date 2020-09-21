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

#ifndef XREGVARIABLESPACEDSLICES_H_
#define XREGVARIABLESPACEDSLICES_H_

#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include <itkImage.h>
#include <itkImageRegionIterator.h>

#include "xregCommon.h"
#include "xregExceptionUtils.h"
#include "xregTBBUtils.h"

namespace xreg
{

/// \brief Helper function to retrieve the 2D in-plane components from a 3D point.
Pt2 GetInPlanePt3D2D(const Pt3& p, const unsigned out_of_plane_dim);

/// \brief Represents a stack of 2D image slices, which may be unevenly spaced,
///        and allows interpolating at arbitrary locations.
///
/// Originally written to resample DICOM series.
template <class tPixelType>
class VariableSpacedSlices
{
public:
  typedef std::size_t size_type;

  typedef tPixelType PixelType;
  typedef itk::Image<PixelType,2> SliceType;
  typedef typename SliceType::Pointer SlicePointer;
  typedef itk::Image<PixelType,3> VolumeType;

  typedef int InterpMethodType;

  static const InterpMethodType kNEAREST_NEIGHBOR = 0;
  static const InterpMethodType kLINEAR           = 1;
  static const InterpMethodType kCUBIC_BSPLINE    = 2;

  template <class tSliceItr, class tCoordItr>
  VariableSpacedSlices(const size_type out_of_plane_dim,
                       tSliceItr slice_begin, tSliceItr slice_end,
                       tCoordItr slice_loc_begin, tCoordItr slice_loc_end,
                       const PixelType& default_val)
    : out_of_plane_dim_(out_of_plane_dim),
      slices_(slice_begin, slice_end),
      slice_locs_(slice_loc_begin, slice_loc_end),
      num_slices_(slices_.size()),
      default_val_(default_val)
  {
    xregASSERT(out_of_plane_dim_ < 3);
    xregASSERT(num_slices_ > 0);
    xregASSERT(num_slices_ == slice_locs_.size());

    if (num_slices_ > 1)
    {
      // TODO: the pads should really use one half of the spacings?
      lower_pad_ = slice_locs_[0] - (slice_locs_[1] - slice_locs_[0]);
      upper_pad_ = slice_locs_[num_slices_ - 1] + (slice_locs_[num_slices_ - 1]
                    - slice_locs_[num_slices_ - 2]);
    }
    else
    {
      // no mercy - very strange case to encounter, why would anyone interpolate
      // off the plane?
      lower_pad_ = upper_pad_ = slice_locs_[0];
    }

    const typename SliceType::SizeType slice_size =
                                      slices_[0]->GetLargestPossibleRegion().GetSize();
    slice_num_cols_ = slice_size[0];
    slice_num_rows_ = slice_size[1];
  }

  struct SliceIndexHint
  {
    bool valid;
    size_type lower;
    size_type upper;
  };

  PixelType interp_cubic_bspline(const Pt3& p, SliceIndexHint* hint = 0) const
  {
    xregThrow("Cubic B-Spline Interpolation NOT Implemented for non-uniform grid!");
    return default_val_;
  }

  PixelType interp_linear(const Pt3& p, SliceIndexHint* hint = 0) const
  {
    PixelType val = default_val_;

    size_type in_plane_dim1 = 0;
    size_type in_plane_dim2 = 0;
    get_in_plane_dims(&in_plane_dim1, &in_plane_dim2);

    size_type slice_idx_lower = kINVALID_SLICE_IDX;
    size_type slice_idx_upper = kINVALID_SLICE_IDX;

    const CoordScalar p_out_of_plane = p[out_of_plane_dim_];

    if (hint && hint->valid)
    {
      slice_idx_upper = hint->upper;
      slice_idx_lower = hint->lower;
    }
    else
    {
      CoordScalarListConstIterator out_of_plane_upper_it =
              std::upper_bound(slice_locs_.begin(), slice_locs_.end(), p_out_of_plane);
      if (out_of_plane_upper_it != slice_locs_.end())
      {
        // There is some slice that lies above this point

        CoordScalarListConstIterator slice_locs_begin_it = slice_locs_.begin();

        slice_idx_upper = out_of_plane_upper_it - slice_locs_begin_it;

        if (out_of_plane_upper_it != slice_locs_begin_it)
        {
          // There is some other slice that is less than, or equal to, this point
          slice_idx_lower = slice_idx_upper - 1;
        }
        else
        {
          // The only slice that lies above this point is the first

          // therefore the lower slice must be invalid, but we yet try to
          // interpolate using default values if it is within a slice spacing
          slice_idx_lower = kINVALID_SLICE_IDX;

          if (p_out_of_plane < lower_pad_)
          {
            // the point lies outside of the grace region, mark both slices as
            // invalid, this will result in the default value being returned
            // without interpolation
            slice_idx_upper = kINVALID_SLICE_IDX;
          }
        }
      }
      else
      {
        // every slice lies below this point
        slice_idx_lower = num_slices_ - 1;

        // therefore the upper slice must be invalid, but we will try to interpolate
        // using default values if it is within a slice spacing
        slice_idx_upper = kINVALID_SLICE_IDX;

        if (p_out_of_plane > upper_pad_)
        {
          // the point lies outside of the grace region, mark both slices as
          // invalid, this will result in the default value being returned
          // without interpolation
          slice_idx_lower = kINVALID_SLICE_IDX;
        }
      }

      if (hint)
      {
        hint->valid = true;

        hint->upper = slice_idx_upper;
        hint->lower = slice_idx_lower;
      }
    }

    // WLOG, the notation treats x,y as in-plane and z as out-of-plane, this
    // allows the reference formulas to be more easily used.
    // https://en.wikipedia.org/wiki/Trilinear_interpolation

    const bool z_0_in_bounds = slice_idx_lower != kINVALID_SLICE_IDX;
    const bool z_1_in_bounds = slice_idx_upper != kINVALID_SLICE_IDX;

    if (z_0_in_bounds || z_1_in_bounds)
    {
      // there is a valid slice available to attempt interpolation
      const SlicePointType in_plane_pt = get_in_plane_pt(p);

      SliceContIdx in_plane_idx;

      CoordScalar z_d = 0;

      if (!z_0_in_bounds)
      {
        // lower slice is invalid, only use values from the upper slice
        z_d = 1;
        slices_[slice_idx_upper]->TransformPhysicalPointToContinuousIndex(
                                                             in_plane_pt, in_plane_idx);
      }
      else if (!z_1_in_bounds)
      {
        // upper slice is invalid, only use values from the lower slice
        z_d = 0;
        slices_[slice_idx_lower]->TransformPhysicalPointToContinuousIndex(
                                                             in_plane_pt, in_plane_idx);
      }
      else
      {
        // both slices are valid
        z_d = (p_out_of_plane - slice_locs_[slice_idx_lower]) /
              (slice_locs_[slice_idx_upper]- slice_locs_[slice_idx_lower]);
        slices_[slice_idx_lower]->TransformPhysicalPointToContinuousIndex(
                                                             in_plane_pt, in_plane_idx);
      }

      const CoordScalar x_0 = std::floor(in_plane_idx[0]);
      const CoordScalar y_0 = std::floor(in_plane_idx[1]);
      const CoordScalar x_1 = x_0 + 1;
      const CoordScalar y_1 = y_0 + 1;

      const bool x_0_in_bounds = (x_0 >= 0) && (x_0 < slice_num_cols_);
      const bool y_0_in_bounds = (y_0 >= 0) && (y_0 < slice_num_rows_);
      const bool x_1_in_bounds = (x_1 >= 0) && (x_1 < slice_num_cols_);
      const bool y_1_in_bounds = (y_1 >= 0) && (y_1 < slice_num_rows_);

      const CoordScalar x_d = in_plane_idx[0] - x_0;  // / (x_1 - x_0) == 1
      const CoordScalar y_d = in_plane_idx[1] - y_0;  // / (x_1 - x_0) == 1

      CoordScalar c_00 = 0;
      CoordScalar c_01 = 0;
      CoordScalar c_10 = 0;
      CoordScalar c_11 = 0;

      SliceIndexType tmp_idx;

      // c00 computation
      if (x_0_in_bounds && y_0_in_bounds && z_0_in_bounds)
      {
        tmp_idx[0] = x_0;
        tmp_idx[1] = y_0;

        c_00 += slices_[slice_idx_lower]->GetPixel(tmp_idx) * (1 - x_d);
      }
      else
      {
        c_00 += default_val_ * (1 - x_d);
      }

      if (x_1_in_bounds && y_0_in_bounds && z_0_in_bounds)
      {
        tmp_idx[0] = x_1;
        tmp_idx[1] = y_0;

        c_00 += slices_[slice_idx_lower]->GetPixel(tmp_idx) * x_d;
      }
      else
      {
        c_00 += default_val_ * x_d;
      }

      // c01 computation
      if (x_0_in_bounds && y_0_in_bounds && z_1_in_bounds)
      {
        tmp_idx[0] = x_0;
        tmp_idx[1] = y_0;

        c_01 += slices_[slice_idx_upper]->GetPixel(tmp_idx) * (1 - x_d);
      }
      else
      {
        c_01 += default_val_ * (1 - x_d);
      }

      if (x_1_in_bounds && y_0_in_bounds && z_1_in_bounds)
      {
        tmp_idx[0] = x_1;
        tmp_idx[1] = y_0;

        c_01 += slices_[slice_idx_upper]->GetPixel(tmp_idx) * x_d;
      }
      else
      {
        c_01 += default_val_ * x_d;
      }

      // c10 computation
      if (x_0_in_bounds && y_1_in_bounds && z_0_in_bounds)
      {
        tmp_idx[0] = x_0;
        tmp_idx[1] = y_1;

        c_10 += slices_[slice_idx_lower]->GetPixel(tmp_idx) * (1 - x_d);
      }
      else
      {
        c_10 += default_val_ * (1 - x_d);
      }

      if (x_1_in_bounds && y_1_in_bounds && z_0_in_bounds)
      {
        tmp_idx[0] = x_1;
        tmp_idx[1] = y_1;

        c_10 += slices_[slice_idx_lower]->GetPixel(tmp_idx) * x_d;
      }
      else
      {
        c_10 += default_val_ * x_d;
      }

      // c11 computation
      if (x_0_in_bounds && y_1_in_bounds && z_1_in_bounds)
      {
        tmp_idx[0] = x_0;
        tmp_idx[1] = y_1;

        c_11 += slices_[slice_idx_upper]->GetPixel(tmp_idx) * (1 - x_d);
      }
      else
      {
        c_11 += default_val_ * (1 - x_d);
      }

      if (x_1_in_bounds && y_1_in_bounds && z_1_in_bounds)
      {
        tmp_idx[0] = x_1;
        tmp_idx[1] = y_1;

        c_11 += slices_[slice_idx_upper]->GetPixel(tmp_idx) * x_d;
      }
      else
      {
        c_11 += default_val_ * x_d;
      }

      const PixelType c_0 = (c_00 * (1 - y_d)) + (c_10 * y_d);
      const PixelType c_1 = (c_01 * (1 - y_d)) + (c_11 * y_d);

      val = (c_0 * (1 - z_d)) + (c_1 * z_d);
    }

    return val;
  }

  PixelType interp_nn(const Pt3& p, SliceIndexHint* hint = 0) const
  {
    PixelType val = default_val_;

    size_type slice_idx = 0;

    if (hint && hint->valid)
    {
      slice_idx = hint->upper;
    }
    else
    {
      if (num_slices_ > 1)
      {
        const CoordScalar p_out_of_plane = p[out_of_plane_dim_];

        // Try to determine where this point lies with respect to the set of slices
        // e.g. what is the closest slice index

        CoordScalarListConstIterator out_of_plane_upper_it =
              std::upper_bound(slice_locs_.begin(), slice_locs_.end(), p_out_of_plane);
        
        if (out_of_plane_upper_it != slice_locs_.end())
        {
          CoordScalarListConstIterator slice_locs_begin_it = slice_locs_.begin();

          if (out_of_plane_upper_it != slice_locs_begin_it)
          {
            slice_idx = out_of_plane_upper_it - slice_locs_begin_it;

            // there is a chance that the preceding element could be closer, so compare
            // both distances.
            const CoordScalar d1 = *out_of_plane_upper_it - p_out_of_plane; 
            const CoordScalar d2 = p_out_of_plane - slice_locs_[slice_idx-1];

            if (d1 > d2)
            {
              --slice_idx;
            }
          }
          else
          {
            // every slice has an out of plane component larger than p's,
            // take the least if p lies within a slice spacing
            // TODO: see note below about half spacing.
            slice_idx = (p_out_of_plane > lower_pad_) ? 0 : kINVALID_SLICE_IDX;
          }
        }
        else
        {
          // No slice has an out of plane location component larger than p's,
          // take the greatest if p lies within a slice spacing
          // TODO: upper_pad_ represents a full spacing, but we really want within
          //       a half spacing here.
          slice_idx = (p_out_of_plane < upper_pad_) ? (num_slices_ - 1)
                                                    : kINVALID_SLICE_IDX;
        }
      }

      if (hint)
      {
        hint->valid = true;
        hint->upper = slice_idx;
      }
    }

    if (slice_idx != kINVALID_SLICE_IDX)
    {
      // use a native pointer to avoid overhead with smart pointer, especially since
      // this will be invoked from many threads
      SliceType* slice = slices_[slice_idx].GetPointer();

      // NOTE: it may actually be better to round the continuous index to
      //       determine if a default value should be used
      SliceIndexType slice_img_idx;
      if (slice->TransformPhysicalPointToIndex(get_in_plane_pt(p), slice_img_idx))
      {
        val = slice->GetPixel(slice_img_idx);
      }
    }

    return val;
  }

  void resample(VolumeType* vol, const InterpMethodType interp_method) const
  {
    if (interp_method == kNEAREST_NEIGHBOR)
    {
      resample_interp_template<kNEAREST_NEIGHBOR>(vol);
    }
    else if (interp_method == kLINEAR)
    {
      resample_interp_template<kLINEAR>(vol);
    }
    else if (interp_method == kCUBIC_BSPLINE)
    {
      resample_interp_template<kCUBIC_BSPLINE>(vol);
    }
  }

  /// templating on the interpolation method, allows the compiler to eliminate branches
  template <int tInterpMethod>
  void resample_interp_template(VolumeType* vol) const
  {
    InterpPts<tInterpMethod> interp_pts = { this,
                                            vol->GetLargestPossibleRegion().GetSize(),
                                            vol };

    ParallelFor(interp_pts, RangeType(0, interp_pts.vol_size[0] *
                                         interp_pts.vol_size[1] *
                                         interp_pts.vol_size[2]));
  }

private:
  typedef std::vector<SlicePointer> SliceList;
  typedef typename SliceList::iterator SliceListIterator;
  typedef typename SliceList::const_iterator SliceListConstIterator;
  typedef std::vector<CoordScalar> CoordScalarList;
  typedef typename CoordScalarList::iterator CoordScalarListIterator;
  typedef typename CoordScalarList::const_iterator CoordScalarListConstIterator;

  typedef typename SliceType::PointType SlicePointType;
  typedef typename SliceType::IndexType SliceIndexType;
  typedef typename itk::ContinuousIndex<CoordScalar,2> SliceContIdx;

  typedef typename VolumeType::PointType VolPointType;
  typedef typename VolumeType::SizeType VolSizeType;

  typedef itk::ImageRegionIterator<VolumeType> VolumeRegionIterator;

  SlicePointType get_in_plane_pt(const Pt3& p) const
  {
    Pt2 q = GetInPlanePt3D2D(p, out_of_plane_dim_);
    
    SlicePointType qq;
    qq[0] = q[0];
    qq[1] = q[1];

    return qq;
  }

  void get_in_plane_dims(size_type* dim1, size_type* dim2) const
  {
    if (out_of_plane_dim_ == 2)
    {
      *dim1 = 0;
      *dim2 = 1;
    }
    else if (out_of_plane_dim_ == 1)
    {
      *dim1 = 0;
      *dim2 = 2;
    }
    else  // out_of_plane_dim_ == 0
    {
      *dim1 = 1;
      *dim2 = 2;
    }
  }

  template <int tInterpMethod>
  struct InterpPts
  {
    const VariableSpacedSlices* var_slices;

    const VolSizeType vol_size;

    VolumeType* vol;

    void operator()(const RangeType r) const
    {
      typedef RangeType::size_type size_type;

      const size_type num_vox_slice = vol_size[0] * vol_size[1];

      typename VolumeType::IndexType vol_idx;

      size_type tmp_1d_slice_idx = 0;

      Pt3 prev_pt;
      Pt3 cur_pt;
      VolPointType tmp_vol_pt;

      SliceIndexHint hint = { false, kINVALID_SLICE_IDX, kINVALID_SLICE_IDX };

      for (size_type vox_1d_idx = r.begin(); vox_1d_idx < r.end(); ++vox_1d_idx)
      {
        vol_idx[2] = vox_1d_idx / num_vox_slice;

        tmp_1d_slice_idx = vox_1d_idx - (vol_idx[2] * num_vox_slice);

        vol_idx[1] = tmp_1d_slice_idx / vol_size[0];
        vol_idx[0] = tmp_1d_slice_idx - (vol_idx[1] * vol_size[0]);

        vol->TransformIndexToPhysicalPoint(vol_idx, tmp_vol_pt);
        cur_pt[0] = tmp_vol_pt[0];
        cur_pt[1] = tmp_vol_pt[1];
        cur_pt[2] = tmp_vol_pt[2];

        // if there was a previous hint and the current point has moved out
        // of the previous point's plane, invalidate the hint so that the interp_XX
        // function calculates the appropriate slice index
        if (hint.valid &&
            (std::abs(cur_pt[var_slices->out_of_plane_dim_] -
                      prev_pt[var_slices->out_of_plane_dim_]) > 1.0e-6))
        {
          hint.valid = false;
        }

        if (tInterpMethod == kNEAREST_NEIGHBOR)
        {
          vol->SetPixel(vol_idx, var_slices->interp_nn(cur_pt, &hint));
        }
        else if (tInterpMethod == kLINEAR)
        {
          vol->SetPixel(vol_idx, var_slices->interp_linear(cur_pt, &hint));
        }
        else if (tInterpMethod == kCUBIC_BSPLINE)
        {
          vol->SetPixel(vol_idx, var_slices->interp_cubic_bspline(cur_pt, &hint));
        }

        prev_pt = cur_pt;
      }
    }
  };

  const size_type out_of_plane_dim_;

  const SliceList slices_;

  const CoordScalarList slice_locs_;

  const size_type num_slices_;

  const PixelType default_val_;

  size_type slice_num_cols_;  // size dim 0
  size_type slice_num_rows_;  // size dim 1

  CoordScalar lower_pad_;
  CoordScalar upper_pad_;

  static const size_type kINVALID_SLICE_IDX = ~size_type(0);
};

}  // xreg

#endif

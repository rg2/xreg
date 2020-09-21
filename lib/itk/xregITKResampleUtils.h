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

#ifndef XREGITKRESAMPLEUTILS_H_
#define XREGITKRESAMPLEUTILS_H_

#include <itkLinearInterpolateImageFunction.h>
#include <itkBSplineInterpolateImageFunction.h>
#include <itkWindowedSincInterpolateImageFunction.h>
#include <itkConstantBoundaryCondition.h>
#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkInterpolateImageFunction.h>
#include <itkResampleImageFilter.h>
#include <itkIdentityTransform.h>
#include <itkDiscreteGaussianImageFilter.h>
#include <itkGrayscaleDilateImageFilter.h>
#include <itkGrayscaleErodeImageFilter.h>
#include <itkBinaryBallStructuringElement.h>

namespace xreg
{

/// \brief Downsample an ITK volume
///
/// factor < 1.0 -> downsample ratio, e.g. 0.5 implies each dimension is cut by half,
/// resulting in an 8x decrease in total number of elements (for a 3D volume).
/// Uses a user specified interpolation function.
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleImage(const itk::Image<T,N>* src, const double factor,
                itk::InterpolateImageFunction<itk::Image<T,N>>* interp_fn, const double sigma = -1)
{
  using ImageType             = itk::Image<T,N>;
  using ImagePointer          = typename ImageType::Pointer;
  using ImageSpacingType      = typename ImageType::SpacingType;
  using ImageSizeType         = typename ImageType::SizeType;
  using IdentityTransformType = itk::IdentityTransform<double,N>;
  using ResampleFilterType    = itk::ResampleImageFilter<ImageType,ImageType>;

  using SmoothingFilter        = itk::DiscreteGaussianImageFilter<ImageType,ImageType>;
  using SmoothingFilterPointer = typename SmoothingFilter::Pointer;

  const ImageType* img_to_resample = src;

  ImagePointer smoothed_img;

  // smooth the image before any downsampling
  if ((factor < 1) && (std::abs(sigma) > 1.0e-6))
  {
    double sigma_to_use = sigma;

    if (sigma < 0)
    {
      sigma_to_use = 0.5 / factor;
    }

    SmoothingFilterPointer smoother = SmoothingFilter::New();
    smoother->SetInput(src);
    smoother->SetUseImageSpacing(false);
    smoother->SetVariance(sigma_to_use * sigma_to_use);
    smoother->Update();

    smoothed_img = smoother->GetOutput();

    img_to_resample = smoothed_img.GetPointer();
  }

  typename IdentityTransformType::Pointer id_xform = IdentityTransformType::New();

  typename ResampleFilterType::Pointer resample_filter = ResampleFilterType::New();

  resample_filter->SetInput(img_to_resample);
  resample_filter->SetTransform(id_xform);
  resample_filter->SetInterpolator(interp_fn);

  resample_filter->SetOutputOrigin(src->GetOrigin());
  resample_filter->SetOutputDirection(src->GetDirection());

  ImageSpacingType dst_spacing = src->GetSpacing();
  ImageSizeType dst_size = src->GetLargestPossibleRegion().GetSize();

  for (unsigned int i = 0; i < N; ++i)
  {
    dst_spacing[i] /= factor;
    dst_size[i] = static_cast<unsigned long>((dst_size[i] * factor) + 0.5);
  }

  resample_filter->SetOutputSpacing(dst_spacing);
  resample_filter->SetSize(dst_size);

  resample_filter->Update();

  return resample_filter->GetOutput();
}

/// \brief Downsample an ITK volume
///
/// factor < 1.0 -> downsample ratio, e.g. 0.5 implies each dimension is cut by half,
/// resulting in an 8x decrease in total number of elements (for a 3D volume).
/// Uses B-Spline interpolation.
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleImageBSplineInterp(const itk::Image<T,N>* src, const double factor, const double sigma = -1)
{
  using InterpFnType = itk::BSplineInterpolateImageFunction<itk::Image<T,N>>;

  typename InterpFnType::Pointer interp_fn = InterpFnType::New();
  interp_fn->SetSplineOrder(3);

  return DownsampleImage(src, factor, interp_fn.GetPointer(), sigma);
}

/// factor < 1.0 -> downsample ratio, e.g. 0.5 implies each dimension is cut by half,
/// resulting in an 8x decrease in total number of elements (for a 3D volume).
/// Uses Linear interpolation.
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleImageLinearInterp(const itk::Image<T,N>* src, const double factor, const double sigma = -1)
{
  using InterpFnType = itk::LinearInterpolateImageFunction<itk::Image<T,N>>;

  typename InterpFnType::Pointer interp_fn = InterpFnType::New();

  return DownsampleImage(src, factor, interp_fn.GetPointer(), sigma);
}

/// factor < 1.0 -> downsample ratio, e.g. 0.5 implies each dimension is cut by half,
/// resulting in an 8x decrease in total number of elements (for a 3D volume).
/// Uses Nearest Neighbor interpolation.
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleImageNNInterp(const itk::Image<T,N>* src, const double factor, const double sigma = -1)
{
  using InterpFnType = itk::NearestNeighborInterpolateImageFunction<itk::Image<T,N>>;

  typename InterpFnType::Pointer interp_fn = InterpFnType::New();

  return DownsampleImage(src, factor, interp_fn.GetPointer(), sigma);
}

/// factor < 1.0 -> downsample ratio, e.g. 0.5 implies each dimension is cut by half,
/// resulting in an 8x decrease in total number of elements (for a 3D volume).
/// Uses sinc interpolation.
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleImageSincInterp(const itk::Image<T,N>* src, const double factor, const double sigma = -1)
{
  using InterpFnType = itk::WindowedSincInterpolateImageFunction<itk::Image<T,N>, 4,
                                                                 itk::Function::LanczosWindowFunction<4>,
                                                                 itk::ConstantBoundaryCondition<itk::Image<T,N>>>;

  typename InterpFnType::Pointer interp_fn = InterpFnType::New();

  return DownsampleImage(src, factor, interp_fn.GetPointer(), sigma);
}

/// \brief Downsample an ITK volume
///
/// factor < 1.0 -> downsample ratio, e.g. 0.5 implies each dimension is cut by half,
/// resulting in an 8x decrease in total number of elements (for a 3D volume).
/// B-Spline interpolation is used by default.
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleImage(const itk::Image<T,N>* src, const double factor, const double sigma = -1)
{
  return DownsampleImageBSplineInterp(src, factor, sigma);
}

template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
DownsampleBinaryImage(const itk::Image<T,N>* src, const double factor,
                      const double morph_kern_rad = -1.0,
                      const bool do_dilate = true)
{
  using ImageType = itk::Image<T,N>;
  using ImagePtr = typename ImageType::Pointer;
  using InterpFn = itk::NearestNeighborInterpolateImageFunction<ImageType>;

  using MorphKernel  = itk::BinaryBallStructuringElement<T,N>;
  using DilateFilter = itk::GrayscaleDilateImageFilter<ImageType,ImageType,MorphKernel>;
  using ErodeFilter  = itk::GrayscaleErodeImageFilter<ImageType,ImageType,MorphKernel>;
  using KernelFilter = itk::KernelImageFilter<ImageType,ImageType,MorphKernel>;

  const double kern_rad_to_use = (morph_kern_rad < 0) ?
                                        ( 0.5 / factor) : morph_kern_rad;
 
  const ImageType* img_to_interp = src;

  ImagePtr morph_img;

  if (kern_rad_to_use > 1.e-8)
  {
    MorphKernel kern;
    kern.SetRadius(kern_rad_to_use);
    kern.CreateStructuringElement();
 
    typename KernelFilter::Pointer filter_fn;

    if (do_dilate)
    { 
      filter_fn = DilateFilter::New();
    }
    else
    {
      filter_fn = ErodeFilter::New();
    }
    
    filter_fn->SetInput(src);
    filter_fn->SetKernel(kern);

    filter_fn->Update();
    morph_img = filter_fn->GetOutput();

    img_to_interp = morph_img.GetPointer();
  }

  typename InterpFn::Pointer interp_fn = InterpFn::New();

  return DownsampleImage(img_to_interp, factor, interp_fn.GetPointer(), 0.0);
}

}  // xreg

#endif


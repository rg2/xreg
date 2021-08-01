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

#ifndef XREGITKIOUTILS_H_
#define XREGITKIOUTILS_H_

#include <itkImageFileWriter.h>
#include <itkImageFileReader.h>
#include <itkGDCMImageIO.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>

#include "xregCommon.h"
#include "xregITKRemapUtils.h"
#include "xregAssert.h"

namespace xreg
{

// some forward declarations to avoid full includes here:
void SplitPathPrefixExtension(const std::string& path, std::string* prefix,
                              std::string* ext);
std::string ToLowerCase(const std::string& s);
// END forward declarations from other files

/// \brief Read an affine transformation saved to disk from 3D Slicer
///
/// This calls ReadITKAffineTransformFromFile and then inverts the transform as
/// 3D Slicer saves the inverse to disk.
FrameTransform ReadSlicerAffineTransformFromFile(const std::string& path);

/// \brief Read a 3D Affine Transformation from an ITK supported file format.
FrameTransform ReadITKAffineTransformFromFile(const std::string& path);

/// \brief Writes an Affine transform to disk in an ITK supported file format.
void WriteITKAffineTransform(const std::string& path, const FrameTransform& xform);

/// \brief Writes a 2D ITK image object to disk after remapping the scalar values
///        to be in 8bpp.
///
/// The file format is determined by the file extension
template <class T>
void WriteITKImageRemap8bpp(const itk::Image<T,2>* img, const std::string& path)
{
  using Image8bppType = itk::Image<unsigned char,2>;
  using Image8bppWriterType = itk::ImageFileWriter<Image8bppType>;

  auto remapped = ITKImageRemap8bpp(img);

  typename Image8bppWriterType::Pointer writer = Image8bppWriterType::New();
  writer->SetInput(remapped);
  writer->SetFileName(path);
  writer->Update();
}

/// \brief Writes an ITK image object to disk using the current/raw voxel values.
///
/// The file format is determined by the file extension.
template <class T, unsigned int N>
void WriteITKImageToDisk(const itk::Image<T,N>* img, const std::string& path,
                         const bool force_no_compression = false)
{
  using ImageWriter = itk::ImageFileWriter<itk::Image<T,N>>;

  typename ImageWriter::Pointer writer = ImageWriter::New();
  writer->SetInput(img);
  writer->SetFileName(path);

  if (force_no_compression)
  {
    writer->UseCompressionOff();
  }
  else
  {
    // If the output format is MHD/MHA, enable compression (by default ITK does not)

    std::string ext;
    SplitPathPrefixExtension(path, 0, &ext);
    ext = ToLowerCase(ext);

    if ((ext == ".mhd") || (ext == ".mha"))
    {
      writer->UseCompressionOn();
    }
  }

  writer->Update();
}

/// \brief Reads an image from disk into an ITK image object
template <class tImage>
typename tImage::Pointer ReadITKImageFromDisk(const std::string& path)
{
  using ImageReader = itk::ImageFileReader<tImage>;

  typename ImageReader::Pointer reader = ImageReader::New();
  reader->SetFileName(path);
  reader->Update();

  return reader->GetOutput();
}

template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ReadDICOMNDFromDisk(const std::string& path)
{
  using ImgReader = itk::ImageFileReader<itk::Image<tPixelType,tN>>;
  typename ImgReader::Pointer img_reader = ImgReader::New();
  img_reader->SetFileName(path);

  itk::GDCMImageIO::Pointer gdcm_io = itk::GDCMImageIO::New();
  img_reader->SetImageIO(gdcm_io);

  img_reader->Update();

  return img_reader->GetOutput();
}


template <class tPixelType>
typename itk::Image<tPixelType,2>::Pointer
ReadDICOM2DFromDisk(const std::string& path)
{
  return ReadDICOMNDFromDisk<tPixelType,2>(path);
}

template <class tPixelType>
typename itk::Image<tPixelType,3>::Pointer
ReadDICOM3DFromDisk(const std::string& path)
{
  return ReadDICOMNDFromDisk<tPixelType,3>(path);
}

template <unsigned int tN>
void WriteITKLabelMapAsRGB(const itk::Image<unsigned char,tN>* img, const std::string& path)
{
  auto lut = GenericAnatomyLUT();

  using RGBImg = itk::Image<typename decltype(lut)::value_type,tN>;
 
  auto dst_img = RGBImg::New();

  dst_img->SetDirection(img->GetDirection());
  dst_img->SetSpacing(img->GetSpacing());
  dst_img->SetOrigin(img->GetOrigin());
  dst_img->SetRegions(img->GetLargestPossibleRegion());
  dst_img->Allocate();

  itk::ImageRegionConstIterator<itk::Image<unsigned char,tN>> src_it(img, img->GetLargestPossibleRegion());
  itk::ImageRegionIterator<RGBImg> dst_it(dst_img, dst_img->GetLargestPossibleRegion());

  for (; !src_it.IsAtEnd(); ++src_it, ++dst_it)
  {
    dst_it.Set(lut[src_it.Get()]);
  }

  WriteITKImageToDisk(dst_img.GetPointer(), path);
}

template <class tIntensityScalar, unsigned int tN>
void RemapImageAndOverlayLabelsAndWriteToDisk(const itk::Image<tIntensityScalar,tN>* intensity_img,
                                              const itk::Image<unsigned char,tN>* label_map,
                                              const std::string& path,
                                              const double label_alpha = 1.0)
{
  auto lut = GenericAnatomyLUT();
  
  using IntensScalar = tIntensityScalar;
  using IntensImg    = itk::Image<IntensScalar,tN>;
  using RemapImg     = itk::Image<unsigned char,tN>;
  using LabelImg     = RemapImg;
  using RGBVal       = typename decltype(lut)::value_type;
  using RGBImg       = itk::Image<RGBVal,tN>;
 
  auto remap_img = ITKImageRemap8bpp(intensity_img);

  auto dst_img = RGBImg::New();

  dst_img->SetDirection(remap_img->GetDirection());
  dst_img->SetSpacing(remap_img->GetSpacing());
  dst_img->SetOrigin(remap_img->GetOrigin());
  dst_img->SetRegions(remap_img->GetLargestPossibleRegion());
  dst_img->Allocate();

  itk::ImageRegionConstIterator<RemapImg> remap_it(remap_img, remap_img->GetLargestPossibleRegion());
  itk::ImageRegionConstIterator<LabelImg> label_it(label_map, label_map->GetLargestPossibleRegion());
  itk::ImageRegionIterator<RGBImg> dst_it(dst_img, dst_img->GetLargestPossibleRegion());

  RGBVal tmp_rgb; 

  xregASSERT((label_alpha >= 0) && (label_alpha <= 1.0));
  const double intensity_alpha = 1.0 - label_alpha;

  for (; !remap_it.IsAtEnd(); ++remap_it, ++label_it, ++dst_it)
  {
    const auto& cur_label  = label_it.Get();
    const auto& cur_intens = remap_it.Get();
      
    tmp_rgb[0] = cur_intens;
    tmp_rgb[1] = cur_intens;
    tmp_rgb[2] = cur_intens;

    if (!cur_label)
    {
      dst_it.Set(tmp_rgb);
    }
    else
    {
      const auto& label_rgb = lut[label_it.Get()];

      tmp_rgb[0] = static_cast<unsigned char>(std::lround((intensity_alpha * tmp_rgb[0]) +
                                                          (label_alpha * label_rgb[0])));
      tmp_rgb[1] = static_cast<unsigned char>(std::lround((intensity_alpha * tmp_rgb[1]) +
                                                          (label_alpha * label_rgb[1])));
      tmp_rgb[2] = static_cast<unsigned char>(std::lround((intensity_alpha * tmp_rgb[2]) +
                                                          (label_alpha * label_rgb[2])));

      dst_it.Set(tmp_rgb);
    } 
  }

  WriteITKImageToDisk(dst_img.GetPointer(), path);
}

}  // xreg

#endif


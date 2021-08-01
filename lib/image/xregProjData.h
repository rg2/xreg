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

#ifndef XREGPROJDATA_H_
#define XREGPROJDATA_H_

#include <boost/optional.hpp>

#include "xregCommon.h"

#include "xregPerspectiveXform.h"

namespace xreg
{

// Forward declarations:
struct DICOMFIleBasicFields;

enum class ProjDataRotToPatUp
{
  kZERO = 0,
  kNINETY = 90,
  kONE_EIGHTY = 180,
  kTWO_SEVENTY = 270
};

/// \brief Stores all relevant information for a single projection.
///
/// The camera model, the pixel data, annotated landmarks, etc.
template <class tPixelScalar>
struct ProjData
{
  using PixelScalar = tPixelScalar;
  using Proj        = itk::Image<PixelScalar,2>;
  using ProjPtr     = typename Proj::Pointer;

  using LandmarksMap = LandMap2;

  CameraModel cam;
  ProjPtr     img;

  // name -> landmark continuous pixel indices
  LandMap2 landmarks;

  // Indicates a rotation that is required to orient the patient "up,"
  // so that the superior portion is approximately located in the top
  // of the image and the inferior portion is approximately located in
  // the bottom of the image.
  boost::optional<ProjDataRotToPatUp> rot_to_pat_up;
  
  // Indicates that the detector spacings specified in cam were explicitly
  // defined from metadata fields in the original source. Examples where this
  // can be false are when converting from DICOM and the user overrides the
  // spacing value manually or the spacing values are guessed from other
  // metadata values (e.g. the detector diameter).
  boost::optional<bool> det_spacings_from_orig_meta;

  // Original DICOM metadata this image - does not need
  // to be set, e.g. for the case of simulated data
  std::shared_ptr<DICOMFIleBasicFields> orig_dcm_meta;
};

using ProjDataF32 = ProjData<float>;
using ProjDataU16 = ProjData<unsigned short>;
using ProjDataU8  = ProjData<unsigned char>;

using ProjDataF32List = std::vector<ProjDataF32>;
using ProjDataU16List = std::vector<ProjDataU16>;
using ProjDataU8List  = std::vector<ProjDataU8>;

ProjDataF32 DownsampleProjData(const ProjDataF32& src_proj, const CoordScalar ds_factor,
                               const bool force_even_dims = false);
ProjDataU16 DownsampleProjData(const ProjDataU16& src_proj, const CoordScalar ds_factor,
                               const bool force_even_dims = false);
ProjDataU8 DownsampleProjData(const ProjDataU8& src_proj, const CoordScalar ds_factor,
                              const bool force_even_dims = false);

ProjDataF32List DownsampleProjData(const ProjDataF32List& src_projs, const CoordScalar ds_factor,
                                   const bool force_even_dims = false);
ProjDataU16List DownsampleProjData(const ProjDataU16List& src_projs, const CoordScalar ds_factor,
                                   const bool force_even_dims = false);
ProjDataU8List DownsampleProjData(const ProjDataU8List& src_projs, const CoordScalar ds_factor,
                                  const bool force_even_dims = false);

template <class tPixelScalar>
using CamImgPair = std::tuple<CameraModel,typename itk::Image<tPixelScalar,2>::Pointer>;

using CamImgPairF32 = CamImgPair<float>;
using CamImgPairU16 = CamImgPair<unsigned short>;
using CamImgPairU8  = CamImgPair<unsigned char>;

using CamImgPairF32List = std::vector<CamImgPairF32>;
using CamImgPairU16List = std::vector<CamImgPairU16>;
using CamImgPairU8List  = std::vector<CamImgPairU8>;

/// \brief Extracts a list of camera models from a list of camera
///        model/projection image pairs.
std::vector<CameraModel>
ExtractCamModels(const CamImgPairF32List& cam_img_pairs);

/// \brief Extracts a list of camera models from a list of camera
///        model/projection image pairs.
std::vector<CameraModel>
ExtractCamModels(const CamImgPairU16List& cam_img_pairs);

/// \brief Extracts a list of camera models from a list of camera
///        model/projection image pairs.
std::vector<CameraModel>
ExtractCamModels(const CamImgPairU8List& cam_img_pairs);

/// \brief Extracts a list of projection images from a list of camera
///        model/projection image pairs.
std::vector<itk::Image<float,2>::Pointer>
ExtractImgs(const CamImgPairF32List& cam_img_pairs);

/// \brief Extracts a list of projection images from a list of camera
///        model/projection image pairs.
std::vector<itk::Image<unsigned short,2>::Pointer>
ExtractImgs(const CamImgPairU16List& cam_img_pairs);

/// \brief Extracts a list of projection images from a list of camera
///        model/projection image pairs.
std::vector<itk::Image<unsigned char,2>::Pointer>
ExtractImgs(const CamImgPairU8List& cam_img_pairs);

/// \brief Extract/copy a list of camera model/image pairs from a list of projection metadatas and images.
CamImgPairF32List ExtractCamImagePairs(const ProjDataF32List& proj_data);

/// \brief Extract/copy a list of camera model/image pairs from a list of projection metadatas and images.
CamImgPairU16List ExtractCamImagePairs(const ProjDataU16List& proj_data);

/// \brief Extract/copy a list of camera model/image pairs from a list of projection metadatas and images.
CamImgPairU8List ExtractCamImagePairs(const ProjDataU8List& proj_data);

ProjDataF32List CamImgPairsToProjData(const CamImgPairF32List& pairs);

ProjDataU16List CamImgPairsToProjData(const CamImgPairU16List& pairs);

ProjDataU8List CamImgPairsToProjData(const CamImgPairU8List& pairs);

/// \brief Extract/copy a list of camera models from a list of projection metadatas and images.
std::vector<CameraModel>
ExtractCamModels(const ProjDataF32List& proj_data);

/// \brief Extract/copy a list of camera models from a list of projection metadatas and images.
std::vector<CameraModel>
ExtractCamModels(const ProjDataU16List& proj_data);

/// \brief Extract/copy a list of camera models from a list of projection metadatas and images.
std::vector<CameraModel>
ExtractCamModels(const ProjDataU8List& proj_data);

/// \brief Updates each camera model associated with an image, using updated
///        image dimensions and pixel spacings.
///
/// This is generally used when the images have been resized.
void UpdateCamModelDetParamsFromImgs(ProjDataF32List* cam_model_img_pairs,
                                     const bool keep_focal_len_const = true);

/// \brief Updates each camera model associated with an image, using updated
///        image dimensions and pixel spacings.
///
/// This is generally used when the images have been resized.
void UpdateCamModelDetParamsFromImgs(ProjDataU16List* cam_model_img_pairs,
                                     const bool keep_focal_len_const = true);

/// \brief Updates each camera model associated with an image, using updated
///        image dimensions and pixel spacings.
///
/// This is generally used when the images have been resized.
void UpdateCamModelDetParamsFromImgs(ProjDataU8List* cam_model_img_pairs,
                                     const bool keep_focal_len_const = true);

/// \brief Updates each camera model associated with an image, using updated
///        image dimensions and pixel spacings.
///
/// This is generally used when the images have been resized.
void UpdateCamModelDetParamsFromImgs(CamImgPairF32List* cam_model_img_pairs,
                                     const bool keep_focal_len_const = true);

/// \brief Updates each camera model associated with an image, using updated
///        image dimensions and pixel spacings.
///
/// This is generally used when the images have been resized.
void UpdateCamModelDetParamsFromImgs(CamImgPairU16List* cam_model_img_pairs,
                                     const bool keep_focal_len_const = true);

/// \brief Updates each camera model associated with an image, using updated
///        image dimensions and pixel spacings.
///
/// This is generally used when the images have been resized.
void UpdateCamModelDetParamsFromImgs(CamImgPairU8List* cam_model_img_pairs,
                                     const bool keep_focal_len_const = true);

itk::Image<float,2>::Pointer
MakeImageF32FromCam(const CameraModel& cam);

itk::Image<unsigned short,2>::Pointer
MakeImageU16FromCam(const CameraModel& cam);

itk::Image<unsigned char,2>::Pointer
MakeImageU8FromCam(const CameraModel& cam);

void ModifyForPatUp(ProjDataF32::Proj* img, const ProjDataRotToPatUp rot_to_pat_up);

void ModifyForPatUp(ProjDataU16::Proj* img, const ProjDataRotToPatUp rot_to_pat_up);

void ModifyForPatUp(ProjDataU8::Proj* img, const ProjDataRotToPatUp rot_to_pat_up);

/// \brief Crops out boundary pixels using a constant sized border; updates the image and
///        the camera model.
///
/// This is useful for removing collimation effections.
std::tuple<CameraModel, itk::Image<unsigned short,2>::Pointer>
CropBoundaryPixels(const CameraModel& src_cam, const itk::Image<unsigned short,2>* src_img,
                   const size_type boundary_width = 20);

/// \brief Crops out boundary pixels using a constant sized border; updates the image and
///        the camera model.
///
/// This is useful for removing collimation effections.
std::tuple<CameraModel, itk::Image<float,2>::Pointer>
CropBoundaryPixels(const CameraModel& src_cam, const itk::Image<float,2>* src_img,
                   const size_type boundary_width = 20);


}  // xreg

#endif


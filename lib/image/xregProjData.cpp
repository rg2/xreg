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

#include "xregProjData.h"

#include "xregAssert.h"
#include "xregITKResampleUtils.h"
#include "xregITKCropPadUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"

namespace  // un-named
{

using namespace xreg;

template <class tPixelScalar>
ProjData<tPixelScalar>
DownsampleProjDataHelper(const ProjData<tPixelScalar>& src_proj, const CoordScalar ds_factor,
                         const bool force_even_dims)
{
  ProjData<tPixelScalar> dst_proj;
 
  dst_proj.cam = DownsampleCameraModel(src_proj.cam, ds_factor, force_even_dims); 

  if (src_proj.img)
  {
    dst_proj.img = DownsampleImage(src_proj.img.GetPointer(), ds_factor);
    
    if (force_even_dims)
    {
      using ProjImg   = typename ProjData<tPixelScalar>::Proj;
      using ROIFilter = itk::RegionOfInterestImageFilter<ProjImg,ProjImg>;

      auto ds_sz = dst_proj.img->GetLargestPossibleRegion().GetSize();
      
      const bool cols_are_odd = ds_sz[0] % 2;
      const bool rows_are_odd = ds_sz[1] % 2;

      if (cols_are_odd || rows_are_odd)
      {
        if (cols_are_odd)
        {
          --ds_sz[0];
        }
        
        if (rows_are_odd)
        {
          --ds_sz[1];
        }

        typename ProjImg::RegionType roi;
        roi.SetSize(ds_sz);
        roi.SetIndex(0,0);
        roi.SetIndex(1,0);

        auto roi_filter = ROIFilter::New();
        roi_filter->SetInput(dst_proj.img);
        roi_filter->SetRegionOfInterest(roi);
        roi_filter->Update();
        
        dst_proj.img = roi_filter->GetOutput();
      }
    }

    const auto ds_sz = dst_proj.img->GetLargestPossibleRegion().GetSize();
    xregASSERT(ds_sz[0] == dst_proj.cam.num_det_cols);
    xregASSERT(ds_sz[1] == dst_proj.cam.num_det_rows);
  }

  for (const auto& src_land : src_proj.landmarks)
  {
    dst_proj.landmarks.insert(LandMap2::value_type(src_land.first,
                LandMap2::mapped_type(src_land.second * ds_factor)));
  }
  
  dst_proj.rot_to_pat_up = src_proj.rot_to_pat_up;

  return dst_proj;
}

template <class tPixelScalar>
std::vector<ProjData<tPixelScalar>>
DownsampleProjDataHelper(const std::vector<ProjData<tPixelScalar>>& src_projs, const CoordScalar ds_factor,
                         const bool force_even_dims)
{
  std::vector<ProjData<tPixelScalar>> dst_projs;
  dst_projs.reserve(src_projs.size());

  for (const auto& src_proj : src_projs)
  {
    dst_projs.push_back(DownsampleProjDataHelper(src_proj, ds_factor, force_even_dims));
  }

  return dst_projs;
}

template <class tPixelScalar>
std::vector<CameraModel>
ExtractCamModelsHelper(const std::vector<CamImgPair<tPixelScalar>>& cam_img_pairs)
{
  const size_type num_cams = cam_img_pairs.size();

  std::vector<CameraModel> cams(num_cams);

  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    cams[cam_idx] = std::get<0>(cam_img_pairs[cam_idx]);
  }

  return cams;
}

template <class tPixelScalar>
std::vector<typename itk::Image<tPixelScalar,2>::Pointer>
ExtractImgsHelper(const std::vector<CamImgPair<tPixelScalar>>& cam_img_pairs)
{
  const size_type num_imgs = cam_img_pairs.size();

  std::vector<typename itk::Image<tPixelScalar,2>::Pointer> imgs(num_imgs);

  for (size_type img_idx = 0; img_idx < num_imgs; ++img_idx)
  {
    imgs[img_idx] = std::get<1>(cam_img_pairs[img_idx]);
  }

  return imgs;
}

template <class tPixelScalar>
std::vector<CamImgPair<tPixelScalar>>
ExtractCamImagePairsHelper(const std::vector<ProjData<tPixelScalar>>& proj_data)
{
  const size_type num_projs = proj_data.size();

  std::vector<CamImgPair<tPixelScalar>> cam_img_pairs;
  cam_img_pairs.reserve(num_projs);

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    cam_img_pairs.push_back(std::make_tuple(proj_data[proj_idx].cam, proj_data[proj_idx].img));
  }

  return cam_img_pairs;
}

template <class tPixelScalar>
std::vector<ProjData<tPixelScalar>>
CamImgPairsToProjDataHelper(const std::vector<CamImgPair<tPixelScalar>>& pairs)
{
  const size_type num_projs = pairs.size();

  std::vector<ProjData<tPixelScalar>> pd(num_projs);

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    const auto& cur_pair = pairs[proj_idx];

    pd[proj_idx].cam = std::get<0>(cur_pair);
    pd[proj_idx].img = std::get<1>(cur_pair);
  }

  return pd;
}

template <class tPixelScalar>
std::vector<CameraModel>
ExtractCamModelsHelper(const std::vector<ProjData<tPixelScalar>>& proj_data)
{
  const size_type num_projs = proj_data.size();

  std::vector<CameraModel> cams(num_projs);

  for (size_type i = 0; i < num_projs; ++i)
  {
    cams[i] = proj_data[i].cam;
  }

  return cams;
}

template <class tPixelScalar>
CameraModel& GetCamHelper(CamImgPair<tPixelScalar>& cam_img_pair)
{
  return std::get<0>(cam_img_pair);
}

template <class tPixelScalar>
CameraModel& GetCamHelper(ProjData<tPixelScalar>& pd)
{
  return pd.cam;
}

template <class tPixelScalar>
typename itk::Image<tPixelScalar,2>::Pointer& GetImgHelper(CamImgPair<tPixelScalar>& cam_img_pair)
{
  return std::get<1>(cam_img_pair);
}

template <class tPixelScalar>
typename itk::Image<tPixelScalar,2>::Pointer& GetImgHelper(ProjData<tPixelScalar>& pd)
{
  return pd.img;
}

template <class T>
struct RecoverPixelScalarHelper;

template <class T>
struct RecoverPixelScalarHelper<std::tuple<CameraModel,T>>
{
  using type = typename T::ObjectType::PixelType;
};

template <class tPixelScalar>
struct RecoverPixelScalarHelper<ProjData<tPixelScalar>>
{
  using type = tPixelScalar;
};

template <class tProjData>
void UpdateCamModelDetParamsFromImgsHelper(std::vector<tProjData>* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  using PixelScalar = typename RecoverPixelScalarHelper<tProjData>::type;

  const size_type num_pairs = cam_model_img_pairs->size();

  for (size_type i = 0; i < num_pairs; ++i)
  {
    auto& p = cam_model_img_pairs->operator[](i);

    auto& cam = GetCamHelper<PixelScalar>(p);
    auto& img = GetImgHelper<PixelScalar>(p);

    const auto new_img_size = img->GetLargestPossibleRegion().GetSize();

    const auto new_img_spacing = img->GetSpacing();

    Mat3x3 intrins = cam.intrins;

    if (keep_focal_len_const)
    {
      // we need to scale the intrinsic matrix to keep the focal length
      // constant

      const CoordScalar xps_scale = cam.det_col_spacing / new_img_spacing[0];
      const CoordScalar yps_scale = cam.det_row_spacing / new_img_spacing[1];

      // these are usually the same as the pixel size scalings,
      // e.g. since we usually downsample images by a global scale factor, but
      // could be different in other cases.
      const CoordScalar x0_scale = static_cast<CoordScalar>(new_img_size[0])
                                      / static_cast<CoordScalar>(cam.num_det_cols);
      const CoordScalar y0_scale = static_cast<CoordScalar>(new_img_size[1])
                                      / static_cast<CoordScalar>(cam.num_det_rows);

      intrins(0,0) *= xps_scale;
      intrins(1,1) *= yps_scale;
      intrins(0,2) *= x0_scale;
      intrins(1,2) *= y0_scale;
    }

    cam.setup(intrins, Mat4x4(cam.extrins.matrix()),
                new_img_size[1], new_img_size[0],
                new_img_spacing[1], new_img_spacing[0]);
  }
}

template <class tPixelScalar>
typename itk::Image<tPixelScalar,2>::Pointer
MakeImageFromCam(const CameraModel& cam)
{
  using Img    = itk::Image<tPixelScalar,2>;
  using ImgPtr = typename Img::Pointer;

  ImgPtr img = Img::New();
  
  typename Img::RegionType reg;
  reg.SetIndex(0, 0);
  reg.SetIndex(1, 0);
  reg.SetSize(0, cam.num_det_cols);
  reg.SetSize(1, cam.num_det_rows);
  
  img->SetRegions(reg);

  std::array<double,2> sp = { cam.det_col_spacing, cam.det_row_spacing };
  img->SetSpacing(sp.data());

  img->Allocate();

  return img;
}

}  // un-named

xreg::ProjDataF32 xreg::DownsampleProjData(const ProjDataF32& src_proj, const CoordScalar ds_factor,
                                           const bool force_even_dims)
{
  return DownsampleProjDataHelper(src_proj, ds_factor, force_even_dims);
}

xreg::ProjDataU16 xreg::DownsampleProjData(const ProjDataU16& src_proj, const CoordScalar ds_factor,
                                           const bool force_even_dims)
{
  return DownsampleProjDataHelper(src_proj, ds_factor, force_even_dims);
}

xreg::ProjDataU8 xreg::DownsampleProjData(const ProjDataU8& src_proj, const CoordScalar ds_factor,
                                          const bool force_even_dims)
{
  return DownsampleProjDataHelper(src_proj, ds_factor, force_even_dims);
}

xreg::ProjDataF32List
xreg::DownsampleProjData(const ProjDataF32List& src_projs, const CoordScalar ds_factor,
                         const bool force_even_dims)
{
  return DownsampleProjDataHelper(src_projs, ds_factor, force_even_dims);
}

xreg::ProjDataU16List
xreg::DownsampleProjData(const ProjDataU16List& src_projs, const CoordScalar ds_factor,
                         const bool force_even_dims)
{
  return DownsampleProjDataHelper(src_projs, ds_factor, force_even_dims);
}

xreg::ProjDataU8List
xreg::DownsampleProjData(const ProjDataU8List& src_projs, const CoordScalar ds_factor,
                         const bool force_even_dims)
{
  return DownsampleProjDataHelper(src_projs, ds_factor, force_even_dims);
}

std::vector<xreg::CameraModel>
xreg::ExtractCamModels(const CamImgPairF32List& cam_img_pairs)
{
  return ExtractCamModelsHelper<float>(cam_img_pairs);
}

std::vector<xreg::CameraModel>
xreg::ExtractCamModels(const CamImgPairU16List& cam_img_pairs)
{
  return ExtractCamModelsHelper<unsigned short>(cam_img_pairs);
}

std::vector<xreg::CameraModel>
xreg::ExtractCamModels(const CamImgPairU8List& cam_img_pairs)
{
  return ExtractCamModelsHelper<unsigned char>(cam_img_pairs);
}

std::vector<itk::Image<float,2>::Pointer>
xreg::ExtractImgs(const CamImgPairF32List& cam_img_pairs)
{
  return ExtractImgsHelper<float>(cam_img_pairs);
}

std::vector<itk::Image<unsigned short,2>::Pointer>
xreg::ExtractImgs(const CamImgPairU16List& cam_img_pairs)
{
  return ExtractImgsHelper<unsigned short>(cam_img_pairs);
}

std::vector<itk::Image<unsigned char,2>::Pointer>
xreg::ExtractImgs(const CamImgPairU8List& cam_img_pairs)
{
  return ExtractImgsHelper<unsigned char>(cam_img_pairs);
}

xreg::CamImgPairF32List xreg::ExtractCamImagePairs(const ProjDataF32List& proj_data)
{
  return ExtractCamImagePairsHelper(proj_data);
}

xreg::CamImgPairU16List xreg::ExtractCamImagePairs(const ProjDataU16List& proj_data)
{
  return ExtractCamImagePairsHelper(proj_data);
}

xreg::CamImgPairU8List xreg::ExtractCamImagePairs(const ProjDataU8List& proj_data)
{
  return ExtractCamImagePairsHelper(proj_data);
}

xreg::ProjDataF32List xreg::CamImgPairsToProjData(const CamImgPairF32List& pairs)
{
  return CamImgPairsToProjDataHelper<float>(pairs);
}

xreg::ProjDataU16List xreg::CamImgPairsToProjData(const CamImgPairU16List& pairs)
{
  return CamImgPairsToProjDataHelper<unsigned short>(pairs);
}

xreg::ProjDataU8List xreg::CamImgPairsToProjData(const CamImgPairU8List& pairs)
{
  return CamImgPairsToProjDataHelper<unsigned char>(pairs);
}

std::vector<xreg::CameraModel>
xreg::ExtractCamModels(const ProjDataF32List& proj_data)
{
  return ExtractCamModelsHelper<float>(proj_data);
}

std::vector<xreg::CameraModel>
xreg::ExtractCamModels(const ProjDataU16List& proj_data)
{
  return ExtractCamModelsHelper<unsigned short>(proj_data);
}

std::vector<xreg::CameraModel>
xreg::ExtractCamModels(const ProjDataU8List& proj_data)
{
  return ExtractCamModelsHelper<unsigned char>(proj_data);
}

itk::Image<float,2>::Pointer
xreg::MakeImageF32FromCam(const CameraModel& cam)
{
  return MakeImageFromCam<float>(cam);
}

itk::Image<unsigned short,2>::Pointer
xreg::MakeImageU16FromCam(const CameraModel& cam)
{
  return MakeImageFromCam<unsigned short>(cam);
}

itk::Image<unsigned char,2>::Pointer
xreg::MakeImageU8FromCam(const CameraModel& cam)
{
  return MakeImageFromCam<unsigned char>(cam);
}

void xreg::UpdateCamModelDetParamsFromImgs(ProjDataF32List* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  UpdateCamModelDetParamsFromImgsHelper(cam_model_img_pairs, keep_focal_len_const);
}

void xreg::UpdateCamModelDetParamsFromImgs(ProjDataU16List* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  UpdateCamModelDetParamsFromImgsHelper(cam_model_img_pairs, keep_focal_len_const);
}

void xreg::UpdateCamModelDetParamsFromImgs(ProjDataU8List* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  UpdateCamModelDetParamsFromImgsHelper(cam_model_img_pairs, keep_focal_len_const);
}

void xreg::UpdateCamModelDetParamsFromImgs(CamImgPairF32List* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  UpdateCamModelDetParamsFromImgsHelper(cam_model_img_pairs, keep_focal_len_const);
}

void xreg::UpdateCamModelDetParamsFromImgs(CamImgPairU16List* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  UpdateCamModelDetParamsFromImgsHelper(cam_model_img_pairs, keep_focal_len_const);
}

void xreg::UpdateCamModelDetParamsFromImgs(CamImgPairU8List* cam_model_img_pairs,
                                           const bool keep_focal_len_const)
{
  UpdateCamModelDetParamsFromImgsHelper(cam_model_img_pairs, keep_focal_len_const);
}

namespace  // un-named
{

using namespace xreg;

template <class tPixelScalar>
void ModifyForPatUpHelper(itk::Image<tPixelScalar,2>* img,
                          const ProjDataRotToPatUp rot_to_pat_up)
{
  cv::Mat img_ocv = ShallowCopyItkToOpenCV(img);

  if (rot_to_pat_up == ProjDataRotToPatUp::kONE_EIGHTY)
  {
    FlipImageRows(&img_ocv);
    FlipImageColumns(&img_ocv);
  }
  else if (rot_to_pat_up == ProjDataRotToPatUp::kZERO)
  {
    // nothing to do
  }
  else
  {
    xregThrow("Unsupported undo pat rot up for undo: %d", static_cast<int>(rot_to_pat_up));
  }
}

}  // un-named

void xreg::ModifyForPatUp(ProjDataF32::Proj* img, const ProjDataRotToPatUp rot_to_pat_up)
{
  ModifyForPatUpHelper(img, rot_to_pat_up);
}

void xreg::ModifyForPatUp(ProjDataU16::Proj* img, const ProjDataRotToPatUp rot_to_pat_up)
{
  ModifyForPatUpHelper(img, rot_to_pat_up);
}

void xreg::ModifyForPatUp(ProjDataU8::Proj* img, const ProjDataRotToPatUp rot_to_pat_up)
{
  ModifyForPatUpHelper(img, rot_to_pat_up);
}

namespace  // un-named
{

template <class tPixelScalar>
std::tuple<CameraModel, typename itk::Image<tPixelScalar,2>::Pointer>
CropBoundaryPixelsHelper(const CameraModel& src_cam, const itk::Image<tPixelScalar,2>* src_img,
                         const size_type boundary_width)
{
  using PixelScalar = tPixelScalar;
  
  const auto dst_cam = UpdateCameraModelFor2DROI(src_cam, boundary_width, boundary_width,
                                                 src_cam.num_det_cols - boundary_width - 1,
                                                 src_cam.num_det_rows - boundary_width - 1); 

  typename itk::Image<PixelScalar,2>::Pointer dst_img;
  
  if (src_img)
  {
    const auto src_img_size = src_img->GetLargestPossibleRegion().GetSize();
    xregASSERT((src_img_size[0] == src_cam.num_det_cols) && (src_img_size[1] == src_cam.num_det_rows));

    dst_img = CropImage2DBoundary(src_img, boundary_width);
    
    const auto dst_img_size = dst_img->GetLargestPossibleRegion().GetSize();
    xregASSERT(dst_img_size[0] == dst_cam.num_det_cols);
    xregASSERT(dst_img_size[1] == dst_cam.num_det_rows);
  }

  return std::make_tuple(dst_cam, dst_img);
}

}

std::tuple<xreg::CameraModel, itk::Image<unsigned short,2>::Pointer>
xreg::CropBoundaryPixels(const CameraModel& src_cam, const itk::Image<unsigned short,2>* src_img,
                         const size_type boundary_width)
{
  return CropBoundaryPixelsHelper(src_cam, src_img, boundary_width);
}

std::tuple<xreg::CameraModel, itk::Image<float,2>::Pointer>
xreg::CropBoundaryPixels(const CameraModel& src_cam, const itk::Image<float,2>* src_img,
                         const size_type boundary_width)
{
  return CropBoundaryPixelsHelper(src_cam, src_img, boundary_width);
}


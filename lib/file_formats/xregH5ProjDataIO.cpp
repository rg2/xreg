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

#include "xregH5ProjDataIO.h"

#include <fmt/format.h>

#include "xregDICOMUtils.h"
#include "xregHDF5.h"
#include "xregHDF5Internal.h"
#include "xregH5CamModelIO.h"

const char* xreg::kXREG_PROJ_DATA_ATTR_STR = "proj-data";

namespace
{

using namespace xreg;

void AddProjDataLandsHelper(const LandMap2& lands, H5::Group* h5,
                            const bool delete_existing = false)
{
  if (delete_existing && ObjectInGroupH5("landmarks", *h5))
  {
    h5->unlink("landmarks");
  }
  
  if (!lands.empty())
  {
    H5::Group lands_g = h5->createGroup("landmarks");
    
    for (const auto& name_and_pt : lands)
    {
      WriteMatrixH5(name_and_pt.first, name_and_pt.second, &lands_g, false);
    }
  }
}

template <class tPixelScalar>
void WriteProjDataH5Helper(const std::vector<ProjData<tPixelScalar>>& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  SetStringAttr("xreg-type", kXREG_PROJ_DATA_ATTR_STR, h5);

  const size_type num_projs = proj_data.size();

  WriteSingleScalarH5("num-projs", num_projs, h5);

  for (size_type i = 0; i < num_projs; ++i)
  {
    H5::Group proj_g = h5->createGroup(fmt::format("proj-{:03d}", i));

    // only add the image if it is non-null
    if (proj_data[i].img)
    {  
      H5::Group img_g = proj_g.createGroup("img");
      WriteImageH5(proj_data[i].img.GetPointer(), &img_g, compress);
    }

    H5::Group cam_g = proj_g.createGroup("cam");
    WriteCamModelH5(proj_data[i].cam, &cam_g);
  
    AddProjDataLandsHelper(proj_data[i].landmarks, &proj_g);

    if (proj_data[i].rot_to_pat_up)
    {
      WriteSingleScalarH5("rot-to-pat-up",
                          static_cast<int>(*proj_data[i].rot_to_pat_up), &proj_g);
    }

    if (proj_data[i].det_spacings_from_orig_meta)
    {
      WriteSingleScalarH5("det-spacings-from-orig-meta",
                          *proj_data[i].det_spacings_from_orig_meta, &proj_g);
    }

    if (proj_data[i].orig_dcm_meta)
    {
      H5::Group orig_meta_g = proj_g.createGroup("orig-dcm-meta");
      
      SetStringAttr("meta-type", "dicom", &orig_meta_g);

      WriteDICOMFieldsH5(*proj_data[i].orig_dcm_meta, &orig_meta_g);
    }
  }
}

template <class tPixelScalar>
void WriteProjDataH5Helper(const std::vector<CamImgPair<tPixelScalar>>& cam_img_pairs,
                           H5::Group* h5, const bool compress)
{
  WriteProjDataH5Helper(CamImgPairsToProjData(cam_img_pairs), h5, compress);
}

}  // un-named

void xreg::CopyProjDataH5(const H5::Group& src_proj_h5, H5::Group* dst_proj_h5)
{
  const auto* src_proj_g = dynamic_cast<const H5::IdComponent*>(&src_proj_h5);
  xregASSERT(src_proj_g);

  auto* dst_proj_g = dynamic_cast<H5::IdComponent*>(dst_proj_h5);
  xregASSERT(dst_proj_g);

  SetStringAttr("xreg-type", kXREG_PROJ_DATA_ATTR_STR, dst_proj_h5);
  
  const auto src_id = src_proj_g->getId();
  const auto dst_id = dst_proj_g->getId();

  auto copy_h5 = [&src_id, &dst_id](const std::string& n)
  {
    H5Ocopy(src_id, n.c_str(), dst_id, n.c_str(), H5P_DEFAULT, H5P_DEFAULT);
  };

  copy_h5("num-projs");

  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", src_proj_h5);

  for (size_type i = 0; i < num_projs; ++i)
  {
    copy_h5(fmt::format("proj-{:03d}", i));
  }
}

void xreg::ReadProjDataFromH5AndWriteToDisk(const H5::Group& h5, const std::string& dst_disk_path)
{
  H5::H5File dst_h5(dst_disk_path, H5F_ACC_TRUNC);
 
  CopyProjDataH5(h5, &dst_h5);

  dst_h5.flush(H5F_SCOPE_GLOBAL);
  dst_h5.close();
}

void xreg::WriteProjDataH5(const ProjDataF32List& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper(proj_data, h5, compress);
}

void xreg::WriteProjDataH5(const ProjDataU16List& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper(proj_data, h5, compress);
}

void xreg::WriteProjDataH5(const ProjDataU8List& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper(proj_data, h5, compress);
}

void xreg::WriteProjDataH5(const ProjDataF32& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5(ProjDataF32List(1, proj_data), h5, compress);
}

void xreg::WriteProjDataH5(const ProjDataU16& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5(ProjDataU16List(1, proj_data), h5, compress);
}

void xreg::WriteProjDataH5(const ProjDataU8& proj_data,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5(ProjDataU8List(1, proj_data), h5, compress);
}

void xreg::WriteProjDataH5(const CamImgPairF32List& cam_img_pairs,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper<float>(cam_img_pairs, h5, compress);
}

void xreg::WriteProjDataH5(const CamImgPairU16List& cam_img_pairs,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper<unsigned short>(cam_img_pairs, h5, compress);
}

void xreg::WriteProjDataH5(const CamImgPairU8List& cam_img_pairs,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper<unsigned char>(cam_img_pairs, h5, compress);
}

void xreg::WriteProjDataH5(const CamImgPairF32& cam_img_pair,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper<float>(CamImgPairF32List(1, cam_img_pair), h5, compress);
}

void xreg::WriteProjDataH5(const CamImgPairU16& cam_img_pair,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper<unsigned short>(CamImgPairU16List(1, cam_img_pair), h5, compress);
}

void xreg::WriteProjDataH5(const CamImgPairU8& cam_img_pair,
                           H5::Group* h5,
                           const bool compress)
{
  WriteProjDataH5Helper<unsigned char>(CamImgPairU8List(1, cam_img_pair), h5, compress);
}

namespace  // un-named
{

template <class tProjData>
void WriteProjDataH5ToDiskHelper(const tProjData& pd, const std::string& path, const bool compress)
{
  H5::H5File h5(path, H5F_ACC_TRUNC);
  
  xreg::WriteProjDataH5(pd, &h5, compress);
  
  h5.flush(H5F_SCOPE_GLOBAL);
  h5.close();
}

}  // un-named

void xreg::WriteProjDataH5ToDisk(const ProjDataF32List& proj_data,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(proj_data, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const ProjDataU16List& proj_data,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(proj_data, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const ProjDataU8List& proj_data,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(proj_data, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const ProjDataF32& proj_data,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(proj_data, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const ProjDataU16& proj_data,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(proj_data, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const ProjDataU8& proj_data,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(proj_data, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const CamImgPairF32List& cam_img_pairs,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(cam_img_pairs, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const CamImgPairU16List& cam_img_pairs,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(cam_img_pairs, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const CamImgPairU8List& cam_img_pairs,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(cam_img_pairs, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const CamImgPairF32& cam_img_pair,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(cam_img_pair, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const CamImgPairU16& cam_img_pair,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(cam_img_pair, path, compress);
}

void xreg::WriteProjDataH5ToDisk(const CamImgPairU8& cam_img_pair,
                                 const std::string& path,
                                 const bool compress)
{
  WriteProjDataH5ToDiskHelper(cam_img_pair, path, compress);
}

namespace  // un-named
{

using namespace xreg;

template <class tPixelScalar>
std::vector<ProjData<tPixelScalar>>
ReadProjDataHelper(const H5::Group& h5, const bool read_pixels)
{
  using PixelScalar  = tPixelScalar;
  using ProjDataType = ProjData<PixelScalar>;
  using ProjDataList = std::vector<ProjDataType>;

  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", h5);

  ProjDataList projs(num_projs);

  for (size_type i = 0; i < num_projs; ++i)
  {
    const std::string proj_g_path = fmt::format("proj-{:03d}", i);

    const H5::Group proj_g = h5.openGroup(proj_g_path);
    
    projs[i].cam = ReadCamModelH5(proj_g.openGroup("cam"));

    if (read_pixels)
    {
      projs[i].img = detail::ReadNDImageH5Helper<PixelScalar,2>(proj_g.openGroup("img"));
    }

    // Read landmarks if present
    if (ObjectInGroupH5("landmarks", proj_g))
    {
      const H5::Group lands_g = proj_g.openGroup("landmarks");
      
      const hsize_t num_lands = lands_g.getNumObjs();

      if (num_lands)
      {
        auto& m = projs[i].landmarks;

        m.reserve(num_lands);

        for (hsize_t l = 0; l < num_lands; ++l)
        {
          const std::string land_name = lands_g.getObjnameByIdx(l);

          m.emplace(land_name, ReadMatrixH5CoordScalar(land_name, lands_g));
        }
      }
    }
    
    if (ObjectInGroupH5("rot-to-pat-up", proj_g))
    {
      projs[i].rot_to_pat_up = static_cast<ProjDataRotToPatUp>(
                                  ReadSingleScalarH5Int("rot-to-pat-up", proj_g));
    }
    
    if (ObjectInGroupH5("det-spacings-from-orig-meta", proj_g))
    {
      projs[i].det_spacings_from_orig_meta = ReadSingleScalarH5Bool(
                                                "det-spacings-from-orig-meta", proj_g);
    }

    if (ObjectInGroupH5("orig-dcm-meta", proj_g))
    {
      H5::Group orig_meta_g = proj_g.openGroup("orig-dcm-meta");

      // double-check that DICOM metadata was written here
      xregASSERT(GetStringAttr("meta-type", orig_meta_g) == "dicom");
      
      projs[i].orig_dcm_meta = std::make_shared<DICOMFIleBasicFields>();
      
      *projs[i].orig_dcm_meta = ReadDICOMFieldsH5(orig_meta_g);
    }
  }

  return projs;
}

template <class tPixelScalar>
typename itk::Image<tPixelScalar,2>::Pointer
ReadSingleImgFromProjDataHelper(const H5::Group& h5, const size_type proj_idx)
{
  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", h5);

  xregASSERT(proj_idx < num_projs);
  
  return detail::ReadNDImageH5Helper<tPixelScalar,2>(h5.openGroup(fmt::format("proj-{:03d}/img", proj_idx)));
}

template <class tPixelScalar>
std::vector<ProjData<tPixelScalar>>
ReadProjDataH5FromDiskHelper(const std::string& path, const bool read_pixels)
{
  return ReadProjDataHelper<tPixelScalar>(H5::H5File(path, H5F_ACC_RDONLY), read_pixels);
}

template <class tPixelScalar>
typename itk::Image<tPixelScalar,2>::Pointer
ReadSingleImgFromProjDataFromDiskHelper(const std::string& path, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataHelper<tPixelScalar>(H5::H5File(path, H5F_ACC_RDONLY), proj_idx);
}

}  // un-named

xreg::ProjDataF32List xreg::ReadProjDataH5F32(const H5::Group& h5, const bool read_pixels)
{
  return ReadProjDataHelper<float>(h5, read_pixels);
}

xreg::ProjDataU16List xreg::ReadProjDataH5U16(const H5::Group& h5, const bool read_pixels)
{
  return ReadProjDataHelper<unsigned short>(h5, read_pixels);
}

xreg::ProjDataU8List xreg::ReadProjDataH5U8(const H5::Group& h5, const bool read_pixels)
{
  return ReadProjDataHelper<unsigned char>(h5, read_pixels);
}

xreg::ProjDataF32::ProjPtr xreg::ReadSingleImgFromProjDataH5F32(const H5::Group& h5, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataHelper<float>(h5, proj_idx);
}

xreg::ProjDataU16::ProjPtr xreg::ReadSingleImgFromProjDataH5U16(const H5::Group& h5, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataHelper<unsigned short>(h5, proj_idx);
}

xreg::ProjDataU8::ProjPtr xreg::ReadSingleImgFromProjDataH5U8(const H5::Group& h5, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataHelper<unsigned char>(h5, proj_idx);
}

std::vector<xreg::CameraModel> xreg::ReadCamModelsFromProjData(const H5::Group& h5)
{
  return ExtractCamModels(ReadProjDataH5F32(h5, false));
}

xreg::ProjDataF32List xreg::ReadProjDataH5F32FromDisk(const std::string& path, const bool read_pixels)
{
  return ReadProjDataH5FromDiskHelper<float>(path, read_pixels);
}

xreg::ProjDataU16List xreg::ReadProjDataH5U16FromDisk(const std::string& path, const bool read_pixels)
{
  return ReadProjDataH5FromDiskHelper<unsigned short>(path, read_pixels);
}

xreg::ProjDataU8List xreg::ReadProjDataH5U8FromDisk(const std::string& path, const bool read_pixels)
{
  return ReadProjDataH5FromDiskHelper<unsigned char>(path, read_pixels);
}

xreg::ProjDataF32::ProjPtr
xreg::ReadSingleImgFromProjDataH5F32FromDisk(const std::string& path, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataFromDiskHelper<float>(path, proj_idx);
}

xreg::ProjDataU16::ProjPtr
xreg::ReadSingleImgFromProjDataH5U16FromDisk(const std::string& path, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataFromDiskHelper<unsigned short>(path, proj_idx);
}

xreg::ProjDataU8::ProjPtr
xreg::ReadSingleImgFromProjDataH5U8FromDisk(const std::string& path, const size_type proj_idx)
{
  return ReadSingleImgFromProjDataFromDiskHelper<unsigned char>(path, proj_idx);
}

std::vector<CameraModel> xreg::ReadCamModelsFromProjDataFromDisk(const std::string& path)
{
  return ExtractCamModels(ReadProjDataH5F32FromDisk(path, false));
}

xreg::ProjDataScalarType xreg::GetProjDataScalarTypeH5(const H5::Group& h5)
{
  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", h5);
  xregASSERT(num_projs > 0);

  auto get_data_type = [&h5] (const size_type proj_idx)
  {
    const H5::Group img_g = h5.openGroup(fmt::format("proj-{:03d}/img", proj_idx));

    const H5::DataType data_type_h5 = img_g.openDataSet("pixels").getDataType();

    if (LookupH5DataType<float>() == data_type_h5)
    {
      return kPROJ_DATA_TYPE_FLOAT32;
    }
    else if (LookupH5DataType<unsigned short>() == data_type_h5)
    {
      return kPROJ_DATA_TYPE_UINT16;
    }
    else if (LookupH5DataType<unsigned char>() == data_type_h5)
    {
      return kPROJ_DATA_TYPE_UINT8;
    }
    else
    {
      xregThrow("unsupported HDF5 datatype for proj data!");
    }
  };
 
  const auto pd_dt = get_data_type(0);

  for (size_type i = 1; i < num_projs; ++i)
  {
    if (pd_dt != get_data_type(i))
    {
      xregThrow("Inconsistent proj data pixel types!");
    }
  }

  return pd_dt;
}

xreg::ProjDataScalarType xreg::GetProjDataScalarTypeFromDisk(const std::string& path)
{
  return GetProjDataScalarTypeH5(H5::H5File(path, H5F_ACC_RDONLY));
}

namespace  // un-named
{

using namespace xreg;

template <class tDstScalar, class tSrcScalar>
std::vector<ProjData<tDstScalar>>
CastProjData(const std::vector<ProjData<tSrcScalar>>& src_pd)
{
  const size_type num_projs = src_pd.size();

  std::vector<ProjData<tDstScalar>> dst_pd(num_projs);

  return dst_pd;
}

template <class tScalar>
typename ProjData<tScalar>::ProjPtr
DeferredProjReaderHelpler(std::vector<ProjData<tScalar>>* pd_ptr,
                          const size_type proj_idx,
                          const std::string& pd_path_on_disk,
                          const bool cache_imgs)
{
  using Scalar  = tScalar;
  using PD      = ProjData<Scalar>;
  using ProjPtr = typename PD::ProjPtr;

  auto& pd = *pd_ptr;

  xregASSERT(proj_idx < pd.size());
  
  ProjPtr to_ret;
  
  if (!cache_imgs || !pd[proj_idx].img)
  {
    to_ret = ReadSingleImgFromProjDataFromDiskHelper<Scalar>(pd_path_on_disk, proj_idx);
  }

  if (cache_imgs)
  {
    pd[proj_idx].img = to_ret;
  }

  return to_ret;
}

}  // un-namd

xreg::DeferredProjReader::DeferredProjReader(const std::string& path, const bool cache_imgs)
  : orig_path_(path), cache_imgs_(cache_imgs)
{
  {
    H5::H5File h5(path, H5F_ACC_RDONLY);

    scalar_type_on_disk_ = GetProjDataScalarTypeH5(h5);

    // only reads camera models and landmarks - no image pixels
    proj_data_F32_ = ReadProjDataH5F32(h5, false);
  }

  proj_data_U16_ = CastProjData<unsigned short>(proj_data_F32_);
  proj_data_U8_  = CastProjData<unsigned char>(proj_data_F32_);
}

xreg::ProjDataScalarType xreg::DeferredProjReader::scalar_type_on_disk() const
{
  return scalar_type_on_disk_;
}

xreg::size_type xreg::DeferredProjReader::num_projs_on_disk() const
{
  return proj_data_F32_.size();
}

const xreg::ProjDataF32List&
xreg::DeferredProjReader::proj_data_F32()
{
  return proj_data_F32_;
}

const xreg::ProjDataU16List&
xreg::DeferredProjReader::proj_data_U16()
{
  return proj_data_U16_;
}

const xreg::ProjDataU8List&
xreg::DeferredProjReader::proj_data_U8()
{
  return proj_data_U8_;
}

xreg::ProjDataF32::ProjPtr
xreg::DeferredProjReader::read_proj_F32(const size_type proj_idx)
{
  return DeferredProjReaderHelpler<float>(&proj_data_F32_, proj_idx, orig_path_, cache_imgs_);
}

xreg::ProjDataU16::ProjPtr
xreg::DeferredProjReader::read_proj_U16(const size_type proj_idx)
{
  return DeferredProjReaderHelpler<unsigned short>(&proj_data_U16_, proj_idx, orig_path_, cache_imgs_);
}

xreg::ProjDataU8::ProjPtr
xreg::DeferredProjReader::read_proj_U8(const size_type proj_idx)
{
  return DeferredProjReaderHelpler<unsigned char>(&proj_data_U8_, proj_idx, orig_path_, cache_imgs_);
}

void xreg::AddLandsToProjDataH5(const LandMap2& lands, const size_type proj_idx,
                                H5::Group* h5,
                                const bool delete_existing)
{
  const size_type num_projs = ReadSingleScalarH5ULong("num-projs", *h5);
  xregASSERT(proj_idx < num_projs);

  H5::Group proj_g = h5->openGroup(fmt::format("proj-{:03d}", proj_idx));
  
  AddProjDataLandsHelper(lands, &proj_g, delete_existing);
}


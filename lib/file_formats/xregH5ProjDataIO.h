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

#ifndef XREGH5PROJDATAIO_H_
#define XREGH5PROJDATAIO_H_

#include "xregProjData.h"

// forward declaration
namespace H5
{

class Group;

}  // H5

namespace xreg
{

extern const char* kXREG_PROJ_DATA_ATTR_STR;

void CopyProjDataH5(const H5::Group& src_proj_h5, H5::Group* dst_proj_h5);

void ReadProjDataFromH5AndWriteToDisk(const H5::Group& h5, const std::string& dst_disk_path);

//////////////////////////////////////////////////
// Write to HDF5 Data Structures
//////////////////////////////////////////////////

void WriteProjDataH5(const ProjDataF32List& proj_data,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const ProjDataU16List& proj_data,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const ProjDataU8List& proj_data,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const ProjDataF32& proj_data,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const ProjDataU16& proj_data,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const ProjDataU8& proj_data,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const CamImgPairF32List& cam_img_pairs,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const CamImgPairU16List& cam_img_pairs,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const CamImgPairU8List& cam_img_pairs,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const CamImgPairF32& cam_img_pair,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const CamImgPairU16& cam_img_pair,
                     H5::Group* h5,
                     const bool compress = true);

void WriteProjDataH5(const CamImgPairU8& cam_img_pair,
                     H5::Group* h5,
                     const bool compress = true);

//////////////////////////////////////////////////
// Write to HDF5 Files
//////////////////////////////////////////////////

void WriteProjDataH5ToDisk(const ProjDataF32List& proj_data,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const ProjDataU16List& proj_data,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const ProjDataU8List& proj_data,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const ProjDataF32& proj_data,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const ProjDataU16& proj_data,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const ProjDataU8& proj_data,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const CamImgPairF32List& cam_img_pairs,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const CamImgPairU16List& cam_img_pairs,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const CamImgPairU8List& cam_img_pairs,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const CamImgPairF32& cam_img_pair,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const CamImgPairU16& cam_img_pair,
                           const std::string& path,
                           const bool compress = true);

void WriteProjDataH5ToDisk(const CamImgPairU8& cam_img_pair,
                           const std::string& path,
                           const bool compress = true);

//////////////////////////////////////////////////
// Read from HDF5 Data Structures
//////////////////////////////////////////////////


ProjDataF32List ReadProjDataH5F32(const H5::Group& h5, const bool read_pixels = true);

ProjDataU16List ReadProjDataH5U16(const H5::Group& h5, const bool read_pixels = true);

ProjDataU8List ReadProjDataH5U8(const H5::Group& h5, const bool read_pixels = true);

ProjDataF32::ProjPtr ReadSingleImgFromProjDataH5F32(const H5::Group& h5, const size_type proj_idx);

ProjDataU16::ProjPtr ReadSingleImgFromProjDataH5U16(const H5::Group& h5, const size_type proj_idx);

ProjDataU8::ProjPtr ReadSingleImgFromProjDataH5U8(const H5::Group& h5, const size_type proj_idx);

std::vector<CameraModel> ReadCamModelsFromProjData(const H5::Group& h5);

//////////////////////////////////////////////////
// Read from HDF5 Files
//////////////////////////////////////////////////

ProjDataF32List ReadProjDataH5F32FromDisk(const std::string& path, const bool read_pixels = true);

ProjDataU16List ReadProjDataH5U16FromDisk(const std::string& path, const bool read_pixels = true);

ProjDataU8List ReadProjDataH5U8FromDisk(const std::string& path, const bool read_pixels = true);

ProjDataF32::ProjPtr ReadSingleImgFromProjDataH5F32FromDisk(const std::string& path, const size_type proj_idx);

ProjDataU16::ProjPtr ReadSingleImgFromProjDataH5U16FromDisk(const std::string& path, const size_type proj_idx);

ProjDataU8::ProjPtr ReadSingleImgFromProjDataH5U8FromDisk(const std::string& path, const size_type proj_idx);

std::vector<CameraModel> ReadCamModelsFromProjDataFromDisk(const std::string& path);

// Get pixel scalar type from proj data HDF5

enum ProjDataScalarType
{
  kPROJ_DATA_TYPE_FLOAT32,
  kPROJ_DATA_TYPE_UINT16,
  kPROJ_DATA_TYPE_UINT8
};

ProjDataScalarType GetProjDataScalarTypeH5(const H5::Group& h5);

ProjDataScalarType GetProjDataScalarTypeFromDisk(const std::string& path);

// Reader that does not read all projections in simultaneously - they may be read in
// sequentially as they need to be processed

class DeferredProjReader
{
public:
  explicit DeferredProjReader(const std::string& path, const bool cache_imgs = false);
  
  DeferredProjReader(const DeferredProjReader&) = delete;
  DeferredProjReader& operator=(const DeferredProjReader&) = delete;

  ProjDataScalarType scalar_type_on_disk() const;

  size_type num_projs_on_disk() const;

  const ProjDataF32List& proj_data_F32();

  const ProjDataU16List& proj_data_U16();

  const ProjDataU8List& proj_data_U8();

  ProjDataF32::ProjPtr read_proj_F32(const size_type proj_idx);
  
  ProjDataU16::ProjPtr read_proj_U16(const size_type proj_idx);
  
  ProjDataU8::ProjPtr read_proj_U8(const size_type proj_idx);

private:

  const std::string orig_path_;

  bool cache_imgs_;

  ProjDataScalarType scalar_type_on_disk_;

  ProjDataF32List proj_data_F32_;
  ProjDataU16List proj_data_U16_;
  ProjDataU8List  proj_data_U8_;
};

void AddLandsToProjDataH5(const LandMap2& lands, const size_type proj_idx,
                          H5::Group* h5,
                          const bool delete_existing = false);

}  // xreg

#endif


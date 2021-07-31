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

#ifndef XREGCIOSFUSIONDICOM_H_
#define XREGCIOSFUSIONDICOM_H_

#include "xregDICOMUtils.h"
#include "xregPerspectiveXform.h"

namespace xreg
{

Mat4x4 CIOSFusionCBCTExtrins();

/// \brief Populate a DICOM metadata struct with some basic params that match
///        a typical Digital Radiograph (DR) shot for the CIOS fusion.
///
/// The source to detector distance, pixel spacings, and rows/columns are all set to
/// nominal values. Not rotation/flipping flags are set.
DICOMFIleBasicFields MakeNaiveCIOSFusionMetaDR();

/// \brief Create a naive camera model using nominal values in the CIOS Fusion DICOM metadata.
///
/// The extrinsic coordinate frame is the same as the camera coordinate frame
/// (e.g. the transformation is identity), unless the second argument is passed true, in which
/// case the origin will lie at the isocenter of the c-arm (previously computed offline).
CameraModel NaiveCamModelFromCIOSFusion(const DICOMFIleBasicFields& meta,
                                        const bool iso_center_at_origin = false);

///  \brief Create a naive camera model using nominal values in the CIOS Fusion
///         DICOM metadata and Extrinsic xform.
/// 
/// The extrinsic coordinate frame is read in from program
CameraModel NaiveCamModelFromCIOSFusionExtrins(const DICOMFIleBasicFields& meta,
                                               const Mat4x4 extrins);

}  // xreg

#endif


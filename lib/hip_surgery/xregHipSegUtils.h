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

#ifndef XREGHIPSEGUTILS_H_
#define XREGHIPSEGUTILS_H_

#include "xregCommon.h"

namespace xreg
{

// Forward Declarations
struct PAOCutPlanes;

/// \brief Given a label map, guess the pelvis, left, and right femur labels.
///
/// Only non-zero labels are considered
/// The most common label is assigned pelvis. The next two most common labels
/// are candidates for femurs - left and right is determined by looking at the
/// X coordinate of the centroids, where the coordinate frame is assumed to be
/// LPS.
/// The output tuple's elements are
/// (pelvis label, left femur label, right femur label).
std::tuple<unsigned char,unsigned char,unsigned char>
GuessPelvisLeftRightFemurLabels(const itk::Image<unsigned char,3>* label_img);

std::tuple<unsigned short,unsigned short,unsigned short>
GuessPelvisLeftRightFemurLabels(const itk::Image<unsigned short,3>* label_img);

/// \brief Given a label map, that may or may not have fragment and cut labels,
///        guess what each label is, or will be.
/// 
/// The output tuple's elements are
/// (pelvis label, fragment label (if present), cut label (if present)
std::tuple<unsigned char,unsigned char,unsigned char>
GuessPelvisPAOFragCutLabels(const itk::Image<unsigned char,3>* labels,
                            const bool labels_has_frag,
                            const bool labels_has_cut);

std::tuple<unsigned short,unsigned short,unsigned short>
GuessPelvisPAOFragCutLabels(const itk::Image<unsigned short,3>* labels,
                            const bool labels_has_frag,
                            const bool labels_has_cut);

/// \brief Given a label map and femur point, guess the pelvis, femur, and fragment labels.
///
/// The output tuple's elements are
/// (pelvis label, femur label, fragment label, cut label)
std::tuple<unsigned char,unsigned char,unsigned char,unsigned char>
GuessPelvisFemurPAOFragLabels(const itk::Image<unsigned char,3>* labels,
                              const Pt3& femur_pt,
                              const bool labels_has_frag = true,
                              const bool labels_has_cut = true);

std::tuple<unsigned short,unsigned short,unsigned short,unsigned short>
GuessPelvisFemurPAOFragLabels(const itk::Image<unsigned short,3>* labels,
                              const Pt3& femur_pt,
                              const bool labels_has_frag = true,
                              const bool labels_has_cut = true);

/// \brief Given a volume and a label map, return three separate volumes representing
///        the pelvis, femur, and fragment sub-volumes.
///
/// If 0 is passed for any label, then it is inferred (guessed) from the label map.
/// Passing 0 for the femur label, requires that a physical point (in volume space)
/// representing the femur be provided.
/// The return tuple ordering is pelvis, femur, fragment.
std::tuple<itk::Image<float,3>::Pointer,
           itk::Image<float,3>::Pointer,
           itk::Image<float,3>::Pointer>
SplitIntoPelvisFemurFragVols(const itk::Image<float,3>* img,
                             const itk::Image<unsigned char,3>* labels,
                             const unsigned char pelvis_label,
                             const unsigned char femur_label,
                             const unsigned char frag_label,
                             const Pt3* femur_pt = nullptr);

/// \brief (PAO specific) Given a 3D segementation of all PAO cut voxels and
///        cutting plane definitions, populate lists of 3D points by the cut
///        they belong to.
///        
/// The input segmentation has one label assigned to all cuts (e.g. ilium,
/// ischium, pubis, and posterior all have the same label).
/// Null pointers may be passed for lists that do not need to be populated.
/// The physical points are in coordinate frame of the label map (volume).
void PAOExtract3DCutPointsForEachCut(const itk::Image<unsigned char,3>* labels,
                                     const unsigned char cut_label,
                                     const PAOCutPlanes& cut_defs,
                                     const FrameTransform& vol_to_app,
                                     Pt3List* ilium_cut_pts,
                                     Pt3List* ischium_cut_pts,
                                     Pt3List* pubis_cut_pts,
                                     Pt3List* post_cut_pts,
                                     std::vector<itk::Image<unsigned char,3>::IndexType>* ilium_cut_inds,
                                     std::vector<itk::Image<unsigned char,3>::IndexType>* ischium_cut_inds,
                                     std::vector<itk::Image<unsigned char,3>::IndexType>* pubis_cut_inds,
                                     std::vector<itk::Image<unsigned char,3>::IndexType>* post_cut_inds);

}  // xreg

#endif


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

#ifndef XREGLANDMARKMAPUTILS_H_
#define XREGLANDMARKMAPUTILS_H_

#include "xregCommon.h"

namespace xreg
{

/// \brief Prints a string -> 2D point mapping to a stream.
void PrintLandmarkMap(const LandMap2& pts, std::ostream& out);

/// \brief Prints a string -> 2D point mapping to a stream.
void PrintLandmarkMap(const LandMultiMap2& pts, std::ostream& out);

/// \brief Prints a string -> 3D point mapping to a stream.
void PrintLandmarkMap(const LandMap3& pts, std::ostream& out);

/// \brief Prints a string -> 3D point mapping to a stream.
void PrintLandmarkMap(const LandMultiMap3& pts, std::ostream& out);

void PrintLandmarkMap(const std::map<std::string,Pt3>& pts, std::ostream& out);

void PrintLandmarkMap(const std::multimap<std::string,Pt3>& pts, std::ostream& out);

LandMap2 CastToFloat(const std::unordered_map<std::string,Pt2_d>& m);
LandMap3 CastToFloat(const std::unordered_map<std::string,Pt3_d>& m);
LandMap4 CastToFloat(const std::unordered_map<std::string,Pt4_d>& m);
LandMultiMap2 CastToFloat(const std::unordered_multimap<std::string,Pt2_d>& m);
LandMultiMap3 CastToFloat(const std::unordered_multimap<std::string,Pt3_d>& m);
LandMultiMap4 CastToFloat(const std::unordered_multimap<std::string,Pt4_d>& m);

std::unordered_map<std::string,Pt2_d> CastToDouble(const LandMap2& m);
std::unordered_map<std::string,Pt3_d> CastToDouble(const LandMap3& m);
std::unordered_map<std::string,Pt4_d> CastToDouble(const LandMap4& m);
std::unordered_multimap<std::string,Pt2_d> CastToDouble(const LandMultiMap2& m);
std::unordered_multimap<std::string,Pt3_d> CastToDouble(const LandMultiMap3& m);
std::unordered_multimap<std::string,Pt4_d> CastToDouble(const LandMultiMap4& m);

/// \brief Given two different point sets defined by name mappings, create two
///        output point sets that are in correspondence by index using the input
///        names to determine correspondence.
///
/// \param pts1_map name/point mapping of point set 1
/// \param pts2_map name/point mapping of point set 2
/// \param dst_pts1 output subset of points from pts1_map that is in correspondence with dst_pts2
/// \param dst_pts2 output subset of points from pts2_map that is in correspondence with dst_pts1
/// \param names output list of landmark names in correspondence order
std::tuple<Pt3List,Pt3List,StrList>
CreateCorrespondencePointLists(const LandMap3& pts1_map, const LandMap3& pts2_map);

std::tuple<Pt3List,Pt2List,StrList>
CreateCorrespondencePointLists(const LandMap3& pts1_map, const LandMap2& pts2_map);

std::tuple<Pt2List,Pt3List,StrList>
CreateCorrespondencePointLists(const LandMap2& pts1_map, const LandMap3& pts2_map);

std::tuple<Pt2List,Pt2List,StrList>
CreateCorrespondencePointLists(const LandMap2& pts1_map, const LandMap2& pts2_map);

LandMap2 DropPtDim(const LandMap3& src_pts, const size_type dim);

LandMap2 PhysPtsToInds(const LandMap2& src_pts,
                       const CoordScalar pixel_spacing_x,
                       const CoordScalar pixel_spacing_y);

LandMap3 PhysPtsToInds(const LandMap3& src_pts,
                       const CoordScalar pixel_spacing_x,
                       const CoordScalar pixel_spacing_y,
                       const CoordScalar pixel_spacing_z);

}  // xreg

#endif


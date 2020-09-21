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

#include "xregLandmarkMapUtils.h"

#include <unordered_set>

#include <fmt/format.h>

#include "xregAssert.h"

namespace
{

template <class tIter>
void PrintLandmark2DMapHelper(tIter begin_it, tIter end_it, std::ostream& out)
{
  for (tIter it = begin_it; it != end_it; ++it)
  {
    out << fmt::format("{:10s}: {:+12.6f} , {:+12.6f}\n",
                              it->first,
                              it->second[0], it->second[1]);
  }
}

template <class tIter>
void PrintLandmark3DMapHelper(tIter begin_it, tIter end_it, std::ostream& out)
{
  for (tIter it = begin_it; it != end_it; ++it)
  {
    out << fmt::format("{:10s}: {:+12.6f} , {:+12.6f} , {:+12.6f}\n",
                              it->first,
                              it->second[0], it->second[1], it->second[2]);
  }
}

}  // un-named

void xreg::PrintLandmarkMap(const LandMap2& pts, std::ostream& out)
{
  PrintLandmark2DMapHelper(pts.begin(), pts.end(), out);
}

void xreg::PrintLandmarkMap(const LandMultiMap2& pts, std::ostream& out)
{
  PrintLandmark2DMapHelper(pts.begin(), pts.end(), out);
}

void xreg::PrintLandmarkMap(const LandMap3& pts, std::ostream& out)
{
  PrintLandmark3DMapHelper(pts.begin(), pts.end(), out);
}

void xreg::PrintLandmarkMap(const LandMultiMap3& pts, std::ostream& out)
{
  PrintLandmark3DMapHelper(pts.begin(), pts.end(), out);
}

void xreg::PrintLandmarkMap(const std::map<std::string,Pt3>& pts, std::ostream& out)
{
  PrintLandmark3DMapHelper(pts.begin(), pts.end(), out);
}


void xreg::PrintLandmarkMap(const std::multimap<std::string,Pt3>& pts, std::ostream& out)
{
  PrintLandmark3DMapHelper(pts.begin(), pts.end(), out);
}

namespace
{

template <class tSrcMap, class tDstMap>
void CastLandmarksMapHelper(const tSrcMap& src_map, tDstMap* dst_map)
{
  using DstMap    = tDstMap;
  using DstPoint  = typename DstMap::mapped_type;
  using DstScalar = typename DstPoint::Scalar;

  dst_map->clear();
  dst_map->reserve(src_map.size());

  for (const auto& l : src_map)
  {
    dst_map->emplace(l.first, l.second. template cast<DstScalar>());
  }
}

}  // un-named

xreg::LandMap2 xreg::CastToFloat(const std::unordered_map<std::string,Pt2_d>& m)
{
  LandMap2 dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

xreg::LandMap3 xreg::CastToFloat(const std::unordered_map<std::string,Pt3_d>& m)
{
  LandMap3 dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

xreg::LandMap4 xreg::CastToFloat(const std::unordered_map<std::string,Pt4_d>& m)
{
  LandMap4 dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

xreg::LandMultiMap2 xreg::CastToFloat(const std::unordered_multimap<std::string,Pt2_d>& m)
{
  LandMultiMap2 dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

xreg::LandMultiMap3 xreg::CastToFloat(const std::unordered_multimap<std::string,Pt3_d>& m)
{
  LandMultiMap3 dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

xreg::LandMultiMap4 xreg::CastToFloat(const std::unordered_multimap<std::string,Pt4_d>& m)
{
  LandMultiMap4 dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

std::unordered_map<std::string,xreg::Pt2_d> xreg::CastToDouble(const LandMap2& m)
{
  std::unordered_map<std::string,Pt2_d> dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

std::unordered_map<std::string,xreg::Pt3_d> xreg::CastToDouble(const LandMap3& m)
{
  std::unordered_map<std::string,Pt3_d> dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

std::unordered_map<std::string,xreg::Pt4_d> xreg::CastToDouble(const LandMap4& m)
{
  std::unordered_map<std::string,Pt4_d> dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

std::unordered_multimap<std::string,xreg::Pt2_d> xreg::CastToDouble(const LandMultiMap2& m)
{
  std::unordered_multimap<std::string,Pt2_d> dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

std::unordered_multimap<std::string,xreg::Pt3_d> xreg::CastToDouble(const LandMultiMap3& m)
{
  std::unordered_multimap<std::string,Pt3_d> dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

std::unordered_multimap<std::string,xreg::Pt4_d> xreg::CastToDouble(const LandMultiMap4& m)
{
  std::unordered_multimap<std::string,Pt4_d> dst_m;
  CastLandmarksMapHelper(m, &dst_m);
  return dst_m;
}

namespace
{

template <class tMap1, class tMap2, class tPointList1, class tPointList2, class tStringList>
void CreateCorrespondencePointListsHelper(const tMap1& pts1_map, const tMap2& pts2_map,
                                          tPointList1* dst_pts1, tPointList2* dst_pts2,
                                          tStringList* names)
{
  using map1_type  = tMap1;
  using key_type   = typename map1_type::key_type;
  using string_set = std::unordered_set<key_type>;
  using size_type  = typename string_set::size_type;

  // find the landmarks that are in both maps
  string_set common_names;

  for (typename map1_type::const_iterator m1_it = pts1_map.begin();
       m1_it != pts1_map.end(); ++m1_it)
  {
    if (pts2_map.find(m1_it->first) != pts2_map.end())
    {
      common_names.insert(m1_it->first);
    }
  }

  // loop through the keys that are common to both maps and
  // create the output lists with correspondence specified by index.

  const size_type num_pts = common_names.size();

  dst_pts1->clear();
  dst_pts2->clear();

  if (names)
  {
    names->clear();
    names->reserve(num_pts);
  }

  dst_pts1->reserve(num_pts);
  dst_pts2->reserve(num_pts);

  for (const auto& name : common_names)
  {
    dst_pts1->push_back(pts1_map.find(name)->second);
    dst_pts2->push_back(pts2_map.find(name)->second);

    if (names)
    {
      names->push_back(name);
    }
  }
}

}  // un-named

std::tuple<xreg::Pt3List,xreg::Pt3List,xreg::StrList>
xreg::CreateCorrespondencePointLists(const LandMap3& pts1_map, const LandMap3& pts2_map)
{
  Pt3List pts1;
  Pt3List pts2;
  StrList names;
  
  CreateCorrespondencePointListsHelper(pts1_map, pts2_map, &pts1, &pts2, &names);

  return std::make_tuple(pts1, pts2, names);
}

std::tuple<xreg::Pt3List,xreg::Pt2List,xreg::StrList>
xreg::CreateCorrespondencePointLists(const LandMap3& pts1_map, const LandMap2& pts2_map)
{
  Pt3List pts1;
  Pt2List pts2;
  StrList names;
  
  CreateCorrespondencePointListsHelper(pts1_map, pts2_map, &pts1, &pts2, &names);

  return std::make_tuple(pts1, pts2, names);
}

std::tuple<xreg::Pt2List,xreg::Pt3List,xreg::StrList>
xreg::CreateCorrespondencePointLists(const LandMap2& pts1_map, const LandMap3& pts2_map)
{
  Pt2List pts1;
  Pt3List pts2;
  StrList names;
  
  CreateCorrespondencePointListsHelper(pts1_map, pts2_map, &pts1, &pts2, &names);

  return std::make_tuple(pts1, pts2, names);
}

std::tuple<xreg::Pt2List,xreg::Pt2List,xreg::StrList>
xreg::CreateCorrespondencePointLists(const LandMap2& pts1_map, const LandMap2& pts2_map)
{
  Pt2List pts1;
  Pt2List pts2;
  StrList names;
  
  CreateCorrespondencePointListsHelper(pts1_map, pts2_map, &pts1, &pts2, &names);

  return std::make_tuple(pts1, pts2, names);
}

namespace
{

using namespace xreg;

Pt2 DropPtX(const Pt3& p)
{
  return p.tail(2);
}

Pt2 DropPtY(const Pt3& p)
{
  Pt2 q;
  q[0] = p[0];
  q[1] = p[2];
  
  return q;
}

Pt2 DropPtZ(const Pt3& p)
{
  return p.head(2);
}

}  // un-named

xreg::LandMap2 xreg::DropPtDim(const LandMap3& src_pts, const size_type dim)
{
  xregASSERT(dim < 3);

  LandMap2 dst_lands;
  dst_lands.reserve(src_pts.size());
  
  auto drop_fn = (dim == 2) ? DropPtZ : ((dim == 0) ? DropPtX : DropPtY);

  for (const auto& kv3 : src_pts)
  {
    dst_lands.emplace(kv3.first, drop_fn(kv3.second));
  }

  return dst_lands;
}

xreg::LandMap2 xreg::PhysPtsToInds(const LandMap2& src_pts,
                                   const CoordScalar pixel_spacing_x,
                                   const CoordScalar pixel_spacing_y)
{
  auto dst_inds = src_pts;
  
  for (auto& kv : dst_inds)
  {
    kv.second[0] /= pixel_spacing_x;
    kv.second[1] /= pixel_spacing_y;
  }

  return dst_inds;
}

xreg::LandMap3 xreg::PhysPtsToInds(const LandMap3& src_pts,
                                   const CoordScalar pixel_spacing_x,
                                   const CoordScalar pixel_spacing_y,
                                   const CoordScalar pixel_spacing_z)
{
  auto dst_inds = src_pts;
  
  for (auto& kv : dst_inds)
  {
    kv.second[0] /= pixel_spacing_x;
    kv.second[1] /= pixel_spacing_y;
    kv.second[2] /= pixel_spacing_z;
  }

  return dst_inds;
}


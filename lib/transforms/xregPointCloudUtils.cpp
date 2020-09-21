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

#include "xregPointCloudUtils.h"

#include "xregAssert.h"
#include "xregTBBUtils.h"

void xreg::ApplyTransform(const FrameTransform& xform, const Pt3List& src_pts, Pt3List* dst_pts)
{
  xregASSERT(src_pts.size() == dst_pts->size());

  auto xform_pts = [&xform, &src_pts, dst_pts] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      dst_pts->operator[](i) = xform * src_pts[i];
    }
  };

  ParallelFor(xform_pts, RangeType(0, src_pts.size()));
}

xreg::Pt3List xreg::ApplyTransform(const FrameTransform& xform, const Pt3List& src_pts)
{
  Pt3List dst_pts(src_pts.size());
  
  ApplyTransform(xform, src_pts, &dst_pts);

  return dst_pts;
}

namespace
{

using namespace xreg;

template <class tPt>
struct SumPointsAccFn
{
  using Pt     = tPt;
  using PtList = std::vector<Pt>;

  const PtList& pts;
  Pt sum;

  SumPointsAccFn(const Pt3List& pts_arg, const Pt& cur_sum = Pt::Zero())
    : pts(pts_arg), sum(cur_sum)
  { }

  SumPointsAccFn(SumPointsAccFn& other, xregSplitMarker)
    : pts(other.pts), sum(Pt::Zero())
  { }

  void operator()(const RangeType& r)
  {
    Pt tmp_sum = sum;

    for (size_type i = r.begin(); i < r.end(); ++i)
    {
      tmp_sum += pts[i];
    }

    sum = tmp_sum;
  }

  void join(SumPointsAccFn& rhs)
  {
    sum += rhs.sum;
  }
};

template <class tPt>
tPt ComputeCentroidHelper(const std::vector<tPt>& pts)
{
  const size_type num_pts = pts.size();

  SumPointsAccFn<tPt> acc_fn(pts);

  ParallelReduce(acc_fn, RangeType(0,num_pts));

  return acc_fn.sum / num_pts;
}

template <class tPt>
void OffsetPointsHelper(const tPt& pt, const std::vector<tPt>& src_pts, std::vector<tPt>* dst_pts)
{
  const size_type num_pts = src_pts.size();

  dst_pts->resize(num_pts);
  //xregASSERT(num_pts == dst_pts->size());

  auto offset_pts = [&pt, &src_pts, dst_pts] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      dst_pts->operator[](i) = src_pts[i] + pt;
    }
  };

  ParallelFor(offset_pts, RangeType(0,num_pts));
}

}  // un-named

xreg::Pt3 xreg::ComputeCentroid(const Pt3List& pts)
{
  return ComputeCentroidHelper(pts);
}

void xreg::OffsetPoints(const Pt3& pt, const Pt3List& src_pts, Pt3List* dst_pts)
{
  OffsetPointsHelper(pt, src_pts, dst_pts);
}

xreg::Pt3List xreg::OffsetPoints(const Pt3& pt, const Pt3List& src_pts)
{
  Pt3List dst_pts;

  OffsetPoints(pt, src_pts, &dst_pts);

  return dst_pts;
}

xreg::CoordScalar xreg::InnerProductAboutDimsOfPts(const Pt3List& pts1, const Pt3List& pts2,
                                                   const size_type d1, const size_type d2)
{
  const size_type num_pts = pts1.size();

  xregASSERT(num_pts == pts2.size());
  xregASSERT((d1 < 3) && (d2 < 3));
  
  auto reduce_fn = [&pts1,&pts2,d1,d2] (const RangeType& r, const CoordScalar& init_val)
  {
    CoordScalar ret = init_val;

    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      ret += pts1[i][d1] * pts2[i][d2];
    }

    return ret;
  };

  return ParallelReduce(CoordScalar(0), reduce_fn, std::plus<CoordScalar>(), RangeType(0,num_pts));
}

xreg::CoordScalar xreg::SumOfNormsSquared(const Pt3List& pts)
{
  auto reduce_fn = [&pts] (const RangeType& r, const CoordScalar& init_val)
  {
    CoordScalar ret = init_val;
  
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      ret += pts[i].squaredNorm();
    }

    return ret;
  };

  return ParallelReduce(CoordScalar(0), reduce_fn, std::plus<CoordScalar>(), RangeType(0, pts.size()));
}

void xreg::ComputeUnitVectors(const Pt3List& vecs, Pt3List* unit_vecs)
{
  const size_type num_vecs = vecs.size();

  xregASSERT(num_vecs == unit_vecs->size());

  auto compute_unit_vecs_fn = [&vecs, unit_vecs] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      unit_vecs->operator[](i) = vecs[i] / vecs[i].norm();
    }
  };

  ParallelFor(compute_unit_vecs_fn, RangeType(0, num_vecs));
}

xreg::Pt3List xreg::ComputeUnitVectors(const Pt3List& vecs)
{
  Pt3List unit_vecs(vecs.size());

  ComputeUnitVectors(vecs, &unit_vecs);

  return unit_vecs;
}

void xreg::AddScaledPoints(Pt3List* in_pts, const CoordScalar s,
                           const Pt3List& add_pts)
{
  const size_type num_pts = in_pts->size();
  xregASSERT(num_pts == add_pts.size());

  auto add_scaled_pts_fn = [in_pts, s, &add_pts](const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      in_pts->operator[](i) += s * add_pts[i];
    }
  };

  ParallelFor(add_scaled_pts_fn, RangeType(0, num_pts));
}

void xreg::ComputeL2Norms(const Pt3List& pts, CoordScalarList* norms)
{
  const size_type num_pts = pts.size();

  norms->resize(num_pts);

  auto norms_fn = [&pts, norms] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      norms->operator[](i) = pts[i].norm();
    }
  };

  ParallelFor(norms_fn, RangeType(0, num_pts));
}

void xreg::ScalePts(const CoordScalar& s, const Pt3List& src_pts, Pt3List* dst_pts)
{
  xregASSERT(src_pts.size() == dst_pts->size());

  if (&src_pts != dst_pts)
  {
    auto scale_fn = [s,&src_pts,dst_pts] (const RangeType& r)
    {
      for (size_type i = r.begin(); i != r.end(); ++i)
      {
        dst_pts->operator[](i) = src_pts[i] * s;
      }
    };

    ParallelFor(scale_fn, RangeType(0, src_pts.size()));
  }
  else
  {
    // The src and destination lists are the same, use the *= operator for greater
    // efficiency
    auto scale_fn = [s,dst_pts] (const RangeType& r)
    {
      for (size_type i = r.begin(); i != r.end(); ++i)
      {
        dst_pts->operator[](i) *= s;
      }
    };

    ParallelFor(scale_fn, RangeType(0, src_pts.size()));
  }
}

std::tuple<xreg::Pt3,xreg::CoordScalar>
xreg::FindClosestPtToPtCloudExhaustive(const Pt3& x, const Pt3List& pt_cloud)
{
  xregASSERT(!pt_cloud.empty());

  Pt3 cur_pt   = pt_cloud[0];
  Pt3 tmp_pt   = x - cur_pt;
  CoordScalar  cur_dist = tmp_pt.squaredNorm();
  CoordScalar  tmp_dist = 0;

  const size_type num_pts = pt_cloud.size();

  for (size_type i = 1; i < num_pts; ++i)
  {
    tmp_pt   = x - pt_cloud[i];
    tmp_dist = tmp_pt.squaredNorm();

    if (tmp_dist < cur_dist)
    {
      cur_pt   = pt_cloud[i];
      cur_dist = tmp_dist;
    }
  }

  return std::make_tuple(cur_pt, std::sqrt(cur_dist));
}

std::tuple<xreg::Pt3List,xreg::CoordScalarList>
xreg::FindClosestPointsAndDistsToPointCloudExhaustive(const Pt3List& cloud_pts, const Pt3List& query_pts)
{
  Pt3List closest_pts;
  CoordScalarList dists;

  FindClosestPointsAndDistsToPointCloudExhaustive(cloud_pts, query_pts, &closest_pts, &dists);

  return std::make_tuple(closest_pts, dists);
}

void xreg::FindClosestPointsAndDistsToPointCloudExhaustive(const Pt3List& cloud_pts, const Pt3List& query_pts,
                                                           Pt3List* closest_pts, CoordScalarList* dists)
{
  xregASSERT(closest_pts);

  const size_type num_query_pts = query_pts.size();

  closest_pts->resize(num_query_pts);
  
  if (dists)
  {
    dists->resize(num_query_pts);
  }

  Pt3 tmp_pt;
  CoordScalar tmp_dist;

  for (size_type i = 0; i < num_query_pts; ++i)
  {
    std::tie(tmp_pt, tmp_dist) = FindClosestPtToPtCloudExhaustive(query_pts[i], cloud_pts);

    closest_pts->operator[](i) = tmp_pt;

    if (dists)
    {
      dists->operator[](i) = tmp_dist;
    }
  }
}

xreg::Pt3List xreg::FindClosestPointsToPointCloudExhaustive(const Pt3List& cloud_pts, const Pt3List& query_pts)
{
  Pt3List closest_pts;

  FindClosestPointsAndDistsToPointCloudExhaustive(cloud_pts, query_pts, &closest_pts, nullptr);

  return closest_pts;
}



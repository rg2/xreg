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

#include "xregMetalObjs.h"
  
xreg::FrameTransform xreg::NaiveScrewModel::cap_to_body_xform() const
{
  FrameTransform xform = FrameTransform::Identity();
  
  xform.matrix()(1,3) = (body.height + cap.height) / -2;
  
  return xform;
}

xreg::FrameTransform xreg::NaiveScrewModel::body_to_cap_xform() const
{
  FrameTransform xform = FrameTransform::Identity();
  
  xform.matrix()(1,3) = (body.height + cap.height) / 2;
  
  return xform;
}

xreg::FrameTransform xreg::NaiveScrewModel::tip_to_body_xform() const
{
  FrameTransform xform = FrameTransform::Identity();

  xform.matrix()(1,3) = body.height / 2;

  return xform;
}

xreg::FrameTransform xreg::NaiveScrewModel::body_to_tip_xform() const
{
  FrameTransform xform = FrameTransform::Identity();

  xform.matrix()(1,3) = body.height / -2;

  return xform;
}

bool xreg::PointInNaiveScrew(const NaiveScrewModel& s, const Pt3& p)
{
  return PointInCylinder(s.body, p) ||
         PointInCylinder(s.cap, s.body_to_cap_xform() * p) ||
         PointInCone(s.tip, s.body_to_tip_xform() * p);
}

xreg::BoundBox3 xreg::ComputeBoundingBox(const NaiveScrewModel& s)
{
  return CombineBoundBoxes(TransformBoundBox(ComputeBoundingBox(s.cap),
                                             s.cap_to_body_xform()),
            CombineBoundBoxes(TransformBoundBox(ComputeBoundingBox(s.tip),
                                                s.tip_to_body_xform()),
                              ComputeBoundingBox(s.body)));
}

xreg::FrameTransform xreg::NaiveKWireModel::tip_to_body_xform() const
{
  FrameTransform xform = FrameTransform::Identity();

  xform.matrix()(1,3) = body.height / 2;

  return xform;
}

xreg::FrameTransform xreg::NaiveKWireModel::body_to_tip_xform() const
{
  FrameTransform xform = FrameTransform::Identity();

  xform.matrix()(1,3) = body.height / -2;

  return xform;
}

bool xreg::PointInNaiveKWire(const NaiveKWireModel& s, const Pt3& p)
{
  return PointInCylinder(s.body, p) ||
         PointInCone(s.tip, s.body_to_tip_xform() * p);
}

xreg::BoundBox3 xreg::ComputeBoundingBox(const NaiveKWireModel& s)
{
  return CombineBoundBoxes(TransformBoundBox(ComputeBoundingBox(s.tip),
                                             s.tip_to_body_xform()),
                           ComputeBoundingBox(s.body));
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::RayNaiveKWireIntersect(const Ray3& ray,
                             const NaiveKWireModel& kwire,
                             const CoordScalar max_dist_thresh)
{
  constexpr CoordScalar kTOL = 1.0e-6;

  CoordScalar d_cyl;
  SurIntersectInfo inter_info_cyl;

  std::tie(d_cyl,inter_info_cyl) = RayCylIntersect(ray, kwire.body, max_dist_thresh);

  const bool inter_cyl = d_cyl > -kTOL;

  const auto body_to_tip = kwire.body_to_tip_xform();
  
  Ray3 cone_ray;
  cone_ray.pt  = body_to_tip * ray.pt;
  cone_ray.dir = body_to_tip.matrix().block(0,0,3,3) * ray.dir;
  cone_ray.dir /= cone_ray.dir.norm();
  
  CoordScalar d_cone;
  SurIntersectInfo inter_info_cone;
  
  std::tie(d_cone,inter_info_cone) = RayConeIntersect(cone_ray, kwire.tip, max_dist_thresh);

  const bool inter_cone = d_cone > -kTOL;

  CoordScalar d = -1;
  SurIntersectInfo inter_info;
    
  if (inter_cyl)
  {
    if (!inter_cone || (d_cyl < d_cone))
    {
      d = d_cyl;
      inter_info = inter_info_cyl;
    }
    else
    {
      d = d_cone;
      inter_info = inter_info_cone;
    }
  }
  else if (inter_cone)
  {
    d = d_cone;
    inter_info = inter_info_cone;
  }
  
  return std::make_tuple(d,inter_info);
}


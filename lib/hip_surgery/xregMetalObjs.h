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

#ifndef XREGMETALOBJS_H_
#define XREGMETALOBJS_H_

#include "xregSpatialPrimitives.h"

namespace xreg
{

// Y-Axis is the screw axis with increasing values as
// you move from the cap of the screw to the tip.
// Origin is at the origin of the main cylinder body
// (e.g. halfway through the body's height axis).
struct NaiveScrewModel
{
  CylinderModel cap;
  CylinderModel body;
  ConeModel     tip;

  FrameTransform cap_to_body_xform() const;

  FrameTransform body_to_cap_xform() const;

  FrameTransform tip_to_body_xform() const;

  FrameTransform body_to_tip_xform() const;
};

bool PointInNaiveScrew(const NaiveScrewModel& s, const Pt3& p);

BoundBox3 ComputeBoundingBox(const NaiveScrewModel& s);

// Y-Axis is the k-wire cylinder axis with increasing values as
// you move towards the pointed tip.
// Origin is at the origin of the cylinder body (halfway along the cylinder height/y-axis).
struct NaiveKWireModel
{
  CylinderModel body;
  ConeModel     tip;

  FrameTransform tip_to_body_xform() const;

  FrameTransform body_to_tip_xform() const;
};

bool PointInNaiveKWire(const NaiveKWireModel& s, const Pt3& p);

BoundBox3 ComputeBoundingBox(const NaiveKWireModel& s);

std::tuple<CoordScalar,SurIntersectInfo>
RayNaiveKWireIntersect(const Ray3& ray,
                       const NaiveKWireModel& kwire,
                       const CoordScalar max_dist_thresh = -1);

}  // xreg

#endif


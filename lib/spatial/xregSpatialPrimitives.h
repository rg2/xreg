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

#ifndef XREGSPATIALPRIMITIVES_H_
#define XREGSPATIALPRIMITIVES_H_

#include "xregCommon.h"

#include <array>

namespace xreg
{

/// \brief Axis-Aligned rectangular slab/volume
struct AASlab3
{
  /// \brief Front lower left corner point
  Pt3 fll_pt;

  /// \brief Back upper right corner point
  Pt3 bur_pt;
};

/// \brief Check if a point lies within an axis-aligned bounding box
///
/// This excludes points on the boundary.
bool PointInAASlab3(const AASlab3& slab, const Pt3& pt);

/// \brief Sample points lying within the volume defined by an axis-aligned bounding box
///
/// An isotropic sampling space is used.
Pt3List SampleAASlab3PtsIso(const AASlab3& slab, const CoordScalar iso_spacing);

struct Plane3
{
  Pt3 normal;

  CoordScalar scalar;
};

/// \brief Applies a rigid transformation to a plane
///
/// 4x4 homogenous matrix is used to represent the transformation.
/// It is permitted to provide the inputs as outputs.
Plane3 TransformPlane(const Plane3& src_plane, const FrameTransform& xform);

/// \brief Applies a translation to a plane
Plane3 TranslatePlane(const Plane3& src_plane, const Pt3& translation);

/// \brief Finds the closest point on a plane to a given point (and plane).
///
/// The plane normal does not necessarily have to be a unit vector.
Pt3 FindClosestOnPlane(const Pt3& query_pt, const Plane3& plane);

/// \brief Compute the distance between a point and the closest point lying
///        on a plane
///
/// The plane normal does not necessarily have to be a unit vector.
/// This is NOT a signed distance, it is unsigned.
CoordScalar DistToPlane(const Pt3& query_pt, const Plane3& plane);

/// \brief Finds the closest point on a plane to a given point and the
///        separating distance.
///
/// The plane normal does not necessarily have to be a unit vector.
std::tuple<Pt3,CoordScalar>
FindClosestOnPlaneAndDist(const Pt3& query_pt, const Plane3& plane);

// Returns ( theta_rad, phi_rad, r )
std::tuple<CoordScalar,CoordScalar,CoordScalar>
PlaneNormalScalarToSpherical(const Plane3& plane);

Plane3
PlaneSphericalToNormalScalar(const CoordScalar theta_rad,
                             const CoordScalar phi_rad,
                             const CoordScalar r);

/// \brief Compute intersection of a ray, or line segment, with an axis aligned
///        3D rectangle.
/// Returns ( intersect flag , t_start, t_stop )
std::tuple<bool,CoordScalar,CoordScalar>
RayRectIntersect(const Pt3& min_rect_corner, const Pt3& max_rect_corner,
                 const Pt3& line_start_pt, const Pt3& line_vec, // should be of full length
                 const bool limit_to_segment = true);

struct BoundBox3
{
  Pt3 lower;
  Pt3 upper;
};

bool PointInBoundBox(const BoundBox3& bb, const Pt3& p);

BoundBox3 TransformBoundBox(const BoundBox3& bb, const FrameTransform& bb_to_world);

// Union of bounding boxes
BoundBox3 CombineBoundBoxes(const BoundBox3& bb1, const BoundBox3& bb2);

// origin is at center of cylinder circle and mid-height,
// y axis is cone height
struct CylinderModel
{
  CoordScalar radius;

  CoordScalar height;
};

bool PointInCylinder(const CylinderModel& c, const Pt3& p);

BoundBox3 ComputeBoundingBox(const CylinderModel& c);

// Origin of cone is on the base, y direction increases towards tip
struct ConeModel
{
  CoordScalar radius;

  CoordScalar height;
};

bool PointInCone(const ConeModel& c, const Pt3& p);

BoundBox3 ComputeBoundingBox(const ConeModel& c);

struct Ray3
{
  Pt3 pt;
  Pt3 dir;

  Pt3 operator()(const CoordScalar t) const;
};

// Origin is at the center of the disk, and the y axis is orthogonal
// to the plane
struct PlanarDisk3
{
  Plane3 plane;

  CoordScalar radius;
};

struct SurIntersectInfo
{
  Pt3 inter_pt = Pt3(Pt3::Zero());
  Pt3 normal   = Pt3(Pt3::Zero());  // outward facing

  // TODO: some kind of other info needed for rendering
};

struct Tri3ForRay3Intersect
{
  std::array<Pt3*,3> verts;

  // Placeholder
  //std::array<Pt3*,3> normals;

  Pt3 v1;
  Pt3 v2;

  Plane3 plane;

  // the following code is derived from my solution in the graphics class

  void init();

  // The direction component of the ray should be unit
  std::tuple<CoordScalar,SurIntersectInfo>
  intersect(const Ray3& ray, const CoordScalar max_dist_thresh = -1) const; 
};

std::tuple<CoordScalar,SurIntersectInfo>
ExhaustiveRayTrisIntersect(const Ray3& ray,
                           const std::vector<Tri3ForRay3Intersect>& tris,
                           const CoordScalar max_dist_thresh = -1);

// The direction component of the ray should be unit
std::tuple<CoordScalar,SurIntersectInfo>
RayPlaneIntersect(const Ray3& ray,
                  const Plane3& plane,
                  const CoordScalar max_dist_thresh = -1);

// The direction component of the ray should be unit
std::tuple<CoordScalar,SurIntersectInfo>
RayPlanarDiskIntersect(const Ray3& ray,
                       const PlanarDisk3& disk,
                       const CoordScalar max_dist_thresh = -1);

Plane3 MakePlane3(const Pt3& p0, const Pt3& p1, const Pt3& p2);

Plane3 MakePlane3(const std::array<Pt3,3>& pts);

// The direction component of the ray should be unit
std::tuple<CoordScalar,SurIntersectInfo>
RayBoxIntersect(const Ray3& ray,
                const BoundBox3& box,
                const CoordScalar max_dist_thresh = -1);

// The direction component of the ray should be unit
std::tuple<CoordScalar,SurIntersectInfo>
RayCylIntersect(const Ray3& ray,
                const CylinderModel& cyl,
                const CoordScalar max_dist_thresh = -1);

// The direction component of the ray should be unit
std::tuple<CoordScalar,SurIntersectInfo>
RayConeIntersect(const Ray3& ray,
                 const ConeModel& cone,
                 const CoordScalar max_dist_thresh = -1);

std::tuple<CoordScalar,SurIntersectInfo>
LineSegPlaneIntersect(const Pt3& start_pt,
                      const Pt3& end_pt,
                      const Plane3& plane,
                      const CoordScalar max_dist_thresh = -1);

/**
 * Finds the closest point between a query point and a triangle. The point type
 * assumes an interface that overloads the +,-,() operators and provides an
 * inner product method, dot(). An Eigen vector will meet these requirements.
 * @param x The query point
 * @param p Vertex 1 of the triangle
 * @param q Vertex 2 of the triangle
 * @param r Vertex 3 of the triangle
 * @param tri_pt The closest point on the triangle
 **/
Pt3 FindClosestPtInTri(const Pt3& x, const Pt3& p, const Pt3& q, const Pt3& r);

Pt3 FindClosestPtOnLineSegment(const Pt3& line_pt1, const Pt3& line_pt2, const Pt3& pt);

/// \brief Find the closest point to two line segments.
///
/// The two line segments are defined by their endpoints.
/// Derived from the naive, slow, example given in:
/// http://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf
std::tuple<Pt3,Pt3,Pt3>
FindClosestPtBetweenTwoLineSegments(const Pt3& l1_pt1, const Pt3& l1_pt2,
                                    const Pt3& l2_pt1, const Pt3& l2_pt2);

}  // xreg

#endif


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

#include "xregSpatialPrimitives.h"

#include "xregAssert.h"
#include "xregPolyFindZeros.h"

bool xreg::PointInAASlab3(const AASlab3& slab, const Pt3& pt)
{
  return (slab.fll_pt(0) < pt(0)) && (pt(0) < slab.bur_pt(0)) &&
         (slab.fll_pt(1) < pt(1)) && (pt(1) < slab.bur_pt(1)) &&
         (slab.fll_pt(2) < pt(2)) && (pt(2) < slab.bur_pt(2));
}

xreg::Pt3List xreg::SampleAASlab3PtsIso(const AASlab3& slab, const CoordScalar iso_spacing)
{
  const size_type num_samples_x = static_cast<size_type>(
                                    std::abs(slab.bur_pt(0) - slab.fll_pt(0))
                                                                / iso_spacing) + 1;
  
  const size_type num_samples_y = static_cast<size_type>(
                                    std::abs(slab.bur_pt(1) - slab.fll_pt(1))
                                                                / iso_spacing) + 1;

  const size_type num_samples_z = static_cast<size_type>(
                                    std::abs(slab.bur_pt(2) - slab.fll_pt(2))
                                                                / iso_spacing) + 1;

  Pt3 fll_to_bur = slab.bur_pt - slab.fll_pt;
  fll_to_bur(0) = (fll_to_bur(0) > 0) ? 1 : -1;
  fll_to_bur(1) = (fll_to_bur(1) > 0) ? 1 : -1;
  fll_to_bur(2) = (fll_to_bur(2) > 0) ? 1 : -1;
  fll_to_bur *= iso_spacing;

  Pt3List pts;
  pts.reserve(num_samples_x * num_samples_y * num_samples_z);

  Pt3 cur_pt = slab.fll_pt;

  for (size_type z = 0; z < num_samples_z; ++z, cur_pt(2) += fll_to_bur(2))
  {
    cur_pt(1) = slab.fll_pt(1); 
    for (size_type y = 0; y < num_samples_y; ++y, cur_pt(1) += fll_to_bur(1))
    { 
      cur_pt(0) = slab.fll_pt(0); 
      for (size_type x = 0; x < num_samples_x; ++x, cur_pt(0) += fll_to_bur(0))
      {
        pts.push_back(cur_pt);
      }
    }
  }
  
  return pts;
}

xreg::Plane3 xreg::TransformPlane(const Plane3& src_plane, const FrameTransform& xform)
{
  Plane3 rot_plane;

  rot_plane.normal = xform.matrix().block(0,0,3,3) * src_plane.normal;
  
  rot_plane.scalar = src_plane.scalar + (xform.matrix().block(0,3,3,1).transpose() * rot_plane.normal)(0);

  return rot_plane;
}

xreg::Plane3 xreg::TranslatePlane(const Plane3& src_plane, const Pt3& translation)
{
  Plane3 trans_plane;
  
  trans_plane.normal = src_plane.normal;
  trans_plane.scalar = src_plane.scalar + src_plane.normal.dot(translation);

  return trans_plane;
}

xreg::Pt3 xreg::FindClosestOnPlane(const Pt3& query_pt, const Plane3& plane)
{
  return query_pt - ((plane.normal.dot(query_pt) - plane.scalar) * (plane.normal / plane.normal.norm()));
}

xreg::CoordScalar xreg::DistToPlane(const Pt3& query_pt, const Plane3& plane)
{
  return std::abs(plane.normal.dot(query_pt) - plane.scalar) / plane.normal.norm();
}

std::tuple<xreg::Pt3,xreg::CoordScalar>
xreg::FindClosestOnPlaneAndDist(const Pt3& query_pt, const Plane3& plane)
{
  const CoordScalar signed_dist = (plane.normal.dot(query_pt) - plane.scalar) / plane.normal.norm();
  
  return std::make_tuple(Pt3(query_pt - (signed_dist * plane.normal)), std::abs(signed_dist));
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar,xreg::CoordScalar>
xreg::PlaneNormalScalarToSpherical(const Plane3& plane)
{
  //           /|
  //          / |
  //         /  |
  //        /   |
  //       /    |
  //      /     |
  //     /      |
  //    /       | n_y
  //   / theta  |
  //  /---------+ 
  //      n_x
  
  //      +---------/|
  //      |        / |
  //      |       /  |
  //      |      /   |
  //      |     /    |
  //      |    / |n| |
  // n_z  | phi      |
  //      |  /       | n_z
  //      | /        |
  //      |/---------+ 
  //      
  
  const CoordScalar n_norm = plane.normal.norm();
  
  return std::make_tuple(std::atan2(plane.normal[1], plane.normal[0]),
                         std::acos(plane.normal[2] / n_norm),
                         plane.scalar * n_norm);
}

xreg::Plane3
xreg::PlaneSphericalToNormalScalar(const CoordScalar theta_rad,
                                   const CoordScalar phi_rad,
                                   const CoordScalar r)
{
  Pt3 n;

  n[2] = std::cos(phi_rad);

  // Normal is unit, norm 1, therefore n_x^2 + n_y^2 + n_z^2 = 1
  // and n_x^2 + n_y^2 = 1 - n_z^2 = 1 - (cos(phi)^2) = sin(phi)^2
  // cos(theta) = n_x / sqrt(1 - n_z^2) = n_x / sin(phi) -> n_x = cos(theta) * sin(phi)
  // sin(theta) = n_y / sqrt(1 - n_z^2) = n_y / sin(phi) -> n_y = sin(theta) * sin(phi)
  
  const CoordScalar sin_phi = std::sin(phi_rad);

  n[0] = std::cos(theta_rad) * sin_phi;
  n[1] = std::sin(theta_rad) * sin_phi;

  xregASSERT(std::abs(n.norm() - 1) < 1.0e-6);

  return Plane3{n, r};
}

std::tuple<bool,xreg::CoordScalar,xreg::CoordScalar>
xreg::RayRectIntersect(const Pt3& min_rect_corner, const Pt3& max_rect_corner,
                       const Pt3& line_start_pt, const Pt3& line_vec, // should be of full length
                       const bool limit_to_segment)
{
  // derived from IntersectRayAABB() on page 180-181 (Section 5.3.3) of Real Time Collision Detection

  bool does_intersect = true;

  CoordScalar t_start = 0;
  CoordScalar t_stop  = limit_to_segment ? CoordScalar(1) : std::numeric_limits<CoordScalar>::infinity();

  std::array<CoordScalar,2> t = { 0, 0 };

  for (size_type slab_idx = 0; slab_idx < 3; ++slab_idx)
  {
    if (std::abs(line_vec[slab_idx]) > 1.0e-8)
    {
      const CoordScalar line_vec_inv = CoordScalar(1) / line_vec[slab_idx];

      t[0] = (min_rect_corner[slab_idx] - line_start_pt[slab_idx]) * line_vec_inv;
      t[1] = (max_rect_corner[slab_idx] - line_start_pt[slab_idx]) * line_vec_inv;

      if (t[1] < t[0])
      {
        std::swap(t[0], t[1]);
      }

      t_start = std::max(t_start, t[0]);
      t_stop  = std::min(t_stop,  t[1]);

      if (t_start > t_stop)
      {
        does_intersect = false;
        break;
      }
    }
    else if ((line_start_pt[slab_idx] < min_rect_corner[slab_idx]) ||
             (line_start_pt[slab_idx] > max_rect_corner[slab_idx]))
    {
      // The line segment is parallel to the slab
      does_intersect = false;
      break;
    }
  }

  return std::make_tuple(does_intersect, t_start, t_stop);
}

bool xreg::PointInBoundBox(const BoundBox3& bb, const Pt3& p)
{
  return (bb.lower(0) <= p(0)) && (p(0) <= bb.upper(0)) &&
         (bb.lower(1) <= p(1)) && (p(1) <= bb.upper(1)) &&
         (bb.lower(2) <= p(2)) && (p(2) <= bb.upper(2));
}

xreg::BoundBox3 xreg::TransformBoundBox(const BoundBox3& bb, const FrameTransform& bb_to_world)
{
  const Pt3 lll = bb_to_world * Pt3{ bb.lower(0), bb.lower(1), bb.lower(2) };
  const Pt3 llu = bb_to_world * Pt3{ bb.lower(0), bb.lower(1), bb.upper(2) };
  const Pt3 luu = bb_to_world * Pt3{ bb.lower(0), bb.upper(1), bb.upper(2) };
  const Pt3 lul = bb_to_world * Pt3{ bb.lower(0), bb.upper(1), bb.lower(2) };
  
  const Pt3 ull = bb_to_world * Pt3{ bb.upper(0), bb.lower(1), bb.lower(2) };
  const Pt3 ulu = bb_to_world * Pt3{ bb.upper(0), bb.lower(1), bb.upper(2) };
  const Pt3 uuu = bb_to_world * Pt3{ bb.upper(0), bb.upper(1), bb.upper(2) };
  const Pt3 uul = bb_to_world * Pt3{ bb.upper(0), bb.upper(1), bb.lower(2) };

  return BoundBox3{
    { std::min(lll(0), std::min(llu(0), std::min(luu(0), std::min(lul(0),
        std::min(ull(0), std::min(ulu(0), std::min(uuu(0), uul(0)))))))),
      std::min(lll(1), std::min(llu(1), std::min(luu(1), std::min(lul(1),
        std::min(ull(1), std::min(ulu(1), std::min(uuu(1), uul(1)))))))),
      std::min(lll(2), std::min(llu(2), std::min(luu(2), std::min(lul(2),
        std::min(ull(2), std::min(ulu(2), std::min(uuu(2), uul(2))))))))  },
    { std::max(lll(0), std::max(llu(0), std::max(luu(0), std::max(lul(0),
        std::max(ull(0), std::max(ulu(0), std::max(uuu(0), uul(0)))))))),
      std::max(lll(1), std::max(llu(1), std::max(luu(1), std::max(lul(1),
        std::max(ull(1), std::max(ulu(1), std::max(uuu(1), uul(1)))))))),
      std::max(lll(2), std::max(llu(2), std::max(luu(2), std::max(lul(2),
        std::max(ull(2), std::max(ulu(2), std::max(uuu(2), uul(2))))))))  } };
}

xreg::BoundBox3
xreg::CombineBoundBoxes(const BoundBox3& bb1, const BoundBox3& bb2)
{
  return BoundBox3{ { std::min(bb1.lower(0), bb2.lower(0)),
                      std::min(bb1.lower(1), bb2.lower(1)),
                      std::min(bb1.lower(2), bb2.lower(2)) },
                    { std::max(bb1.upper(0), bb2.upper(0)),
                      std::max(bb1.upper(1), bb2.upper(1)),
                      std::max(bb1.upper(2), bb2.upper(2)) } };
}

bool xreg::PointInCylinder(const CylinderModel& c, const Pt3& p)
{
  const CoordScalar half_height = c.height / 2;

  return (-half_height <= p(1)) && (p(1) <= half_height) &&
         (std::sqrt((p(0) * p(0)) + (p(2) * p(2))) <= c.radius); 
}

xreg::BoundBox3 xreg::ComputeBoundingBox(const CylinderModel& c)
{
  constexpr CoordScalar kEPS = 1.0e-6;

  const CoordScalar half_height = c.height / 2;

  return BoundBox3{ { -c.radius - kEPS, -half_height - kEPS, -c.radius - kEPS },
                    {  c.radius + kEPS,  half_height + kEPS,  c.radius + kEPS } };
}

bool xreg::PointInCone(const ConeModel& c, const Pt3& p)
{
  const CoordScalar y = p(1);

  const CoordScalar radius_at_y = (c.radius * (c.height - y)) / c.height;

  return (0 <= y) && (y <= c.height) &&
         (std::sqrt((p(0) * p(0)) + (p(2) * p(2))) <= radius_at_y);
}

xreg::BoundBox3 xreg::ComputeBoundingBox(const ConeModel& c)
{
  constexpr CoordScalar kEPS = 1.0e-6;

  return BoundBox3{ { -c.radius - kEPS, -kEPS,           -c.radius - kEPS },
                    {  c.radius + kEPS, c.height + kEPS,  c.radius + kEPS } };
}
  
xreg::Pt3 xreg::Ray3::operator()(const CoordScalar t) const
{
  return pt + (t * dir);
}

void xreg::Tri3ForRay3Intersect::init()
{
  // I will find the vectors which will be re-used for computing
  // barycentric coordinates on the triangle.
  // This assumes all points have been adjusted so that
  // p1 is at the origin.
  //
  // Set up the least squares problem using the normal equations:
  // B = [p2 p3] [beta; gamma] = q
  // [beta; gamma] = inv(B^T * B) * B^T * q
  //
  // In this function, we will compute inv(A^T * A) * A^T, storing
  // the first row in v1, and the second row in v2.
  
  const Pt3 p2 = *verts[1] - *verts[0];
  const Pt3 p3 = *verts[2] - *verts[0];

  // Let A = B^T * B
  const CoordScalar a11 = p2.squaredNorm();
  const CoordScalar a12 = p2.dot(p3);  // this is equal to a21
  const CoordScalar a22 = p3.squaredNorm();
  
  // This cannot be zero for a valid triangle.
  const CoordScalar det_A = (a11 * a22) - (a12 * a12);

  v1[0] = (a22 * p2[0]) - (a12 * p3[0]);
  v1[1] = (a22 * p2[1]) - (a12 * p3[1]);
  v1[2] = (a22 * p2[2]) - (a12 * p3[2]);

  v1 /= det_A;

  v2[0] = (a11 * p3[0]) - (a12 * p2[0]);
  v2[1] = (a11 * p3[1]) - (a12 * p2[1]);
  v2[2] = (a11 * p3[2]) - (a12 * p2[2]);

  v2 /= det_A;

  // pre-compute the plane this triangle lies in
  plane.normal = p2.cross(p3);
  plane.normal /= plane.normal.norm();
  plane.scalar = plane.normal.dot(*verts[0]);
  // TODO: ensure this creates an outward facing normal
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::Tri3ForRay3Intersect::intersect(const Ray3& ray, const CoordScalar max_dist_thresh) const
{
  const CoordScalar kTOL = 1.0e-6;

  CoordScalar dist = -1;
  CoordScalar t    = 0;

  SurIntersectInfo inter_info;

  // first we need to check that this ray can intersect the plane containing
  // this triangle
  
  const CoordScalar ray_dir_dot_plane_normal = ray.dir.dot(plane.normal);

  xregASSERT(std::abs(ray.dir.norm() - 1) < kTOL);
  bool intersects_plane = std::abs(ray_dir_dot_plane_normal - 1) > kTOL;

  Pt3 intersect_pt;

  if (intersects_plane)
  {
    // this ray intersects the plane since the ray direction is not orthogonal
    // to the plane normal, check to make sure the plane lies in the forward
    // direction of the ray and compute the intersection point.
    
    t = (plane.scalar - plane.normal.dot(ray.pt)) / ray_dir_dot_plane_normal;
    
    // make sure the triangle is in front of us: t > 0
    // also prune based on the max_dist_thresh parameter, no point in computing
    // barycentrics if the plane is too far away.
    if ((t > -kTOL) && ((max_dist_thresh < -kTOL) || (t < max_dist_thresh)))
    {
      intersect_pt = ray(t);
    }
    else
    {
      intersects_plane = false;
    }
  }
  else if (std::abs(plane.normal.dot(ray.pt) - plane.scalar) < kTOL)
  {
    // The ray is orthogonal to the direction of the plane, but the ray reference
    // point lies on the plane
    
    intersects_plane = true;
    intersect_pt = ray.pt;
  }

  if (intersects_plane)
  {
    const Pt3 p = intersect_pt - *verts[0];

    const CoordScalar beta = v1.dot(p);

    if ((-kTOL < beta) && (beta < (kTOL + 1)))
    {
      const CoordScalar gamma = v2.dot(p);

      if ((-kTOL < gamma) && (gamma < (kTOL + 1)))
      {
        const CoordScalar alpha = 1.0 - beta - gamma;

        if (alpha > -kTOL)
        {
          // The point on the plane is a valid point in the triangle
          dist = t;
         
          inter_info.inter_pt = intersect_pt;
          inter_info.normal   = plane.normal;

          // TODO: normal should actually be a linear combination of
          // the normals at each vertex:
          // inter_info.normal = (*normals[0] * alpha) +
          //                     (*normals[1] * beta)  +
          //                     (*normals[2] * gamma);

          // TODO: other stuff useful for rendering
        }
      }
    }
  }

  return std::make_tuple(dist,inter_info);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::ExhaustiveRayTrisIntersect(const Ray3& ray,
                                 const std::vector<Tri3ForRay3Intersect>& tris,
                                 const CoordScalar max_dist_thresh)
{
  const CoordScalar kTOL = 1.0e-6;

  CoordScalar      best_inter_dist = -1;
  SurIntersectInfo best_inter_info;
  
  CoordScalar      cur_inter_dist = -1;
  SurIntersectInfo cur_inter_info;
 
  CoordScalar cur_max_dist_thresh = max_dist_thresh;

  for (const auto& tri : tris)
  {
    std::tie(cur_inter_dist, cur_inter_info) = tri.intersect(ray, cur_max_dist_thresh);
  
    if (cur_inter_dist > -kTOL)
    {
      bool replace = false;

      if (best_inter_dist > -kTOL)
      {
        if (cur_inter_dist < best_inter_dist)
        {
          replace = true;
        }
      }
      else
      {
        replace = true;
      }

      if (replace)
      {
        best_inter_dist = cur_inter_dist;
        best_inter_info = cur_inter_info;
      
        cur_max_dist_thresh = best_inter_dist;
      }
    }
  }
  
  return std::make_tuple(best_inter_dist, best_inter_info);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::RayPlaneIntersect(const Ray3& ray,
                        const Plane3& plane,
                        const CoordScalar max_dist_thresh)
{
  constexpr CoordScalar kTOL = 1.0e-6;
  
  xregASSERT(std::abs(ray.dir.norm() - 1) < kTOL);

  const CoordScalar ray_dir_dot_plane_norm = ray.dir.dot(plane.normal);

  CoordScalar d = -1;
  SurIntersectInfo inter_info;

  if (std::abs(ray_dir_dot_plane_norm / plane.normal.norm()) > kTOL)
  {
    const CoordScalar t = (plane.scalar - plane.normal.dot(ray.pt)) / ray_dir_dot_plane_norm;
    
    // check that the ray intersects in the positive direction, and the distance
    // is smaller than the max dist thresh (if it is specified)
    // since the ray direction is unit, t is the distance
    if ((t > -kTOL) && ((max_dist_thresh < -kTOL) || (t < max_dist_thresh)))
    {
      d = t;
            
      inter_info.inter_pt = ray(t);
      inter_info.normal   = plane.normal;
    }
  }

  return std::make_tuple(d,inter_info);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::RayPlanarDiskIntersect(const Ray3& ray,
                             const PlanarDisk3& disk,
                             const CoordScalar max_dist_thresh)
{
  constexpr CoordScalar kTOL = 1.0e-6;
  
  xregASSERT(std::abs(ray.dir.norm() - 1) < kTOL);

  CoordScalar d;
  SurIntersectInfo inter_info;

  std::tie(d,inter_info) = RayPlaneIntersect(ray, disk.plane, max_dist_thresh);

  if (d > -kTOL)
  {
    // intersects plane, check to see if it lies within the circle
    if (((inter_info.inter_pt(0) * inter_info.inter_pt(0)) +
         (inter_info.inter_pt(2) * inter_info.inter_pt(2)) + kTOL) > (disk.radius * disk.radius))
    {
      d = -1;
    }
  }
  
  return std::make_tuple(d,inter_info);
}

xreg::Plane3 xreg::MakePlane3(const Pt3& p0, const Pt3& p1, const Pt3& p2)
{
  Plane3 p;
  p.normal = (p1 - p0).cross(p2 - p0);
  p.normal /= p.normal.norm();

  p.scalar = p.normal.dot(p0);

  return p;
}

xreg::Plane3 xreg::MakePlane3(const std::array<Pt3,3>& pts)
{
  return MakePlane3(pts[0], pts[1], pts[2]);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::RayBoxIntersect(const Ray3& ray,
                      const BoundBox3& box,
                      const CoordScalar max_dist_thresh)
{
  constexpr CoordScalar kTOL = 1.0e-6;

  auto check_bounds = [&box,kTOL] (const Pt3& x, const unsigned int dim1, const unsigned int dim2)
  {
    return ((box.lower(dim1) - kTOL) < x(dim1)) && (x(dim1) < (box.upper(dim1) + kTOL)) &&
           ((box.lower(dim2) - kTOL) < x(dim2)) && (x(dim2) < (box.upper(dim2) + kTOL));
  };

  using ThreePts = std::array<Pt3,3>;

  const std::array<ThreePts,6> side_pts =
    {
      ThreePts{
        box.lower,
        Pt3({ box.upper(0), box.lower(1), box.lower(2) }),
        Pt3({ box.lower(0), box.upper(1), box.lower(2) })
      },
      ThreePts{
        Pt3({ box.lower(0), box.lower(1), box.upper(2) }),
        Pt3({ box.upper(0), box.lower(1), box.upper(2) }), 
        Pt3({ box.lower(0), box.upper(1), box.upper(2) })
      },
      ThreePts{
        Pt3({ box.upper(0), box.lower(1), box.lower(2) }),
        Pt3({ box.upper(0), box.lower(1), box.upper(2) }), 
        Pt3({ box.upper(0), box.upper(1), box.lower(2) })
      },
      ThreePts{
        box.lower,
        Pt3({ box.lower(0), box.upper(1), box.lower(2) }),
        Pt3({ box.lower(0), box.lower(1), box.upper(2) }) 
      },
      ThreePts{
        box.lower,
        Pt3({ box.lower(0), box.lower(1), box.upper(2) }),
        Pt3({ box.upper(0), box.lower(1), box.lower(2) })
      },
      ThreePts{
        Pt3({ box.lower(0), box.upper(1), box.lower(2) }),
        Pt3({ box.upper(0), box.upper(1), box.lower(2) }),
        Pt3({ box.lower(0), box.upper(1), box.upper(2) })
      }
    };
 
  using Ind2 = std::array<unsigned int,2>;

  constexpr std::array<Ind2,6> side_inds =
    {
      Ind2({ 0, 1 }),
      Ind2({ 0, 1 }),
      Ind2({ 1, 2 }),
      Ind2({ 1, 2 }),
      Ind2({ 0, 2 }),
      Ind2({ 0, 2 })
    };
  
  CoordScalar d = -1;
  SurIntersectInfo inter_info;

  for (unsigned int i = 0; i < 6; ++i)
  {
    CoordScalar      cur_d;
    SurIntersectInfo cur_inter;

    std::tie(cur_d, cur_inter) = RayPlaneIntersect(ray, MakePlane3(side_pts[i]), max_dist_thresh);
    
    if ((cur_d > -kTOL) && check_bounds(cur_inter.inter_pt, side_inds[i][0], side_inds[i][1]))
    {
      if ((d < -kTOL) || (cur_d < d))
      {
        d = cur_d;
        inter_info = cur_inter;
      }
    }
  }
  
  return std::make_tuple(d, inter_info);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::RayCylIntersect(const Ray3& ray,
                      const CylinderModel& cyl,
                      const CoordScalar max_dist_thresh)
{
  constexpr CoordScalar kTOL = 1.0e-6;

  xregASSERT(std::abs(ray.dir.norm() - 1) < kTOL);

  const CoordScalar half_height = cyl.height / CoordScalar(2);

  CoordScalar top_d = -1;
  SurIntersectInfo top_inter_info;

  CoordScalar bot_d = -1;
  SurIntersectInfo bot_inter_info;

  PlanarDisk3 tmp_disk;
  tmp_disk.plane.normal(0) = 0;
  tmp_disk.plane.normal(1) = 1;
  tmp_disk.plane.normal(2) = 0;
  tmp_disk.plane.scalar = half_height;
  tmp_disk.radius = cyl.radius;

  std::tie(top_d,top_inter_info) = RayPlanarDiskIntersect(ray, tmp_disk);

  const bool inter_top = top_d > -kTOL;
  
  // next, check intersection with bottom plane
  tmp_disk.plane.normal(1) = -1;
  
  std::tie(bot_d,bot_inter_info) = RayPlanarDiskIntersect(ray, tmp_disk);

  const bool inter_bot = bot_d > -kTOL;

  CoordScalar d = -1;
  SurIntersectInfo inter_info;

  const bool inter_cap = inter_top || inter_bot;

  if (inter_cap)
  {
    if (!inter_bot)
    {
      // only intersected the top
      d = top_d;
      inter_info = top_inter_info;
    }
    else if (!inter_top)
    {
      // only intersected the bottom
      d = bot_d;
      inter_info = bot_inter_info;
    }
    else if (top_d < bot_d)
    {
      // intersected both caps, top intersection is nearest
      d = top_d;
      inter_info = top_inter_info;
    }
    else
    {
      // intersected both caps, bottom intersection is nearest
      d = bot_d;
      inter_info = bot_inter_info;
    }
  }

  // check against cylinder body if we do not hit both caps
  if (!inter_top || !inter_bot)
  {
    // setup the quadratic equation for the ray intersecting the cylinder:
    // (ray.pt(0) + t * ray.dir(0))^2 + (ray.pt(2) + t * ray.dir(2))^2 = cyl.radius^2
    
    const CoordScalar qf_a = (ray.dir(0) * ray.dir(0)) + (ray.dir(2) * ray.dir(2));
    const CoordScalar qf_b = 2 * ((ray.dir(0) * ray.pt(0)) + (ray.dir(2) * ray.pt(2)));
    const CoordScalar qf_c = (ray.pt(0) * ray.pt(0)) + (ray.pt(2) * ray.pt(2))
                                - (cyl.radius * cyl.radius);

    const auto new_ts = QuadraticFnRealRoots(qf_a, qf_b, qf_c);
    
    CoordScalar new_d = -1;
    SurIntersectInfo new_inter;

    // choose the best t that intersects the infinite cylinder
    for (const CoordScalar new_t : new_ts)
    {
      if (new_t > -kTOL)
      {
        // current estimate of t intersects the infinite cylinder
        
        new_inter.inter_pt = ray(new_t);
        if (std::abs(new_inter.inter_pt(1)) < (half_height + kTOL))
        {
          // intersection point is within height bounds, now verify that this point is a good candidate
          // (first point checked, or shorter distance than the first point)
          if ((new_d < -kTOL) || (new_t < new_d))
          {
            new_d = new_t;
          }
        }
      }
    }
    
    if (new_d > -kTOL)
    {
      // we have a valid collision with the cylinder body
      
      // set the normal vector at the collision point with cylinder body
      new_inter.normal(0) = new_inter.inter_pt(0);
      new_inter.normal(2) = new_inter.inter_pt(2);
      new_inter.normal /= new_inter.normal.norm();

      if (!inter_cap || (new_d < d))
      {
        // there was no intersection with a cap, or the intersection with the cylinder body
        // is closer, so use the intersection with the body
        d = new_d;
        inter_info = new_inter;
      }
    }
  }

  // check against the maximum distance threshold
  if ((max_dist_thresh > -kTOL) && (d > max_dist_thresh))
  {
    d = -1;
  }

  return std::make_tuple(d,inter_info);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::RayConeIntersect(const Ray3& ray,
                       const ConeModel& cone,
                       const CoordScalar max_dist_thresh)
{
  constexpr CoordScalar kTOL = 1.0e-6;

  xregASSERT(std::abs(ray.dir.norm() - 1) < kTOL);

  CoordScalar d = -1;
  SurIntersectInfo inter_info;
 
  // First check for intersection on the bottom disk

  PlanarDisk3 tmp_disk;
  tmp_disk.plane.normal(0) = 0;
  tmp_disk.plane.normal(1) = -1;
  tmp_disk.plane.normal(2) = 0;
  tmp_disk.plane.scalar = 0;
  tmp_disk.radius = cone.radius;

  std::tie(d,inter_info) = RayPlanarDiskIntersect(ray, tmp_disk);

  // now check against the cone part

  const CoordScalar r_sq = cone.radius * cone.radius;

  const CoordScalar h_sq = cone.height * cone.height;

  const CoordScalar r_sq_div_h_sq = r_sq / h_sq;

  // setup the quadratic formula to solve for the ray distance component (t)
  
  const CoordScalar qf_a = (ray.dir(0) * ray.dir(0))
                              + (ray.dir(2) * ray.dir(2))
                              - ((ray.dir(1) * ray.dir(1)) * r_sq) / h_sq;
  
  const CoordScalar qf_b = (2 * ray.dir(0) * ray.pt(0))
                              + (2 * ray.dir(2) * ray.pt(2))
                              + ((2 * ray.dir(1) * r_sq * (cone.height - ray.pt(1))) / h_sq);

  const CoordScalar qf_c = (ray.pt(0) * ray.pt(0))
                              + (ray.pt(2) * ray.pt(2))
                              - ((r_sq * (cone.height - ray.pt(1)) * (cone.height - ray.pt(1))) / h_sq);

  const auto new_ts = QuadraticFnRealRoots(qf_a, qf_b, qf_c);
  
  for (const auto new_t : new_ts)
  {
    if (new_t > -kTOL)
    {
      const auto new_p = ray(new_t);
      
      if ((new_p(1) > -kTOL) && (new_p(1) < (cone.height + kTOL)))
      {
        if ((d < -kTOL) || (new_t < d))
        {
          d = new_t;
          
          inter_info.inter_pt = new_p;        

          // TODO: set normal!!
        }
      }
    }
  }

  // check against the maximum distance threshold
  if ((max_dist_thresh > -kTOL) && (d > max_dist_thresh))
  {
    d = -1;
  }

  return std::make_tuple(d, inter_info);
}

std::tuple<xreg::CoordScalar,xreg::SurIntersectInfo>
xreg::LineSegPlaneIntersect(const Pt3& start_pt,
                            const Pt3& end_pt,
                            const Plane3& plane,
                            const CoordScalar max_dist_thresh)
{
  Ray3 r;

  r.pt = start_pt;

  r.dir = end_pt - start_pt;
  
  const CoordScalar seg_len = r.dir.norm();

  r.dir /= seg_len;

  auto inter_info = RayPlaneIntersect(r, plane, max_dist_thresh);
 
  auto& d = std::get<0>(inter_info);

  if (d > seg_len)
  {
    d = -1;
  }

  return inter_info;
}

namespace
{

using namespace xreg;

CoordScalar Clamp01(const CoordScalar& x)
{
  return std::max(std::min(x, CoordScalar(1)), CoordScalar(0));
}

}  // un-named

xreg::Pt3 xreg::FindClosestPtInTri(const Pt3& x, const Pt3& p, const Pt3& q, const Pt3& r)
{
  /*
     \    t
      \ 2 |
       \  |
        \ |
         \|
          \
          |\
          | \
          |  \  1
        3 | 0 \
      ____|____\______ s
          |     \
        4 |  5   \  6
          |       \
  */

  const Pt3 E0 = q - p;
  const Pt3 E1 = r - p;

  const Pt3 p_minus_x = p - x;

  const CoordScalar a = E0.dot(E0);
  const CoordScalar b = E0.dot(E1);
  const CoordScalar c = E1.dot(E1);
  const CoordScalar d = E0.dot(p_minus_x);
  const CoordScalar e = E1.dot(p_minus_x);

  const CoordScalar det = (a * c) - (b * b);
  const CoordScalar s   = (b * e) - (c * d);
  const CoordScalar t   = (b * d) - (a * e);

  Pt3 tri_pt;

  // check to make sure the triangle has not degenerated to a single point
  if ((std::abs(a) > 1.0e-8) && (std::abs(c) > 1.0e-8))
  {
    if ((s + t) > det)
    {
      if (t > 0)
      {
        if (s > 0)
        {
          // Region 1
          const CoordScalar s1 = Clamp01((e + c - d - b) / (a - (2 * b) + c));
          tri_pt = p + (s1 * E0) + ((1 - s1) * E1);
        }
        else
        {
          // Region 2
          if ((b + d - c - e) < 0)
          {
            const CoordScalar s1 = Clamp01((e + c - d - b) / (a - (2 * b) + c));
            tri_pt = p + (s1 * E0) + ((1 - s1) * E1);
          }
          else
          {
            tri_pt = p + (Clamp01(-e / c) * E1);
          }
        }
      }
      else
      {
        // Region 6
        if ((b + e - a - d) < 0)
        {
          const CoordScalar s1 = Clamp01((e + c - d - b) / (a - (2 * b) + c));
          tri_pt = p + (s1 * E0) + ((1 - s1) * E1);
        }
        else
        {
          tri_pt = p + (Clamp01(-d / a) * E0);
        }
      }
    }
    else
    {
      if (s > 0)
      {
        if (t > 0)
        {
          // Region 0
          const CoordScalar det_scale = 1 / det;
          tri_pt = p + (s * det_scale * E0) + (t * det_scale * E1);
        }
        else
        {
          // Region 5
          tri_pt = p + (Clamp01(-d / a) * E0);
        }
      }
      else
      {
        if (t > 0)
        {
          // Region 3
          tri_pt = p + (Clamp01(-e / c) * E1);
        }
        else
        {
          // Region 4
          if (d < 0)
          {
            tri_pt = p + (Clamp01(-d / a) * E0);
          }
          else
          {
            tri_pt = p + (Clamp01(-e / c) * E1);
          }
        }
      }
    }
  }
  else
  {
    tri_pt = p;  // triangle has degenerated to a single point
  }
  
  return tri_pt;
}

xreg::Pt3 xreg::FindClosestPtOnLineSegment(const Pt3& line_pt1, const Pt3& line_pt2, const Pt3& pt)
{
  Pt3 closest_pt = line_pt1;

  const Pt3 v = line_pt2 - line_pt1;

  const auto v_dot_v = v.dot(v);

  if (v_dot_v > 1.0e-6)
  {
    const auto t = pt.dot(v) / v_dot_v;
    
    if (t < 1.0e-6)
    {
      // closest point remains line_pt1
    }
    else if (t > (1.0 - 1.0e-6))
    {
      closest_pt = line_pt2;
    }
    else
    {
      closest_pt += t * v;
    }
  }

  return closest_pt;
}

namespace
{

using namespace xreg;

CoordScalar DistBetweenLinePtsHelper(const CoordScalar a, const CoordScalar b,
                                     const CoordScalar c, const CoordScalar d,
                                     const CoordScalar e, const CoordScalar s,
                                     const CoordScalar t)
{
  // NOTE: we don't need the f term, as it is a constant wrt s and t.
  return (a * s * s) - (2 * b * s * t) + (c * t * t) + (2 * d * s) - (2 * e * t);
}

}  // unnamed

std::tuple<xreg::Pt3,xreg::Pt3,xreg::Pt3>
xreg::FindClosestPtBetweenTwoLineSegments(const Pt3& l1_pt1, const Pt3& l1_pt2,
                                          const Pt3& l2_pt1, const Pt3& l2_pt2)
{
  const Pt3 v1 = l1_pt2 - l1_pt1;
  const Pt3 v2 = l2_pt2 - l2_pt1;

  const CoordScalar a = v1.dot(v1);

  const CoordScalar b = v1.dot(v2);

  const CoordScalar c = v2.dot(v2);

  const Pt3 v3 = l1_pt1 - l2_pt1;

  const CoordScalar d = v1.dot(v3);

  const CoordScalar e = v2.dot(v3);

  const CoordScalar delta = (a * c) - (b * b);

  CoordScalarList s(9);
  CoordScalarList t(9);

  s[0] = 0;
  t[0] = 0;

  s[1] = 1;
  t[1] = 0;

  s[2] = 0;
  t[2] = 1;

  s[3] = 1;
  t[3] = 1;

  s[4] = -d / a;
  t[4] = 0;

  s[5] = (b - d) / a;
  t[5] = 1;

  s[6] = 0;
  t[6] = e / c;

  s[7] = 1;
  t[7] = (b + e) / c;

  CoordScalarList dists(9);

  if (std::abs(delta) > 1.0e-6)
  {
    s[8] = ((b * e) - (c * d)) / delta;
    t[8] = ((a * e) - (b * d)) / delta;
  }
  else
  {
    s.resize(8);
    t.resize(8);
    dists.resize(8);
  }

  for (size_type i = 0; i < dists.size(); ++i)
  {
    dists[i] = DistBetweenLinePtsHelper(a, b, c, d, e, s[i], t[i]);
  }

  const size_type min_idx = std::min_element(dists.begin(), dists.end()) - dists.begin();

  const Pt3 l1_pt = l1_pt1 + (s[min_idx] * (l1_pt2 - l1_pt1));
  const Pt3 l2_pt = l2_pt1 + (t[min_idx] * (l2_pt2 - l2_pt1));

  return std::make_tuple(Pt3((l1_pt + l2_pt) / 2), l1_pt, l2_pt);
}


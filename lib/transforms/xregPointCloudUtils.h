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

#ifndef XREGPOINTCLOUDUTILS_H_
#define XREGPOINTCLOUDUTILS_H_

#include "xregCommon.h"

namespace xreg
{

/**
 * @brief Applies a transformation to a collection of points.
 *
 * The transformation is executed using the * operator, so an
 * Eigen matrix or Eigen transformation will work.
 * If a multi-threading library is available, this executes
 * each transformation in parallel.
 * @param xform The transformation to be applied
 * @param src_pts The list of points input to the transform
 * @param dst_pts The list of points output from the transform;
 *                must be initialized to have the appropriate length;
 *                may be \p src_pts
 **/
void ApplyTransform(const FrameTransform& xform, const Pt3List& src_pts, Pt3List* dst_pts);

/**
 * @brief Applies a transformation to a collection of points.
 *
 * The transformation is executed using the * operator, so an
 * Eigen matrix or Eigen transformation will work.
 * If a multi-threading library is available, this executes
 * each transformation in parallel.
 * @param xform The transformation to be applied
 * @param src_pts The list of points input to the transform
 * @param dst_pts The list of points output from the transform;
 *                must be initialized to have the appropriate length;
 *                may be \p src_pts
 **/
Pt3List ApplyTransform(const FrameTransform& xform, const Pt3List& src_pts);

Pt3 ComputeCentroid(const Pt3List& pts);

/**
 * @brief Offsets a point cloud by a translation vector.
 *
 * This routine is threaded.
 * @param src_pts The point cloud to offset
 * @param dst_pts_ptr The output/offset point cloud
 * @param pt The translation/offset vector/point
 **/
void OffsetPoints(const Pt3& pt, const Pt3List& src_pts, Pt3List* dst_pts);

Pt3List OffsetPoints(const Pt3& pt, const Pt3List& src_pts);

/**
 * @brief Computes the "Inner Product" between two point clouds by taking a
 * "slice" about a dimension in each collection.
 *
 * This was originally intended to be used by Horn's Quaternion Method.
 * This routine is threaded.
 * @param pts1 Point Cloud 1
 * @param pts2 Point Cloud 2
 * @param d1 Index into the dimension to slice in pts1
 * @param d2 Index into the dimension to slice in pts2
 * @param s The "inner product" output
 **/
CoordScalar InnerProductAboutDimsOfPts(const Pt3List& pts1, const Pt3List& pts2,
                                       const size_type d1, const size_type d2);

/**
 * @brief Compute the sum of the squares of the norms of each point in a list.
 *
 * This routine is threaded.
 * @param pts The point list to sum
 * @param s The output sum
 **/
CoordScalar SumOfNormsSquared(const Pt3List& pts);

/// \brief Computes the unit vectors for each vector in a collection
///
/// The output may be the same container as the input.
/// The output must already be allocated to the appropriate length.
void ComputeUnitVectors(const Pt3List& vecs, Pt3List* unit_vecs);

Pt3List ComputeUnitVectors(const Pt3List& vecs);

/**
 * @brief Add scaled points to existing points.
 *
 * @param in_pts The set of existing points that will be added to from a scaled
 *               collection of points
 * @param s The scale factor to be applied
 * @param add_pts The collection of points that will be scaled and added to in_pts
 **/
void AddScaledPoints(Pt3List* in_pts, const CoordScalar s,
                     const Pt3List& add_pts);

/// \brief Compute L2 norm values for each point in a collection.
///
/// This is parallelized when multi-threading is enabled.
void ComputeL2Norms(const Pt3List& pts, CoordScalarList* norms);

/**
 * @brief Scales a collection of points.
 *
 * Each point is scaled using the * operator; e.g.
 * dst_pts[i] = src_pts[i] * s;
 * @param s The scale factor
 * @param src_pts The list of points to be scaled (input)
 * @param dst_pts The list of scaled points (output);
 *                must be initialized to have the appropriate length;
 *                may be \p src_pts
 **/
void ScalePts(const CoordScalar& s, const Pt3List& src_pts, Pt3List* dst_pts);

std::tuple<Pt3,CoordScalar>
FindClosestPtToPtCloudExhaustive(const Pt3& x, const Pt3List& pt_cloud);

std::tuple<Pt3List,CoordScalarList>
FindClosestPointsAndDistsToPointCloudExhaustive(const Pt3List& cloud_pts, const Pt3List& query_pts);

void FindClosestPointsAndDistsToPointCloudExhaustive(const Pt3List& cloud_pts, const Pt3List& query_pts,
                                                     Pt3List* closest_pts, CoordScalarList* dists = nullptr);

Pt3List FindClosestPointsToPointCloudExhaustive(const Pt3List& cloud_pts, const Pt3List& query_pts);

}  // xreg

#endif


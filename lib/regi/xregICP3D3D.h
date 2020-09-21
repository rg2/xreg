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

#ifndef XREGICP3D3D_H_
#define XREGICP3D3D_H_

#include "xregCommon.h"
#include "xregObjWithOStream.h"
#include "xregKDTree.h"

namespace xreg
{

struct TriMesh;

struct PointToSurRegiICP : ObjWithOStream
{
  // Initial estimate of the transformation
  FrameTransform init_pts_to_sur_xform = FrameTransform::Identity();

  // The point cloud which is transformed and projected to the target surface
  const Pt3List* pts = nullptr;
 
  // The target surface which is used to compute closest point lookups 
  const TriMesh* sur = nullptr;
 
  // The maximum number of iterations to execute; a value less than 0 indicates there is no limit.
  int max_its = -1;

  /**
   * @brief The stopping tolerance.
   *
   * The stopping tolerance with respect to the change in mean surface distance
   * between iterations. Let \f$d_i\f$ define the mean surface distance at
   * iteration \f$i\f$ and \f$T\f$ denote this stopping tolerance, then a
   * stopping condition for the algorithm is:
   \f[
      T \leq \frac{d_{i+1}}{d_i} \leq 1
   \f]
   **/
  double stop_ratio = 0.999;

  /**
   * @brief Threshold used to identify outliers.
   *
   * This is used to identify outlier points at each ICP iteration. This value is in units of
   * standard deviations from the mean surface distance. Therefore, at each
   * iteration any points (after transformation by the current registration
   * estimate) that are greater than std_dev_outlier_coeff units of
   * standard deviation from the mean surface distance are not used to compute
   * the next registration estimate. A non-positive value indicates that no outlier detection
   * should be performed.
   **/
	double std_dev_outlier_coeff = 0.0;
 
  // Boolean flag indicating that a scaling factor is allowed in the registration transform
  bool allow_similarity = false;

  FrameTransform run();

  std::function<void(PointToSurRegiICP*)> start_of_processing_callback_fn;
  std::function<void(PointToSurRegiICP*)> end_of_processing_callback_fn;

  // Takes (this, iteration #, cur. xform) as arguments
  std::function<void(PointToSurRegiICP*, const size_type, const FrameTransform&)> start_of_iteration_callback_fn;
  std::function<void(PointToSurRegiICP*, const size_type, const FrameTransform&)> end_of_iteration_callback_fn;

private:
 
  // Initializes or allocates working data structures
  void init();

  // Working variables with allocated buffers that may be handy to keep around
  
  // KD-Tree data structures for the target surface
  using KDTree = KDTreeNode<KDTreeTri>;

  std::vector<KDTreeTri> kd_tree_tris;

  std::unique_ptr<KDTree> kd_tree;

  CoordScalarList match_dists;
  
  Pt3List match_pts;
  Pt3List reg_pts;

  Pt3List src_inliers;
  Pt3List match_inliers;
};

}  // xreg

#endif


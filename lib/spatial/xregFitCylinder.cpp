/*
 * MIT License
 *
 * Copyright (c) 2021 Robert Grupp
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

#include "xregFitCylinder.h"

#include <Eigen/Dense>

#include "xregAssert.h"
#include "xregFitCircle.h"

std::tuple<xreg::Pt3,xreg::CoordScalar> xreg::FitOrbitToPts(const Pt3List& pts)
{
  const size_type num_pts = pts.size();

  xregASSERT(num_pts > 2);

  // First, do PCA to get the approximate 3D plane of this orbit
  Pt3 mean_pt = Pt3::Zero();

  for (size_type i = 0; i < num_pts; ++i)
  {
    mean_pt += pts[i];
  }

  mean_pt /= static_cast<CoordScalar>(num_pts);

  MatMxN pts_mat(3, num_pts);
  
  for (size_type i = 0; i < num_pts; ++i)
  {
    pts_mat.col(i) = pts[i] - mean_pt;
  }

  const MatMxN cov_mat = (pts_mat * pts_mat.transpose()) / static_cast<CoordScalar>(num_pts - 1);
  
  Eigen::SelfAdjointEigenSolver<MatMxN> eig_solver(cov_mat);
  
  const Pt3 u1 = eig_solver.eigenvectors().col(1);
  const Pt3 u2 = eig_solver.eigenvectors().col(2);

  // Next, project the 3D points onto the plane to get a collections of 2D points
  // which lie on a circle

  Pt2List pts_2d;
  pts_2d.reserve(num_pts);
 
  Eigen::Matrix<CoordScalar,2,3> u_proj;
  u_proj.row(0) = u1.transpose();
  u_proj.row(1) = u2.transpose();

  for (size_type i = 0; i < num_pts; ++i)
  {
    pts_2d.push_back(u_proj * pts_mat.col(i));
  }

  // Next, fit a 2D circle to these points
  Pt2 center_pt_2d;
  CoordScalar radius = 0;
  std::tie(center_pt_2d, radius) = FitCircle2D(pts_2d);

  // Finally, recover the 3D location of the circle center
  return std::make_tuple(mean_pt + (u1 * center_pt_2d(0)) + (u2 * center_pt_2d(1)), radius);
}


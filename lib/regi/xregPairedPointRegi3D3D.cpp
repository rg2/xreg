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

#include "xregPairedPointRegi3D3D.h"

#include <Eigen/Eigenvalues>

#include "xregAssert.h"
#include "xregRotUtils.h"
#include "xregPointCloudUtils.h"
#include "xregLandmarkMapUtils.h"

xreg::FrameTransform
xreg::PairedPointRegi3D3D(const Pt3List& pts1, const Pt3List& pts2, const CoordScalar scale)
{
  const size_type num_pts = pts1.size();
  xregASSERT(num_pts == pts2.size());

  FrameTransform xform = FrameTransform::Identity();

  // These calls are threaded
  const Pt3 cent1 = ComputeCentroid(pts1);
  const Pt3 cent2 = ComputeCentroid(pts2);

  // These calls are threaded
  const Pt3List pts1_prime = OffsetPoints(-cent1, pts1);
  const Pt3List pts2_prime = OffsetPoints(-cent2, pts2);

  CoordScalar s = scale;
  if (s <= 0)
  {
    s = 0;

    const CoordScalar s1 = SumOfNormsSquared(pts1_prime);  // threaded

    if (std::abs(s1) > 1.0e-6)
    {
      const CoordScalar s2 = SumOfNormsSquared(pts2_prime);  // threaded

      s = std::sqrt(s2 / s1);
    }
  }

  CoordScalar S[9];

  CoordScalar* S_dst = S;
  for (size_type i = 0; i < 3; ++i)
  {
    for (size_type j = 0; j < 3; ++j, ++S_dst)
    {
      // This call is threaded
      *S_dst = InnerProductAboutDimsOfPts(pts1_prime, pts2_prime, i, j);
    }
  }

  const CoordScalar S_xx = S[0];
  const CoordScalar S_xy = S[1];
  const CoordScalar S_xz = S[2];
  const CoordScalar S_yx = S[3];
  const CoordScalar S_yy = S[4];
  const CoordScalar S_yz = S[5];
  const CoordScalar S_zx = S[6];
  const CoordScalar S_zy = S[7];
  const CoordScalar S_zz = S[8];

  Mat4x4 N_matrix;
  N_matrix(0,0) = S_xx + S_yy + S_zz;
  N_matrix(0,1) = S_yz - S_zy;
  N_matrix(0,2) = S_zx - S_xz;
  N_matrix(0,3) = S_xy - S_yx;
  N_matrix(1,0) = N_matrix(0,1);
  N_matrix(1,1) = S_xx - S_yy - S_zz;
  N_matrix(1,2) = S_xy + S_yx;
  N_matrix(1,3) = S_zx + S_xz;
  N_matrix(2,0) = N_matrix(0,2);
  N_matrix(2,1) = N_matrix(1,2);
  N_matrix(2,2) = -S_xx + S_yy - S_zz;
  N_matrix(2,3) = S_yz + S_zy;
  N_matrix(3,0) = N_matrix(0,3);
  N_matrix(3,1) = N_matrix(1,3);
  N_matrix(3,2) = N_matrix(2,3);
  N_matrix(3,3) = -S_xx - S_yy + S_zz;

  Eigen::EigenSolver<Mat4x4> eig_dec(N_matrix, true);

  size_type max_index = 0;
  eig_dec.eigenvalues().real().maxCoeff(&max_index);
  xform.linear().matrix() = QuatToRotMat(eig_dec.eigenvectors().col(max_index).real());

  xform.linear().matrix() *= s;

  xform.matrix().block(0,3,3,1) = cent2 - (xform.linear() * cent1);
  
  return xform;
}

xreg::FrameTransform
xreg::PairedPointRegi3D3D(const LandMap3& pts1, const LandMap3& pts2, const CoordScalar scale)
{
  const auto corr_lists = CreateCorrespondencePointLists(pts1, pts2);

  return PairedPointRegi3D3D(std::get<0>(corr_lists), std::get<1>(corr_lists), scale);
}


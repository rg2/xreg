/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Robert Grupp
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

#include "xregPerspectiveXform.h"

#include <cmath>

#include <fmt/printf.h>

#include "xregRotUtils.h"
#include "xregRigidUtils.h"
#include "xregAssert.h"

std::tuple<xreg::Mat3x3,xreg::Mat4x4,xreg::CoordScalar>
xreg::DecompProjMat(const Mat3x4& P, const bool use_pos_rho)
{
  using RowVec3 = Eigen::Matrix<CoordScalar,1,3>;

  // We'll ask for thin U and V, therefore the matrix type must be dynamic
  using JacSVD = Eigen::JacobiSVD<Eigen::Matrix<CoordScalar, Eigen::Dynamic, Eigen::Dynamic>>;

  Mat3x3 K;
  Mat4x4 T;

  xregASSERT(std::abs(P.block(0,0,3,3).determinant()) > 1.0e-8);

  const RowVec3 a1 = P.block(0, 0, 1, 3);
  const RowVec3 a2 = P.block(1, 0, 1, 3);
  const RowVec3 a3 = P.block(2, 0, 1, 3);

  const CoordScalar rho    = (use_pos_rho ? 1 : -1) / a3.norm();
  const CoordScalar rho_sq = rho * rho;
  
  const RowVec3 r3 = rho * a3;

  const CoordScalar x0 = rho_sq * (a1.dot(a3));
  const CoordScalar y0 = rho_sq * (a2.dot(a3));

  const RowVec3 a1_x_a3 = a1.cross(a3);
  const RowVec3 a2_x_a3 = a2.cross(a3);

  const CoordScalar a1_x_a3_dot_a2_x_a3 = a1_x_a3.dot(a2_x_a3);

  const CoordScalar a1_x_a3_norm = a1_x_a3.norm();
  const CoordScalar a2_x_a3_norm = a2_x_a3.norm();

  const RowVec3 r1 = a2_x_a3 / a2_x_a3_norm;
  const RowVec3 r2 = r3.cross(r1);

  Mat3x3 R;
  R.block(0, 0, 1, 3) = r1;
  R.block(1, 0, 1, 3) = r2;
  R.block(2, 0, 1, 3) = r3;

  // Make sure it's a rotation, these should only fail if the first 3x3 block
  // is not invertible.
  xregASSERT(std::abs(R.determinant() - 1) < 1.0e-4);
  xregASSERT(((R.transpose() * R) - Mat3x3::Identity()).norm() < 1.0e-4);

  CoordScalar sin_theta = 1;
  CoordScalar cos_theta = 0;

  if (std::abs(a1_x_a3_dot_a2_x_a3) > 1.0e-8)
  {
    // there is a shear
    cos_theta = -a1_x_a3_dot_a2_x_a3 / (a1_x_a3_norm * a2_x_a3_norm);
    sin_theta = std::sin(std::acos(cos_theta));
  }

  const CoordScalar alpha = rho_sq * a1_x_a3_norm * sin_theta;

  // beta has the sin(theta) term, but later when populating the matrix we
  // divide beta by sin(theta), so just leave it out here and not divide later.
  const CoordScalar beta  = rho_sq * a2_x_a3_norm; // * sin_theta;

  K.setIdentity();
  K(0,0) = alpha;
  K(0,1) = -alpha * (cos_theta / sin_theta);  // what I call gamma in Matlab
  K(1,1) = beta;
  K(0,2) = x0;
  K(1,2) = y0;

  T.setIdentity();
  T.block(0, 0, 3, 3) = R;

  JacSVD svd(K, Eigen::ComputeThinU | Eigen::ComputeThinV);

  T.block(0, 3, 3, 1) = svd.solve(rho * P.block(0, 3, 3, 1));

  return std::make_tuple(K,T,rho);
}

std::tuple<xreg::Mat3x3,xreg::Mat4x4,xreg::CoordScalar>
xreg::DecompProjMatQR(const Mat3x4& P)
{
  // copied from my MATLAB implementation [K, H, rho] = cisDecompProjMatQR(M)

  Mat3x3 perm = Mat3x3::Zero();
  perm(0,2) = 1;
  perm(1,1) = 1;
  perm(2,0) = 1;

  const Mat3x3 A = P.block(0,0,3,3).transpose() * perm;
  xregASSERT(std::abs(A.determinant()) > 1.0e-8);

  Eigen::HouseholderQR<Mat3x3> qr(A);

  const Mat3x3 Q = qr.householderQ();
  const Mat3x3 R = qr.matrixQR().triangularView<Eigen::Upper>();

  Mat3x3 intrins = perm * R.transpose() * perm;

  CoordScalar rho = intrins(2,2);

  Mat3x3 rot_mat = perm * Q.transpose();
  
  Mat3x3 N = Mat3x3::Identity();
  
  if (rot_mat.determinant() < 0)
  {
    // rotation matrix is actually a reflection; flip the direction of the last column
    N(2,2) = -1;
  }

  if ((intrins(0,0) * intrins(1,1)) < 0)
  {
    // when the first two diagonal elements do not agree on sign
    // flip one direction to get them both in agreement, but flip the direction
    // needed to have the first two elements positive.
    if (intrins(0,0) < 0)
    {
      N(0,0) = -1;
    }
    else
    {
      N(1,1) = -1;
    }
    
    // also flip the third element to maintain handedness
    N(2,2) = -1;
  }
  else if (intrins(0,0) < 0)
  {
    // the first two elements agree on sign, but are negative, flip both
    N(0,0) = -1;
    N(1,1) = -1;
  }
  
  intrins = intrins * N;
  rot_mat = N * rot_mat;

  // set rotation in extrinsic
  Mat4x4 extrins = Mat4x4::Identity();
  extrins.block(0,0,3,3) = rot_mat;
 
  // recover translation
  extrins.block(0,3,3,1) = intrins.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(P.block(0,3,3,1));
 
  // make sure the bottom right element of intrinsics has magnitude 1
  intrins /= std::abs(rho);

  return std::make_tuple(intrins, extrins, rho);
}

xreg::CoordScalar xreg::FocalLenFromIntrins(const Mat3x3& K, CoordScalar xps, CoordScalar yps)
{
  // Average of the focal lengths
  return (std::abs((K(0,0) * xps)) + std::abs((K(1,1) * ((yps < 0) ? xps : yps)))) / CoordScalar(2);
}

xreg::Mat3x4 xreg::ProjMat3x4FromIntrinsExtrins(const Mat3x3& intrins, const Mat4x4& extrins)
{
  Mat3x4 K_aug = Mat3x4::Zero();
  K_aug.block(0,0,3,3) = intrins;

  return K_aug * extrins;
}

xreg::Mat3x3 xreg::MakeNaiveIntrins(const CoordScalar focal_len,
                                    const unsigned long num_rows,
                                    const unsigned long num_cols,
                                    const CoordScalar pixel_row_spacing,
                                    const CoordScalar pixel_col_spacing,
                                    const bool z_is_neg)
{
  xregASSERT(focal_len > CoordScalar(1.0e-8));
  xregASSERT(num_rows && num_cols);
  xregASSERT((pixel_row_spacing > CoordScalar(1.0e-8)) &&
             (pixel_col_spacing > CoordScalar(1.0e-8)));

  Mat3x3 intrins = Mat3x3::Identity();
  intrins(0,0) = focal_len / pixel_col_spacing;
  intrins(1,1) = focal_len / pixel_row_spacing;
  // 2,2 is already one
  
  if (z_is_neg)
  {
    intrins(0,0) *= -1;
    intrins(1,1) *= -1;
  }

  // principal point is at the center pixel
  intrins(0,2) = (num_cols - 1) * 0.5;
  intrins(1,2) = (num_rows - 1) * 0.5;

  return intrins;
}

void xreg::CameraModel::setup(const CoordScalar focal_len_arg, const size_type nr, const size_type nc,
                              const CoordScalar rs, const CoordScalar cs)
{
  xregASSERT(focal_len_arg > CoordScalar(1.0e-8));
  xregASSERT(nr && nc);
  xregASSERT((rs > CoordScalar(1.0e-8)) && (cs > CoordScalar(1.0e-8)));

  focal_len = focal_len_arg;
  num_det_rows = nr;
  num_det_cols = nc;
  det_row_spacing = rs;
  det_col_spacing = cs;

  intrins = MakeNaiveIntrins(focal_len, num_det_rows, num_det_cols,
                             det_row_spacing, det_col_spacing,
                             coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_NEG_Z);
  
  intrins_inv = intrins.inverse();

  // no additional frame
  extrins     = FrameTransform::Identity();
  extrins_inv = FrameTransform::Identity();

  pinhole_pt = Pt3::Zero();
}
  
void xreg::CameraModel::setup(const Mat3x4& proj_mat, const size_type nr, const size_type nc,
                              const CoordScalar rs, const CoordScalar cs,
                              const bool use_extrins)
{
  xregASSERT(nr && nc);
  xregASSERT((rs > CoordScalar(1.0e-8)) && (cs > CoordScalar(1.0e-8)));

  num_det_rows = nr;
  num_det_cols = nc;
  det_row_spacing = rs;
  det_col_spacing = cs;

  Mat4x4 extrins_mat;
  std::tie(intrins,extrins_mat,std::ignore) = DecompProjMat(proj_mat,
                                                (coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_POS_Z) ||
                                                (coord_frame_type == kORIGIN_ON_DETECTOR));

  focal_len = FocalLenFromIntrins(intrins, det_col_spacing, det_row_spacing);

  intrins_inv = intrins.inverse();

  if (use_extrins)
  {
    extrins.matrix() = extrins_mat;
  }
  else
  {
    extrins = FrameTransform::Identity();
  }

  extrins_inv.matrix() = SE3Inv(extrins.matrix());

  if ((coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_POS_Z) ||
      (coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_NEG_Z))
  {
    pinhole_pt = extrins_inv.matrix().block(0, 3, 3, 1);
  }
  else
  {
    Pt3 pinhole_wrt_cam = Pt3::Zero();
    pinhole_wrt_cam(2) = focal_len;

    pinhole_pt = extrins_inv * pinhole_wrt_cam;
  }
}

void xreg::CameraModel::setup(const Mat3x3& intrins_mat, const Mat4x4& extrins_mat,
                              const size_type nr, const size_type nc,
                              const CoordScalar rs, const CoordScalar cs)
{
  xregASSERT(nr && nc);
  xregASSERT((rs > CoordScalar(1.0e-8)) && (cs > CoordScalar(1.0e-8)));

  num_det_rows = nr;
  num_det_cols = nc;
  det_row_spacing = rs;
  det_col_spacing = cs;

  intrins = intrins_mat;
  intrins_inv = intrins.inverse();

  focal_len = FocalLenFromIntrins(intrins, det_col_spacing, det_row_spacing);

  extrins = extrins_mat;
  extrins_inv.matrix() = SE3Inv(extrins.matrix());

  if ((coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_POS_Z) ||
      (coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_NEG_Z))
  {
    pinhole_pt = extrins_inv.matrix().block(0, 3, 3, 1);
  }
  else
  {
    Pt3 pinhole_wrt_cam = Pt3::Zero();
    pinhole_wrt_cam(2) = focal_len;

    pinhole_pt = extrins_inv * pinhole_wrt_cam;
  }
}
  
xreg::Pt3 xreg::CameraModel::proj_pt_to_det_pt(const Pt3& src_pt) const
{
  return ind_pt_to_phys_det_pt(phys_pt_to_ind_pt(src_pt));
}
  
xreg::Pt3List xreg::CameraModel::proj_pts_to_det(const Pt3List& src_pts) const
{
  const size_type num_pts = src_pts.size();

  Pt3List pts_on_det(num_pts);

  for (size_type pt_idx = 0; pt_idx < num_pts; ++pt_idx)
  {
    pts_on_det[pt_idx] = proj_pt_to_det_pt(src_pts[pt_idx]);
  }
  
  return pts_on_det;
}
  
xreg::Pt3 xreg::CameraModel::phys_pt_to_ind_pt(const Pt3& phys_pt) const
{
  Pt3 idx;
  
  if ((coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_POS_Z) ||
      (coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_NEG_Z))
  {
    idx = intrins * (extrins * phys_pt);
  }
  else
  {
    Pt3 p = extrins * phys_pt;
    p(2) = focal_len - p(2);

    idx = intrins * p;
  }
    
  idx /= idx[2];  // normalize for homogeneous continuous index
  
  return idx;
}
  
xreg::Pt3List xreg::CameraModel::phys_pts_to_ind_pts(const Pt3List& phys_pts) const
{
  const size_type num_pts = phys_pts.size();

  Pt3List idx_pts(num_pts);

  for (size_type pt_idx = 0; pt_idx < num_pts; ++pt_idx)
  {
    idx_pts[pt_idx] = phys_pt_to_ind_pt(phys_pts[pt_idx]);
  }
  
  return idx_pts;
}
  
xreg::Pt3 xreg::CameraModel::ind_pt_to_phys_det_pt(const Pt2& ind_pt) const
{
  Pt3 ind_pt3D;
  ind_pt3D(0) = ind_pt(0);
  ind_pt3D(1) = ind_pt(1);
  ind_pt3D(2) = 1;

  return ind_pt_to_phys_det_pt(ind_pt3D);
}
  
xreg::Pt3 xreg::CameraModel::ind_pt_to_phys_det_pt(const Pt3& ind_pt) const
{
  // When the camera coordinate frame origin is on the detector, then we need
  // to subtract [0;0;focal_len] after multiplying by the inverse intrinsic.
  Pt3 z_adjust = Pt3::Zero();
  if (coord_frame_type == kORIGIN_ON_DETECTOR)
  {
    z_adjust(2) = -focal_len;
  }
  
  const CoordScalar det_z_val = ((coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_NEG_Z) ? -1 : 1) * focal_len;

  return extrins_inv * ((intrins_inv * (det_z_val * ind_pt)) + z_adjust);
}
  
xreg::Pt3List xreg::CameraModel::ind_pts_to_phys_det_pts(const Pt3List& ind_pts) const
{
  const size_type num_pts = ind_pts.size();

  Pt3List phys_det_pts(num_pts);

  for (size_type pt_idx = 0; pt_idx < num_pts; ++pt_idx)
  {
    phys_det_pts[pt_idx] = ind_pt_to_phys_det_pt(ind_pts[pt_idx]);
  }
  
  return phys_det_pts;
}
  
bool xreg::CameraModel::operator==(const CameraModel& other) const
{
  return (num_det_cols == other.num_det_cols) &&
         (num_det_rows == other.num_det_rows) &&
         (coord_frame_type == other.coord_frame_type) &&
         (std::abs(det_row_spacing - other.det_row_spacing) < 1.0e-6) &&
         (std::abs(det_col_spacing - other.det_col_spacing) < 1.0e-6) &&
         (std::abs(focal_len - other.focal_len) < 1.0e-6) &&
         ((intrins - other.intrins).norm() < 1.0e-6) &&
         ((extrins.matrix() - other.extrins.matrix()).norm() < 1.0e-6);
}
  
bool xreg::CameraModel::operator!=(const CameraModel& other) const
{
  return !operator==(other);
}

xreg::CameraModel::Point3DGrid xreg::CameraModel::detector_grid() const
{
  Point3DGrid detector_pts(num_det_rows, num_det_cols);

  // for each pixel index find the location of the pixel on the detector plane
  // in "world" coordinates

  // When the camera coordinate frame origin is on the detector, then we need
  // to subtract [0;0;focal_len] after multiplying by the inverse intrinsic.
  Pt3 z_adjust = Pt3::Zero();
  if (coord_frame_type == kORIGIN_ON_DETECTOR)
  {
    z_adjust(2) = -focal_len;
  }

  const CoordScalar det_z_val = ((coord_frame_type == kORIGIN_AT_FOCAL_PT_DET_NEG_Z) ? -1 : 1) * focal_len;

  Pt3 tmp_det_idx = Pt3::Zero();
  tmp_det_idx[2] = det_z_val;

  for (size_type det_row_idx = 0; det_row_idx < num_det_rows; ++det_row_idx)
  {
    tmp_det_idx[1] = det_row_idx * det_z_val;

    for (size_type det_col_idx = 0; det_col_idx < num_det_cols; ++det_col_idx)
    {
      tmp_det_idx[0] = det_col_idx * det_z_val;

      detector_pts(det_row_idx, det_col_idx) = extrins_inv * ((intrins_inv * tmp_det_idx) + z_adjust);
    }
  }

  return detector_pts;
}

xreg::CameraModel xreg::UpdateCameraModelFor2DROI(const CameraModel& src_cam,
                                                  const int roi_start_col,
                                                  const int roi_start_row,
                                                  const int roi_end_col,
                                                  const int roi_end_row)
{
  xregASSERT((roi_end_col > roi_start_col) && (roi_end_row > roi_start_row));

  CameraModel dst_cam;
  dst_cam.coord_frame_type = src_cam.coord_frame_type;

  Mat3x3 dst_intrins = src_cam.intrins;
  dst_intrins(0,2) -= CoordScalar(roi_start_col);
  dst_intrins(1,2) -= CoordScalar(roi_start_row);

  dst_cam.setup(dst_intrins, src_cam.extrins.matrix(),
                roi_end_row - roi_start_row + 1,
                roi_end_col - roi_start_col + 1,
                src_cam.det_row_spacing, src_cam.det_col_spacing);

  return dst_cam;
}

std::tuple<xreg::Pt2,xreg::Pt2>
xreg::GetBoundingBox2DProjPts(const CameraModel& cam, const Pt3List& pts_3d)
{
  Pt2 top_left;
  Pt2 bot_right;
  
  const size_type num_pts = pts_3d.size();

  CoordScalar& min_c = top_left[0];
  CoordScalar& min_r = top_left[1];
  CoordScalar& max_c = bot_right[0];
  CoordScalar& max_r = bot_right[1];

  min_c = std::numeric_limits<CoordScalar>::max();
  min_r = std::numeric_limits<CoordScalar>::max();
  max_c = std::numeric_limits<CoordScalar>::lowest();
  max_r = std::numeric_limits<CoordScalar>::lowest();

  Pt3 proj_pt;

  for (size_type pt_idx = 0; pt_idx < num_pts; ++pt_idx)
  {
    proj_pt = cam.phys_pt_to_ind_pt(pts_3d[pt_idx]);
  
    min_c = std::min(min_c, proj_pt[0]);
    max_c = std::max(max_c, proj_pt[0]);
    
    min_r = std::min(min_r, proj_pt[1]);
    max_r = std::max(max_r, proj_pt[1]);
  }

  return std::make_tuple(top_left, bot_right);
}

xreg::CameraModel xreg::UpdateCameraModelTightBoundsForProjPts(const CameraModel& cam,
                                                               const Pt3List& pts_3d,
                                                               const CoordScalar pad_cols,
                                                               const CoordScalar pad_rows)
{
  Pt2 top_left;
  Pt2 bot_right;

  std::tie(top_left,bot_right) = GetBoundingBox2DProjPts(cam, pts_3d); 
  
  CoordScalar min_c = top_left[0]  - pad_cols;
  CoordScalar max_c = bot_right[0] + pad_cols;
  CoordScalar min_r = top_left[1]  - pad_rows;
  CoordScalar max_r = bot_right[1] + pad_rows;

  // clamp to original image bounds
  min_c = std::max(CoordScalar(0), std::floor(min_c));
  min_r = std::max(CoordScalar(0), std::floor(min_r));
  
  max_c = std::min(CoordScalar(cam.num_det_cols - 1), std::ceil(max_c));
  min_r = std::min(CoordScalar(cam.num_det_rows - 1), std::ceil(max_r));

  return UpdateCameraModelFor2DROI(cam,
                                   static_cast<size_type>(min_c),
                                   static_cast<size_type>(min_r),
                                   static_cast<size_type>(max_c),
                                   static_cast<size_type>(max_r));
}

xreg::CameraModel xreg::MoveFocalPointUpdateCam(const CameraModel& cam,
                                                const Pt3 src_delta)
{
  // No support for skewed cameras yet.
  xregASSERT(std::abs(cam.intrins(0,1)) < 1.0e-6);

  // Need to perform translations in the opposite directions for extrinsics
  // so objects stay in the same relative location with respect to the detector
  Pt3 extrins_trans;
  extrins_trans(0) = -src_delta(0);
  extrins_trans(1) = -src_delta(1);

  const CoordScalar col_spacing = cam.det_col_spacing;
  const CoordScalar row_spacing = cam.det_row_spacing;

  Mat3x3 new_intrins = cam.intrins;

  // X and Y cause the principal point to translate
  new_intrins(0,2) += src_delta(0) / col_spacing;
  new_intrins(1,2) += src_delta(1) / row_spacing;

  // Z causes the focal length to change
  CoordScalar new_focal_len = cam.focal_len;
  if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z)
  {
    new_focal_len -= src_delta(2);
    extrins_trans(2) = -src_delta(2);
  }
  else
  {
    new_focal_len += src_delta(2);

    if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
    {
      extrins_trans(2) = src_delta(2);
    }
    else
    {
      extrins_trans(2) = 0;
    }
  }
  
  new_intrins(0,0) = new_focal_len / col_spacing;
  new_intrins(1,1) = new_focal_len / row_spacing;

  if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
  {
    new_intrins(0,0) *= -1;
    new_intrins(1,1) *= -1;
  }

  const Mat4x4 new_extrins = TransXYZ4x4(extrins_trans) * cam.extrins.matrix();

  CameraModel new_cam;
  new_cam.coord_frame_type = cam.coord_frame_type;

  new_cam.setup(new_intrins, new_extrins, cam.num_det_rows, cam.num_det_cols,
                row_spacing, col_spacing);

  return new_cam;
}

xreg::Pt3 xreg::CalcSourcePositionDelta(const CameraModel& cam1, const CameraModel& cam2)
{
  xregASSERT(std::abs(cam1.intrins(0,1)) < 1.0e-6);  // no skew
  xregASSERT(std::abs(cam2.intrins(0,1)) < 1.0e-6);  // no skew
  xregASSERT(cam1.coord_frame_type == cam2.coord_frame_type);

  Pt3 delta_cam1_src = Pt3::Zero();

  delta_cam1_src(0) = (cam2.intrins(0,2) * cam2.det_col_spacing) -
                      (cam1.intrins(0,2) * cam1.det_col_spacing);

  delta_cam1_src(1) = (cam2.intrins(1,2) * cam2.det_row_spacing) -
                      (cam1.intrins(1,2) * cam1.det_row_spacing);

  delta_cam1_src(2) = cam1.focal_len - cam2.focal_len;
  if ((cam1.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z) ||
      (cam1.coord_frame_type == CameraModel::kORIGIN_ON_DETECTOR))
  {
    delta_cam1_src(2) *= -1;
  }

  return delta_cam1_src;
}

xreg::CameraModel xreg::DownsampleCameraModel(const CameraModel& src_cam, const CoordScalar ds_factor,
                                              const bool force_even_dims)
{
  CameraModel dst_cam;
  dst_cam.coord_frame_type = src_cam.coord_frame_type;

  auto intrins = src_cam.intrins;
  intrins(0,0) *= ds_factor;
  intrins(1,1) *= ds_factor;
  intrins(0,2) *= ds_factor;
  intrins(1,2) *= ds_factor;

  long num_ds_rows = std::lround(src_cam.num_det_rows * ds_factor);
  long num_ds_cols = std::lround(src_cam.num_det_cols * ds_factor);

  if (force_even_dims)
  {
    if (num_ds_rows % 2)
    {
      --num_ds_rows;
    }

    if (num_ds_cols % 2)
    {
      --num_ds_cols;
    }
  }

  dst_cam.setup(intrins, src_cam.extrins.matrix(),
                num_ds_rows, num_ds_cols,
                src_cam.det_row_spacing / ds_factor,
                src_cam.det_col_spacing / ds_factor);

  return dst_cam;
}

std::vector<xreg::CameraModel>
xreg::CreateCameraWorldUsingFiducial(const std::vector<CameraModel>& orig_cams,
                                     const std::vector<FrameTransform>& cams_to_fid)
{
  const size_type num_cams = orig_cams.size();

  std::vector<CameraModel> dst_cams(num_cams);

  for (size_type i = 0; i < num_cams; ++i)
  {
    const CameraModel& src_cam = orig_cams[i];
    CameraModel&       dst_cam = dst_cams[i];

    dst_cam.coord_frame_type = src_cam.coord_frame_type;
    dst_cam.setup(src_cam.intrins,
                  (src_cam.extrins * cams_to_fid[i].inverse()).matrix(),
                  src_cam.num_det_rows, src_cam.num_det_rows,
                  src_cam.det_row_spacing, src_cam.det_col_spacing);
  }

  return dst_cams;
}

void xreg::PrintCam(std::ostream& out, const CameraModel& cam)
{
  std::string coord_frame_str;
  switch (cam.coord_frame_type)
  {
  case CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z:
    coord_frame_str = "origin at pinhole, z axis towards detector";
    break;
  case CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z:
    coord_frame_str = "origin at pinhole, z axis away from detector";
    break;
  case CameraModel::kORIGIN_ON_DETECTOR:
    coord_frame_str = "origin on detector, z axis away from detector";
    break;
  default:
    coord_frame_str = "unknown";
    break;
  }

  out << fmt::sprintf("    num rows: %lu\n"
                      "    num cols: %lu\n"
                      " row spacing: %7.4f\n"
                      " col spacing: %7.4f\n"
                      "focal length: %7.4f\n"
                      "     pinhole: [%+12.4f , %+12.4f , %+12.4f]\n"
                      "coord. frame: %s\n",
                      cam.num_det_rows, cam.num_det_cols,
                      cam.det_row_spacing, cam.det_col_spacing,
                      cam.focal_len,
                      cam.pinhole_pt[0], cam.pinhole_pt[1], cam.pinhole_pt[2],
                      coord_frame_str);
  out << "intrins:\n" << cam.intrins
      << "\nextrins:\n" << cam.extrins.matrix()
      << std::endl;
}

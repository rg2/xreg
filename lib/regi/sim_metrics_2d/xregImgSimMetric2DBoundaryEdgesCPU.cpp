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

#include "xregImgSimMetric2DBoundaryEdgesCPU.h"

#include "xregOpenCVUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregTBBUtils.h"
#include "xregRayCastInterface.h"

void xreg::ImgSimMetric2DBoundaryEdgesCPU::allocate_resources()
{
  ImgSimMetric2DCPU::allocate_resources();

  cv::Mat fixed_img = ShallowCopyItkToOpenCV(this->fixed_img_.GetPointer());
  
  img_num_rows_ = fixed_img.rows;
  img_num_cols_ = fixed_img.cols;

  num_fixed_edges_ = 0;

  cv::Mat fixed_edges = ShallowCopyItkToOpenCV(fixed_img_edges_.GetPointer()).clone();
  
  // Invert, 0 -> edge, 1 -> no edge
  // TODO: parallelize
  for (size_type r = 0; r < img_num_rows_; ++r)
  {
    auto* edge_row = &fixed_edges.at<EdgePixelScalar>(r,0);

    for (size_type c = 0; c < img_num_cols_; ++c)
    {
      if (edge_row[c])
      {
        ++num_fixed_edges_;
      }
      
      edge_row[c] = !edge_row[c];
    }
  }
  
  fixed_edge_dist_map_ = cv::Mat::zeros(fixed_img.size(), cv::DataType<Scalar>::type);

  cv::distanceTransform(fixed_edges, fixed_edge_dist_map_, CV_DIST_L2, CV_DIST_MASK_PRECISE);
 
  // create storage for moving image edges 
  move_edge_imgs_ = AllocContiguousBufferForOpenCVImages<EdgePixelScalar>(img_num_rows_, img_num_cols_,
                                                                          this->num_mov_imgs_,
                                                                          &move_edges_buf_);
}

void xreg::ImgSimMetric2DBoundaryEdgesCPU::compute()
{
  this->pre_compute();

  auto compute_dists_fn = [&] (const RangeType& r)
  {
    const bool reg = this->regularize_;
    const Scalar num_edge_pts_low = 0.5 * this->num_fixed_edges_;

    const cv::Mat& fixed_dist_map = this->fixed_edge_dist_map_;

    const size_type nr = this->img_num_rows_;
    const size_type nc = this->img_num_cols_;

    const size_type num_pix_per_proj = nr * nc;

    for (size_type mov_idx = r.begin(); mov_idx < r.end(); ++mov_idx)
    {
      // compute the moving image edges
      cv::Mat& cur_mov_edges = this->move_edge_imgs_[mov_idx];
      
      cv::Mat cur_mov_depth(nr, nc, cv::DataType<Scalar>::type,
                            this->mov_imgs_buf_ + (mov_idx * num_pix_per_proj)); 

     
      FindPixelsWithAdjacentIntensity(cur_mov_depth, &cur_mov_edges, kRAY_CAST_MAX_DEPTH, true);

      // Now compute the mean distance
      Scalar d = 0;
      size_type num_pts = 0;
     
      for (size_type r = 0; r < nr; ++r)
      {
        const auto* cur_edge_row = &cur_mov_edges.at<EdgePixelScalar>(r,0);
        const auto* cur_dist_row = &fixed_dist_map.at<Scalar>(r,0);

        for (size_type c = 0; c < nc; ++c)
        {
          if (cur_edge_row[c])
          {
            ++num_pts;
            
            d += cur_dist_row[c];
          }
        }
      }

      d /= num_pts;

      // TODO: come up with better regularization
      if (reg)
      {
        if (num_pts < num_edge_pts_low)
        {
          d += 1000;
        }
      }    

      this->sim_vals_[mov_idx] = d;
    }
  };

  ParallelFor(compute_dists_fn, RangeType(0, this->num_mov_imgs_));
}

void xreg::ImgSimMetric2DBoundaryEdgesCPU::set_fixed_image_edges(FixedEdgeImagePtr fixed_edges)
{
  fixed_img_edges_ = fixed_edges;
}

void xreg::ImgSimMetric2DBoundaryEdgesCPU::set_regularize(const bool r)
{
  regularize_ = r;
}

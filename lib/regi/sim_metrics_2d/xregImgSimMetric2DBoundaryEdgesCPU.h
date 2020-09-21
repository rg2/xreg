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

#ifndef XREGIMGSIMMETRIC2DBOUNDARYEDGESCPU_H_
#define XREGIMGSIMMETRIC2DBOUNDARYEDGESCPU_H_

#include <opencv2/core/core.hpp>

#include "xregImgSimMetric2DCPU.h"

namespace xreg
{

/// \brief Edge based similarity metric on 2D images with hand-drawn boundary edges on the fixed images.
///
/// The moving image inputs should actually be depth images, from which this similarity metric will compute
/// the boundary edges.
class ImgSimMetric2DBoundaryEdgesCPU : public ImgSimMetric2DCPU
{
public:
  using EdgePixelScalar   = unsigned char;
  using FixedEdgeImage    = itk::Image<EdgePixelScalar,2>;
  using FixedEdgeImagePtr = FixedEdgeImage::Pointer;

  /// \brief Constructor - trivial, peforms no work.
  ImgSimMetric2DBoundaryEdgesCPU() = default;

  /// \brief Allocates resources, such as edge image buffers.
  ///
  /// This allocates,
  /// the fixed image edge distance map, a buffer for storing every moving image's
  /// edge image.
  /// This computes the fixed image distance map.
  void allocate_resources();

  /// \brief Computes the edge distance similarity values.
  ///
  /// This loops through each moving (depth) image, computes the edges,
  /// and then computes the distance value.
  void compute();

  /// \brief Sets the fixed image edge map.
  ///
  /// Sets the fixed image edges used to compute the distance map.
  void set_fixed_image_edges(FixedEdgeImagePtr fixed_edges);

  void set_regularize(const bool r);

private:
  using EdgeBuffer = std::vector<EdgePixelScalar>;
  using MatList    = std::vector<cv::Mat>;

  size_type img_num_rows_ = 0;
  size_type img_num_cols_ = 0;

  FixedEdgeImagePtr fixed_img_edges_;

  cv::Mat fixed_edge_dist_map_;

  EdgeBuffer move_edges_buf_;

  MatList move_edge_imgs_;

  size_type num_fixed_edges_ = 0;

  bool regularize_ = true;
};

}  // xreg

#endif


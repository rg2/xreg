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

#ifndef XREGLOCALCONTRASTNORM_H_
#define XREGLOCALCONTRASTNORM_H_

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

namespace xreg
{

// Implements local contrast normalization by replacing each pixel with the
// value it would take when remapping a local window about it to have zero
// mean and unit standard deviation. 
cv::Mat LocalContrastNormStdNorm(const cv::Mat& input_img, const int win_len_rows,
                                 const int win_len_cols,
                                 const boost::optional<float>& border_val =
                                   boost::optional<float>());

// Implements local contrast normalization according to:
// What is the Best Multi-Stage Architecture for Object Recognition?
// Jarrett, et al.
// Section 2: Model Architecture, Local Contrast Normalization Layer.
cv::Mat LocalContrastNormJarrett(const cv::Mat& input_img, const int win_len_rows,
                                 const int win_len_cols,
                                 const boost::optional<float>& border_val =
                                   boost::optional<float>());


}  // xreg

#endif


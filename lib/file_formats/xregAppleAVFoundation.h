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

#ifndef XREGAPPLEAVFOUNDATION_H_
#define XREGAPPLEAVFOUNDATION_H_

#include "xregWriteVideo.h"

namespace xreg
{

class WriteImageFramesToVideoAppleAVF : public WriteImageFramesToVideo
{
public:
  void open() override;

  void close() override;

  void write(const cv::Mat& frame) override;
  
  ~WriteImageFramesToVideoAppleAVF() override;

private:
  void* av_asset_writer_ = nullptr;
  
  void* av_asset_writer_input_ = nullptr;
  
  void* av_assest_writer_pix_buf_adaptor_ = nullptr;

  bool input_setup_ = false;
  
  int num_rows_ = 0;
  int num_cols_ = 0;

  int frame_type_ = 0;
  
  long frame_count_ = 0;
};

}  // xreg

#endif


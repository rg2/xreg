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

#include "xregITKLabelUtils.h"

#include "xregCommon.h"

namespace  // un-named
{

using namespace xreg;

template <class tPixelScalar, unsigned int tN, class tLabelScalar>
std::vector<typename itk::Image<tPixelScalar,tN>::Pointer>
MakeVolListFromVolAndLabelsHelper(const itk::Image<tPixelScalar,tN>* vol,
                                  const itk::Image<tLabelScalar,tN>* labels,
                                  const std::vector<tLabelScalar>& labels_to_use,
                                  const tPixelScalar masked_out_val)
{
  using Img     = itk::Image<tPixelScalar,tN>;
  using ImgPtr  = typename Img::Pointer;
  using ImgList = std::vector<ImgPtr>;
  
  const size_type num_vols = labels_to_use.size();
  
  ImgList vols;
  vols.reserve(num_vols);

  for (const auto& l : labels_to_use)
  {
    vols.push_back(ApplyMaskToITKImage(vol, labels, l, masked_out_val, true));
  }

  return vols;
}

}  // un-named

std::vector<itk::Image<unsigned char,3>::Pointer>
xreg::MakeVolListFromVolAndLabels(const itk::Image<unsigned char,3>* vol,
                                  const itk::Image<unsigned char,3>* labels,
                                  const std::vector<unsigned char>& labels_to_use,
                                  const unsigned char masked_out_val)
{
  return MakeVolListFromVolAndLabelsHelper(vol, labels, labels_to_use, masked_out_val);
}

std::vector<itk::Image<float,3>::Pointer>
xreg::MakeVolListFromVolAndLabels(const itk::Image<float,3>* vol,
                                  const itk::Image<unsigned char,3>* labels,
                                  const std::vector<unsigned char>& labels_to_use,
                                  const float masked_out_val)
{
  return MakeVolListFromVolAndLabelsHelper(vol, labels, labels_to_use, masked_out_val);
}



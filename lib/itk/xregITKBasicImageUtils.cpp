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

#include "xregITKBasicImageUtils.h"

itk::AffineTransform<xreg::CoordScalar,3>::Pointer
xreg::ConvertEigenAffineXformToITK(const FrameTransform& eigen_xform)
{
  using ITKAffineXform = itk::AffineTransform<CoordScalar,3>;

  auto itk_xform = ITKAffineXform::New();

  // TODO: handle the "fixed parameters" of the ITK transform - will need an optional image passed in?
  ITKAffineXform::FixedParametersType itk_fixed_params(3);
  itk_fixed_params.Fill(0);

  itk_xform->SetFixedParameters(itk_fixed_params);

  ITKAffineXform::ParametersType itk_params(12);

  // ITK parameters are stored in column major
  itk_params.SetElement(0,  eigen_xform.matrix()(0,0));
  itk_params.SetElement(1,  eigen_xform.matrix()(1,0));
  itk_params.SetElement(2,  eigen_xform.matrix()(2,0));
  itk_params.SetElement(3,  eigen_xform.matrix()(0,1));
  itk_params.SetElement(4,  eigen_xform.matrix()(1,1));
  itk_params.SetElement(5,  eigen_xform.matrix()(2,1));
  itk_params.SetElement(6,  eigen_xform.matrix()(0,2));
  itk_params.SetElement(7,  eigen_xform.matrix()(1,2));
  itk_params.SetElement(8,  eigen_xform.matrix()(2,2));
  itk_params.SetElement(9,  eigen_xform.matrix()(0,3));
  itk_params.SetElement(10, eigen_xform.matrix()(1,3));
  itk_params.SetElement(11, eigen_xform.matrix()(2,3));

  itk_xform->SetParameters(itk_params);

  return itk_xform;
}


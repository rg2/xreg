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

#include "xregITKIOUtils.h"

#include <itkAffineTransform.h>
#include <itkTransformFileReader.h>
#include <itkTransformFileWriter.h>

#include "xregExceptionUtils.h"

xreg::FrameTransform xreg::ReadSlicerAffineTransformFromFile(const std::string& path)
{
  return ReadITKAffineTransformFromFile(path).inverse();
}

namespace
{

using namespace xreg;

template <class U>
bool PopulateEigenXformFromITKAffine(
        itk::TransformFileReader::TransformType* itk_xform_base,
        FrameTransform* xform)
{
  using AffineXform = itk::AffineTransform<U,3>;

  bool success = false;

  AffineXform* itk_xform = dynamic_cast<AffineXform*>(itk_xform_base);

  if (itk_xform)
  {
    const typename AffineXform::MatrixType& A = itk_xform->GetMatrix();
    const typename AffineXform::OutputVectorType& t = itk_xform->GetOffset();

    xform->matrix()(0,0) = static_cast<CoordScalar>(A(0,0));
    xform->matrix()(1,0) = static_cast<CoordScalar>(A(1,0));
    xform->matrix()(2,0) = static_cast<CoordScalar>(A(2,0));
    xform->matrix()(3,0) = 0;

    xform->matrix()(0,1) = static_cast<CoordScalar>(A(0,1));
    xform->matrix()(1,1) = static_cast<CoordScalar>(A(1,1));
    xform->matrix()(2,1) = static_cast<CoordScalar>(A(2,1));
    xform->matrix()(3,1) = 0;

    xform->matrix()(0,2) = static_cast<CoordScalar>(A(0,2));
    xform->matrix()(1,2) = static_cast<CoordScalar>(A(1,2));
    xform->matrix()(2,2) = static_cast<CoordScalar>(A(2,2));
    xform->matrix()(3,2) = 0;

    xform->matrix()(0,3) = static_cast<CoordScalar>(t[0]);
    xform->matrix()(1,3) = static_cast<CoordScalar>(t[1]);
    xform->matrix()(2,3) = static_cast<CoordScalar>(t[2]);
    xform->matrix()(3,3) = 1;

    success = true;
  }

  return success;
}

template <class U>
void PopulateITKAffineXformFromEigen(const FrameTransform& eigen_xform,
                                     itk::AffineTransform<U,3>* itk_xform)
{
  itk_xform->SetIdentity();

  typename itk::AffineTransform<U,3>::MatrixType R;

  for (unsigned int r = 0; r < 3; ++r)
  {
    for (unsigned int c = 0; c < 3; ++c)
    {
      R(r,c) = static_cast<U>(eigen_xform.matrix()(r,c));
    }
  }

  itk_xform->SetMatrix(R);

  typename itk::AffineTransform<U,3>::OutputVectorType t;

  for (unsigned int i = 0; i < 3; ++i)
  {
    t[i] = static_cast<U>(eigen_xform.matrix()(i,3));
  }

  itk_xform->SetOffset(t);
}

template <class U>
bool PopulateEigenXformFromITKMatrixOffset(
            itk::TransformFileReader::TransformType* itk_xform_base,
            FrameTransform* xform)
{
  using MatrixOffsetXform = itk::MatrixOffsetTransformBase<U,3,3>;

  bool success = false;

  MatrixOffsetXform* itk_xform = dynamic_cast<MatrixOffsetXform*>(itk_xform_base);

  if (itk_xform)
  {
    const typename MatrixOffsetXform::MatrixType& A = itk_xform->GetMatrix();
    const typename MatrixOffsetXform::OutputVectorType& t = itk_xform->GetOffset();

    xform->matrix()(0,0) = static_cast<CoordScalar>(A(0,0));
    xform->matrix()(1,0) = static_cast<CoordScalar>(A(1,0));
    xform->matrix()(2,0) = static_cast<CoordScalar>(A(2,0));
    xform->matrix()(3,0) = 0;

    xform->matrix()(0,1) = static_cast<CoordScalar>(A(0,1));
    xform->matrix()(1,1) = static_cast<CoordScalar>(A(1,1));
    xform->matrix()(2,1) = static_cast<CoordScalar>(A(2,1));
    xform->matrix()(3,1) = 0;

    xform->matrix()(0,2) = static_cast<CoordScalar>(A(0,2));
    xform->matrix()(1,2) = static_cast<CoordScalar>(A(1,2));
    xform->matrix()(2,2) = static_cast<CoordScalar>(A(2,2));
    xform->matrix()(3,2) = 0;

    xform->matrix()(0,3) = static_cast<CoordScalar>(t[0]);
    xform->matrix()(1,3) = static_cast<CoordScalar>(t[1]);
    xform->matrix()(2,3) = static_cast<CoordScalar>(t[2]);
    xform->matrix()(3,3) = 1;

    success = true;
  }

  return success;
}

}  // un-named

xreg::FrameTransform xreg::ReadITKAffineTransformFromFile(const std::string& path)
{
  FrameTransform xform = FrameTransform::Identity();

  itk::TransformFileReader::Pointer reader = itk::TransformFileReader::New();

  reader->SetFileName(path);
  reader->Update();

  const itk::TransformFileReader::TransformListType* xforms_list = reader->GetTransformList();

  if (xforms_list && !xforms_list->empty())
  {
    if (xforms_list->size() == 1)
    {
      itk::TransformFileReader::TransformType* itk_xform = xforms_list->front();

      if (itk_xform)
      {
        if (itk_xform->GetTransformCategory() == itk::TransformFileReader::TransformType::Linear)
        {
          if (itk_xform->GetTransformTypeAsString().find("CompositeTransform") == std::string::npos)
          {
            if (!PopulateEigenXformFromITKAffine<double>(itk_xform, &xform))
            {
              std::cout << "not affine double!" << std::endl;
              if (!PopulateEigenXformFromITKAffine<float>(itk_xform, &xform))
              {
                std::cout << "not affine float!" << std::endl;
                if (!PopulateEigenXformFromITKMatrixOffset<double>(itk_xform, &xform))
                {
                  std::cout << "not mat off double!" << std::endl;
                  if (!PopulateEigenXformFromITKMatrixOffset<float>(itk_xform, &xform))
                  {
                    std::cout << "not mat off float!" << std::endl;
                    xregThrow("Unsupported ITK transform class!");
                  }
                }
              }
            }
          }
          else
          {
            xregThrow("Composite transforms are not supported!");
          }
        }
        else
        {
          xregThrow("Not a linear transform!");
        }
      }
      else
      {
        xregThrow("Transform not found in file!");
      }
    }
    else
    {
      xregThrow("Transform file with multiple transforms is not supported!");
    }
  }
  else
  {
    xregThrow("Transform file is empty!");
  }
  
  return xform;
}

void xreg::WriteITKAffineTransform(const std::string& path, const FrameTransform& xform)
{
  using ITKAffineXform = itk::AffineTransform<CoordScalar,3>;

  auto itk_xform = ITKAffineXform::New();

  PopulateITKAffineXformFromEigen(xform, itk_xform.GetPointer());

  using TransformWriter = itk::TransformFileWriterTemplate<CoordScalar>;
  auto writer = TransformWriter::New();

  writer->SetInput(itk_xform);
  writer->SetFileName(path);
  writer->Update();
}


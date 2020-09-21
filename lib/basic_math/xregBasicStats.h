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

#ifndef XREGBASICSTATS_H_
#define XREGBASICSTATS_H_

#include "xregCommon.h"

namespace xreg
{

CoordScalar SampleMean(const CoordScalarList& x);

CoordScalar SampleStdDev(const CoordScalarList& x);

// Computes the sample standard deviation, but saves some computation by using
// a previously computed value for the sample mean
CoordScalar SampleStdDev(const CoordScalarList& x, const CoordScalar sample_mean);

// Computes the sample mean and sample standard deviation, returned as a tuple (mean, std. dev.)
std::tuple<CoordScalar,CoordScalar> SampleMeanAndStdDev(const CoordScalarList& x);

template <class tDerived>
typename tDerived::Scalar
SampleMean(const Eigen::ArrayBase<tDerived>& a)
{
  return a.mean();
}

template <class tDerived>
typename tDerived::Scalar
SampleStdDev(const Eigen::ArrayBase<tDerived>& a, const typename tDerived::Scalar& a_mean)
{
  return std::sqrt((a - a_mean).square().sum() / ((a.rows() * a.cols()) - 1));
}

template <class tDerived>
typename tDerived::Scalar
SampleStdDev(const Eigen::ArrayBase<tDerived>& a)
{
  return SampleStdDev(a, SampleMean(a));
}

template <class tDerived>
typename tDerived::Scalar
SampleMean(const Eigen::MatrixBase<tDerived>& m)
{
  return SampleMean(m.array());
}

template <class tDerived>
typename tDerived::Scalar
SampleStdDev(const Eigen::MatrixBase<tDerived>& m, const typename tDerived::Scalar& m_mean)
{
  return SampleStdDev(m.array(), m_mean);
}

template <class tDerived>
typename tDerived::Scalar
SampleStdDev(const Eigen::MatrixBase<tDerived>& m)
{
  auto a = m.array();

  return SampleStdDev(a, SampleMean(a));
}

}  // xreg

#endif


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

#ifndef XREGTYPEUTILS_H_
#define XREGTYPEUTILS_H_

#include <Eigen/Eigen>

namespace xreg
{

/**
 * @brief Default scalar type for a vector type.
 *
 * This may be used to retrieve the element/component/scalar type
 * for a vector, when only a generic type is used. The default
 * behavior is to use the value_type type in the vector class.
 **/
template <class V>
struct ScalarType
{
  typedef typename V::value_type type;
};

/// \brief Scalar type for a C-style array
template <class T>
struct ScalarType<T*>
{
  typedef T type;
};

#define XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(T) \
template <> \
struct ScalarType<T> \
{ \
  using type = T; \
};

XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(unsigned char)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(unsigned short)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(unsigned int)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(unsigned long)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(char)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(short)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(int)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(long)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(float)
XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER(double)

#undef XREG_MAKE_SCALAR_TYPE_FOR_SCALAR_HELPER

/**
 * @brief Scalar type for an Eigen Matrix/Vector type.
 **/
template <class T, int rows, int cols, int opts, int maxrows, int maxcols>
struct ScalarType<Eigen::Matrix<T,rows,cols,opts,maxrows,maxcols> >
{
  typedef T type;
};

/**
 * @brief Scalar type for an Eigen MatrixBase type.
 **/
template <class Derived>
struct ScalarType<Eigen::MatrixBase<Derived> >
{
  typedef typename Eigen::MatrixBase<Derived>::Scalar type;
};

/**
 * @brief Scalar type for an Eigen Product type.
 **/
template <class LHS, class RHS, int flags>
struct ScalarType<Eigen::Product<LHS,RHS,flags> >
{
  typedef typename Eigen::Product<LHS,RHS,flags>::Scalar type;
};

/**
 * @brief Scalar type for an Eigen Transform type.
 **/
template <class Scalar, int Dim, int Mode, int _Options>
struct ScalarType<Eigen::Transform<Scalar,Dim,Mode,_Options> >
{
  typedef Scalar type;
};

/**
 * @brief Scalar type for an Eigen Array type.
 **/
template <class T, int rows, int cols, int opts, int maxrows, int maxcols>
struct ScalarType<Eigen::Array<T,rows,cols,opts,maxrows,maxcols> >
{
  typedef T type;
};

/**
 * @brief Scalar type for an Eigen ArrayBase type.
 **/
template <class Derived>
struct ScalarType<Eigen::ArrayBase<Derived> >
{
  typedef typename Eigen::ArrayBase<Derived>::Scalar type;
};

/**
 * @brief Scalar type for an Eigen ArrayWrapper type.
 **/
template <class tDerived>
struct ScalarType<Eigen::ArrayWrapper<tDerived> >
{
  typedef typename tDerived::Scalar type;
};

/// \brief Scalar type for an Eigen::CwiseUnaryView
template <class tViewOp, class tMatrixType>
struct ScalarType<Eigen::CwiseUnaryView<tViewOp, tMatrixType> >
{
  // It's actually possible that the matrix type has a different scalar type
  // then the view: e.g. a matrix of complex numbers, but a column of the real
  // real components
  typedef Eigen::CwiseUnaryView<tViewOp, tMatrixType> ColType;
  typedef typename ColType::Scalar type;
};

/// \brief Scalar type for an Eigen::Map
template <class tMatrixType>
struct ScalarType<Eigen::Map<tMatrixType> >
{
  typedef typename tMatrixType::Scalar type;
};

/// \brief Scalar type for an Eigen::Block
template <class tMatrixType>
struct ScalarType<Eigen::Block<tMatrixType> >
{
  typedef typename tMatrixType::Scalar type;
};

/// \brief Value/element type of a list/vector/container
template <class L>
struct ValueType
{
  typedef typename ScalarType<L>::type type;
};

/**
 * @brief Used as a decorator for helper functions to handle different
 *        incompatble types at compile time.
 **/
template <bool b>
struct BoolTypeHelper { };

}  // xreg

#endif

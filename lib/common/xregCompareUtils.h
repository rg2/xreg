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

#ifndef XREGCOMPAREUTILS_H_
#define XREGCOMPAREUTILS_H_

#include <array>
#include <functional>
#include <algorithm>

#include <boost/functional/hash.hpp>

#include "xregCommon.h"

namespace xreg
{

/// \brief Comparator for two N-dimensional fixed size vectors.
///
/// Each vector is treated as a sorted set, so that comparisons are always consistent.
/// The comparator operator (operator ()) performs the sorting out of place,
/// so the user does not need to perform any pre/post processing
template <class T, size_type tN, class tCmp = std::less<T>>
struct OrderIndepArrayCmp
{
  enum { kDIM = tN };

  using Scalar = T;
  using Array  = std::array<Scalar,kDIM>;
  using Cmp    = tCmp;
  
  Cmp comp;

  OrderIndepArrayCmp(const Cmp& c = Cmp())
    : comp(c)
  { }

  bool operator()(const Array& x, const Array& y) const
  {
		// NOTE:
		// very inefficient, but that is not a concern for the applications of finding duplicate triangles.
		// I want to be able to create orderings when I know the entire tetrahedron;
		// this allows me to order the vertex indices so that I have a normal vector
		// pointing "inside." However, for building up my histogram, etc. it could
		// happen that the orderings for the same triangle are different for
		// shared faces. 
    
		// These cannot be member variables, as we could have comparison object references shared amongst several algorithms
    std::array<Scalar,kDIM> tmp_vals1 = x;
    std::array<Scalar,kDIM> tmp_vals2 = y;

    std::sort(tmp_vals1.begin(), tmp_vals1.end(), comp);
    std::sort(tmp_vals2.begin(), tmp_vals2.end(), comp);

    bool to_ret = false;

    for (size_type i = 0; i < kDIM; ++i)
    {
      if (comp(tmp_vals1[i],tmp_vals2[i]))
      {
        // this component in the first set of values is strictly less than the
        // corresponding component in the second set of values
        to_ret = true;
        break;
      }
      else if (comp(tmp_vals2[i],tmp_vals1[i]))
      {
        // this component in the second set of values is strictly less than the
        // corresponding component in the first set of values, this implies
        // they are not equal and the second set of values should be considered
        // greater than the first set -> return false
        break;
      }
    }

    return to_ret;
	}
};

/// \brief Equality comparator for two N-dimensional fixed size vectors.
///
/// This is used instead of running OrderIndepFixedVectorCmp::operator(x,y) and
/// OrderIndepFixedVectorCmp::operator(y,x) so that we only sort the vectors once.
///
/// Each vector is treated as a sorted set, so that comparisons are always consistent.
/// The comparator operator (operator ()) performs the sorting out of place,
/// so the user does not need to perform any pre/post processing
template <class T, size_type tN, class tCmp = std::less<T>>
struct OrderIndepArrayEqualTo
{
  enum { kDIM = tN };

  using Scalar = T;
  using Array  = std::array<Scalar,kDIM>;
  using Cmp    = tCmp;
  
  Cmp comp;

  OrderIndepArrayEqualTo(const Cmp& c = Cmp())
    : comp(c)
  { }

  bool operator()(const Array& x, const Array& y) const
  {
		// NOTE: see above for note about efficiency and not using tmp_valsX as members

    std::array<Scalar,kDIM> tmp_vals1 = x;
    std::array<Scalar,kDIM> tmp_vals2 = y;

    std::sort(tmp_vals1.begin(), tmp_vals1.end(), comp);
    std::sort(tmp_vals2.begin(), tmp_vals2.end(), comp);

		bool to_ret = true;

		for (size_type i = 0; i < kDIM; ++i)
		{
			if (comp(tmp_vals1[i],tmp_vals2[i]) || comp(tmp_vals2[i], tmp_vals1[i]))
			{
				// this component in the first set of values is not equal to the
				// corresponding component in the second set of values
				to_ret = false;
				break;
			}
		}

		return to_ret;
  }
};

/// \brief Hash computation for an order independent fixed size vector.
///
/// The vector is treated as a sorted set, so that hashes are always consistent.
/// The comparator operator (operator ()) performs the sorting out of place,
/// so the user does not need to perform any pre/post processing
template <class T, size_type tN, class tCmp = std::less<T>>
struct OrderIndepArrayHash
{
  enum { kDIM = tN };

  using Scalar = T;
  using Array  = std::array<Scalar,kDIM>;
  using Cmp    = tCmp;
  
  using hash_value_type = size_type;

  Cmp comp;

  OrderIndepArrayHash(const Cmp& c = Cmp())
    : comp(c)
  { }
  
  hash_value_type operator()(const Array& x) const
  {
    // NOTE: see above for note about efficiency and not using tmp_vals as a member

    // derived from: http://www.boost.org/doc/libs/1_59_0/doc/html/hash/combine.html

    std::array<Scalar,kDIM> tmp_vals = x;

    std::sort(tmp_vals.begin(), tmp_vals.end(), comp);

    return boost::hash_range(tmp_vals.begin(), tmp_vals.end());
  }
};

template <class tPt>
struct FixedPtInfo;

template <class tScalar, int rows, int cols, int opts, int maxrows, int maxcols>
struct FixedPtInfo<Eigen::Matrix<tScalar,rows,cols,opts,maxrows,maxcols>>
{
  enum { is_fixed = (rows != Eigen::Dynamic) && (cols != Eigen::Dynamic) };
  
  enum { fixed_length = is_fixed ? (rows * cols) : 0 };
  
  using Scalar = tScalar;
};

template <class tPt>
struct FixedPtHash
{
  using Pt              = tPt;
  using hash_value_type = size_type;

  enum { kDIM = FixedPtInfo<Pt>::fixed_length };

  static_assert(FixedPtInfo<Pt>::is_fixed, "Point dimensions are not fixed/static!");
  
  hash_value_type operator()(const Pt& x) const
  {
    hash_value_type seed = 0;

    // using a for loop here, instead of hash_range, due to not knowing the stride of the data
    for (size_type i = 0; i < kDIM; ++i)
    {
      boost::hash_combine(seed, x[i]);
    }

    return seed;
  }
};

template <class tPt>
struct PtEuclideanNormEqualTo
{
  using Pt = tPt;

  bool operator()(const Pt& x, const Pt& y) const
  {
    return (x - y).norm() < 1.0e-8;
  }
};

}  // xreg

#endif


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

/**
 * @file
 * @brief Contains basic utilities for using Intel's Threading Building Blocks,
 *        while remaining compatible when executing in a single thread without them.
 **/

#ifndef XREGTBBUTILS_H_
#define XREGTBBUTILS_H_

#include <cstddef>  // size_t
#include <iterator>

// define this to switch from the parallel TBB calls
// to serial standard library calls
//#define XREG_NO_TBB

#ifndef XREG_NO_TBB
#include <tbb/tbb.h>
#endif

#include "xregAssert.h"

// Wrap an argument name with this when it is only used when TBB has been found
// This avoids the unused argument compiler warning
#ifndef XREG_NO_TBB
#define XREG_TBB_ARG(x) x
#else
#define XREG_TBB_ARG(x)
#endif

namespace xreg
{

#ifndef XREG_NO_TBB
using RangeType = tbb::blocked_range<size_t>;
#else

/**
 * @brief Range type representing the start and end of a range of elements.
 *
 * The start/begin index is inclusive, while the stop/end index is NOT
 * (similar to begin/end iterators). This should have a similar interface to
 * the blocked_range type in TBB.
 **/
struct RangeType
{
  using size_type = std::size_t;

  size_type begin_;
  size_type end_;

  RangeType(const size_type b, const size_type e)
    : begin_(b), end_(e)
  { }

  size_type begin() const { return begin_; }

  size_type end() const { return end_; }
};
#endif

#ifndef XREG_NO_TBB
#define xregSplitMarker tbb::split
#else
struct SplitMarkerType { };
#define xregSplitMarker xreg::SplitMarkerType
#endif

template <class _fn>
void ParallelFor(_fn& fn_obj, const RangeType& r)
{
#ifndef XREG_NO_TBB
  tbb::parallel_for(r, fn_obj);
#else
  fn_obj(r);
#endif
}

template <class _value, class _fn, class _red>
_value ParallelReduce(const _value& id_val, _fn& fn_obj, _red XREG_TBB_ARG(red_obj), const RangeType& r)
{
#ifndef XREG_NO_TBB
  return tbb::parallel_reduce(r, id_val, fn_obj, red_obj);
#else
  return fn_obj(r, id_val);
#endif
}

template <class _fn>
void ParallelReduce(_fn& fn_obj, const RangeType& r)
{
#ifndef XREG_NO_TBB
  tbb::parallel_reduce(r, fn_obj);
#else
  fn_obj(r);
#endif
}

namespace detail
{

template <class T>
struct IsRandomAccessIterator
{
  enum { value = 0 };
};

template <>
struct IsRandomAccessIterator<std::random_access_iterator_tag>
{
  enum { value = 1 };
};

template <class InputIt, class OutputIt, class UnaryOp>
struct ParallelTransformRangeFn
{
  InputIt begin_in;
  OutputIt begin_out;
  UnaryOp op;

  void operator()(const RangeType& r) const
  {
    InputIt cur_begin_in = begin_in + r.begin();
    InputIt cur_end_in   = begin_in + r.end();

    OutputIt cur_out = begin_out + r.begin();

    for (InputIt cur_in = cur_begin_in; cur_in != cur_end_in; ++cur_in, ++cur_out)
    {
      *cur_out = op(*cur_in);
    }
  }
};

}  // detail

template <class InputIt, class OutputIt, class UnaryOp>
OutputIt ParallelTransform(InputIt begin_in, InputIt end_in, OutputIt begin_out, UnaryOp op)
{
#ifndef XREG_NO_TBB
  using InputItCat = typename std::iterator_traits<InputIt>::iterator_category;
  using OutputItCat = typename std::iterator_traits<OutputIt>::iterator_category;

  static_assert(detail::IsRandomAccessIterator<InputItCat>::value,
                "input iterator must be random access");
  static_assert(detail::IsRandomAccessIterator<OutputItCat>::value,
                "output iterator must be random access");

  detail::ParallelTransformRangeFn<InputIt, OutputIt, UnaryOp> xform_fn = { begin_in, begin_out, op };
  const RangeType::size_type len = std::distance(begin_in, end_in);
  ParallelFor(xform_fn, RangeType(0, len));
  return begin_out + len;
#else
  return std::transform(begin_in, end_in, begin_out, op);
#endif
}

}  // xreg

#endif

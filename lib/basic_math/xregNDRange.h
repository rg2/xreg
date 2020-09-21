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

#ifndef XREGNDRANGE_H_
#define XREGNDRANGE_H_

#include "xregCommon.h"

namespace xreg
{

struct ConstSpacedRange
{
  using Scalar     = CoordScalar;
  using ScalarList = CoordScalarList;

  Scalar start;
  Scalar stop;
  Scalar inc;

  size_type size() const;

  ScalarList vals() const;

  Scalar operator[](const size_type i) const;
};

class ConstSpacedMeshGrid
{
public:
  using Scalar      = ConstSpacedRange::Scalar;
  using Range1D     = ConstSpacedRange;
  using Range1DList = std::vector<Range1D>;
  using Index       = std::vector<size_type>;
  using PtND        = Eigen::Matrix<Scalar,Eigen::Dynamic,1>;

  struct Iterator
  {
    const ConstSpacedMeshGrid* mesh_grid;
    Index cur_ind;

    PtND cur_pt;
    
    const PtND* operator->() const;

    const PtND& operator*() const;

    // prefix
    Iterator& operator++();

    // prefix
    Iterator& operator--();

    // postfix
    Iterator operator++(int);

    // postfix
    Iterator operator--(int);

    bool operator==(const Iterator& it) const;
    
    bool operator!=(const Iterator& it) const;
  };
  
  using iterator       = Iterator;
  using const_iterator = Iterator;

  ConstSpacedMeshGrid() = default;

  explicit
  ConstSpacedMeshGrid(const Range1DList& ranges, const bool row_major = true);

  size_type num_dims() const;

  size_type size() const;

  const Range1D& range(const size_type dim) const;

  Index basic_strides() const;

  PtND operator()(const Index& ind) const;

  const_iterator cbegin() const;

  const_iterator cend() const;

  const_iterator begin() const;

  const_iterator end() const;

  iterator begin();

  iterator end();

private:

  Range1DList ranges_;
  
  Index dims_;

  size_type num_dims_ = 0;

  bool row_major_ = true;
};

}  // xreg

#endif


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

#include "xregNDRange.h"

#include "xregAssert.h"

xreg::size_type xreg::ConstSpacedRange::size() const
{
  size_type sz = 0;
  
  if (std::abs(inc) > 1.0e-12)
  {
    const Scalar signed_dist = stop - start;
    const Scalar num_steps   = signed_dist / inc;

    if (num_steps > -1.0e-12)
    {
      sz = static_cast<size_type>(num_steps) + 1;
    }
    // else the increment will move away from the stop val,
    // and the range is empty
  }
  else if (std::abs(start - stop) < 1.0e-12)
  {
    sz = 1;
  }
  
  return sz;
}

xreg::ConstSpacedRange::ScalarList
xreg::ConstSpacedRange::vals() const
{
  ScalarList v;
  
  const size_type num_elems = size();

  if (num_elems)
  {
    v.reserve(num_elems);

    Scalar cur_val = start;

    for (size_type i = 0; i < num_elems; ++i, cur_val += inc)
    {
      v.push_back(cur_val);
    }
  }

  return v;
}

xreg::ConstSpacedRange::Scalar
xreg::ConstSpacedRange::operator[](const size_type i) const
{
  const size_type num_elems = size();
  xregASSERT(i < num_elems);
  
  return start + (i * inc);
}
  
xreg::ConstSpacedMeshGrid::ConstSpacedMeshGrid(const Range1DList& ranges, const bool row_major)
  : ranges_(ranges), num_dims_(ranges.size()), row_major_(row_major)
{
  dims_.reserve(num_dims_);
  for (const auto& r : ranges_)
  {
    dims_.push_back(r.size());
  }
}

xreg::size_type xreg::ConstSpacedMeshGrid::num_dims() const
{
  return num_dims_;
}

xreg::size_type xreg::ConstSpacedMeshGrid::size() const
{
  size_type tot_size = 0;

  if (num_dims_)
  {
    tot_size = 1;

    for (const auto& d : dims_)
    {
      tot_size *= d;
    }
  }

  return tot_size;
}

const xreg::ConstSpacedMeshGrid::Range1D&
xreg::ConstSpacedMeshGrid::range(const size_type dim) const
{
  xregASSERT(dim < num_dims_);
  return ranges_[dim];
}

xreg::ConstSpacedMeshGrid::Index
xreg::ConstSpacedMeshGrid::basic_strides() const
{
  Index s(num_dims_, 1);

  for (size_type d = 1; d < num_dims_; ++d)
  {
    const size_type cur_dim  = row_major_ ? (num_dims_ - 1 - d) : d;
    const size_type prev_dim = row_major_ ? (cur_dim + 1) : (cur_dim - 1);

    s[cur_dim] = s[prev_dim] * range(prev_dim).size();
  }

  return s;
}

xreg::ConstSpacedMeshGrid::PtND
xreg::ConstSpacedMeshGrid::operator()(const Index& ind) const
{
  xregASSERT(ind.size() == num_dims_);

  PtND p(num_dims_);

  for (size_type i = 0; i < num_dims_; ++i)
  {
    p[i] = ranges_[i][ind[i]];
  }

  return p;
}
  
xreg::ConstSpacedMeshGrid::const_iterator
xreg::ConstSpacedMeshGrid::cbegin() const
{
  Iterator it = { this };
  
  if (num_dims_)
  {
    it.cur_ind.assign(num_dims_, 0);
    it.cur_pt = operator()(it.cur_ind);
  }

  return it;
}

xreg::ConstSpacedMeshGrid::const_iterator
xreg::ConstSpacedMeshGrid::cend() const
{
  Iterator it = { this };

  if (num_dims_)
  {
    it.cur_ind.assign(num_dims_, 0);
    
    // set the index to all zeros, except overflowing on the most significant dimension
    if (row_major_)
    {
      it.cur_ind[0] = dims_[0];
    }
    else
    {
      it.cur_ind[num_dims_ - 1] = dims_[num_dims_ - 1];
    }

    // We do not need to set the cur_pt - when a -- operator is applied to the end iterator,
    // the point will be reset
  }

  return it;
}

xreg::ConstSpacedMeshGrid::const_iterator
xreg::ConstSpacedMeshGrid::begin() const
{
  return cbegin();
}

xreg::ConstSpacedMeshGrid::const_iterator
xreg::ConstSpacedMeshGrid::end() const
{
  return cend();
}

xreg::ConstSpacedMeshGrid::iterator
xreg::ConstSpacedMeshGrid::begin()
{
  return cbegin();
}

xreg::ConstSpacedMeshGrid::iterator
xreg::ConstSpacedMeshGrid::end()
{
  return cend();
}
    
const xreg::ConstSpacedMeshGrid::PtND*
xreg::ConstSpacedMeshGrid::Iterator::operator->() const
{
  return &cur_pt;
}

const xreg::ConstSpacedMeshGrid::PtND&
xreg::ConstSpacedMeshGrid::Iterator::operator*() const
{
  return cur_pt;
}

// prefix
xreg::ConstSpacedMeshGrid::Iterator&
xreg::ConstSpacedMeshGrid::Iterator::operator++()
{
  const size_type nd_minus_1 = mesh_grid->num_dims_ - 1;
  
  for (size_type i = 0; i < mesh_grid->num_dims_; ++i)
  {
    const size_type cur_dim = mesh_grid->row_major_ ? (nd_minus_1 - i) : i;

    size_type& ci = cur_ind[cur_dim];
    ++ci;

    if (ci < mesh_grid->dims_[cur_dim])
    {
      // still in bounds on this dimension
      // we do not need to increment any of the other dims
      cur_pt[cur_dim] = mesh_grid->ranges_[cur_dim][ci];
      break;
    }
    else
    {
      // we've wrapped around
      
      if (i != nd_minus_1)
      {
        // still have extra dimensions to go, reset this index
        ci = 0;
        cur_pt[cur_dim] = mesh_grid->ranges_[cur_dim][0];
      }
      else
      {
        // this is the last dimension - leave the index at out of bounds value
        // to indicate the end, but do not update the point
      }
    }
  }

  return *this;
}

// prefix
xreg::ConstSpacedMeshGrid::Iterator&
xreg::ConstSpacedMeshGrid::Iterator::operator--()
{
  const size_type nd_minus_1 = mesh_grid->num_dims_ - 1;

  // check for end
  if ((mesh_grid->row_major_  && (cur_ind[0] == mesh_grid->dims_[0])) ||
      (!mesh_grid->row_major_ && (cur_ind[nd_minus_1] == mesh_grid->dims_[nd_minus_1])))
  {
    cur_ind = mesh_grid->dims_;
    for (auto& ci : cur_ind)
    {
      --ci;
    }
    cur_pt = mesh_grid->operator()(cur_ind);
  }
  else
  {
    for (size_type i = 0; i < nd_minus_1; ++i)
    {
      const size_type cur_dim = mesh_grid->row_major_ ? nd_minus_1 - i : i;
      
      auto& ci = cur_ind[cur_dim];

      if (ci)
      {
        // still in bounds on this dimension
        // we will not need to decrement any of the other dims
        
        --ci;

        cur_pt[cur_dim] = mesh_grid->ranges_[cur_dim][ci];
        break;
      }
      else
      {
        // we're at zero, so wrap around in this dimension, and keep iterating
        // on the remaining dimensions
        ci = mesh_grid->dims_[cur_dim] - 1;
        cur_pt[cur_dim] = mesh_grid->ranges_[cur_dim][ci];
      }
    }
  }

  return *this;
}

// postfix
xreg::ConstSpacedMeshGrid::Iterator
xreg::ConstSpacedMeshGrid::Iterator::operator++(int)
{
  Iterator it(*this);

  operator++();

  return it;
}

// postfix
xreg::ConstSpacedMeshGrid::Iterator
xreg::ConstSpacedMeshGrid::Iterator::operator--(int)
{
  Iterator it(*this);

  operator--();

  return it;
}

bool xreg::ConstSpacedMeshGrid::Iterator::operator==(const Iterator& it) const
{
  bool eq = mesh_grid == it.mesh_grid;

  if (eq)
  {
    const size_type num_dims = mesh_grid->num_dims();

    for (size_type i = 0; eq && (i < num_dims); ++i)
    {
      eq = cur_ind[i] == it.cur_ind[i];
    }
  }

  return eq;
}

bool xreg::ConstSpacedMeshGrid::Iterator::operator!=(const Iterator& it) const
{
  return !(*this == it);
}

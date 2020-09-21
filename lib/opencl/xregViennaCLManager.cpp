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

#include "xregViennaCLManager.h"

#include <tuple>

#include <boost/container/flat_set.hpp>

#include <viennacl/ocl/backend.hpp>

void xreg::setup_vcl_ctx_if_needed(const long vcl_ctx,
                                   const boost::compute::context& ctx,
                                   const boost::compute::command_queue& queue)
{
  using Key = std::tuple<long,cl_context,cl_command_queue>;

  static boost::container::flat_set<Key> init_set;
  
  Key k(vcl_ctx,ctx.get(),queue.get());

  if (init_set.find(k) == init_set.end())
  {
    viennacl::ocl::setup_context(vcl_ctx, ctx, ctx.get_device().id(), queue);
    init_set.insert(k);
  }
}


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

#ifndef XREGOBJECTUTILS_H_
#define XREGOBJECTUTILS_H_

#include <cstring>      // memcpy
#include <type_traits>  // is_pod

#include "xregTypeUtils.h"

namespace xreg
{

namespace detail
{

template <class T>
void CopyObjectArrayTypeHelper(T* dst, const T* src, const std::size_t num_elems,
                               const BoolTypeHelper<true>)
{
  memcpy(dst, src, sizeof(T) * num_elems);
}

template <class T, class U>
void CopyObjectArrayTypeHelper(U* dst, const T* src, const std::size_t num_elems,
                               const BoolTypeHelper<false>)
{
  U* dst_it = dst;
  const T* src_it = src;
  for (; src_it != (src + num_elems); ++src_it, ++dst_it)
  {
    *dst_it = static_cast<U>(*src_it);
  }
}

}  // detail


/**
 * @brief Copies an array of objects from a memory buffer into a memory buffer
 * of a <b>different</b> type.
 *
 * This is accomplished via the assignment operator (=).
 * @param dst Destination memory buffer
 * @param src Source memory buffer
 * @param num_elems The number of elements to copy from the start of the source
 *                  memory buffer
 */
template <class T, class U>
void CopyObjectArray(U* dst, const T* src, const std::size_t num_elems)
{
  detail::CopyObjectArrayTypeHelper(dst, src, num_elems, BoolTypeHelper<false>());
}

/**
 * @brief Copies an array of objects from one memory buffer to another.
 *
 * If the object type is a primitive type then this call degenerates to a
 * memcpy, otherwise the assignment operator (=) is used in a standard for loop.
 * @param dst Destination memory buffer
 * @param src Source memory buffer
 * @param num_elems The number of elements to copy from the start of the source
 *                  memory buffer
 */
template <class T>
void CopyObjectArray(T* dst, const T* src, const std::size_t num_elems)
{
  detail::CopyObjectArrayTypeHelper(dst, src, num_elems,
                                    BoolTypeHelper<std::is_pod<T>::value>());
}

}  // xreg

#endif

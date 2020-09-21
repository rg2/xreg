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

#ifndef XREGRAYCASTSYNCBUF_H_
#define XREGRAYCASTSYNCBUF_H_

#include <boost/compute/container/vector.hpp>

#include "xregCommon.h"

namespace xreg
{

class RayCastSyncBuf
{
public:
  using BufElem = RayCastPixelScalar;
  
  using HostVec = std::vector<BufElem>;
  
  using OCLBuf   = boost::compute::vector<BufElem>;
  using OCLQueue = boost::compute::command_queue;
  
  struct HostBuf
  {
    BufElem*  buf = nullptr;
    size_type len = 0;

    HostBuf() = default;

    HostBuf(BufElem* b, const size_type l);
  };

  RayCastSyncBuf() = default;

  RayCastSyncBuf(const RayCastSyncBuf&) = delete;
  RayCastSyncBuf& operator=(const RayCastSyncBuf&) = delete;

  virtual ~RayCastSyncBuf() = default;

  void set_modified();

  virtual void sync() = 0;

  virtual void alloc() = 0;

  void set_range(const size_type start, const size_type end);

protected:
  static constexpr size_type kRANGE_AT_BUF_END = ~size_type(0);

  size_type range_start_ = 0;
  size_type range_end_   = kRANGE_AT_BUF_END;

  bool modified_ = false;
};

/// Base class for synchronizing data onto the host
class RayCastSyncHostBuf : public RayCastSyncBuf
{
public:
  virtual HostBuf& host_buf() = 0;
};

/// This should be created by an object working with data on the host and a
/// reference/pointer will be passed to an object that also works with data
/// on the host.
class RayCastSyncHostBufFromHost : public RayCastSyncHostBuf
{
public:
  void set_host(HostVec& h);

  void set_host(BufElem* host_buf, const size_type len);

  void sync();

  void alloc();

  HostBuf& host_buf();

private:
  HostBuf host_buf_;
};

/// This should be created by an object working with data on the open CL device and a
/// reference/pointer will be passed to an object that needs this data, but will
/// process it on the host.
class RayCastSyncHostBufFromOCL : public RayCastSyncHostBuf
{
public:
  RayCastSyncHostBufFromOCL() = default;

  RayCastSyncHostBufFromOCL(OCLBuf& ocl_buf, OCLQueue& ocl_queue);

  void set_host(HostVec& h);

  void set_host(BufElem* host_buf, const size_type len);

  void set_ocl(OCLBuf* ocl_buf, OCLQueue& ocl_queue);

  void sync();

  void alloc();

  HostBuf& host_buf();

private:
  OCLBuf*  ocl_buf_ = nullptr;
  OCLQueue ocl_queue_;

  HostBuf host_buf_;
  HostVec host_vec_;
};

/// \brief Base class for synchronizing data to be processed on a device.
class RayCastSyncOCLBuf : public RayCastSyncBuf
{
public:
  RayCastSyncOCLBuf() = default;

  explicit RayCastSyncOCLBuf(OCLBuf& ocl_buf);

  OCLBuf& ocl_buf();

  bool ocl_buf_valid() const;

  void set_ocl(OCLBuf* ocl_buf, const OCLQueue& ocl_queue);

  OCLQueue& queue();

protected:
  OCLBuf*  ocl_buf_ = nullptr;
  OCLQueue ocl_queue_;
};

/// This should be created by an object working with data on the device and a
/// reference/pointer will be passed to an object that needs this data and will
/// continue to process it on the device.
class RayCastSyncOCLBufFromOCL : public RayCastSyncOCLBuf
{
public:
  RayCastSyncOCLBufFromOCL() = default;

  explicit RayCastSyncOCLBufFromOCL(OCLBuf& ocl_buf);

  void sync();

  void alloc();
};

/// This should be created by an object working with data on the host and a
/// reference/pointer will be passed to an object that needs this data, but will
/// process it on the device.
class RayCastSyncOCLBufFromHost : public RayCastSyncOCLBuf
{
public:
  RayCastSyncOCLBufFromHost() = default;

  explicit RayCastSyncOCLBufFromHost(HostVec& host_buf);

  void set_host(HostVec& host_buf);

  void set_host(BufElem* host_buf, const size_type len);

  void sync();

  void alloc();

private:
  HostBuf host_buf_;
};

}  // xreg

#endif


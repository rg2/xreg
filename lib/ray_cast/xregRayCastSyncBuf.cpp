/*
 * MIT License
 *
 * Copyright (c) 2020-2026 Robert Grupp
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

#include "xregRayCastSyncBuf.h"

xreg::RayCastSyncBuf::HostBuf::HostBuf(BufElem* b, const size_type l)
  : buf(b), len(l)
{ }

void xreg::RayCastSyncBuf::set_modified()
{
  modified_ = true;
}

void xreg::RayCastSyncBuf::set_range(const size_type start, const size_type end)
{
  if ((range_start_ != start) || (range_end_ != end))
  {
    // the range of data has been changed, so a re-sync is
    // appropriate
    modified_ = true;
  }

  range_start_ = start;
  range_end_   = end;
}


void xreg::RayCastSyncHostBufFromHost::set_host(HostVec& h)
{
  host_buf_.buf = &h[0];
  host_buf_.len = h.size();
}

void xreg::RayCastSyncHostBufFromHost::set_host(BufElem* host_buf, const size_type len)
{
  host_buf_.buf = host_buf;
  host_buf_.len = len;
}

void xreg::RayCastSyncHostBufFromHost::sync()
{ }

void xreg::RayCastSyncHostBufFromHost::alloc()
{ }

xreg::RayCastSyncHostBufFromHost::HostBuf&
xreg::RayCastSyncHostBufFromHost::host_buf()
{
  return host_buf_;
}

xreg::RayCastSyncHostBufFromOCL::RayCastSyncHostBufFromOCL(OCLBuf& ocl_buf, OCLQueue& ocl_queue)
  : ocl_buf_(&ocl_buf), ocl_queue_(ocl_queue)
{ }

void xreg::RayCastSyncHostBufFromOCL::set_host(HostVec& h)
{
  host_buf_.buf = &h[0];
  host_buf_.len = h.size();
}

void xreg::RayCastSyncHostBufFromOCL::set_host(BufElem* host_buf, const size_type len)
{
  host_buf_.buf = host_buf;
  host_buf_.len = len;
}

void xreg::RayCastSyncHostBufFromOCL::set_ocl(OCLBuf* ocl_buf, OCLQueue& ocl_queue)
{
  ocl_buf_   = ocl_buf;
  ocl_queue_ = ocl_queue;
}

void xreg::RayCastSyncHostBufFromOCL::sync()
{
  // TODO: MAKE THREAD SAFE
  if (this->modified_)
  {
    auto ocl_buf_end = (this->range_end_ == this->kRANGE_AT_BUF_END) ?
                            ocl_buf_->end() : (ocl_buf_->begin() + this->range_end_);

    boost::compute::copy(ocl_buf_->begin() + this->range_start_, ocl_buf_end,
                         host_buf_.buf + this->range_start_,
                         ocl_queue_);
    this->modified_ = false;
  }
}

void xreg::RayCastSyncHostBufFromOCL::alloc()
{
  if (!host_buf_.buf)
  {
    host_vec_.resize(ocl_buf_->size());
    host_buf_.buf = &host_vec_[0];
    host_buf_.len = ocl_buf_->size();
  }
}

xreg::RayCastSyncHostBufFromOCL::HostBuf& xreg::RayCastSyncHostBufFromOCL::host_buf()
{
  return host_buf_;
}
  
xreg::RayCastSyncOCLBuf::RayCastSyncOCLBuf(OCLBuf& ocl_buf)
  : ocl_buf_(&ocl_buf) // TODO: what about queue?
{ }

xreg::RayCastSyncOCLBuf::OCLBuf& xreg::RayCastSyncOCLBuf::ocl_buf()
{
  return *ocl_buf_;
}

bool xreg::RayCastSyncOCLBuf::ocl_buf_valid() const
{
  return ocl_buf_;
}

void xreg::RayCastSyncOCLBuf::set_ocl(OCLBuf* ocl_buf, const OCLQueue& ocl_queue)
{
  ocl_buf_   = ocl_buf;
  ocl_queue_ = ocl_queue;
}

xreg::RayCastSyncOCLBuf::OCLQueue& xreg::RayCastSyncOCLBuf::queue()
{
  return ocl_queue_;
}
  
xreg::RayCastSyncOCLBufFromOCL::RayCastSyncOCLBufFromOCL(OCLBuf& ocl_buf)
  : RayCastSyncOCLBuf(ocl_buf)
{ }

void xreg::RayCastSyncOCLBufFromOCL::sync()
{ }

void xreg::RayCastSyncOCLBufFromOCL::alloc()
{ }
  
xreg::RayCastSyncOCLBufFromHost::RayCastSyncOCLBufFromHost(HostVec& host_buf)
  : RayCastSyncOCLBuf(), host_buf_(&host_buf[0], host_buf.size())
{ }

void xreg::RayCastSyncOCLBufFromHost::set_host(HostVec& host_buf)
{
  host_buf_.buf = &host_buf[0];
  host_buf_.len = host_buf.size();
}

void xreg::RayCastSyncOCLBufFromHost::set_host(BufElem* host_buf, const size_type len)
{
  host_buf_.buf = host_buf;
  host_buf_.len = len;
}

void xreg::RayCastSyncOCLBufFromHost::sync()
{
  if (this->modified_)
  {
    const size_type host_buf_end = (this->range_end_ == this->kRANGE_AT_BUF_END) ?
                                      host_buf_.len : this->range_end_;

    boost::compute::copy(host_buf_.buf + this->range_start_, host_buf_.buf + host_buf_end,
                         this->ocl_buf_->begin() + this->range_start_,
                         this->ocl_queue_);
  }
}

void xreg::RayCastSyncOCLBufFromHost::alloc()
{
  this->ocl_buf_->resize(host_buf_.len);
}


#ifdef XREG_HAS_METAL

xreg::RayCastSyncHostBufFromMetal::RayCastSyncHostBufFromMetal(MetalBuf& metal_buf,
                                                               MetalQueue& metal_queue)
  : metal_buf_(&metal_buf), metal_queue_(metal_queue)
{ }

void xreg::RayCastSyncHostBufFromMetal::set_host(HostVec& h)
{
  host_buf_.buf = h.data();
  host_buf_.len = h.size();
}

void xreg::RayCastSyncHostBufFromMetal::set_host(BufElem* host_buf, const size_type len)
{
  host_buf_.buf = host_buf;
  host_buf_.len = len;
}

void xreg::RayCastSyncHostBufFromMetal::set_metal(MetalBuf* metal_buf, MetalQueue& metal_queue)
{
  metal_buf_   = metal_buf;
  metal_queue_ = metal_queue;
}

void xreg::RayCastSyncHostBufFromMetal::sync()
{
  // TODO: MAKE THREAD SAFE
  if (this->modified_)
  {
    const size_type metal_buf_end = (this->range_end_ == this->kRANGE_AT_BUF_END) ?
                                        metal_buf_->size() : this->range_end_;

    CopyMetalToHost(*metal_buf_, this->range_start_, metal_buf_end,
                    host_buf_.buf + this->range_start_,
                    metal_queue_);
    this->modified_ = false;
  }
}

void xreg::RayCastSyncHostBufFromMetal::alloc()
{
  if (!host_buf_.buf)
  {
    host_vec_.resize(metal_buf_->size());
    host_buf_.buf = host_vec_.data();
    host_buf_.len = metal_buf_->size();
  }
}

xreg::RayCastSyncHostBufFromMetal::HostBuf& xreg::RayCastSyncHostBufFromMetal::host_buf()
{
  return host_buf_;
}

xreg::RayCastSyncMetalBuf::RayCastSyncMetalBuf(MetalBuf& metal_buf)
  : metal_buf_(&metal_buf)
{ }

xreg::RayCastSyncMetalBuf::MetalBuf& xreg::RayCastSyncMetalBuf::metal_buf()
{
  return *metal_buf_;
}

bool xreg::RayCastSyncMetalBuf::metal_buf_valid() const
{
  return metal_buf_;
}

void xreg::RayCastSyncMetalBuf::set_metal(MetalBuf* metal_buf, const MetalQueue& metal_queue)
{
  metal_buf_   = metal_buf;
  metal_queue_ = metal_queue;
}

xreg::RayCastSyncMetalBuf::MetalQueue& xreg::RayCastSyncMetalBuf::queue()
{
  return metal_queue_;
}

xreg::RayCastSyncMetalBufFromMetal::RayCastSyncMetalBufFromMetal(MetalBuf& metal_buf)
  : RayCastSyncMetalBuf(metal_buf)
{ }

void xreg::RayCastSyncMetalBufFromMetal::sync()
{ }

void xreg::RayCastSyncMetalBufFromMetal::alloc()
{ }

xreg::RayCastSyncMetalBufFromHost::RayCastSyncMetalBufFromHost(HostVec& host_buf)
  : RayCastSyncMetalBuf(), host_buf_(host_buf.data(), host_buf.size())
{ }

void xreg::RayCastSyncMetalBufFromHost::set_host(HostVec& host_buf)
{
  host_buf_.buf = host_buf.data();
  host_buf_.len = host_buf.size();
}

void xreg::RayCastSyncMetalBufFromHost::set_host(BufElem* host_buf, const size_type len)
{
  host_buf_.buf = host_buf;
  host_buf_.len = len;
}

void xreg::RayCastSyncMetalBufFromHost::sync()
{
  if (this->modified_)
  {
    const size_type host_buf_end = (this->range_end_ == this->kRANGE_AT_BUF_END) ?
                                      host_buf_.len : this->range_end_;

    CopyHostToMetal(host_buf_.buf + this->range_start_, host_buf_.buf + host_buf_end,
                    *this->metal_buf_, this->range_start_,
                    this->metal_queue_);
    this->modified_ = false;
  }
}

void xreg::RayCastSyncMetalBufFromHost::alloc()
{
  this->metal_buf_->resize(host_buf_.len);
}

#endif  // XREG_HAS_METAL

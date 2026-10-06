
#include <cstring>
#include <iostream>
#include <vector>

#import <Metal/Metal.h>

#include "xregAssert.h"
#include "xregRayCastSyncBuf.h"

namespace
{

using namespace xreg;

using BufElem  = RayCastSyncBuf::BufElem;
using HostVec  = RayCastSyncBuf::HostVec;
using MetalBuf = RayCastSyncBuf::MetalBuf;

// The value of an element with every byte set to v
BufElem ByteFillVal(const unsigned char v)
{
  BufElem x;
  std::memset(&x, v, sizeof(x));
  return x;
}

bool BitwiseEq(const BufElem a, const BufElem b)
{
  return std::memcmp(&a, &b, sizeof(BufElem)) == 0;
}

BufElem InitVal(const size_type i)
{
  return static_cast<BufElem>(i) * 0.5f + 1.0f;
}

// Write to elements [start, end) of buf using the GPU. This does NOT wait for
// completion, the sync objects are responsible for that.
void GPUFill(MetalBuf& buf, const size_type start, const size_type end,
             const unsigned char v, MetalCmdQueue& queue)
{
  @autoreleasepool
  {
    id<MTLCommandBuffer> cmd_buf = [(__bridge id<MTLCommandQueue>) queue.native_handle() commandBuffer];

    id<MTLBlitCommandEncoder> blit = [cmd_buf blitCommandEncoder];
    [blit fillBuffer:(__bridge id<MTLBuffer>) buf.buffer().native_handle()
               range:NSMakeRange(start * sizeof(BufElem), (end - start) * sizeof(BufElem))
               value:v];
    [blit endEncoding];

    [cmd_buf commit];
  }
}

void TestStorageMode(const MetalDevice& dev, const MetalStorageMode mode)
{
  MetalCmdQueue queue(dev);

  const size_type n = 1000;

  HostVec host_src(n);
  for (size_type i = 0; i < n; ++i)
  {
    host_src[i] = InitVal(i);
  }

  HostVec readback(n, -1);

  MetalBuf metal_buf(dev, 0, mode);
  xregASSERT(metal_buf.empty());
  xregASSERT(!metal_buf.buffer().native_handle());

  if (mode == MetalStorageMode::kMANAGED)
  {
    xregASSERT(metal_buf.buffer().is_managed());
  }
  else if (mode == MetalStorageMode::kSHARED)
  {
    xregASSERT(!metal_buf.buffer().is_managed());
  }

  // host -> Metal, full buffer

  RayCastSyncMetalBufFromHost to_metal(host_src);
  to_metal.set_metal(&metal_buf, queue);
  xregASSERT(to_metal.metal_buf_valid());

  to_metal.alloc();
  xregASSERT(metal_buf.size() == n);
  xregASSERT(metal_buf.buffer().native_handle());

  to_metal.set_modified();
  to_metal.sync();

  CopyMetalToHost(metal_buf, 0, n, readback.data(), queue);
  xregASSERT(readback == host_src);

  // sync without a modification does not transfer

  host_src[0] = -100;
  to_metal.sync();

  CopyMetalToHost(metal_buf, 0, 1, readback.data(), queue);
  xregASSERT(readback[0] == InitVal(0));

  // host -> Metal, sub-range only

  for (auto& x : host_src)
  {
    x = -x;
  }

  to_metal.set_range(100, 200);
  to_metal.sync();

  CopyMetalToHost(metal_buf, 0, n, readback.data(), queue);
  for (size_type i = 0; i < n; ++i)
  {
    xregASSERT(readback[i] == (((i >= 100) && (i < 200)) ? -InitVal(i) : InitVal(i)));
  }

  // Metal (written by the GPU) -> host, full buffer, internally allocated host buffer

  RayCastSyncHostBufFromMetal to_host(metal_buf, queue);
  to_host.alloc();

  auto& host_buf = to_host.host_buf();
  xregASSERT(host_buf.buf);
  xregASSERT(host_buf.len == n);

  GPUFill(metal_buf, 0, n, 0x3F, queue);

  to_host.set_modified();
  to_host.sync();

  for (size_type i = 0; i < n; ++i)
  {
    xregASSERT(BitwiseEq(host_buf.buf[i], ByteFillVal(0x3F)));
  }

  // Metal (written by the GPU) -> host, sub-range only, external host buffer

  HostVec ext_host(n, 0);
  to_host.set_host(ext_host);

  GPUFill(metal_buf, 0, n, 0x40, queue);

  to_host.set_range(250, 500);
  to_host.sync();

  for (size_type i = 0; i < n; ++i)
  {
    xregASSERT(BitwiseEq(ext_host[i], ((i >= 250) && (i < 500)) ? ByteFillVal(0x40) : BufElem(0)));
  }

  // Metal -> Metal is a pass-through

  RayCastSyncMetalBufFromMetal pass_through(metal_buf);
  pass_through.set_metal(&metal_buf, queue);
  pass_through.alloc();
  pass_through.set_modified();
  pass_through.sync();
  xregASSERT(&pass_through.metal_buf() == &metal_buf);
  xregASSERT(metal_buf.size() == n);

  // out of bounds transfers throw

  bool threw = false;
  try
  {
    CopyMetalToHost(metal_buf, n - 1, n + 1, readback.data(), queue);
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  xregASSERT(threw);

  threw = false;
  try
  {
    CopyHostToMetal(host_src.data(), host_src.data() + n, metal_buf, 1, queue);
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  xregASSERT(threw);
}

}  // un-named

int main(int argc, char* argv[])
{
  using namespace xreg;

  const auto all_devs = MetalAllDevices();
  xregASSERT(!all_devs.empty());

  for (const auto& dev : all_devs)
  {
    std::cout << "Metal device: " << dev.id_str()
              << (dev.has_unified_memory() ? " (unified memory)" : " (discrete memory)")
              << std::endl;

    std::cout << "  testing shared storage..." << std::endl;
    TestStorageMode(dev, MetalStorageMode::kSHARED);

    std::cout << "  testing managed storage..." << std::endl;
    TestStorageMode(dev, MetalStorageMode::kMANAGED);

    std::cout << "  testing auto storage..." << std::endl;
    TestStorageMode(dev, MetalStorageMode::kAUTO);
  }

  std::cout << "PASSED" << std::endl;

  return 0;
}

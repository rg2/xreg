/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#if !__has_feature(objc_arc)
#error "xregMetalCompute.mm must be compiled with ARC (-fobjc-arc)"
#endif

#include "xregMetalCompute.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "xregExceptionUtils.h"
#include "xregMetalShaderSrc.h"

struct xreg::MetalLibrary::Impl
{
  MetalDevice dev;

  id<MTLLibrary> lib = nil;
};

struct xreg::MetalComputePipeline::Impl
{
  std::string kernel_name;

  id<MTLComputePipelineState> pipeline = nil;
};

std::string xreg::MetalHelperShadersSrc()
{
  std::string src;

  for (const char* s : { kMETAL_SHADER_SHARED_SRC, kMETAL_MATH_SRC, kMETAL_SPATIAL_SRC,
                         kMETAL_INTERP_SRC, kMETAL_MISC_KERNELS_SRC })
  {
    src += s;
    src += '\n';
  }

  return src;
}

xreg::MetalLibrary::MetalLibrary(const MetalDevice& dev, const std::string& src,
                                 const MacroMap& macros, const bool fast_math)
  : impl_(std::make_shared<Impl>())
{
  if (!dev.valid())
  {
    xregThrow("cannot compile a Metal library with an invalid device!");
  }

  impl_->dev = dev;

  @autoreleasepool
  {
    MTLCompileOptions* opts = [MTLCompileOptions new];

    if (@available(macOS 15.0, *))
    {
      opts.mathMode = fast_math ? MTLMathModeFast : MTLMathModeSafe;
    }
    else
    {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
      opts.fastMathEnabled = fast_math ? YES : NO;
#pragma clang diagnostic pop
    }

    NSMutableDictionary<NSString*,NSObject*>* ns_macros = [NSMutableDictionary dictionary];

    ns_macros[@"XREG_METAL_RUNTIME_SRC"] = @"1";

    for (const auto& m : macros)
    {
      ns_macros[[NSString stringWithUTF8String:m.first.c_str()]] =
                                  [NSString stringWithUTF8String:m.second.c_str()];
    }

    opts.preprocessorMacros = ns_macros;

    NSError* err = nil;

    impl_->lib = [(__bridge id<MTLDevice>) dev.native_handle()
                    newLibraryWithSource:[NSString stringWithUTF8String:src.c_str()]
                                 options:opts
                                   error:&err];

    if (!impl_->lib)
    {
      xregThrow("failed to compile Metal library:\n%s",
                err ? err.localizedDescription.UTF8String : "unknown error");
    }
  }
}

bool xreg::MetalLibrary::valid() const
{
  return impl_ && impl_->lib;
}

const xreg::MetalDevice& xreg::MetalLibrary::device() const
{
  if (!impl_)
  {
    xregThrow("invalid Metal library has no device!");
  }

  return impl_->dev;
}

void* xreg::MetalLibrary::native_handle() const
{
  return valid() ? (__bridge void*) impl_->lib : nullptr;
}

xreg::MetalComputePipeline::MetalComputePipeline(const MetalLibrary& lib,
                                                 const std::string& kernel_name,
                                                 const BoolFnConstMap& bool_fn_consts)
  : impl_(std::make_shared<Impl>())
{
  if (!lib.valid())
  {
    xregThrow("cannot create a Metal compute pipeline from an invalid library!");
  }

  impl_->kernel_name = kernel_name;

  @autoreleasepool
  {
    MTLFunctionConstantValues* const_vals = [MTLFunctionConstantValues new];

    for (const auto& c : bool_fn_consts)
    {
      const bool v = c.second;
      [const_vals setConstantValue:&v type:MTLDataTypeBool atIndex:c.first];
    }

    NSError* err = nil;

    id<MTLFunction> fn = [(__bridge id<MTLLibrary>) lib.native_handle()
                            newFunctionWithName:[NSString stringWithUTF8String:kernel_name.c_str()]
                                 constantValues:const_vals
                                          error:&err];

    if (!fn)
    {
      xregThrow("failed to create Metal function %s: %s", kernel_name.c_str(),
                err ? err.localizedDescription.UTF8String : "function not found");
    }

    impl_->pipeline = [(__bridge id<MTLDevice>) lib.device().native_handle()
                          newComputePipelineStateWithFunction:fn error:&err];

    if (!impl_->pipeline)
    {
      xregThrow("failed to create Metal compute pipeline for %s: %s", kernel_name.c_str(),
                err ? err.localizedDescription.UTF8String : "unknown error");
    }
  }
}

bool xreg::MetalComputePipeline::valid() const
{
  return impl_ && impl_->pipeline;
}

const std::string& xreg::MetalComputePipeline::kernel_name() const
{
  if (!impl_)
  {
    xregThrow("invalid Metal compute pipeline has no kernel name!");
  }

  return impl_->kernel_name;
}

xreg::size_type xreg::MetalComputePipeline::thread_execution_width() const
{
  return valid() ? impl_->pipeline.threadExecutionWidth : 0;
}

xreg::size_type xreg::MetalComputePipeline::max_total_threads_per_threadgroup() const
{
  return valid() ? impl_->pipeline.maxTotalThreadsPerThreadgroup : 0;
}

void* xreg::MetalComputePipeline::native_handle() const
{
  return valid() ? (__bridge void*) impl_->pipeline : nullptr;
}


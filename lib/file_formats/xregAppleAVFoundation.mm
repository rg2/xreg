/*
 * MIT License
 *
 * Copyright (c) 2021 Robert Grupp
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
#error "xregAppleAVFoundation.mm must be compiled with ARC (-fobjc-arc)"
#endif

#include "xregAppleAVFoundation.h"

#import <AVFoundation/AVFoundation.h>

#include <opencv2/imgproc.hpp>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregFilesystemUtils.h"

namespace
{

// The Objective-C objects are stored in void* members (so the header remains
// pure C++) which own a +1 reference, obtained with __bridge_retained. This
// releases that reference.
void ReleaseObjCRef(void*& obj)
{
  if (obj)
  {
    CFBridgingRelease(obj);
    obj = nullptr;
  }
}

}  // un-named

void xreg::WriteImageFramesToVideoAppleAVF::release_objc_refs()
{
  ReleaseObjCRef(av_asset_writer_);
  ReleaseObjCRef(av_asset_writer_input_);
  ReleaseObjCRef(av_assest_writer_pix_buf_adaptor_);

  input_setup_ = false;
}

void xreg::WriteImageFramesToVideoAppleAVF::open()
{
  @autoreleasepool
  {
    // drop any writer from a previous call to open()
    release_objc_refs();

    if (Path(dst_vid_path).exists())
    {
      std::remove(dst_vid_path.c_str());
    }

    NSError* error = nil;

    AVAssetWriter* writer = [AVAssetWriter assetWriterWithURL:
                              [NSURL fileURLWithPath:
                                [NSString stringWithUTF8String:dst_vid_path.c_str()].stringByExpandingTildeInPath
                                          isDirectory:NO]
                              fileType:AVFileTypeMPEG4
                              error:&error];

    if (writer)
    {
      av_asset_writer_ = (__bridge_retained void*) writer;
    }
    else
    {
      xregThrow("Failed to initialize AVAssetWriter: %s", error.localizedDescription.UTF8String);
    }

    frame_count_ = 0;
  }
}

xreg::WriteImageFramesToVideoAppleAVF::~WriteImageFramesToVideoAppleAVF()
{
  if (input_setup_)
  {
    close();
  }

  release_objc_refs();
}

void xreg::WriteImageFramesToVideoAppleAVF::close()
{
  @autoreleasepool
  {
    // these are strong references, so the objects remain valid in this scope
    // after the members are released below
    AVAssetWriterInput* writer_input = (__bridge AVAssetWriterInput*) av_asset_writer_input_;

    if (writer_input)
    {
      [writer_input markAsFinished];
    }
    else
    {
      xregThrow("Cannot close writer when not opened!");
    }

    AVAssetWriter* writer = (__bridge AVAssetWriter*) av_asset_writer_;

    // the writer is finished with (successfully or not) after this call, so
    // always release the members, even when an exception is thrown below
    release_objc_refs();

    if (writer)
    {
      __block BOOL finished_writing = NO;

      [writer finishWritingWithCompletionHandler:^{ finished_writing = YES; }];

      while (!finished_writing)
      {
        [NSThread sleepForTimeInterval:0.125];
      }

      if (writer.status == AVAssetWriterStatusFailed)
      {
        xregThrow("Failed to finish writing with error: %s", writer.error.localizedDescription.UTF8String);
      }
      else if (writer.status == AVAssetWriterStatusCancelled)
      {
        xregThrow("Failed to finish writing - was cancelled!");
      }
      else if (writer.status == AVAssetWriterStatusUnknown)
      {
        xregThrow("Failed to finish writing - status unknown!");
      }
      else if (writer.status == AVAssetWriterStatusWriting)
      {
        xregThrow("Failed to finish writing - still writing!");
      }
    }
    else
    {
      xregThrow("writer object is null! cannot finish writing!");
    }
  }
}

void xreg::WriteImageFramesToVideoAppleAVF::write(const cv::Mat& frame)
{
  @autoreleasepool
  {
    AVAssetWriter* writer = (__bridge AVAssetWriter*) av_asset_writer_;

    if (writer)
    {
      if (!input_setup_)
      {
        num_rows_ = frame.rows;
        num_cols_ = frame.cols;

        frame_type_ = frame.type();

        NSNumber* frame_width  = [NSNumber numberWithInt:frame.cols];
        NSNumber* frame_height = [NSNumber numberWithInt:frame.rows];

        NSDictionary* out_settings = @{ AVVideoCodecKey:AVVideoCodecTypeH264,
                                        AVVideoWidthKey:frame_width,
                                        AVVideoHeightKey:frame_height };


        AVAssetWriterInput* writer_input = [AVAssetWriterInput
                                              assetWriterInputWithMediaType:AVMediaTypeVideo
                                              outputSettings:out_settings];
        xregASSERT(writer_input);

        av_asset_writer_input_ = (__bridge_retained void*) writer_input;

        NSDictionary* src_buf_attr = @{ (__bridge NSString*) kCVPixelBufferPixelFormatTypeKey:
                                            [NSNumber numberWithInt:((frame_type_ == CV_8UC3) ?
                                              kCVPixelFormatType_24RGB : kCVPixelFormatType_OneComponent8)],
                                        (__bridge NSString*) kCVPixelBufferWidthKey:frame_width,
                                        (__bridge NSString*) kCVPixelBufferHeightKey:frame_height };

        AVAssetWriterInputPixelBufferAdaptor* pix_buf_adaptor = [AVAssetWriterInputPixelBufferAdaptor
                            assetWriterInputPixelBufferAdaptorWithAssetWriterInput:writer_input
                            sourcePixelBufferAttributes:src_buf_attr];
        xregASSERT(pix_buf_adaptor);

        av_assest_writer_pix_buf_adaptor_ = (__bridge_retained void*) pix_buf_adaptor;

        xregASSERT([writer canAddInput:writer_input]);
        [writer addInput:writer_input];

        if ([writer startWriting])
        {
          input_setup_ = true;

          [writer startSessionAtSourceTime:kCMTimeZero];
        }
        else
        {
          if (writer.status == AVAssetWriterStatusFailed)
          {
            xregThrow("Failed to start writing with error: %s", writer.error.localizedDescription.UTF8String);
          }
          else
          {
            xregThrow("Unable to start writing video (no error provided)!");
          }
        }
      }

      AVAssetWriterInput* writer_input = (__bridge AVAssetWriterInput*) av_asset_writer_input_;
      xregASSERT(writer_input);

      AVAssetWriterInputPixelBufferAdaptor* pix_buf_adaptor =
                      (__bridge AVAssetWriterInputPixelBufferAdaptor*) av_assest_writer_pix_buf_adaptor_;

      xregASSERT(pix_buf_adaptor);
      xregASSERT(pix_buf_adaptor.pixelBufferPool);

      xregASSERT(num_rows_ == frame.rows);
      xregASSERT(num_cols_ == frame.cols);
      xregASSERT(frame_type_ == frame.type());

      while (!writer_input.readyForMoreMediaData)
      {
        [NSThread sleepForTimeInterval:0.05];
      }

      CVPixelBufferRef pixel_buf = nullptr;
      CVPixelBufferPoolCreatePixelBuffer(nullptr, pix_buf_adaptor.pixelBufferPool, &pixel_buf);
      xregASSERT(pixel_buf);

      CVPixelBufferLockBaseAddress(pixel_buf, 0);

      cv::Mat dst_mat(num_rows_, num_cols_, frame_type_,
                      CVPixelBufferGetBaseAddress(pixel_buf),
                      CVPixelBufferGetBytesPerRow(pixel_buf));

      if (frame.channels() == 1)
      {
        frame.copyTo(dst_mat);
      }
      else
      {
        xregASSERT(frame.channels() == 3);

        cv::cvtColor(frame, dst_mat, cv::COLOR_BGR2RGB);
      }

      CVPixelBufferUnlockBaseAddress(pixel_buf, 0);

      // TODO: consider switching to CMTimeMakeWithSeconds
      const BOOL appended = [pix_buf_adaptor appendPixelBuffer:pixel_buf
                                          withPresentationTime:CMTimeMake(frame_count_, static_cast<int32_t>(fps))];

      CVPixelBufferRelease(pixel_buf);

      if (!appended)
      {
        xregThrow("Failed to append pixel buffer!");
      }

      ++frame_count_;
    }
    else
    {
      xregThrow("cannot write a frame before AVAssetWriter setup!");
    }
  }
}


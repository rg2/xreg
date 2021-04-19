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

#include "xregAppleAVFoundation.h"

#import <AVFoundation/AVFoundation.h>

#include <opencv2/imgproc.hpp>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregFilesystemUtils.h"

void xreg::WriteImageFramesToVideoAppleAVF::open()
{
  if (Path(dst_vid_path).exists())
  {
    std::remove(dst_vid_path.c_str());
  }
  
  NSError* error = nil;

  AVAssetWriter* writer = [AVAssetWriter assetWriterWithURL:
                            [NSURL fileURLWithPath:
                              [NSString stringWithUTF8String:dst_vid_path.c_str()].stringByAbbreviatingWithTildeInPath
                                        isDirectory:NO]
                            fileType:AVFileTypeMPEG4
                            error:&error];

  if (writer)
  {
    av_asset_writer_ = writer;
  }
  else
  {
    xregThrow("Failed to initialize AVAssetWriter: %s", error.localizedDescription.UTF8String); 
  }

  frame_count_ = 0;
}

xreg::WriteImageFramesToVideoAppleAVF::~WriteImageFramesToVideoAppleAVF()
{
  if (input_setup_)
  {
    close();
  }
}

void xreg::WriteImageFramesToVideoAppleAVF::close()
{
  AVAssetWriterInput* writer_input = static_cast<AVAssetWriterInput*>(av_asset_writer_input_);

  if (writer_input)
  {
    [writer_input markAsFinished];
  }
  else
  {
    xregThrow("Cannot close writer when not opened!");
  }
  
  AVAssetWriter* writer = static_cast<AVAssetWriter*>(av_asset_writer_);

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
 
  av_asset_writer_                  = nullptr;
  av_asset_writer_input_            = nullptr;
  av_assest_writer_pix_buf_adaptor_ = nullptr;
  
  input_setup_ = false;
}

void xreg::WriteImageFramesToVideoAppleAVF::write(const cv::Mat& frame)
{
  AVAssetWriter* writer = static_cast<AVAssetWriter*>(av_asset_writer_);
  
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

      av_asset_writer_input_ = writer_input;

      NSDictionary* src_buf_attr = @{ (__bridge NSString*) kCVPixelBufferPixelFormatTypeKey:
                                          [NSNumber numberWithInt:((frame_type_ == CV_8UC3) ?
                                            kCVPixelFormatType_24RGB : kCVPixelFormatType_OneComponent8)],
                                      (__bridge NSString*) kCVPixelBufferWidthKey:frame_width,
                                      (__bridge NSString*) kCVPixelBufferHeightKey:frame_height };
     
      AVAssetWriterInputPixelBufferAdaptor* pix_buf_adaptor = [AVAssetWriterInputPixelBufferAdaptor
                          assetWriterInputPixelBufferAdaptorWithAssetWriterInput:writer_input
                          sourcePixelBufferAttributes:src_buf_attr];
      xregASSERT(pix_buf_adaptor);

      av_assest_writer_pix_buf_adaptor_ = pix_buf_adaptor;

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
    
    AVAssetWriterInput* writer_input = static_cast<AVAssetWriterInput*>(av_asset_writer_input_);
    xregASSERT(writer_input);

    AVAssetWriterInputPixelBufferAdaptor* pix_buf_adaptor =
                    static_cast<AVAssetWriterInputPixelBufferAdaptor*>(av_assest_writer_pix_buf_adaptor_);
    
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
    if (![pix_buf_adaptor appendPixelBuffer:pixel_buf
                       withPresentationTime:CMTimeMake(frame_count_, static_cast<int32_t>(fps))])
    {
      xregThrow("Failed to append pixel buffer!");
    }

    CVPixelBufferRelease(pixel_buf);

    ++frame_count_;
  }
  else
  {
    xregThrow("cannot write a frame before AVAssetWriter setup!");
  }
}


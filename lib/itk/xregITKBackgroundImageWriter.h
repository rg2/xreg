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

#ifndef XREGITKBACKGROUNDIMAGEWRITER_H_
#define XREGITKBACKGROUNDIMAGEWRITER_H_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>

#include "xregITKIOUtils.h"

namespace xreg
{

/// \brief Utility class for writing images to disk in the background
///
/// Useful when writing out large compressed images, so that processing on other
/// image may continue while waiting from previously processed image to be
/// written.
template <class tImage>
class ITKBackgroundImageWriter
{
public:
  using ImageType    = tImage;
  using ImagePointer = typename ImageType::Pointer;

  using size_type = std::size_t;

  /// \brief Constructor - starts up the image writing threads
  ///
  /// If the number of writer threads passed is 0, then the number of threads
  /// is determined by the number of virtual cores.
  explicit ITKBackgroundImageWriter(const size_type num_writer_threads = 0)
  {
    num_threads_to_use_ = num_writer_threads;

    if (!num_writer_threads)
    {
      num_threads_to_use_ = std::thread::hardware_concurrency();
      
      if (!num_threads_to_use_)
      {
        num_threads_to_use_ = 1;
      }
    }

    running_ = true;
    ThreadObjFn tmp_thread_obj_fn_init = { &running_, &queue_mutex_, &imgs_to_write_, &cur_num_bytes_, &num_imgs_writing_ };

    writer_threads_.resize(num_threads_to_use_);
    for (size_type thread_idx = 0; thread_idx < num_threads_to_use_; ++thread_idx)
    {
      writer_threads_[thread_idx].reset(new std::thread(tmp_thread_obj_fn_init));
    }
  }

  /// \brief Destructor - waits for all images to be written and stops threads
  ~ITKBackgroundImageWriter()
  {
    // wait for all images to be written
    wait();

    // signal the threads to exit
    running_ = false;

    // wait for the threads to exit
    for (size_type thread_idx = 0; thread_idx < num_threads_to_use_; ++thread_idx)
    {
      writer_threads_[thread_idx]->join();
    }
  }
  
  // no copying
  ITKBackgroundImageWriter(const ITKBackgroundImageWriter&) = delete;
  ITKBackgroundImageWriter& operator=(const ITKBackgroundImageWriter&) = delete;

  /// \brief Adds an image to the queue
  void add(ImagePointer img, const std::string& path,
           const bool force_no_compression = false)
  {
    ImagePathPair img_path_pair = { img, path, force_no_compression };

    const size_type cur_img_num_bytes = GetImageNumBytes(img.GetPointer());

    if (max_num_bytes_)
    {
      // we're enforcing the maximum amount of memory to queue, we'll need to block
      // and keep checking until the image is added.

      while (true)
      {
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);

          // we can add this image if the queue is empty and no image is currently
          // being written (we always need to be able to add a single image,
          // otherwise no work will be done) or adding the current image does not
          // violate the memory limit.
          if ((imgs_to_write_.empty() && !num_imgs_writing_) || ((cur_num_bytes_ + cur_img_num_bytes) <= max_num_bytes_))
          {
            imgs_to_write_.push(img_path_pair);
            cur_num_bytes_ += cur_img_num_bytes;
            break;
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    else
    {
      // we're not enforcing the maximum amount of memory to queue, this case is
      // simpler.
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        imgs_to_write_.push(img_path_pair);
        cur_num_bytes_ += cur_img_num_bytes;
      }
    }
  }

  /// \brief Blocks until the queue is empty - e.g. all images written.
  void wait()
  {
    bool queue_is_empty = false;

    while (!queue_is_empty || num_imgs_writing_)
    {
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_is_empty = imgs_to_write_.empty();
      }

      if (!queue_is_empty || num_imgs_writing_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  /// \brief The maximum number of bytes of raw image buffers to queue for writing
  ///
  /// When this limit is reached, calls to add become blocking until enough images
  /// are written to disk and their memory reclaimed. This should be used to ensure
  /// that system memory is not exhausted. 0 -> no limit.
  void set_memory_limit(const size_type max_num_bytes)
  {
    max_num_bytes_ = max_num_bytes;
  }

private:

  struct ImagePathPair
  {
    ImagePointer img;
    std::string path;
    bool force_no_compression;
  };

  using ImageQueue = std::queue<ImagePathPair>;

  struct ThreadObjFn
  {
    bool* running;
    std::mutex* queue_mutex;
    ImageQueue* img_queue;

    size_type* cur_num_bytes_in_queue;

    size_type* num_imgs_writing;

    void operator()()
    {
      ImagePathPair cur_img_path_pair;

      while (*running)
      {
        {
          // get lock on queue, get an image to write if there is one
          std::lock_guard<std::mutex> lock(*queue_mutex);

          if (!img_queue->empty())
          {
            cur_img_path_pair = img_queue->front();
            img_queue->pop();
            ++*num_imgs_writing;
          }
        }

        if (cur_img_path_pair.img && !cur_img_path_pair.path.empty())
        {
          // a valid image and path was removed from the queue, write it!
          WriteITKImageToDisk(cur_img_path_pair.img.GetPointer(),
                              cur_img_path_pair.path,
                              cur_img_path_pair.force_no_compression);

          {
            std::lock_guard<std::mutex> lock(*queue_mutex);
            *cur_num_bytes_in_queue -= GetImageNumBytes(cur_img_path_pair.img.GetPointer());
            --*num_imgs_writing;
          }

          // reset
          cur_img_path_pair.img = 0;
          cur_img_path_pair.path.clear();
        }
        else
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }
  };

  static size_type GetImageNumBytes(ImageType* img)
  {
    return sizeof(typename ImageType::PixelType) * img->GetPixelContainer()->Capacity();
  }

  std::mutex queue_mutex_;
  ImageQueue imgs_to_write_;

  // TODO: I think this may be changed to std::vector<std::thread> due to
  //       move ctors in c++11
  std::vector<std::shared_ptr<std::thread>> writer_threads_;

  size_type num_threads_to_use_ = 0;

  bool running_ = false;

  size_type cur_num_bytes_ = 0;
  size_type max_num_bytes_ = 0;

  size_type num_imgs_writing_ = 0;
};


}  // xreg

#endif


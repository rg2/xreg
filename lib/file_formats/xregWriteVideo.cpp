/*
 * MIT License
 *
 * Copyright (c) 2020,2021 Robert Grupp
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

#include "xregWriteVideo.h"

#ifndef _WIN32

#define BOOST_ERROR_CODE_HEADER_ONLY
#include <boost/process.hpp>
#undef BOOST_ERROR_CODE_HEADER_ONLY

#include <fmt/format.h>

#endif

#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include "xregAssert.h"
#include "xregFilesystemUtils.h"

#ifdef __APPLE__
#include "xregAppleAVFoundation.h"
#endif

void xreg::WriteImageFramesToVideo::write(const std::vector<cv::Mat>& frames)
{
  for (const cv::Mat& f : frames)
  {
    write(f);
  }
}

#ifndef _WIN32

namespace xreg
{
namespace detail
{

struct BoostProc
{
  std::unique_ptr<boost::process::child> ffmpeg_p;
  
  boost::process::pipe ffmpeg_p_in;
};

}
}
  
xreg::WriteImageFramesToVideoWithFFMPEG::WriteImageFramesToVideoWithFFMPEG()
{
  ffmpeg_path = FindExeOnSystemPath("ffmpeg");
}

xreg::WriteImageFramesToVideoWithFFMPEG::WriteImageFramesToVideoWithFFMPEG(
                                            const std::string& ffmpeg_path_arg)
{
  ffmpeg_path = ffmpeg_path_arg;
}

void xreg::WriteImageFramesToVideoWithFFMPEG::open()
{
  namespace bp = boost::process;
 
  xregASSERT(!ffmpeg_path.empty()); 
  xregASSERT(p == nullptr);

  p = std::make_shared<detail::BoostProc>();

  const std::string fps_str = fmt::format("{}", fps);
  const std::string q_str   = fmt::format("{}", quality);

  p->ffmpeg_p.reset(new bp::child(ffmpeg_path,
                                  "-y",                   // overwrite existing output files
                                  "-f", "image2pipe",     // inputs are images piped in
                                  //"-vcodec", "png",     // input codec (not required)
                                  "-framerate", fps_str,  // fps
                                  "-i", "-",              // read from stdin
                                  "-vcodec", "h264",      // output codec
                                  "-pix_fmt", "yuv420p",  // need this for compatibility
                                  "-an",                  // disable audio
                                  "-crf", q_str,          // constant quality
                                  "-preset", "slow",      // try for better compression, but slower
                                  dst_vid_path,           // output file
                                  bp::std_in < p->ffmpeg_p_in,
                                  bp::std_out > bp::null,
                                  bp::std_err > bp::null)); 
}

void xreg::WriteImageFramesToVideoWithFFMPEG::close()
{
  xregASSERT(p.get());

  p->ffmpeg_p_in.close();

  p->ffmpeg_p->wait();
 
  p = nullptr;
}

void xreg::WriteImageFramesToVideoWithFFMPEG::write(const cv::Mat& frame)
{
  xregASSERT(p->ffmpeg_p->running());
  
  cv::imencode(".png", frame, png_buf);
  
  p->ffmpeg_p_in.write(reinterpret_cast<char*>(&png_buf[0]), png_buf.size());
}

xreg::WriteImageFramesToVideoWithFFMPEG::~WriteImageFramesToVideoWithFFMPEG()
{
  if (p)
  {
    close();
  }
}

#endif

void xreg::WriteImageFramesToVideoWithOpenCV::open()
{ }

void xreg::WriteImageFramesToVideoWithOpenCV::close()
{
  writer = nullptr;
}

void xreg::WriteImageFramesToVideoWithOpenCV::write(const cv::Mat& frame)
{
  if (!writer)
  {
    if (Path(dst_vid_path).exists())
    {
      std::remove(dst_vid_path.c_str());
    }

    writer = std::make_shared<cv::VideoWriter>(dst_vid_path, cv::VideoWriter::fourcc('a','v','c','1'),
                                               fps, frame.size());
  
    xregASSERT(writer->isOpened());
 
    writer->set(cv::VIDEOWRITER_PROP_QUALITY, 100.0); 
  }

  *writer << frame;
}

std::unique_ptr<xreg::WriteImageFramesToVideo> xreg::GetWriteImageFramesToVideo()
{
  std::unique_ptr<WriteImageFramesToVideo> writer;

#ifndef _WIN32
  const std::string ffmpeg_path = FindExeOnSystemPath("ffmpeg");

  if (!ffmpeg_path.empty())
  {
    writer.reset(new WriteImageFramesToVideoWithFFMPEG);
  }
  else
#endif
  {
#ifdef __APPLE__
    std::cerr << "WARNING: could not find FFMPEG executable, falling back to "
                 "Apple AVFoundation video writer!" << std::endl;
    writer.reset(new WriteImageFramesToVideoAppleAVF);
#else

#ifndef _WIN32
    std::cerr << "WARNING: could not find FFMPEG executable, falling back to "
                 "OpenCV video writer!" << std::endl;
#endif

    writer.reset(new WriteImageFramesToVideoWithOpenCV);
#endif
  }

  return writer;
}

void xreg::WriteAllImageFramesToVideo(const std::string& vid_path,
                                      const std::vector<cv::Mat>& frames,
                                      const double fps_or_len,
                                      const bool is_fps)
{
  if (!frames.empty())
  {
    auto writer = GetWriteImageFramesToVideo();

    writer->dst_vid_path = vid_path;
    writer->fps = is_fps ? fps_or_len : (frames.size() / fps_or_len);
    
    writer->open();

    writer->write(frames);
   
    writer->close();
  }
  else
  {
    xregThrow("No frames provided to create video!");
  }
}

void xreg::WriteImageFilesToVideo(const std::string& vid_path,
                                  const std::vector<std::string>& img_paths,
                                  const double fps_or_len,
                                  const bool is_fps)
{
  std::vector<cv::Mat> frames;

  const size_type num_frames = img_paths.size();

  frames.reserve(num_frames);

  for (size_type i = 0; i < num_frames; ++i)
  {
    frames.push_back(cv::imread(img_paths[i]));
  }

  WriteAllImageFramesToVideo(vid_path, frames, fps_or_len, is_fps);
}

void xreg::WriteDirOfImagesToVideo(const std::string& vid_path,
                                   const std::string& img_dir,
                                   const bool lex_sort,
                                   const std::vector<std::string>& img_exts,
                                   const double fps_or_len,
                                   const bool is_fps)
{
  FileExtensions file_exts;
  
  for (const auto& ext : img_exts)
  {
    file_exts.add(ext);
  }
  
  std::vector<std::string> img_paths;

  GetFilePathsFromDir(img_dir, &img_paths, file_exts);

  if (lex_sort)
  {
    std::sort(img_paths.begin(), img_paths.end());
  }

  WriteImageFilesToVideo(vid_path, img_paths, fps_or_len, is_fps);
}


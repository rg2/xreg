
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <unistd.h>

#include <opencv2/core.hpp>

#include "xregAssert.h"
#include "xregAppleAVFoundation.h"

namespace
{

using namespace xreg;

// Writes a short video with frames of the provided type and checks that a
// non-empty file was created at expected_path
void TestWriteVideo(const std::string& dst_path, const std::string& expected_path,
                    const int frame_type)
{
  std::cout << "writing: " << dst_path << std::endl;

  std::filesystem::remove(expected_path);

  {
    WriteImageFramesToVideoAppleAVF writer;
    writer.dst_vid_path = dst_path;
    writer.fps = 15;

    writer.open();

    for (int i = 0; i < 30; ++i)
    {
      cv::Mat frame(120, 160, frame_type, cv::Scalar::all(0));

      // a moving bright band
      frame.colRange(i * 4, (i * 4) + 16).setTo(cv::Scalar::all(255));

      writer.write(frame);
    }

    writer.close();
  }

  xregASSERT(std::filesystem::exists(expected_path));
  xregASSERT(std::filesystem::file_size(expected_path) > 0);

  std::filesystem::remove(expected_path);
}

}  // un-named

int main(int argc, char* argv[])
{
  const std::string pid_str = std::to_string(getpid());

  const char* home_dir = std::getenv("HOME");
  xregASSERT(home_dir);

  // absolute path outside of the home directory
  {
    const std::string tmp_path = (std::filesystem::temp_directory_path() /
                                    ("xreg_avf_test_rgb_" + pid_str + ".mp4")).string();

    TestWriteVideo(tmp_path, tmp_path, CV_8UC3);
  }

  // absolute path in the home directory
  {
    const std::string home_path = std::string(home_dir) + "/.xreg_avf_test_gray_" + pid_str + ".mp4";

    TestWriteVideo(home_path, home_path, CV_8UC1);
  }

  // path starting with ~
  {
    const std::string file_name = ".xreg_avf_test_tilde_" + pid_str + ".mp4";

    TestWriteVideo("~/" + file_name, std::string(home_dir) + "/" + file_name, CV_8UC3);
  }

  std::cout << "PASSED" << std::endl;

  return 0;
}

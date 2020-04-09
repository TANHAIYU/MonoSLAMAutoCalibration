#ifndef FILEGRABBER_H
#define FILEGRABBER_H

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include "framegrabber.h"

using namespace std;

class FileGrabber {
 public:
  FileGrabber();
  ~FileGrabber();

  void Init(const string &path, FrameGrabber *frame_grabber, int init_frame_id, int max_frame);
  void SetFrameRange(int init_frame_id, int max_frame);
  void operator()();

 private:
  void ProcessFiles(const boost::filesystem::path &directory);
  cv::Mat GetImageFile(const string &file_full_path);

  vector<string>  files_vec_;
  // boost::thread   fg_thread_; by haiyu
  boost::thread   fg_thread_;

  bool            initialised_;
  int             frame_id_;
  int             max_frame_;
  int             init_frame_id_;
  FrameGrabber    *frame_grabber_;
};


#endif

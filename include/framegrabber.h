#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H

#include <queue>

#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>

using namespace std;

struct Frame_ {
  int     frame_id;
  cv::Mat data;
};

class FileGrabber;
// class UsbCamGrabber; by haiyu

class FrameGrabber {
 public:
  FrameGrabber();
  ~FrameGrabber();

  void Init(const string &dev, const bool mode, int init_frame_id, int max_frame);

  bool GetFrame(int frame_id, Frame_ *frame);
  void SetFrame(const Frame_ &frame);
  bool IsFrameBufferFull();

 private:
  queue<Frame_>  frame_buffer_; //by haiyu
  boost::mutex  fg_mutex_;      // by haiyu

  FileGrabber   *file_grabber_;
//  UsbCamGrabber *usb_cam_grabber_;  by haiyu
};

#endif // FRAMEGRABBER_H

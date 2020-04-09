#include "framegrabber.h"
#include "filegrabber.h"
//#include "usbcamgrabber.h" by haiyu

FrameGrabber::FrameGrabber()
{
	file_grabber_ = NULL;
//	usb_cam_grabber_ = NULL; by haiyu
}

FrameGrabber::~FrameGrabber()
{
	if (file_grabber_ != NULL)
		delete  file_grabber_;

//	if (usb_cam_grabber_ != NULL)      by haiyu
//		delete  usb_cam_grabber_;  by haiyu

	while (!frame_buffer_.empty()) {
		frame_buffer_.pop();
	}
}

void FrameGrabber::Init(const string &dev, const bool mode, int init_frame_id, int max_frame)
{
	if (!mode) {
		file_grabber_ = new FileGrabber;
		file_grabber_->Init(dev, this, init_frame_id, max_frame);
	}
	else {
//		usb_cam_grabber_ = new UsbCamGrabber; by haiyu
//		usb_cam_grabber_->Init(dev, this);    by haiyu
		std::cout<<"no video or frame pics file"<<endl;
	}
}

bool FrameGrabber::GetFrame(int frame_id, Frame_ *frame)
{
	boost::mutex::scoped_lock lock(fg_mutex_);

	if (frame_buffer_.size() < 1) {
		return  false;
	}

	*frame = frame_buffer_.front();
	//  assert(frame->frame_id == frame_id);
	frame_buffer_.pop();
	return  true;
}

void FrameGrabber::SetFrame(const Frame_ &frame)
{
	boost::mutex::scoped_lock lock(fg_mutex_);

	frame_buffer_.push(frame);
}

bool FrameGrabber::IsFrameBufferFull()
{
	boost::mutex::scoped_lock lock(fg_mutex_);

	if (frame_buffer_.size() < 10) {
		return  false;
	}

	return  true;
}



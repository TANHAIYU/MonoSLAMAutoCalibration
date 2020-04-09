#include "filegrabber.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>


FileGrabber::FileGrabber():init_frame_id_(0), max_frame_(0)
{
	initialised_ = false;
	frame_id_ = 0;
	frame_grabber_ = NULL;
}

FileGrabber::~FileGrabber()
{
	if(!files_vec_.empty()) {
		files_vec_.clear();
	}
	if(frame_grabber_ != NULL)
		delete frame_grabber_;
}

void FileGrabber::SetFrameRange(int init_frame_id, int max_frame)
{
	init_frame_id_ = init_frame_id;
	frame_id_ = init_frame_id_ - 1;
	max_frame_ = max_frame;
}


void FileGrabber::Init(const string &path, FrameGrabber *frame_grabber, int init_frame_id, int max_frame)
{
	ProcessFiles(path);
	frame_grabber_  = frame_grabber;
	initialised_    = true;
	SetFrameRange(init_frame_id, max_frame);
	fg_thread_ = boost::thread(boost::ref(*this));
	//fg_thread_.swap(boost::thread(boost::ref(*this)));
	//cout << "here is the first thread" << endl;
}

void FileGrabber::ProcessFiles(const boost::filesystem::path &directory)
{
	if (exists(directory)) {
		boost::filesystem::directory_iterator end;

		for (boost::filesystem::directory_iterator iter(directory); iter!=end; ++iter) {
			if (is_directory(*iter)) {
				ProcessFiles(*iter);
			} else {
				string  path_name = iter->path().string();
				/*cout << "path name is " << path_name <<endl;*/
				files_vec_.push_back(path_name);
			}
		}
		sort(files_vec_.begin(), files_vec_.end());
	}
}

void FileGrabber::operator ()()
{

	while (initialised_) {
		if (frame_grabber_->IsFrameBufferFull()==false) {
			if (files_vec_.size() > (unsigned int)frame_id_ && max_frame_ > frame_id_) {
				Frame_ frame;
				string  file_full_path = files_vec_.at(frame_id_);
				frame.frame_id = frame_id_;
				frame.data = GetImageFile(file_full_path);
				++frame_id_;
				frame_grabber_->SetFrame(frame);
				
			}
		}
		else {
			boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		}
	}
}

cv::Mat FileGrabber::GetImageFile(const string &file_full_path)
{
	return  cv::imread(file_full_path, 0); // reads the image without color
}



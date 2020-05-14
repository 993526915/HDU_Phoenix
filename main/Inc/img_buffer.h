/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify *  it under the terms of the GNU General Public License as published by *  the Free Software Foundation, either version 3 of the License, or *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef IMG_BUFFER_H_
#define IMG_BUFFER_H_

#include "iostream"
#include <vector>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

enum BufferState
{
	kIdle = 0,
	kWrite,
	kRead
};

class ImgBuffer
{
public:
	ImgBuffer();
	~ImgBuffer() = default;

	int GetImage(cv::Mat &src_img);
	void ReadComplete(int return_index);
	bool ImgEnterBuffer(cv::Mat &src);
	int get_headIdx()
	{
		return this->_headIdx;
	}

private:
	unsigned int buffer_size;
	std::vector<cv::Mat> image_buffer_;
	std::vector<BufferState> buffer_state_;
	int latest_index_;
	std::mutex lock_;

	std::vector<cv::Mat> _frames;
	std::vector<std::timed_mutex> _mutexs;

	int _tailIdx;
	int _headIdx;
};

#endif //IMG_BUFFER_H_

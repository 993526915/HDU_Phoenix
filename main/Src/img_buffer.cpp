
#include "iostream"
#include "string"
#include "log.h"
#include "img_buffer.h"

ImgBuffer::ImgBuffer() : _frames(6), _mutexs(6), _tailIdx(0), _headIdx(0)
{
    // buffer_size = 2;
    // image_buffer_.resize(buffer_size);
    // buffer_state_.resize(buffer_size);
    // for (int i = 0; i < buffer_state_.size(); ++i)
    // {
    //     buffer_state_[i] = BufferState::kIdle;
    //     image_buffer_[i] = cv::Mat(640, 480, CV_8UC3, (0, 0, 0));
    // }
    // latest_index_ = -1;
}

int ImgBuffer::GetImage(cv::Mat &src)
{
    // if (latest_index_ < 0)
    // {
    //     return -1;
    // }
    // int temp_index = -1;
    // lock_.lock();
    // if (buffer_state_[latest_index_] == BufferState::kWrite)
    // {
    //     buffer_state_[latest_index_] = BufferState::kRead;
    // }
    // else
    // {
    //     lock_.unlock();
    //     return temp_index;
    // }
    // temp_index = latest_index_;
    // lock_.unlock();

    // src = image_buffer_[temp_index];
    // return temp_index;
    lock_.lock();
    volatile const size_t headIdx = _headIdx;

    //try for 2ms to lock
    // unique_lock<timed_mutex> lock(_mutexs[headIdx],chrono::milliseconds(2));
    // if(!lock.owns_lock() ||
    //    _frames[headIdx].img.empty() ||
    //    _frames[headIdx].timeStamp == _lastGetTimeStamp)
    // {
    //     return false;
    // }
    
    src = _frames[headIdx];
    // DLOG_WARNING << src.size();
    lock_.unlock();
    // _lastGetTimeStamp = _frames[headIdx].timeStamp;

    return true;
}

void ImgBuffer::ReadComplete(int return_index)
{

    if (return_index < 0 || return_index > (buffer_state_.size() - 1))
    {
        return;
    }

    buffer_state_[return_index] = BufferState::kIdle;
}

bool ImgBuffer::ImgEnterBuffer(cv::Mat &src)
{
    // std::cout << buffer_size << std::endl;
    // for (int i = 0; i < buffer_state_.size(); ++i)
    // {
    //     LOG_WARNING<<"QQQQQ";
    //     if (buffer_state_[i] != BufferState::kRead)
    //     {
    //         LOG_ERROR << "11111";
    //         image_buffer_[i] = src.clone();
    //         // src.copyTo(image_buffer_[i]);
    //         buffer_state_[i] = BufferState::kWrite;
    //         lock_.lock();
    //         latest_index_ = i;
    //         lock_.unlock();
    //     }
    // }
    lock_.lock();
    const int newHeadIdx = (_headIdx + 1) % _frames.size();

    //try for 2ms to lock
    // unique_lock<timed_mutex> lock(_mutexs[newHeadIdx],chrono::milliseconds(2));
    // if(!lock.owns_lock())
    // {
    //     return false;
    // }
    DLOG_WARNING << src.size();
    _frames[newHeadIdx] = src;
    if (newHeadIdx == _tailIdx)
    {
        _tailIdx = (_tailIdx + 1) % _frames.size();
    }
    _headIdx = newHeadIdx;
    lock_.unlock();
    return true;
}

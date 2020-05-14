
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
    lock_.lock();
    const int newHeadIdx = (_headIdx + 1) % _frames.size();
    _frames[newHeadIdx] = src;
    if (newHeadIdx == _tailIdx)
    {
        _tailIdx = (_tailIdx + 1) % _frames.size();
    }
    _headIdx = newHeadIdx;
    lock_.unlock();
    return true;
}

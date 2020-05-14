
#include "iostream"
#include "string"
#include "log.h"
#include "img_buffer.h"

ImgBuffer::ImgBuffer() : _lock(6),_frames(6), _tailIdx(0), _headIdx(0)
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

bool ImgBuffer::GetImage(cv::Mat &src)
{
    std::unique_lock<std::mutex> lock(_lock[_headIdx],std::try_to_lock);
    if(!lock.owns_lock())
    {
        return false;
    }
    volatile const size_t headIdx = _headIdx;
    src = _frames[headIdx];
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
    const int newHeadIdx = (_headIdx + 1) % _frames.size();
    std::unique_lock<std::mutex> lock(_lock[newHeadIdx],std::try_to_lock);
    if(!lock.owns_lock())
    {
        return false;
    }
    _frames[newHeadIdx] = src;
    if (newHeadIdx == _tailIdx)
    {
        _tailIdx = (_tailIdx + 1) % _frames.size();
    }
    _headIdx = newHeadIdx;
    return true;
}

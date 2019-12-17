#ifndef __TOOL__H
#define __TOOL_H
#include<vector>
#include <thread>
#include <mutex>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
enum BufferState
{
  IDLE = 0,
  WRITE,
  READ
};
class cv_tool
{
public:
        explicit cv_tool(std::string camera_name,unsigned int buffer_size = 3) : get_img_info_(false)
        {
                image_buffer_.resize(buffer_size);
                buffer_state_.resize(buffer_size);
                index_ = 0;
                capture_time_ = 0;
                capture_time_=-1;
                for (int i = 0; i < buffer_state_.size(); i++)
                {
                        buffer_state_[i] = BufferState::IDLE;
                }
                latest_index_ = -1; 
        }
        void ImageCallback(const cv::Mat img)
        {
                if (!get_img_info_)
                {
                        capture_begin_ = std::chrono::high_resolution_clock::now();
                        get_img_info_ = true;
                }
                else
                {
                        capture_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>(std::chrono::high_resolution_clock::now() - capture_begin_).count();
                        capture_begin_ = std::chrono::high_resolution_clock::now();
                }
                capture_begin_ = std::chrono::high_resolution_clock::now();
                auto write_begin = std::chrono::high_resolution_clock::now();
                for (int i = 0; i < buffer_state_.size(); ++i)
                {
                        if (buffer_state_[i] != BufferState::READ)
                        {
                                // TODO 格式转换成mat到buff
                                image_buffer_[i]=img;
                                buffer_state_[i] = BufferState::WRITE;
                                lock_.lock();
                                latest_index_ = i;
                                lock_.unlock();
                        }
                }
                
        }
        int NextImage(cv::Mat &src_img)
        {
                if (latest_index_<0)
                {
                        std::cout<<"Call image when no image received"<<std::endl;
                        return -1;
                }
                int temp_index = -1;
                lock_.lock();
                if (buffer_state_[latest_index_] == BufferState::WRITE)
                {
                        buffer_state_[latest_index_] = BufferState::READ;
                }
                else
                {
                        std::cout<<"No image is availabel"<<std::endl;
                        lock_.unlock();
                        return temp_index;
                }
                temp_index = latest_index_;
                lock_.unlock();
                src_img = image_buffer_[temp_index];
                //cv::imshow("src_img",src_img);
                return temp_index;
        }
        void GetCaptureTime(double &capture_time)
        {
                if (!get_img_info_)
                {
                        std::cout<<"The first image doesn't receive"<<std::endl;
                        return;
                }
                if (capture_time_ < 0)
                {
                        std::cout<<"The second image doesn't receive"<<std::endl;
                        return;
                }
                capture_time = capture_time_;
        }
        void ReadComplete(int return_index)
        {

                if (return_index < 0 || return_index > (buffer_state_.size() - 1))
                {
                std::cout<<"Return index error, please check the return_index"<<std::endl;
                return;
                }
                buffer_state_[return_index] = BufferState::IDLE;
        }
private:
        std::vector<cv::Mat> image_buffer_;
        std::vector<BufferState> buffer_state_;
        int latest_index_;
        int index_;
        std::chrono::high_resolution_clock::time_point capture_begin_;//时钟
        double capture_time_;//时间
        bool get_img_info_;
        std::mutex lock_;
};

#endif
#include"frame.h"


namespace myslam{
    
            Frame::Frame(){
            }

            Frame::Frame(unsigned long id ,double time_stamp,const SE3 &pose,const cv::Mat &left_img,cv::Mat &right_img)
            :id(id),time_stamp(time_stamp),pose_(pose),left_img(left_img),right_img(right_img){}

            SE3 Frame::pose(){
                std::unique_lock<std::mutex> lck(pose_mutex);
                return pose_;
            }

            void Frame::SetKeyFrame(){
                is_keyframe = true;
                static unsigned keyframe_factory_id=0;
                keyframe_id = keyframe_factory_id++;//先赋值再+1，所以从0开始
            }

            void Frame::SetPose(const SE3& pose){
                std::unique_lock<std::mutex> lck(pose_mutex);
                pose_ = pose;
            }

            Frame::Ptr Frame::CreateFrame(){
                static unsigned long factory_id=0;
                Frame::Ptr new_frame(new Frame);//创建一个帧指针
                new_frame->id = factory_id++;
                return new_frame;//返回帧指针
            }
    
}
#pragma once

#ifndef FRAME_H
#define FRAME_H

#include"camera.h"
#include"common_include.h"  
#include"feature.h"

namespace myslam{

struct MapPoint;
struct Feature;

    struct Frame{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Frame> Ptr;

            unsigned long id=0;//帧id
            unsigned long keyframe_id=0;//关键帧id
            bool is_keyframe = false;//是否关键帧
            double time_stamp;//时间戳
            SE3 pose_;
            std::mutex pose_mutex;
            cv::Mat left_img,right_img;

            std::vector<std::shared_ptr<Feature>> features_left;
            std::vector<std::shared_ptr<Feature>> features_right;

            Frame();

            Frame(unsigned long id ,double time_stamp,const SE3 &pose,const cv::Mat &left_img,cv::Mat &right_img);

            SE3 pose();

            void SetPose(const SE3 &pose);

            void SetKeyFrame();

            static std::shared_ptr<Frame> CreateFrame();


    };
}


#endif

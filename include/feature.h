#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include"common_include.h"
#include<opencv2/features2d.hpp>
namespace myslam{

struct Frame;
struct MapPoint;

    struct Feature{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Feature> Ptr;

            std::weak_ptr<Frame> frame_;//持有该特征点的frame
            cv::KeyPoint kp_;//position_
            std::weak_ptr<MapPoint> mappoint_;

            bool is_outlier = false;
            bool is_on_left_img = true;

            Feature();
            Feature(std::shared_ptr<Frame> frame,const cv::KeyPoint &kp);
    };
}//namespace
#endif

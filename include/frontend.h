#pragma once
#ifndef FRONTEND_H
#define FRONTEND_H

#include"common_include.h"
#include"frame.h"
#include"map.h"
#include"g2o_types.h"
#include"viewer.h"
#include<opencv2/opencv.hpp>
#include"backend.h"
#include"config.h"
namespace myslam{
class Backend;
class Viewer;

enum class FrontendStatus{INITING,TRACKING_GOOD,TRACKING_BAD,LOST};//强制枚举，C++推荐使用

    class Frontend{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Frontend> Ptr;

            Frontend();
            bool AddFrame(Frame::Ptr frame);

            void SetMap(Map::Ptr map);

            void SetViewer(std::shared_ptr<Viewer> viewer);

            void SetBackend(std::shared_ptr<Backend> backend);

            FrontendStatus GetStatus() const;

            void SetCameras(Camera::Ptr left,Camera::Ptr right);

        private:
            bool Track();

            bool Reset();

            int TrackLastFrame();

            int EstimateCurrentPose();

            bool InsertKeyframe();

            bool StereoInit();

            int DetectFeatures();

            int FindFeaturesInRight();

            bool BuildInitMap();

            int TriangulateNewPoints();

            void SetObservationsForKeyFrame();

            FrontendStatus status_ = FrontendStatus::INITING;

            Frame::Ptr current_frame_ = nullptr;

            Frame::Ptr last_frame_ = nullptr;

            Camera::Ptr camera_left_ = nullptr;

            Camera::Ptr camera_right_ = nullptr;

            Map::Ptr map_ = nullptr;

            std::shared_ptr<Backend> backend_ = nullptr;

            std::shared_ptr<Viewer> viewer_ = nullptr;

            SE3 relative_motion_;

            int tracking_inliers_ = 0;

            int num_features_=200;

            int num_features_init_=100;

            int num_features_tracking_=50;

            int num_features_tracking_bad_=20;
            
            int num_features_needed_for_keyframe_=80;

            cv::Ptr<cv::GFTTDetector> gftt_;
            
            inline bool triangulation(const std::vector<SE3> &poses,const std::vector<Vec3> &points,Vec3 &pworld);

    };//class
}//namespace


#endif

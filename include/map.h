#pragma once
#ifndef MAP_H
#define MAP_H

#include"common_include.h"
#include"feature.h"
#include"mappoint.h"
#include"frame.h"

namespace myslam{
    class Map {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Map> Ptr;
            typedef std::unordered_map<unsigned long,MapPoint::Ptr> LandmarksType;//hasing
            typedef std::unordered_map<unsigned long,Frame::Ptr> KeyframesType;

            Map();
            //增加关键帧
            void InsertKeyFrame(Frame::Ptr frame);
            //增加一个地图顶点
            void InsertMapPoint(MapPoint::Ptr mappoint);
            //获取所有地图点
            LandmarksType GetAllMapPoints();    
            //获取所有关键帧
            KeyframesType GetAllKeyFrames();
            //获取激活地图点
            LandmarksType GetActiveMapPoints();
            //获取激活关键帧
            KeyframesType GetActiveKeyFrames();
            //清理观测数量为零的点
            void CleanMap();

        private:
            void RemoveOldKeyFrame();
            std::mutex data_mutex;
            LandmarksType landmarks_;
            LandmarksType active_landmarks_;
            KeyframesType Keyframes_;
            KeyframesType active_Keyframes_;

            Frame::Ptr current_frame = nullptr;
            
            unsigned int num_active_keyframes_=7;

    };//class Map
}//namespace


#endif

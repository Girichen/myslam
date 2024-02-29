#pragma once

#ifndef BACKEND_H
#define BACKEND_H

#include"common_include.h"
#include"frame.h"
#include"map.h"
namespace myslam{
class Camera;
class Map;
    class Backend{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Backend> Ptr;

            Backend();

            void SetMap(Map::Ptr map);

            void SetCameras(Camera::Ptr left,Camera::Ptr right);

            void UpdateMap();

            void Stop();

        private:
            void BackendLoop();

            void Optimize(Map::KeyframesType &keyframes,Map::LandmarksType &landmarks);

            Map::Ptr map_;

            std::thread backend_thread_;//线程

            std::mutex data_mutex_;//互斥锁

            std::condition_variable map_update_;//条件变量，它用于在多线程环境中实现线程之间的通知和等待

            std::atomic<bool> backend_running_;//std::atomic确保共享变量在不同线程中原子化，就是这个线程访问这个共享变量是，别的线程不访问

            Camera::Ptr cam_left_ = nullptr;

            Camera::Ptr cam_right_ = nullptr;
            
    };//class Backend
}//myslam


#endif

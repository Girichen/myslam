#include"backend.h"

namespace myslam{

    Backend::Backend(){
        backend_running_.store(true);//保证操作原子性，和backend_running_=true有区别。atomic类的store函数
        std::thread backend_thread_(std::bind(&Backend::BackendLoop,this));
    }

    void Backend::SetCameras(Camera::Ptr left,Camera::Ptr right){
        cam_left_ = left ;
        cam_right_ = right ;
    }//SetCameras
        

    void Backend::SetMap(Map::Ptr map){
        map_ = map ;
    }//SetMap
     
    void Backend::UpdateMap(){
        std::unique_lock<mutex> lock(data_mutex_);
        map_update_.notify_one();
    }//

    void Backend::Stop(){
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::BackendLoop(){
        //当后端运行时
        while(backend_running_.load()){
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            Map::KeyframesType active_kfs = map_ ->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_ ->GetActiveMapPoints();
            Optimize(active_kfs,active_landmarks);
        }
    }//BackendLoop

    void Backend::Optimize( Map::KeyframesType &active_kfs, Map::LandmarksType &active_landmarks){
        //setup g2o
    }//Optimize
}//myslam
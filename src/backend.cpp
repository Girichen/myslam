#include"backend.h"

namespace myslam{

    Backend::Backend(){
        backend_running_.store(true);
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
        
        
    }
}//myslam
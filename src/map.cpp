#include"map.h"

namespace myslam{
    Map::Map(){}
    
    void Map::InsertKeyFrame(Frame::Ptr frame){
        current_frame = frame;
        if(Keyframes_.find(frame->keyframe_id) == Keyframes_.end()){
            Keyframes_.insert(make_pair(frame->keyframe_id,frame));
            active_Keyframes_.insert(make_pair(frame->keyframe_id,frame));
        }else{
            Keyframes_[frame->keyframe_id] = frame;
            active_Keyframes_[frame->keyframe_id]=frame;
        }

        if(active_Keyframes_.size()>num_active_keyframes_){
            RemoveOldKeyFrame();
        }
    }
    void Map::RemoveOldKeyFrame(){
        if(current_frame == nullptr) return;
        double max_dis =0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id=0;
        auto Twc = current_frame->pose().inverse();
        for ( auto &kf : active_Keyframes_){
            if(kf.second == current_frame)continue;
            auto dis = (kf.second->pose()*Twc).log().norm();
            if(dis>max_dis){
                max_dis=dis;
                max_kf_id  = kf.first;
            }
            if(dis<min_dis){
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }
        
        const double min_dis_th = 0.2;
        Frame::Ptr frame_to_remove = nullptr;
        
        if(min_dis<min_dis_th){
            frame_to_remove = Keyframes_.at(min_kf_id);
        }else{
            frame_to_remove = Keyframes_.at(max_kf_id);
        }
        active_Keyframes_.erase(frame_to_remove->keyframe_id);
        
    for(auto feature : frame_to_remove->features_left){
        auto mp = feature->mappoint_.lock();
        
        if(mp){
            mp->RemoveObservation(feature);
            }
        }
    //error
    for(auto feature:frame_to_remove->features_right){
        
        if(feature = nullptr) 
            continue;
        auto mp = feature->mappoint_.lock();
        if(mp){
            
            mp->RemoveObservation(feature);
        }
   }
        
        CleanMap();
 }

       void Map::CleanMap(){
            int cnt_landmark_removed = 0;
            for(auto iter = active_landmarks_.begin();iter!=active_landmarks_.end();){
                if(iter->second->observed_times == 0){
                    iter = active_landmarks_.erase(iter);
                    cnt_landmark_removed++;
                }else{
                    ++iter;
                }
            }
       }//void Map::CleanMap     

        void Map::InsertMapPoint(MapPoint::Ptr mappoint){
            if(landmarks_.find(mappoint->id_)==landmarks_.end()){
                landmarks_.insert(make_pair(mappoint->id_,mappoint));
                active_landmarks_.insert(make_pair(mappoint->id_,mappoint));
            }else{
                landmarks_[mappoint->id_]=mappoint;
                active_landmarks_[mappoint->id_]=mappoint;
            }
        }//void Map::InesertMapPoint

        std::unordered_map<unsigned long,MapPoint::Ptr> Map::GetActiveMapPoints(){
            std::unique_lock<std::mutex> lock(data_mutex);
            return active_landmarks_;
        }// Map::GetActiveMapPoints()

        std::unordered_map<unsigned long,Frame::Ptr> Map::GetActiveKeyFrames(){
            std::unique_lock<std::mutex> lck(data_mutex);
            return active_Keyframes_;
        }// Map::GetActiveKeyFrames()

        std::unordered_map<unsigned long,MapPoint::Ptr> Map::GetAllMapPoints(){
            std::unique_lock<std::mutex> lck(data_mutex);
            return landmarks_;
        }// Map::GetAllMapPoints()

        std::unordered_map<unsigned long,Frame::Ptr> Map::GetAllKeyFrames(){
            std::unique_lock<std::mutex> lck(data_mutex);
            return Keyframes_;
        }// Map::GetAllKeyFrames()
}//myslam




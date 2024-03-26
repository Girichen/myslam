#include"mappoint.h"

namespace myslam{
    MapPoint::MapPoint(){}

    MapPoint::MapPoint(unsigned long id,Vec3 &pos)
    :id_(id),pos_(pos){}

    Vec3 MapPoint::Pos(){
        std::unique_lock<std::mutex> lock(data_mutex);
       return pos_;  
    }

    void MapPoint::SetPos(const Vec3 &pos){
        std::unique_lock<std::mutex> lck(data_mutex);
        pos_=pos;
    }

    void MapPoint::AddObservation(std::shared_ptr<Feature> feature){
        std::unique_lock<std::mutex> lck(data_mutex);
        observations_.push_back(feature);
        observed_times++;
        lck.unlock();
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature){
        std::unique_lock<std::mutex> lck(data_mutex);
        for(auto iter = observations_.begin();iter != observations_.end();iter++){
            if(iter->lock() == feature){
                observations_.erase(iter);
                feature->mappoint_.reset();
                observed_times--;
                break;
            }
        }//for
    }

    std::list<std::weak_ptr<Feature>> MapPoint::GetObs(){
        std::unique_lock<std::mutex> lck(data_mutex);
        return observations_;
    }

    MapPoint::Ptr MapPoint::CreateNewMappoint(){
        static unsigned long factory_id = 0;//static的话在该类公用
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++;
        return new_mappoint;
    }


}//namespace
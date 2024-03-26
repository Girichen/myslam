#pragma once
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"common_include.h"
#include"feature.h"
namespace myslam{

struct Feature;
struct Frame;

    struct MapPoint{
            public:
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<MapPoint> Ptr;

            unsigned long id_ = 0;
            bool is_outlier = false;
            Vec3 pos_ = Vec3::Zero();
            std::mutex data_mutex;
            int observed_times = 0;
            std::list<std::weak_ptr<Feature>> observations_;
            
            MapPoint();

            MapPoint(unsigned long id, Vec3 &pos);

            Vec3 Pos();

            void SetPos(const Vec3 &pos);

            void AddObservation(std::shared_ptr<Feature> feature);

            void RemoveObservation(std::shared_ptr<Feature> feature);

            std::list<std::weak_ptr<Feature>> GetObs();

            static MapPoint::Ptr CreateNewMappoint();
    };
}//namespace


#endif

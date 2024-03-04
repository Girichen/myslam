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
        
        typedef g2o::BlockSolver_6_3  BlockSolverType;
        
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
        
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        //稀疏优化器
        g2o::SparseOptimizer optimizer;
        //设置求解器
        optimizer.setAlgorithm(solver);

        //pose顶点
        std::map<unsigned long,VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for(auto &keyframe : active_kfs){
            auto kf = keyframe.second;//关键帧（id，指针),这里second是指针,即关键帧
            VertexPose * vertex_pose = new VertexPose();

            vertex_pose->setId(kf->keyframe_id);
            vertex_pose->setEstimate(kf->pose());
            optimizer.addVertex(vertex_pose);
            if(kf->keyframe_id>max_kf_id){
                max_kf_id = kf->keyframe_id;
            }
        vertices.insert({kf->keyframe_id,vertex_pose});
        }//for
        //landmark顶点
        std::map<unsigned long , VertexXYZ *> vertices_landmarks;

        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();


        //edge
        int index = 1;
        double chi2_th = 5.991;//robust kernel
        std::map<EdgeProjection *,Feature::Ptr> edges_and_features;

        for(auto &landmark :active_landmarks){
            if(landmark.second->is_outlier) continue;
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetObs();
            for(auto &obs :observations){
                if(obs.lock()==nullptr)continue;
                auto feat = obs.lock();
                if(feat->is_outlier || feat->frame_.lock()==nullptr)continue;

                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;
                if(feat->is_on_left_img){
                    edge = new EdgeProjection(K,left_ext);
                }else{
                    edge = new EdgeProjection(K,right_ext);
                }//if

                if(vertices_landmarks.find(landmark_id)==vertices_landmarks.end()){
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id +max_kf_id +1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id,v});
                    optimizer.addVertex(v);
                }//if

                
            }//for

        }//for

    }//Optimize


}//myslam

//创建一个线性求解器LinearSolver。
//创建BlockSolver，并用上面定义的线性求解器初始化。
//创建总求解器solver，并从GN/LM/DogLeg 中选一个作为迭代策略，再用上述块求解器BlockSolver初始化。
//创建图优化的核心：稀疏优化器（SparseOptimizer）。
//定义图的顶点和边，并添加到SparseOptimizer中。
//设置优化参数，开始执行优化。

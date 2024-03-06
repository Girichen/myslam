#include"frontend.h"
//"frame.h"
//"map.h"
//"camera.h"
//"feature.h"
//"common_include.h"
//"opencv2/features2d.hpp"
//"g2o_types.h"
//"viewer.h"
namespace myslam{
    
        Frontend::Frontend(){
                gftt_= cv::GFTTDetector::create(Config::Get<int>("num_features"),0.01,20);
                num_features_init_ = Config::Get<int>("num_features_init");
                num_features_ = Config::Get<int>("num_features");
        }

        FrontendStatus Frontend::GetStatus() const {return status_;}
        
        void Frontend::SetMap(Map::Ptr map){map_ = map;}

        void Frontend::SetViewer(std::shared_ptr<Viewer> viewer){viewer_ = viewer;}

        void Frontend::SetBackend(std::shared_ptr<Backend> backend){backend_ = backend;}

        void Frontend::SetCameras(Camera::Ptr left,Camera::Ptr right){
                camera_left_ = left;
                camera_right_= right;
        }

        bool Frontend::AddFrame(Frame::Ptr frame){
                current_frame_ = frame;//将传入的帧设置为当前帧
                switch(status_){//查询当前帧的状态
                        case FrontendStatus::INITING://如果当前是初始化则调用双目初始化
                                StereoInit();
                                break;
                        case FrontendStatus::TRACKING_GOOD://跟踪
                                Track();
                                break;
                        case FrontendStatus::TRACKING_BAD:
                                Track();
                                break;
                        case FrontendStatus::LOST://重启
                                Reset();
                                break;
                }//switch
                last_frame_=current_frame_;//当前帧变为上一帧数
                return true;
        }//bool AddFrame()

        bool Frontend::Track(){
                //如果有上一帧，则计算该帧的pose
                if(last_frame_){
                        current_frame_->SetPose(relative_motion_ * last_frame_->pose());
                }
                //判断跟踪状态
                if(tracking_inliers_>num_features_tracking_){
                        status_ = FrontendStatus::TRACKING_GOOD;
                }else if(tracking_inliers_>num_features_tracking_bad_){
                        status_ = FrontendStatus::TRACKING_BAD;
                }else{
                        status_ = FrontendStatus::LOST;
                }
                //插入关键帧，该函数内部会判断是否需要关键帧
                InsertKeyframe();
                //计算相对运动
                relative_motion_ = current_frame_->pose() * last_frame_->pose().inverse();
                return true;
        }//bool Track() 

        bool Frontend::InsertKeyframe(){
                //判断是否需要关键帧
                if(tracking_inliers_>=num_features_needed_for_keyframe_){
                       return false;
                }//if
                //如果需要关键帧
                current_frame_->SetKeyFrame();//将当前帧设置为关键帧
                map_->Map::InsertKeyFrame(current_frame_);//在地图插入当前帧
                SetObservationsForKeyFrame();
                DetectFeatures();//对当前帧检测特征点
                FindFeaturesInRight();//在右图寻找特征点
                TriangulateNewPoints();//三角化特征点
                backend_->UpdateMap();//后端优化地图

                return true;
        }//bool InsertKeyframe()

        void Frontend::SetObservationsForKeyFrame(){
                //遍历当前帧所有左图特征点，Feature
                for(const auto &feature : current_frame_->features_left){
                        auto mp = feature->mappoint_.lock();
                        if(mp) mp->AddObservation(feature);
                }
        }//void SetObservationsForKeyFrame()

        int Frontend::TriangulateNewPoints(){
                std::vector<SE3> poses{camera_left_->pose(),camera_right_->pose()};
                SE3 current_pose_Twc = camera_left_->pose().inverse();
                int cnt_triangulated_pts =0;
                for(unsigned int i = 0;i<current_frame_->features_left.size();++i){
                        if(current_frame_->features_left[i]->mappoint_.expired() &&
                        current_frame_->features_right[i] != nullptr){
                                std::vector<Vec3> points{
                                        camera_left_->pixel2camera(
                                                Vec2(current_frame_->features_left[i]->kp_.pt.x,
                                                     current_frame_->features_left[i]->kp_.pt.y)),
                                        camera_right_->pixel2camera(                               
                                                Vec2(current_frame_->features_right[i]->kp_.pt.x,
                                                     current_frame_->features_right[i]->kp_.pt.y))};
                                Vec3 pworld = Vec3::Zero();

                                if(triangulation(poses,points,pworld) && pworld[2]>0){
                                        auto new_mappoint = MapPoint::CreateNewMappoint();
                                        pworld = current_pose_Twc * pworld;
                                        new_mappoint->SetPos(pworld);
                                        new_mappoint->AddObservation(current_frame_->features_left[i]);
                                        new_mappoint->AddObservation(current_frame_->features_right[i]);
                                        current_frame_->features_left[i]->mappoint_ = new_mappoint;
                                        current_frame_->features_right[i]->mappoint_ = new_mappoint;
                                        map_->InsertMapPoint(new_mappoint);
                                        cnt_triangulated_pts++;
                                }//if
                        }//if
                }//for
                return cnt_triangulated_pts;
        }// int TriangulateNewPoints

        inline bool Frontend::triangulation(const std::vector<SE3> &poses,
                                const std::vector<Vec3> &points,
                                Vec3 &pworld){
                
                MatXX A(2 * poses.size(), 4);
                VecX b(2 * poses.size());
                b.setZero();
                for (size_t i = 0; i < poses.size(); ++i) {
                        Mat34 m = poses[i].matrix3x4();
                        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
                        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
                }
                auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
                pworld = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

                if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
                        // 解质量不好，放弃
                        return true;
                }
                return false;
        }//triangulation

        int Frontend::EstimateCurrentPose(){
                //初始化g2o
                typedef g2o::BlockSolver_6_3 BlockSolverType;
                typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSloverType;
                auto solver = new g2o::OptimizationAlgorithmLevenberg(
                        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSloverType>()));
                g2o::SparseOptimizer optimizer;
                optimizer.setAlgorithm(solver);
                
                //vertex
                VertexPose * vertexpose = new VertexPose();
                vertexpose->setId(0);
                vertexpose->setEstimate(current_frame_->pose());//设置估计量
                optimizer.addVertex(vertexpose);

                //K
                Mat33 K = camera_left_->K();

                //edges
                int index = 1;
                std::vector<EdgeProjectionPoseOnly *>edges;
                std::vector<Feature::Ptr> features;
                for(unsigned int i =0;i<current_frame_->features_left.size();++i){
                        auto mp = current_frame_->features_left[i]->mappoint_.lock();
                                if(mp){
                                        features.push_back(current_frame_->features_left[i]);
                                        EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_,K);
                                        edge->setId(index);
                                        cv::Point2f &p = current_frame_->features_left[i]->kp_.pt;
                                        edge->setMeasurement(Vec2(p.x,p.y));
                                        edge->setInformation(Eigen::Matrix2d::Identity());
                                        edge->setRobustKernel(new g2o::RobustKernelHuber);
                                        edges.push_back(edge);
                                        optimizer.addEdge(edge);
                                        index++;
                                }//if  
                }//for
                const double chi12_th = 5.991;
                int cnt_outlier=0;
                for(int iter = 0;iter<4;++iter){
                        vertexpose->setEstimate(current_frame_->pose());
                        optimizer.initializeOptimization();
                        optimizer.optimize(10);
                        cnt_outlier = 0;
                        for(unsigned long i =0;i<edges.size();++i){
                                auto e  = edges[i];
                                if(features[i]->is_outlier){
                                        e->computeError();
                                }
                                if(e->chi2()>chi12_th){
                                        features[i]->is_outlier = true;
                                        e->setLevel(1);
                                        cnt_outlier ++;
                                }else{
                                        features[i]->is_outlier = false;
                                        e->setLevel(0);
                                }
                                if(iter==2){
                                        e->setRobustKernel(nullptr);
                                }
                                
                        }//for
                }//for
                current_frame_->SetPose(vertexpose->estimate());
                for(auto &feat:features){
                        if(feat->is_outlier){
                                feat->mappoint_.reset();
                                feat->is_outlier = false;

                        }
                }//for
                return features.size() - cnt_outlier;
        }//Frontend::EstimateCurrentPose()

        bool Frontend::StereoInit(){
                
                int num_features_right = FindFeaturesInRight();
                if(num_features_right < num_features_init_){
                        return false;
                }
                if(BuildInitMap()){
                        status_ = FrontendStatus::TRACKING_GOOD;
                        if(viewer_){
                                viewer_->AddCurrentFrame(current_frame_);
                                viewer_->UpdateMap();
                        }
                        return true;
                }
                return false;
        }//bool StereoInit

        int Frontend::DetectFeatures(){
                cv::Mat mask(current_frame_->left_img.size(),CV_8UC1,255);
                for(auto &feat : current_frame_->features_left){
                cv::rectangle(mask,feat->kp_.pt - cv::Point2f(10,10),
                                feat->kp_.pt +cv::Point2f(10,10),0,CV_FILLED);

                }
                std::vector<cv::KeyPoint> keypoints;
                gftt_->detect(current_frame_->left_img,keypoints,mask);
                int cnt_detected =0;
                for(auto &kp:keypoints){
                        current_frame_->features_left.push_back(Feature::Ptr(new Feature(current_frame_,kp)));
                        cnt_detected++;
                }
                
               return cnt_detected;
        }

        int Frontend::FindFeaturesInRight(){
                std::vector<cv::Point2f>kps_left,kps_right;
                for(auto &kp:current_frame_->features_right){
                        kps_left.push_back(kp->kp_.pt);
                        auto mp = kp->mappoint_.lock();//weak_ptr需要这样用
                        if(mp){
                                auto px = camera_right_->world2pixel(mp->pos_,current_frame_->pose());
                                kps_right.push_back(cv::Point2f(px[0],px[1]));
                        }else{
                                kps_right.push_back(kp->kp_.pt);
                        }

                }
                std::vector<uchar> status;
                Mat error;
                
                cv::calcOpticalFlowPyrLK(current_frame_->left_img,current_frame_->right_img,kps_left,kps_right,status,error,
                                        cv::Size(11,11),3,
                                        cv::TermCriteria(cv::TermCriteria::COUNT +cv::TermCriteria::EPS,30,0.01),
                                        cv::OPTFLOW_USE_INITIAL_FLOW);
                
                int num_good_pts=0;
                for(unsigned long int i =0;i<status.size();++i){
                        if(status[i]){
                                cv::KeyPoint kp(kps_right[i],7);
                                Feature::Ptr feat(new Feature(current_frame_,kp));
                                feat->is_on_left_img = false;
                                current_frame_->features_right.push_back(feat);
                                ++num_good_pts;
                        }else{
                                current_frame_->features_right.push_back(nullptr);
                        }
                        
                }
                return num_good_pts;         
        }//FindFeaturesInRight

        bool Frontend::BuildInitMap(){
                std::vector<SE3> poses{camera_left_->pose(),camera_right_->pose()};
                unsigned long cnt_init_landmarks = 0;
                for(unsigned int i= 0;i<current_frame_->features_left.size();++i){
                        if(current_frame_->features_left[i] = nullptr) continue;
                        std::vector<Vec3> points{
                                        camera_left_->pixel2camera(
                                                Vec2(current_frame_->features_left[i]->kp_.pt.x,
                                                     current_frame_->features_left[i]->kp_.pt.y)),
                                        camera_right_->pixel2camera(                               
                                                Vec2(current_frame_->features_right[i]->kp_.pt.x,
                                                     current_frame_->features_right[i]->kp_.pt.y))};
                        Vec3 pworld = Vec3::Zero();
                        if(triangulation(poses,points,pworld) && pworld[2]>0){
                                        auto new_mappoint = MapPoint::CreateNewMappoint();
                                        new_mappoint->SetPos(pworld);
                                        new_mappoint->AddObservation(current_frame_->features_left[i]);
                                        new_mappoint->AddObservation(current_frame_->features_right[i]);
                                        current_frame_->features_left[i]->mappoint_ = new_mappoint;
                                        current_frame_->features_right[i]->mappoint_ = new_mappoint;
                                        cnt_init_landmarks++;
                                        map_->InsertMapPoint(new_mappoint);
                                }//if         
                }//for
                current_frame_->SetKeyFrame();
                map_->InsertKeyFrame(current_frame_);
                backend_->UpdateMap();
                return true;
        }//BuildInitMap

        bool Frontend::Reset(){
                
                return 0;
        }//Reset()

        int Frontend::TrackLastFrame(){
                //创建上一关键点、当前关键点
                std::vector<cv::Point2f> kps_last,kps_current;
                //遍历上一关键点，如果地图点存在就
                for(auto &kp : last_frame_->features_left){
                       if(auto mp = kp->mappoint_.lock()){
                                auto px = camera_left_->world2pixel(mp->pos_,current_frame_->pose());
                                kps_last.push_back(kp->kp_.pt);
                                kps_current.push_back(cv::Point2f(px[0],px[1]));
                        }else{
                                kps_last.push_back(kp->kp_.pt);
                                kps_current.push_back(kp->kp_.pt);
                        }//else
             }//for

             std::vector<uchar> status;
             cv::Mat error;
             cv::calcOpticalFlowPyrLK(
                        last_frame_->left_img,current_frame_->left_img,kps_last,
                        kps_current,status,error,cv::Size(11,11),3,
                        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,30,0.01),
                        cv::OPTFLOW_USE_INITIAL_FLOW
             );
             
             int num_good_pts = 0;

             for(size_t i = 0;i<status.size();++i){
                if(status[i]){
                        cv::KeyPoint kp(kps_current[i],7);
                        Feature::Ptr feature(new Feature(current_frame_,kp));
                        feature->mappoint_ = last_frame_->features_left[i]->mappoint_;
                        current_frame_->features_left.push_back(feature);
                        num_good_pts++;
                }//if
             }//for
             return num_good_pts;
        }//TrackLastFrame
}//namespace
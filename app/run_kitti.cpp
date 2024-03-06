#include "visual_odometry.h"

int main(int argc, char **argv){
    std::string config_path = argv[1];
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(config_path));
    
    vo->Init();
    vo->Run();
    return 0;
}
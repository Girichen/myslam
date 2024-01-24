#include"feature.h"

namespace myslam{
    Feature::Feature(){
    }

    Feature::Feature(std::shared_ptr<Frame> frame,const cv::KeyPoint &kp)
    :frame_(frame),kp_(kp){}


}//namespace
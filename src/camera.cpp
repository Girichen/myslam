#include"camera.h"

namespace myslam{

    Camera::Camera(){ 
    }

    Camera::Camera(double fx,double fy,double cx,double cy,double baseline,const SE3 &pose_)
                :fx(fx),fy(fy),cx(cx),cy(cy),baseline(baseline),pose_(pose_){
                    pose_inv_ = pose_.inverse();
                };

    SE3 Camera::pose() const{
        return pose_;
    }
    Mat33 Camera::K() const{
        Mat33 K;
        K<<fx,0,cx,0,fy,cy,0,0,1;
        return K;
    }

    Vec3 Camera::world2camera(const Vec3 &p_w,const SE3 &T_c_w){
        return pose_ *T_c_w *p_w;
    }

    Vec3 Camera::camera2world(const Vec3 &p_c,const SE3 &T_c_w){
        return T_c_w.inverse() * pose_inv_ *p_c;
    }

    Vec2 Camera::camera2pixel(const Vec3 &p_c){
        return Vec2(
            fx *p_c(0,0) / p_c(2,0)+cx,
            fy *p_c(1,0) / p_c(2,0)+cy
            );
    }

    Vec3 Camera::pixel2camera(const Vec2 &p_p,double depth){
        return Vec3(
            (p_p(0,0)-cx) * depth / fx,
            (p_p(1,0)-cy) * depth / fy,
            depth
        );
    }

    Vec2 Camera::world2pixel(const Vec3 &p_w,const SE3 &T_c_w){
        return camera2pixel(world2camera(p_w,T_c_w)); 
    }

    Vec3 Camera::pixel2world(const Vec2 &p_p,const SE3 &T_c_w,double depth){
        return camera2world(pixel2camera(p_p,depth),T_c_w);
    }   



}
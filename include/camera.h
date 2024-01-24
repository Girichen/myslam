#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include"common_include.h"

namespace myslam{

    class Camera{
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                typedef std::shared_ptr<Camera> Ptr;

                double fx=0,fy=0,cx=0,cy=0,baseline=0;
                SE3 pose_;
                SE3 pose_inv_;

                Camera();

                Camera(double fx,double fy,double cx,double cy,double baseline,const SE3 &pose_);
                
                SE3 pose() const;//const 使用函数时不改变自身成员变量的值
                
                Mat33 K() const;

                Vec3 world2camera(const Vec3&p_w,const SE3 &T_c_w);

                Vec3 camera2world(const Vec3 &p_c,const SE3 &T_c_w);

                Vec2 camera2pixel(const Vec3 &p_c);

                Vec3 pixel2camera(const Vec2 &p_p,double depth=1);
            
                Vec3 pixel2world(const Vec2 &p_p,const SE3 &T_c_w,double depth=1);

                Vec2 world2pixel(const Vec3 &p_w,const SE3 &T_c_w);

    };//class
}//namespace


#endif

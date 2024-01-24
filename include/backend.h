#pragma once

#ifndef BACKEND_H
#define BACKEND_H

#include"common_include.h"
namespace myslam{
    class Backend{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Backend> Ptr;

            Backend::Backend();
    }//class Backend
}//myslam


#endif

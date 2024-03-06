#pragma once 
#ifndef CONFIG_H
#define CONFIG_H
#include "common_include.h"
namespace myslam{
    class Config{
        private:
            static std::shared_ptr<Config> config_;
            
            cv::FileStorage file_;

            Config(){}
        public:
            ~Config();
            static bool SetParameterFile(const std::string &filename);

            template <typename T>
            static T Get(const std::string &key){
                return T(Config::config_->file_[key]);
            }
            

    };//config

}

#endif

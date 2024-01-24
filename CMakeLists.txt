cmake_minimum_required(VERSION 2.8)
project(myslam)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)


#直接包含.h文件
include_directories(
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/thirdparty/g2o
                    ${Sophus_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                     "/usr/include/suitesparse"#cs.h
                    )


add_library(${PROJECT_NAME} SHARED 
            src/camera.cpp
            src/frame.cpp
            src/feature.cpp
            src/mappoint.cpp
            src/map.cpp
            src/frontend.cpp
            src/viewer.cpp
            include/common_include.h
            include/camera.h
            include/frame.h
            include/feature.h
            include/mappoint.h
            include/map.h
            include/frontend.h
            include/viewer.h
            include/g2o_types.h
            )
target_link_libraries(${PROJECT_NAME}
                    ${OpenCV_LIBS}
                    ${EIGEN3_LIBS}
                    ${Pangolin_LIBRARIES}
                   
            )
            


add_executable(main src/main.cpp)
target_link_libraries(main  ${PROJECT_NAME})

#G2O库放在thridparty文件夹，编译然后只需要target就好了
#sudo apt-get install libsuitesparse-dev，解决没有cs.h文件
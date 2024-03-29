cmake_minimum_required(VERSION 2.8)
project(myslam)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/thirdparty/g2o/cmake_modules)
find_package(G2O REQUIRED)

include_directories(
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/thirdparty/g2o
                    ${Sophus_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                     "/usr/include/suitesparse"#cs.h
                    )

aux_source_directory(./src SRC_LIST)

file(GLOB g2olib ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/g2o/lib/*.so)
set(LIBS 
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        pthread
        ${EIGEN3_LIBS}
        ${g2olib}
        )

add_library(myslam SHARED ${SRC_LIST})
target_link_libraries(myslam ${LIBS})


add_subdirectory(app)

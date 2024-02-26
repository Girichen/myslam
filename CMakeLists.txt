cmake_minimum_required(VERSION 2.8)
project(myslam)
set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/thirdparty/g2o/cmake_modules)
find_package(G2O REQUIRED)

include_directories(
                    ${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/thirdparty/g2o
                    ${G2O_INCLUDE_DIRRS}
                    ${Sophus_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${G2O_INCLUDE_DIRS}
                     "/usr/include/suitesparse"#cs.h
                    )

aux_source_directory(./src SRC_LIST)
message(${SRC_LIST})
set(LIBS 
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${G2O_LIBRARIS}
        pthread
        )

add_library(myslam SHARED ${SRC_LIST})
target_link_libraries(myslam ${LIBS})

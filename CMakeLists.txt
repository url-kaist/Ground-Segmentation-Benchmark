cmake_minimum_required(VERSION 3.5.0) #3.0.2)
project(gseg_benchmark)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        dynamic_reconfigure
        pcl_conversions
        pcl_ros
        geometry_msgs
        sensor_msgs
        message_generation
        velodyne_pointcloud
        )

add_message_files(
        FILES
        node.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(classviz src/utils/viz_each_class.cpp include/common.hpp)
target_link_libraries(classviz ${catkin_LIBRARIES})
add_dependencies(classviz gseg_benchmark_generate_messages_cpp)

add_executable(estimateviz src/utils/viz_estimates.cpp include/common.hpp)
target_link_libraries(estimateviz ${catkin_LIBRARIES})
add_dependencies(estimateviz gseg_benchmark_generate_messages_cpp)

set(alg_source
    include/common.hpp
    include/gpf/groundplanefit.hpp
    include/r_gpf/r_gpf.hpp
    include/ransac/ransac_gpf.hpp
    include/patchwork/patchwork.hpp
    include/gpregression/GaussianFloorSegmentation.h
    include/cascadedseg/cascaded_groundseg.hpp
    include/linefit/bin.cc
    include/linefit/ground_segmentation.cc
    include/linefit/segment.cc
    )

add_executable(benchmark src/main_rosbag.cpp ${alg_source})
target_link_libraries(benchmark ${catkin_LIBRARIES} ${PCL_LIBRARIES} stdc++fs)
add_dependencies(benchmark gseg_benchmark_generate_messages_cpp)

add_executable(benchmark_offline src/main_offline.cpp ${alg_source})
target_link_libraries(benchmark_offline ${catkin_LIBRARIES} ${PCL_LIBRARIES} stdc++fs)
add_dependencies(benchmark_offline gseg_benchmark_generate_messages_cpp)


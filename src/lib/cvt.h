//
// Created by jeewon on 22. 3. 20..
//

#ifndef GSEG_BENCHMARK_CVT_H
#define GSEG_BENCHMARK_CVT_H

#include <pcl_conversions/pcl_conversions.h>

namespace cvt {
    template<typename T>
    pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
        pcl::PointCloud<T> cloudresult;
        pcl::fromROSMsg(cloudmsg,cloudresult);
        return cloudresult;
    }
    template<typename T>
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
    {
        sensor_msgs::PointCloud2 cloud_ROS;
        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.frame_id = frame_id;
        return cloud_ROS;
    }
}

#endif //GSEG_BENCHMARK_CVT_H

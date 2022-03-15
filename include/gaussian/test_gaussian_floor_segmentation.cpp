#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

#include "GaussianFloorSegmentation.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_up(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    // Fill in the cloud data
    pcl::PCDReader reader;

    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZI>("apollo.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << cloud->points.size() << std::endl;

    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    pcl::GaussianFloorSegmentationParams params;

    pcl::GaussianFloorSegmentation<pcl::PointXYZI> ground_segmentation{params};
    ground_segmentation.setInputCloud(cloud);

    ground_segmentation.setKeepGround(true);
    ground_segmentation.setKeepObstacle(false);
    ground_segmentation.setKeepOverHanging(false);
    ground_segmentation.filter(*cloud_filtered);

    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;


    auto endTime      = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "Ellapse-Time: " << ellapsedTime.count() << " milliseconds." << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>("apollo-utm_ground.pcd", *cloud_filtered, false);
    // writer.write<pcl::PointXYZI>("apollo-utm_noground.pcd", *cloud_filtered, false);

    return 0;
}
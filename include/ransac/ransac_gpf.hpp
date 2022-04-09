#ifndef RANSAC_GPF_H
#define RANSAC_GPF_H

#include "../common.hpp"

class RansacGPF {
public:
    RansacGPF() {};

    RansacGPF(ros::NodeHandle *nh) : node_handle_(*nh) {
        // Init ROS related
        ROS_INFO("Inititalizing Ransac GPF...");

        node_handle_.param("/sensor_height", sensor_height_, 1.723);

        // RANSAC PARAMETERS
        node_handle_.param("/ransac/th_dist", ransac_th_dist_, 0.01);

        ROS_INFO("Sensor Height: %f", sensor_height_);
        ROS_INFO("RANSAC dist threshold: %f", ransac_th_dist_);
        node_handle_.param("/ransac/margin", ransac_margin_, 0.3);
        ROS_INFO("RANSAC dist margin: %f", ransac_margin_);

        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_th_dist_);

        std::cout << "Target mode: \033[1;32m" << mode_ << "\033[0m" << std::endl;
        PlaneViz = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/gpf/plane", 100);
    }

    void estimate_ground(
            pcl::PointCloud<PointXYZILID> &cloudIn,
            pcl::PointCloud<PointXYZILID> &cloudOut,
            pcl::PointCloud<PointXYZILID> &cloudNonground,
            double &time_taken);
    void estimate_ground(
            pcl::PointCloud<PointXYZILID> &cloudIn,
            vector<int> &labels);

    geometry_msgs::PolygonStamped set_plane_polygon(const Eigen::Vector3f &normal_v, const float &d);

private:
    ros::NodeHandle                    node_handle_;
    double                             sensor_height_;
    int                                num_iter_;
    int                                num_lpr_;
    double                             th_seeds_;
    double                             th_dist_;
    std::string                        mode_;
    // For RANSAC
    pcl::SACSegmentation<PointXYZILID> seg;
    double                             ransac_th_dist_;
    double                             ransac_margin_;

    ros::Publisher PlaneViz;

    // void extract_ground_Ceres(const pcl::PointCloud<PointXYZILID>& cloudIn,
    //             pcl::PointCloud<PointXYZILID>& dst, pcl::PointCloud<PointXYZILID>& outlier);
    void extract_ground_RANSAC(
            const pcl::PointCloud<PointXYZILID> &cloudIn,
            pcl::PointCloud<PointXYZILID> &dst, pcl::PointCloud<PointXYZILID> &outlier);

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d 
    float    d_;
    MatrixXf normal_;
    float    th_dist_d_;
};

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void RansacGPF::estimate_ground(
        pcl::PointCloud<PointXYZILID> &cloudIn,
        pcl::PointCloud<PointXYZILID> &cloudOut,
        pcl::PointCloud<PointXYZILID> &cloudNonground,
        double &time_taken) {

    auto start = chrono::high_resolution_clock::now();

    extract_ground_RANSAC(cloudIn, cloudOut, cloudNonground);

    auto end = chrono::high_resolution_clock::now();
    time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;

    // Visualization of plane
    jsk_recognition_msgs::PolygonArray plane_marker;
    plane_marker.header.frame_id = "/map";
    plane_marker.header.stamp    = ros::Time::now();

    auto polygons = set_plane_polygon(normal_, d_);
    polygons.header = plane_marker.header;
    plane_marker.polygons.push_back(polygons);
    plane_marker.likelihood.push_back(BLUE_COLOR);
    PlaneViz.publish(plane_marker);
}

void RansacGPF::estimate_ground(
        pcl::PointCloud<PointXYZILID> &cloudIn,
        vector<int> &labels) {

    pcl::PointCloud<PointXYZILID> cloudOut;
    pcl::PointCloud<PointXYZILID> cloudNonground;
    int ground = 0;
    if (!labels.empty()) labels.clear();

    auto start = chrono::high_resolution_clock::now();
    extract_ground_RANSAC(cloudIn, cloudOut, cloudNonground);

    pcl::KdTreeFLANN<PointXYZILID> kdtree;
    std::vector<int> idxes;
    std::vector<float> sqr_dists;

    auto cloudGround = boost::make_shared<pcl::PointCloud<PointXYZILID>>(cloudOut);
    kdtree.setInputCloud(cloudGround);

    for (int i = 0; i<cloudIn.points.size(); i++) {
        PointXYZILID query = cloudIn.points[i];
        kdtree.nearestKSearch(query, 1, idxes, sqr_dists);
        if (sqr_dists[0]==0) labels.push_back(1);
        else labels.push_back(0);
    }

    auto end = chrono::high_resolution_clock::now();
}
void RansacGPF::extract_ground_RANSAC(
        const pcl::PointCloud<PointXYZILID> &cloudIn,
        pcl::PointCloud<PointXYZILID> &dst, pcl::PointCloud<PointXYZILID> &outlier) {
    pcl::ModelCoefficients::Ptr        coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr             inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::PointCloud<PointXYZILID>::Ptr cloud(new pcl::PointCloud<PointXYZILID>);
    *cloud = cloudIn;
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    static Eigen::Vector3f abc(3);
    abc(0) = coefficients->values[0];
    abc(1) = coefficients->values[1];
    abc(2) = coefficients->values[2];
    d_ = coefficients->values[3];

    dst.points.clear();
    outlier.points.clear();
    int outlier_idx          = 0;

    static Eigen::Vector3f xyz_point;
    // Note that inliers are sorted
    for (int               i = 0; i < inliers->indices.size(); ++i) {
        auto inlier_idx = inliers->indices[i];
        dst.points.push_back(cloud->points[inliers->indices[i]]);

        // Fill outliers!
        while (outlier_idx < inlier_idx) {
            xyz_point(0) = cloud->points[outlier_idx].x;
            xyz_point(1) = cloud->points[outlier_idx].y;
            xyz_point(2) = cloud->points[outlier_idx].z;
            if (abc.dot(xyz_point) + d_ < ransac_margin_) {
                dst.points.push_back(cloud->points[outlier_idx]);
            } else {
                outlier.points.push_back(cloud->points[outlier_idx]);
            }
            ++outlier_idx;
        }
        ++outlier_idx;
    }
    normal_ = abc;
}

geometry_msgs::PolygonStamped RansacGPF::set_plane_polygon(const Eigen::Vector3f &normal_v, const float &d) {
    // using (50, 50), (50, -50), (-50, 50), (-50, -50)
    const static vector<float>    xs = {50, 50, -50, -50};
    const static vector<float>    ys = {50, -50, -50, 50};
    geometry_msgs::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::Point32        point;

    for (uint8_t i = 0; i < 4; ++i) {
        float x = xs[i];
        float y = ys[i];
        point.x = x;
        point.y = y;
        // ax + by + cz = d;
        // => z = -(ax + by + d)/c
        point.z = -1 * (normal_v(0) * x + normal_v(1) * y + d) / normal_v(2);
        polygons.polygon.points.push_back(point);
    }
    return polygons;
}


#endif

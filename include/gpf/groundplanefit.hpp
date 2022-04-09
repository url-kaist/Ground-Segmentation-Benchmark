#ifndef GPF_H
#define GPF_H

#include "../common.hpp"

/*
 * This code is from below github page:
 * https://github.com/VincentCheungM/Run_based_segmentation
 */

#define NUM_TRISECTION 3
#define X_FRONT 20.0 // (80 + 80) / 3
#define X_REAR -20.0

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

typedef std::vector<pcl::PointCloud<PointXYZILID> > TrisectionPatches;

pcl::PointCloud<PointXYZILID>::Ptr g_seeds_pc(new pcl::PointCloud<PointXYZILID>());
pcl::PointCloud<PointXYZILID>::Ptr g_ground_pc(new pcl::PointCloud<PointXYZILID>());
pcl::PointCloud<PointXYZILID>::Ptr g_not_ground_pc(new pcl::PointCloud<PointXYZILID>());
//pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<SLRPointXYZIRL>());

bool point_cmp(PointXYZILID a, PointXYZILID b) {
    return a.z < b.z;
}

/*
    @brief Ground Plane fitting ROS Node.
    @param Velodyne Pointcloud topic.
    @param Sensor Model.
    @param Sensor height for filtering error mirror points.
    @param Num of segment, iteration, LPR
    @param Threshold of seeds distance, and ground plane distance
    
    @subscirbe:/velodyne_points
    @publish:/points_no_ground, /points_ground
*/
class GroundPlaneFit {
public:
    GroundPlaneFit() {};

    GroundPlaneFit(ros::NodeHandle *nh) : node_handle_(*nh) {
        // Init ROS related
        ROS_INFO("Inititalizing Ground Plane Fitter...");

        node_handle_.param("/sensor_height", sensor_height_, 1.723);
        ROS_INFO("Sensor Height: %f", sensor_height_);

        node_handle_.param<string>("/gpf/mode", mode_, "single");

        node_handle_.param("/gpf/num_iter", num_iter_, 3);
        ROS_INFO("Num of Iteration: %d", num_iter_);

        node_handle_.param("/gpf/num_lpr", num_lpr_, 20);
        ROS_INFO("Num of LPR: %d", num_lpr_);

        node_handle_.param("/gpf/th_seeds", th_seeds_, 1.2);
        ROS_INFO("Seeds Threshold: %f", th_seeds_);

        node_handle_.param("/gpf/th_dist", th_dist_, 0.3);
        ROS_INFO("Distance Threshold: %f", th_dist_);
        std::cout << "Mode: \033[1;32m " << mode_ << "\033[0m" << std::endl;

        if (mode_ == "multiple") {
            init_regionwise_patches();
            regionwise_ground_.reserve(100000);
            regionwise_nonground_.reserve(100000);
        }
        PlaneViz = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/gpf/plane", 100);

    }

    void estimate_ground(
            pcl::PointCloud<PointXYZILID> &cloudIn,
            pcl::PointCloud<PointXYZILID> &cloudOut,
            pcl::PointCloud<PointXYZILID> &cloudNonground,
            double &time_taken);

    void estimate_ground(
            const pcl::PointCloud<PointXYZILID> &cloudIn,
            vector<int> &labels);

    geometry_msgs::PolygonStamped set_plane_polygon(const MatrixXf &normal_v, const float &d);

    geometry_msgs::PolygonStamped set_plane_polygon4trisection(const MatrixXf &normal_v, const float &d, uint8_t i);

private:
    ros::NodeHandle node_handle_;
    double          sensor_height_;
    int             num_iter_;
    int             num_lpr_;
    double          th_seeds_;
    double          th_dist_;
    ros::Publisher  PlaneViz;
    string          mode_;

    // When mode is multiple;
    std::vector<std::pair<Eigen::MatrixXf, double> > normals_;
    pcl::PointCloud<PointXYZILID>                    regionwise_ground_;
    pcl::PointCloud<PointXYZILID>                    regionwise_nonground_;
    TrisectionPatches                                patches;

    void init_regionwise_patches();

    void clear_patches();

    void estimate_plane_(void);

    void extract_initial_seeds_(const pcl::PointCloud<PointXYZILID> &p_sorted);

    void pc2patches(const pcl::PointCloud<PointXYZILID> &src, TrisectionPatches &patches);

    void extract_initial_seeds_(
            const pcl::PointCloud<PointXYZILID> &p_sorted,
            pcl::PointCloud<PointXYZILID> &init_seeds);

    void extract_piecewiseground(
            const pcl::PointCloud<PointXYZILID> &src,
            pcl::PointCloud<PointXYZILID> &dst,
            pcl::PointCloud<PointXYZILID> &non_ground_dst);

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d 
    float                                            d_;
    MatrixXf                                         normal_;
    float                                            th_dist_d_;
};

void GroundPlaneFit::init_regionwise_patches() {
    pcl::PointCloud<PointXYZILID> cloud;
    for (int                      i = 0; i < NUM_TRISECTION; i++) {
        patches.emplace_back(cloud);
    }
}

void GroundPlaneFit::clear_patches() {
    for (int i = 0; i < NUM_TRISECTION; i++) {
        if (!patches[i].points.empty()) patches[i].points.clear();

    }
}

void GroundPlaneFit::pc2patches(const pcl::PointCloud<PointXYZILID> &src, TrisectionPatches &patches) {
    for (auto const &pt : src.points) {
        if (pt.x > X_FRONT) {
            patches[0].points.push_back(pt);
        } else if (pt.x < X_REAR) {
            patches[2].points.push_back(pt);
        } else {
            patches[1].points.push_back(pt);
        }

    }
}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated 
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.
    
*/
void GroundPlaneFit::estimate_plane_(void) {
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_         = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;

    // return the equation parameters
}


/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::

*/
void GroundPlaneFit::extract_initial_seeds_(const pcl::PointCloud<PointXYZILID> &p_sorted) {
    // LPR is the mean of low point representative
    double   sum        = 0;
    int      cnt        = 0;
    // Calculate the mean height value.
    for (int i          = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}


void GroundPlaneFit::extract_initial_seeds_(
        const pcl::PointCloud<PointXYZILID> &p_sorted,
        pcl::PointCloud<PointXYZILID> &init_seeds) {
    init_seeds.points.clear();

    // LPR is the mean of low point representative
//  std::cout<<"hey!! p sorted"<<p_sorted.points.size()<<std::endl;
    double sum = 0;
    int    cnt = 0;
    // Calculate the mean height value.

    for (int i          = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}


/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void GroundPlaneFit::estimate_ground(
        pcl::PointCloud<PointXYZILID> &cloudIn,
        pcl::PointCloud<PointXYZILID> &cloudOut,
        pcl::PointCloud<PointXYZILID> &cloudNonground,
        double &time_taken) {

    auto                          start = chrono::high_resolution_clock::now();
    // 1.Msg to pointcloud
    pcl::PointCloud<PointXYZILID> laserCloudIn;
    laserCloudIn = cloudIn;
    pcl::PointCloud<PointXYZILID> laserCloudIn_org = cloudIn;
    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
    // 3.Error point removal
    pcl::PointCloud<PointXYZILID>::iterator it = laserCloudIn.points.begin();
    for (int                                i  = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1.5 * sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    if (mode_ == "single") {
        // 4. Extract init ground seeds.
        extract_initial_seeds_(laserCloudIn);
        g_ground_pc = g_seeds_pc;

        // 5. Ground plane fitter mainloop
        for (int i = 0; i < num_iter_; i++) {
            estimate_plane_();
            g_ground_pc->clear();
            g_not_ground_pc->clear();

            //pointcloud to matrix
            MatrixXf  points(laserCloudIn_org.points.size(), 3);
            int       j            = 0;
            for (auto p:laserCloudIn_org.points) {
                points.row(j++) << p.x, p.y, p.z;
            }
            // ground plane model
            VectorXf  result       = points * normal_;
            // threshold filter
            int       num_inliers  = 0;
            int       num_outliers = 0;
            for (int  r            = 0; r < result.rows(); r++) {
                if (result[r] < th_dist_d_) {
                    //                g_all_pc->points[r].label = 1u;// means ground
                    g_ground_pc->points.push_back(laserCloudIn_org[r]);
                    //                ++num_inliers;
                } else {
                    //                g_all_pc->points[r].label = 0u;// means not ground and non clusterred
                    if (i == num_iter_ - 1) {
                        g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
                    }
                }
            }
        }
        cloudOut       = *g_ground_pc;
        cloudNonground = *g_not_ground_pc;
    } else if (mode_ == "multiple") {
        clear_patches();
        pc2patches(laserCloudIn, patches);
        normals_.clear();
        cloudOut.clear();
        cloudNonground.clear();

        for (uint16_t sector_idx = 0; sector_idx < NUM_TRISECTION; ++sector_idx) {

            if (!patches[sector_idx].empty()) {
                extract_piecewiseground(patches[sector_idx], regionwise_ground_, regionwise_nonground_);
                normals_.emplace_back(std::pair<Eigen::MatrixXf, double>(normal_, d_));
                cloudOut += regionwise_ground_;
                cloudNonground += regionwise_nonground_;
            }
        }
    }

    auto end = chrono::high_resolution_clock::now();
    time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;

    // Visualization of plane
    jsk_recognition_msgs::PolygonArray plane_marker;
    plane_marker.header.frame_id = "/map";
    plane_marker.header.stamp    = ros::Time::now();
    if (mode_ == "single") {
        auto polygons   = set_plane_polygon(normal_, d_);
        polygons.header = plane_marker.header;
        plane_marker.polygons.push_back(polygons);
        plane_marker.likelihood.push_back(BLUE_COLOR);
    } else if (mode_ == "multiple") {
        for (uint8_t i = 0; i < NUM_TRISECTION; ++i) {
            auto polygons = set_plane_polygon4trisection(normals_[i].first, normals_[i].second, i);
            polygons.header = plane_marker.header;
            plane_marker.polygons.push_back(polygons);
            plane_marker.likelihood.push_back(BLUE_COLOR);
        }
    }

    PlaneViz.publish(plane_marker);

}
void GroundPlaneFit::estimate_ground(
        const pcl::PointCloud<PointXYZILID> &cloudIn,
        vector<int> &labels) {

    auto start = chrono::high_resolution_clock::now();
    pcl::PointCloud<PointXYZILID> laserCloudIn;
    laserCloudIn = cloudIn;
    pcl::PointCloud<PointXYZILID> laserCloudIn_org = cloudIn;

    pcl::PointCloud<PointXYZILID> cloudOut;
    pcl::PointCloud<PointXYZILID> cloudNonground;

    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_cmp);
    pcl::PointCloud<PointXYZILID>::iterator it = laserCloudIn.points.begin();
    for (int i  = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1.5 * sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
    if (mode_ == "single") {
        extract_initial_seeds_(laserCloudIn);
        g_ground_pc = g_seeds_pc;
        for (int i = 0; i < num_iter_; i++) {
            estimate_plane_();
            g_ground_pc->clear();
            g_not_ground_pc->clear();

            MatrixXf points(laserCloudIn_org.points.size(), 3);
            int       j            = 0;
            for (auto p:laserCloudIn_org.points) {
                points.row(j++) << p.x, p.y, p.z;
            }
            VectorXf  result       = points * normal_;
            int       num_inliers  = 0;
            int       num_outliers = 0;
            for (int  r            = 0; r < result.rows(); r++) {
                if (result[r] < th_dist_d_) {
                    //                g_all_pc->points[r].label = 1u;// means ground
                    g_ground_pc->points.push_back(laserCloudIn_org[r]);
                    //                ++num_inliers;
                } else {
                    //                g_all_pc->points[r].label = 0u;// means not ground and non clusterred
                    if (i == num_iter_ - 1) {
                        g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
                    }
                }
            }
        }
        cloudOut       = *g_ground_pc;
        cloudNonground = *g_not_ground_pc;
    } else if (mode_ == "multiple") {
        clear_patches();
        pc2patches(laserCloudIn, patches);
        normals_.clear();
        cloudOut.clear();
        cloudNonground.clear();

        for (uint16_t sector_idx = 0; sector_idx < NUM_TRISECTION; ++sector_idx) {
            if (!patches[sector_idx].empty()) {
                extract_piecewiseground(patches[sector_idx], regionwise_ground_, regionwise_nonground_);
                normals_.emplace_back(std::pair<Eigen::MatrixXf, double>(normal_, d_));
                cloudOut += regionwise_ground_;
                cloudNonground += regionwise_nonground_;
            }
        }
    }
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

void GroundPlaneFit::extract_piecewiseground(
        const pcl::PointCloud<PointXYZILID> &src,
        pcl::PointCloud<PointXYZILID> &dst,
        pcl::PointCloud<PointXYZILID> &non_ground_dst) {
    // 0. Initialization
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();
    // 1. set seeds!
    extract_initial_seeds_(src);
    g_ground_pc = g_seeds_pc;
    // 2. Extract ground
    for (int i = 0; i < num_iter_; i++) {
        estimate_plane_();
        g_ground_pc->clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int             j = 0;
        for (auto &p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int        r      = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    g_ground_pc->points.push_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.push_back(src[r]);
                } else {
                    non_ground_dst.push_back(src[r]);

                }
            }
        }
    }
}

geometry_msgs::PolygonStamped GroundPlaneFit::set_plane_polygon(const MatrixXf &normal_v, const float &d) {
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

geometry_msgs::PolygonStamped GroundPlaneFit::set_plane_polygon4trisection(const MatrixXf &normal_v, const float &d, uint8_t i) {
    vector<float> xs, ys;
    if (i == 0) {
        xs = {70, 70, X_FRONT, X_FRONT};
        ys = {70, -70, -70, 70};
    } else if (i == 1) {
        xs = {X_FRONT, X_FRONT, X_REAR, X_REAR};
        ys = {70, -70, -70, 70};
    } else if (i == 2) {
        xs = {X_REAR, X_REAR, -70, -70};
        ys = {70, -70, -70, 70};
    }

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

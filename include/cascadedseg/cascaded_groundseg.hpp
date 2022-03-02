#ifndef CASCADESEG_H
#define CASCADESEG_H

/*
 * This code is from below github page:
 * https://github.com/n-patiphon/cascaded_ground_seg
 */

#include "../common.hpp"

struct PointXYZILIDR {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t label;                     ///< point label
    uint16_t id;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILIDR,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, label, label)
                                          (uint16_t, id, id)
                                          (uint16_t, ring, ring))

class CascadedGroundSeg {
public:
    CascadedGroundSeg() {};

    CascadedGroundSeg(ros::NodeHandle *nh) : node_handle_(*nh) {


        ROS_INFO("Initialization Cascaded Ground Segmentation...");

        node_handle_.param("/sensor_height", sensor_height_, 1.723);
        ROS_INFO("Sensor Height: %f", sensor_height_);

        node_handle_.param("/cascaded_gseg/sensor_model", sensor_model_, 64);
        ROS_INFO("Sensor Model: %d", sensor_model_);

        node_handle_.param("/cascaded_gseg/max_slope", max_slope_, 10.0);
        ROS_INFO("Max Slope: %f", max_slope_);

        node_handle_.param("/cascaded_gseg/vertical_thres", vertical_thres_, 0.08);
        ROS_INFO("Vertical Threshold: %f", vertical_thres_);

        node_handle_.param("/cascaded_gseg/remove_floor", floor_removal_, true);
        ROS_INFO("Floor Removal: %d", floor_removal_);

        node_handle_.param("/cascaded_gseg/plane_dis_thres", plane_dis_thres_, 0.3);
        ROS_INFO("Plane distance threhold: %f", plane_dis_thres_);

        node_handle_.param("/cascaded_gseg/n_section", n_section_, 4);
        ROS_INFO("Number of Sections: %d", n_section_);

        node_handle_.param("/cascaded_gseg/plane_height_thres", plane_height_thres_, 0.3);
        ROS_INFO("Height difference threshold: %f", plane_height_thres_);

        node_handle_.param("/cascaded_gseg/plane_ang_thres", plane_ang_thres_, 5.0);
        ROS_INFO("Angular difference threhold: %f", plane_ang_thres_);

        vertical_res_ = sensor_model_;

        // Calculate expected radius for each laser
        InitRadiusTable(sensor_model_);

        // Calculate section bounds
        GetSectionBounds();

        switch (sensor_model_) {
            case 64:
                horizontal_res_ = 2083;
                break;
            case 32:
                horizontal_res_ = 2250;
                break;
            case 16:
                horizontal_res_ = 1800;
                break;
            default:
                horizontal_res_ = DEFAULT_HOR_RES;
                break;
        }

        // Create index map
        index_map_ = new int *[vertical_res_];
        for (int i = 0; i < vertical_res_; i++) {
            index_map_[i] = new int[horizontal_res_];
        }

        // Create region index
        region_index_ = new std::vector<int> *[4];
        for (int i = 0; i < 4; i++) {
            region_index_[i] = new std::vector<int>[n_section_];
        }

        PlaneViz = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/cascadedgseg/plane", 100);

    }

    void estimate_ground(
            pcl::PointCloud<PointXYZILID> &cloudIn,
            pcl::PointCloud<PointXYZILID> &cloudOut,
            pcl::PointCloud<PointXYZILID> &cloudNonground,
            double &time_taken);

private:
    ros::NodeHandle node_handle_;
    ros::Publisher  PlaneViz;

    // Inter-ring filter parameters
    int    sensor_model_;
    double sensor_height_;
    double max_slope_;
    double vertical_thres_;

    // Multi-region plane fitting parameters
    double plane_dis_thres_;
    int    n_section_;
    double plane_height_thres_;
    double plane_ang_thres_;

    bool floor_removal_;

    time_t              t0_, t1_, t2_, t3_, t4_, start_, end_, total_t_;
    time_t              t5_, t6_, t7_, t8_, t9_;
    time_t              sps_;
    double              indexing_, extracting_, fill_regions_;
    double              quad_extract_, plane_seg_, rest_;
    int                 vertical_res_;
    int                 horizontal_res_;
    double              radius_table_[64];
    int                 **index_map_;
    std::vector<int>    **region_index_;
    std::vector<double> section_bounds_;
    const int           DEFAULT_HOR_RES = 2000;

    void PointXYZILID2XYZILIDR(
            pcl::PointCloud<PointXYZILID> &src,
            pcl::PointCloud<PointXYZILIDR> &dst);

    void PointXYZILIDR2XYZILID(
            pcl::PointCloud<PointXYZILIDR> &src,
            pcl::PointCloud<PointXYZILID> &dst);

    void SegmentGround(
            const pcl::PointCloud<PointXYZILIDR>::ConstPtr &pc_in,
            pcl::PointCloud<PointXYZILID> &nonground_out,
            pcl::PointCloud<PointXYZILID> &ground_out);


    void InitRadiusTable(int in_model);

    double RadiusCal(double theta, double alpha, double beta);

    void GetSectionBounds();

    void InitIndexMap();

    void FillIndexMap(const pcl::PointCloud<PointXYZILIDR>::ConstPtr &pc_in);

    void ColumnSegment(int i, const pcl::PointCloud<PointXYZILIDR>::ConstPtr &pc_in, std::vector<int> &v_ring);

    double EstimatedRad(int index_tar, int index_ref);

    void InitRegionIndex();

    void FillRegionIndex(pcl::PointCloud<PointXYZILID>::Ptr &ground_pc_in);

    int GetSection(double r);

    void SectionPlaneSegment(
            int i, pcl::PointCloud<PointXYZILID>::Ptr &ground_pc_in,
            pcl::PointCloud<PointXYZILID> &ground_pc_out,
            pcl::PointCloud<PointXYZILID>::Ptr &non_ground_pc_out);

    void PlaneSeg(
            int q, int s, pcl::PointCloud<PointXYZILID>::Ptr &region_pc_in,
            pcl::PointCloud<PointXYZILID> &ground_pc_out,
            pcl::PointCloud<PointXYZILID>::Ptr &non_ground_pc_out,
            pcl::ModelCoefficients::Ptr &prev_coefficients);

    bool isContinuous(int q, int s, pcl::ModelCoefficients::Ptr curr, pcl::ModelCoefficients::Ptr prev);

};

void CascadedGroundSeg::estimate_ground(
        pcl::PointCloud<PointXYZILID> &cloudIn,
        pcl::PointCloud<PointXYZILID> &cloudOut,
        pcl::PointCloud<PointXYZILID> &cloudNonground,
        double &time_taken) {


    pcl::PointCloud<PointXYZILIDR>::Ptr laserCloudIn(new pcl::PointCloud<PointXYZILIDR>);
    PointXYZILID2XYZILIDR(cloudIn, *laserCloudIn);

    pcl::PointCloud<PointXYZILID> nonground_points;
    pcl::PointCloud<PointXYZILID> ground_points;
    nonground_points.header = cloudIn.header;
    ground_points.header    = cloudIn.header;
    nonground_points.reserve(200000);
    ground_points.reserve(200000);

    auto start = chrono::high_resolution_clock::now();

    SegmentGround(laserCloudIn, nonground_points, ground_points);

    if (!floor_removal_) {
        PointXYZILIDR2XYZILID(*laserCloudIn, nonground_points);
    }
    cloudOut       = ground_points;
    cloudNonground = nonground_points;

    auto end = chrono::high_resolution_clock::now();
    time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;
}


void CascadedGroundSeg::SegmentGround(
        const pcl::PointCloud<PointXYZILIDR>::ConstPtr &pc_in,
        pcl::PointCloud<PointXYZILID> &nonground_out,
        pcl::PointCloud<PointXYZILID> &ground_out) {

    // Convert velodyne_pointcloud to PCL PointCloud
    pcl::PointCloud<PointXYZILID>::Ptr pc_in_noR(new pcl::PointCloud<PointXYZILID>);
    pcl::copyPointCloud(*pc_in, *pc_in_noR);

    ///////////////////////////////////////////
    //////////// Inter-ring filter ////////////
    ///////////////////////////////////////////
    indexing_     = 0;
    extracting_   = 0;
    fill_regions_ = 0;
    quad_extract_ = 0;
    plane_seg_    = 0;
    rest_         = 0;

    t0_ = clock();
    // Declare vectors to store non ground points index
    std::vector<int> v_ring;
    // Clear index map
    InitIndexMap();
    // Fill Index map
    FillIndexMap(pc_in);
    // Loop over columns

    for (int i = 0; i < horizontal_res_; i++) {
        // Segment each column of the index map separately
        ColumnSegment(i, pc_in, v_ring);
    }
    t1_ = clock();
    indexing_ += (double) (t1_ - t0_) / CLOCKS_PER_SEC;

    // Extract vectical points and remaining gruond points from the input cloud
    pcl::PointCloud<PointXYZILID>::Ptr remaining_ground(new pcl::PointCloud<PointXYZILID>);
    pcl::PointIndices::Ptr             ground_indices(new pcl::PointIndices);
    ground_indices->indices = v_ring;
    pcl::ExtractIndices<PointXYZILID> extract_ring_ground;
    extract_ring_ground.setInputCloud(pc_in_noR);
    extract_ring_ground.setIndices(ground_indices);
    extract_ring_ground.setNegative(true);
    extract_ring_ground.filter(*remaining_ground);
    extract_ring_ground.setNegative(false);
    extract_ring_ground.filter(nonground_out);

    t2_ = clock();
    extracting_ += (double) (t2_ - t1_) / CLOCKS_PER_SEC;
    ///////////////////////////////////////////////
    /////////// Quandrants plane fitting //////////
    ///////////////////////////////////////////////

    // Create empty pointcloud to store remaining vertical points
    pcl::PointCloud<PointXYZILID>::Ptr remaining_nonground(new pcl::PointCloud<PointXYZILID>);
    // Clear region index
    InitRegionIndex();
    // Fill region Index
    FillRegionIndex(remaining_ground);

    t3_ = clock();
    fill_regions_ += (double) (t3_ - t2_) / CLOCKS_PER_SEC;
    // Loop over Quandrants
    for (int i = 0; i < 4; i++) {
        // Segment each section separately
        SectionPlaneSegment(i, remaining_ground, ground_out, remaining_nonground);
    }

    //////////////////////////////////////////
    ////////////// Outputs merge /////////////
    //////////////////////////////////////////
    nonground_out += *remaining_nonground;


}

void CascadedGroundSeg::InitIndexMap() {
    for (int i = 0; i < vertical_res_; i++) {
        for (int j = 0; j < horizontal_res_; j++) {
            index_map_[i][j] = -1;
        }
    }
}

void CascadedGroundSeg::FillIndexMap(const pcl::PointCloud<PointXYZILIDR>::ConstPtr &pc_in) {
    for (size_t i = 0; i < pc_in->points.size(); i++) {
        double u = atan2(pc_in->points[i].y, pc_in->points[i].x) * 180 / M_PI;
        if (u < 0) { u = 360 + u; }

        int column = horizontal_res_ - (int) ((double) horizontal_res_ * u / 360.0) - 1;
        int row    = vertical_res_ - 1 - pc_in->points[i].ring;
        index_map_[row][column] = i;
    }
}

void CascadedGroundSeg::ColumnSegment(int i, const pcl::PointCloud<PointXYZILIDR>::ConstPtr &pc_in, std::vector<int> &v_ring) {
    int    index_ref = -1;
    double r_ref;
    double z_max;
    double z_min;

    std::vector<int> candidates;
    candidates.clear();

    for (int j = 0; j < vertical_res_; j++) {
        if (index_map_[j][i] != -1) {
            if (index_ref == -1) {

                index_ref = j;
                PointXYZILIDR point_ref = pc_in->points[index_map_[index_ref][i]];
                r_ref = sqrt(point_ref.x * point_ref.x + point_ref.y * point_ref.y);
                z_max = point_ref.z;
                z_min = point_ref.z;
                candidates.push_back(index_map_[index_ref][i]);

            } else {

                int           index_tar = j;
                PointXYZILIDR point_tar = pc_in->points[index_map_[index_tar][i]];
                double        r_tar     = sqrt(point_tar.x * point_tar.x + point_tar.y * point_tar.y);
                double        r_diff    = fabs(r_ref - r_tar);

                if (r_diff < EstimatedRad(index_tar, index_ref)) {

                    candidates.push_back(index_map_[index_tar][i]);
                    if (point_tar.z > z_max) { z_max = point_tar.z; }
                    if (point_tar.z < z_min) { z_min = point_tar.z; }
                    r_ref     = r_tar;
                    index_ref = index_tar;

                } else {
                    if ((candidates.size() > 1) && ((z_max - z_min) > vertical_thres_)) {
                        //append cadidates to v_ring
                        v_ring.insert(v_ring.end(), candidates.begin(), candidates.end());
                        candidates.clear();
                    } else {
                        candidates.clear();
                    }
                    candidates.push_back(index_map_[index_tar][i]);
                    z_max     = point_tar.z;
                    z_min     = point_tar.z;
                    r_ref     = r_tar;
                    index_ref = index_tar;

                }
            }
        }
    }

    if ((candidates.size() > 1) && ((z_max - z_min) > vertical_thres_)) {
        // append candidates to v_ring
        v_ring.insert(v_ring.end(), candidates.begin(), candidates.end());
        candidates.clear();
    }
}

double CascadedGroundSeg::EstimatedRad(int index_tar, int index_ref) {
    double   r = 0;
    for (int i = index_ref; i < index_tar; i++) {
        r += radius_table_[i];
    }
    return r;
}


void CascadedGroundSeg::InitRegionIndex() {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < n_section_; j++) {
            region_index_[i][j].clear();
        }
    }
}

void CascadedGroundSeg::FillRegionIndex(pcl::PointCloud<PointXYZILID>::Ptr &ground_pc_in) {
    for (size_t i = 0; i < ground_pc_in->size(); i++) {
        double y = ground_pc_in->points[i].y;
        double x = ground_pc_in->points[i].x;
        double r = sqrt(x * x + y * y);
        double u = atan2(y, x) * 180 / M_PI;
        if (u < 0) { u = 360 + u; }

        if (u >= 315) {
            //Quadrant 0
            int s = GetSection(r);
            region_index_[0][s].push_back(i);
        } else if (u >= 225) {
            //Quadrant 1
            int s = GetSection(r);
            region_index_[3][s].push_back(i);
        } else if (u >= 135) {
            //Quadrant 2
            int s = GetSection(r);
            region_index_[2][s].push_back(i);
        } else if (u >= 45) {
            //Quadrant 3
            int s = GetSection(r);
            region_index_[1][s].push_back(i);
        } else {
            //Quadrant 0
            int s = GetSection(r);
            region_index_[0][s].push_back(i);
        }
    }
}

int CascadedGroundSeg::GetSection(double r) {
    for (int i = 0; i < n_section_; i++) {
        if (r < section_bounds_[i]) {
            return i;
        }
    }
    return n_section_ - 1;
}

void CascadedGroundSeg::SectionPlaneSegment(
        int i, pcl::PointCloud<PointXYZILID>::Ptr &ground_pc_in, pcl::PointCloud<PointXYZILID> &ground_pc_out,
        pcl::PointCloud<PointXYZILID>::Ptr &non_ground_pc_out) {
    // Coefficients object to store coeffs of a valid plane
    pcl::ModelCoefficients::Ptr prev_coefficients(new pcl::ModelCoefficients);
    prev_coefficients->values.resize(4);


    // Loop through sections
    for (int j = 0; j < n_section_; j++) {
        t4_ = clock();

        // Select points that belong to the current region
        pcl::PointCloud<PointXYZILID>::Ptr region_cloud(new pcl::PointCloud<PointXYZILID>);
        pcl::PointIndices::Ptr             region_point_indices(new pcl::PointIndices);
        region_point_indices->indices = region_index_[i][j];

        pcl::ExtractIndices<PointXYZILID> extract_region_cloud;
        extract_region_cloud.setInputCloud(ground_pc_in);
        extract_region_cloud.setIndices(region_point_indices);
        extract_region_cloud.setNegative(false);
        extract_region_cloud.filter(*region_cloud);

        t5_ = clock();
        quad_extract_ += (double) (t5_ - t4_) / CLOCKS_PER_SEC;
        if (region_cloud->points.size() > 3) {
            t6_ = clock();
            PlaneSeg(i, j, region_cloud, ground_pc_out, non_ground_pc_out, prev_coefficients);
            t7_ = clock();
            plane_seg_ += (double) (t7_ - t6_) / CLOCKS_PER_SEC;
        } else if (region_cloud->points.size() > 0) {
            t8_                                                              = clock();
            // ROS_INFO("Manual");
            // segment using coefficients of the estimated plane from previous section

            // Temporary containers for plane segmentation results
            pcl::PointCloud<PointXYZILID>::Ptr tmp_ground(new pcl::PointCloud<PointXYZILID>);
            pcl::PointCloud<PointXYZILID>::Ptr tmp_vertical(new pcl::PointCloud<PointXYZILID>);

            // Plane segmedntation using known coefficients
            pcl::SampleConsensusModelPlane<PointXYZILID>::Ptr ref_plane(new pcl::SampleConsensusModelPlane<PointXYZILID>(region_cloud));
            Eigen::Vector4f                                   coefficients_v = Eigen::Vector4f(prev_coefficients->values[0],
                                                                                               prev_coefficients->values[1],
                                                                                               prev_coefficients->values[2],
                                                                                               prev_coefficients->values[3]);
            std::vector<int>                                  inliers_v;

            ref_plane->selectWithinDistance(coefficients_v, plane_dis_thres_, inliers_v);

            if (inliers_v.size() == 0) {
                *non_ground_pc_out += *region_cloud;
            } else if (inliers_v.size() == region_cloud->size()) {
                ground_pc_out += *region_cloud;
            } else {
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                inliers->indices = inliers_v;
                pcl::ExtractIndices<PointXYZILID> extract;
                extract.setInputCloud(region_cloud);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*tmp_ground);
                extract.setNegative(true);
                extract.filter(*tmp_vertical);

                //Merge Outputs
                ground_pc_out += *tmp_ground;
                *non_ground_pc_out += *tmp_vertical;
                t9_ = clock();
                rest_ += (double) (t9_ - t8_) / CLOCKS_PER_SEC;
            }
        }
    }
}

void CascadedGroundSeg::PlaneSeg(
        int q, int s, pcl::PointCloud<PointXYZILID>::Ptr &region_pc_in, pcl::PointCloud<PointXYZILID> &ground_pc_out,
        pcl::PointCloud<PointXYZILID>::Ptr &non_ground_pc_out, pcl::ModelCoefficients::Ptr &prev_coefficients) {
    // Temporary containers for plane segmentation results
    pcl::PointCloud<PointXYZILID>::Ptr tmp_ground(new pcl::PointCloud<PointXYZILID>);
    pcl::PointCloud<PointXYZILID>::Ptr tmp_vertical(new pcl::PointCloud<PointXYZILID>);

    // Fitting a plane
    pcl::ModelCoefficients::Ptr        coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr             inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointXYZILID> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(plane_dis_thres_);
    // Perform segmentation
    seg.setInputCloud(region_pc_in);
    seg.segment(*inliers, *coefficients);


    bool discontinuous = !isContinuous(q, s, coefficients, prev_coefficients);

    if ((inliers->indices.size() == 0) || discontinuous) {
        pcl::SampleConsensusModelPlane<PointXYZILID>::Ptr ref_plane(new pcl::SampleConsensusModelPlane<PointXYZILID>(region_pc_in));
        Eigen::Vector4f                                   coefficients_v = Eigen::Vector4f(prev_coefficients->values[0],
                                                                                           prev_coefficients->values[1],
                                                                                           prev_coefficients->values[2],
                                                                                           prev_coefficients->values[3]);
        std::vector<int>                                  inliers_v;
        ref_plane->selectWithinDistance(coefficients_v, plane_dis_thres_, inliers_v);

        inliers->indices     = inliers_v;
        coefficients->values = prev_coefficients->values;
    }

    if (inliers->indices.size() == region_pc_in->size()) {
        // In this condition, every point is ground
        ground_pc_out += *region_pc_in;
        // Update previous plane
        prev_coefficients->values = coefficients->values;
    } else if (inliers->indices.size() == 0) {
        // Cannot fit the plane at all
        *non_ground_pc_out += *region_pc_in;
        // Not update previous plane
    } else {
        // This is general condition. Some are ground, some are not
        // Extract the segmented points
        pcl::ExtractIndices<PointXYZILID> extract;
        extract.setInputCloud(region_pc_in);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*tmp_ground);
        extract.setNegative(true);
        extract.filter(*tmp_vertical);
        // Merge outputs
        ground_pc_out += *tmp_ground;
        *non_ground_pc_out += *tmp_vertical;
        // Update previous plane
        prev_coefficients->values = coefficients->values;
    }
}

bool CascadedGroundSeg::isContinuous(int q, int s, pcl::ModelCoefficients::Ptr curr, pcl::ModelCoefficients::Ptr prev) {
    if (s == 0) {
        return true;
    }

    // Vertice (x,y)
    double rad   = section_bounds_[s];
    double angle = 90.0 * q * M_PI / 180;
    double x     = rad * cos(angle);
    double y     = rad * sin(angle);

    // Check height
    bool   con_height = false;
    double height_gap = fabs(
            (prev->values[0] - curr->values[0]) * x + (prev->values[1] - curr->values[1]) * y + (prev->values[3] - curr->values[3]));
    con_height = height_gap <= plane_height_thres_;
    // ROS_INFO("Height gap: %f", height_gap);

    // Check angle
    bool   con_ang   = false;
    double angle_gap = prev->values[0] * curr->values[0] + prev->values[1] * curr->values[1] + prev->values[2] * curr->values[2];
    angle_gap = acos(angle_gap) * 180 / M_PI;
    con_ang   = angle_gap <= plane_ang_thres_;
    // ROS_INFO("Angle gap: %f", angle_gap);

    return (con_height && con_ang);
}


void CascadedGroundSeg::InitRadiusTable(int in_model) {

    double step          = 0;
    double initial_angle = 0;

    switch (in_model) {
        case 64:
            step          = 1.0 / 3.0;
            initial_angle = -2.0;
            break;
        case 32:
            step          = 4.0 / 3.0;
            initial_angle = -31.0 / 3.0;
            break;
        case 16:
            step          = 2.0;
            initial_angle = -15.0;
            break;
        default:
            step          = 1.0 / 3.0;
            initial_angle = -2.0;
            break;
    }

    double alpha = step / 180 * M_PI;
    double beta  = max_slope_ / 180 * M_PI;

    for (int i = 0; i < in_model; i++) {
        double theta = (i * step + initial_angle) / 180 * M_PI;
        if ((in_model == 64) && (i == 31)) {
            radius_table_[i] = sensor_height_ * RadiusCal(theta, -1.0 * alpha, beta);
            step          = 0.5;
            initial_angle = -15.0 + 8.83;
            alpha         = step / 180 * M_PI;
            beta          = max_slope_ / 180 * M_PI;
        } else {
            radius_table_[i] = sensor_height_ * RadiusCal(theta, alpha, beta);
        }
    }
}

double CascadedGroundSeg::RadiusCal(double theta, double alpha, double beta) {
    return fabs(1.0 / (tan(theta) + tan(beta))) - (1.0 / (tan(alpha + theta) + tan(beta)));
}

void CascadedGroundSeg::GetSectionBounds() {
    // Calculate lasers id which define section boundaries
    int      boundary_indices[n_section_];
    int      section_width = int(ceil(1.0 * vertical_res_ / n_section_));
    for (int i             = 0; i < n_section_; i++) {
        int new_ind = vertical_res_ - section_width * (i + 1);
        if (new_ind < 0) {
            boundary_indices[i] = 0;
        } else {
            boundary_indices[i] = new_ind;
        }
    }

    // Set up geometric parameters according to the sensor model
    double step          = 0;
    double initial_angle = 0;
    switch (sensor_model_) {
        case 64:
            step          = 1.0 / 3.0;
            initial_angle = -2.0;
            break;
        case 32:
            step          = 4.0 / 3.0;
            initial_angle = -31.0 / 3.0;
            break;
        case 16:
            step          = 2.0;
            initial_angle = -15.0;
            break;
        default:
            step          = 1.0 / 3.0;
            initial_angle = -2.0;
            break;
    }

    section_bounds_.clear();
    int      boundary_ind = n_section_ - 1;
    for (int i            = 0; i < sensor_model_; i++) {
        if (i == boundary_indices[boundary_ind]) {
            double theta = (i * step + initial_angle) / 180 * M_PI;
            if (theta != 0) {
                section_bounds_.insert(section_bounds_.begin(), sensor_height_ / tan(theta));
            } else {
                ROS_INFO("Please adjust number of sections: (n_section)");
                section_bounds_.insert(section_bounds_.begin(), sensor_height_ / tan(0.0001));
            }
            boundary_ind--;
            if (boundary_ind < 0) break;
        }
        if ((sensor_model_ == 64) && (i == 31)) {
            step          = 0.5;
            initial_angle = -15.0 + 8.83;
        }
    }
}


void CascadedGroundSeg::PointXYZILID2XYZILIDR(
        pcl::PointCloud<PointXYZILID> &src,
        pcl::PointCloud<PointXYZILIDR> &dst) {
    dst.points.clear();
    for (const auto &pt: src.points) {
        int    scanID   = 0;
        double distance = sqrt(pt.x * pt.x + pt.y * pt.y);
        if (distance <= 3.0 || distance >= 90.0) {
            continue;
        }
        double angle = atan(pt.z / distance) * 180 / M_PI;

        if (sensor_model_ == 16) {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (sensor_model_ - 1) || scanID < 0) { continue; }
        } else if (sensor_model_ == 32) {
            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
            if (scanID > (sensor_model_ - 1) || scanID < 0) { continue; }
        } else if (sensor_model_ == 64) {
            if (angle >= -8.83) { scanID = int((2 - angle) * 3.0 + 0.5); }
            else { scanID = sensor_model_ / 2 + int((-8.83 - angle) * 2.0 + 0.5); }
            if (scanID > 63 || scanID < 0) { continue; }
        } else { printf("wrong scan number\n"); }

        PointXYZILIDR pt_xyzilidr;
        pt_xyzilidr.x         = pt.x;
        pt_xyzilidr.y         = pt.y;
        pt_xyzilidr.z         = pt.z;
        pt_xyzilidr.intensity = pt.intensity;
        pt_xyzilidr.label     = pt.label;
        pt_xyzilidr.id        = pt.id;
        pt_xyzilidr.ring      = scanID;
        dst.points.push_back(pt_xyzilidr);
    }
}

void CascadedGroundSeg::PointXYZILIDR2XYZILID(
        pcl::PointCloud<PointXYZILIDR> &src,
        pcl::PointCloud<PointXYZILID> &dst) {
    dst.points.clear();
    for (const auto &pt: src.points) {
        PointXYZILID pt_xyzilid;
        pt_xyzilid.x         = pt.x;
        pt_xyzilid.y         = pt.y;
        pt_xyzilid.z         = pt.z;
        pt_xyzilid.intensity = pt.intensity;
        pt_xyzilid.label     = pt.label;
        pt_xyzilid.id        = pt.id;

        dst.points.push_back(pt_xyzilid);
    }
}

#endif

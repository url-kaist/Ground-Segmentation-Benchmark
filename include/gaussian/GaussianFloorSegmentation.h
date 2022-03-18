/**
 * @file GaussianFloorSegmentation.h
 * @author smallmunich (qqlpw@hotmail.com)
 * @brief Gaussian Process Cloud Segmentation.
 * @version 0.1
 * @date 2020-09-29
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef GAUSSIAN_FLOOR_SEGMENTATION_H_
#define GAUSSIAN_FLOOR_SEGMENTATION_H_


#include <math.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <vector>

#include <pcl/filters/filter_indices.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include "../common.hpp"

namespace pcl
{
/**
 * @addtogroup segmentation
 *
 * @{
 */

    struct SignalPoint
    {
        double range;
        double height;
        int    index;
        bool   is_ground;
    };

    struct LinCell
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        //
        std::vector<int> bin_indices;
        std::vector<int> ground_indices;

        int prototype_index;
        int cluster_assigned;
    };

    struct AngCell
    {
        std::vector<SignalPoint> sig_points;

        std::vector<LinCell, Eigen::aligned_allocator<LinCell>> lin_cell;

        std::vector<pcl::PointXY> range_height_signal;
    };

    struct PolarBinGrid
    {
        std::vector<AngCell, Eigen::aligned_allocator<AngCell>> ang_cells;
    };

/**
 * @brief Gaussian process-based ground segmentation filter for point cloud data.
 *
 * Based on Chen, Tongtong, et al. "Gaussian-process-based real-time ground
 * segmentation for autonomous land vehicles." Journal of Intelligent & Robotic
 * Systems 76.3-4 (2014)
 *
 * @tparam PointT
 */
    template <typename PointT>
    class GaussianFloorSegmentation : public pcl::Filter<PointT>
    {
    public:
        GaussianFloorSegmentation(){ };
        GaussianFloorSegmentation(ros::NodeHandle *nh) : node_handle_(*nh) {
            node_handle_.param("/sensor_height", sensor_height_, 1.723);
            node_handle_.param("/gaussian/rmax", rmax_, 100.0);

            node_handle_.param("/gaussian/max_bin_points",max_bin_points_, 200);
            node_handle_.param("/gaussian/num_seed_points", num_seed_points_, 10);

            node_handle_.param("/gaussian/num_bins_a", num_bins_a_, 72);
            node_handle_.param("/gaussian/num_bins_l", num_bins_l_, 200);

            node_handle_.param("/gaussian/p_l", p_l_, (float)30.0);
            node_handle_.param("/gaussian/p_sf", p_sf_, (float)1.0);
            node_handle_.param("/gaussian/p_sn", p_sn_, (float)0.3);
            node_handle_.param("/gaussian/p_tmodel", p_tmodel_, (float)5.0);
            node_handle_.param("/gaussian/p_tdata", p_tdata_, (float)5.0);
            node_handle_.param("/gaussian/p_tg", p_tg_, (float)0.3);

            node_handle_.param("/gaussian/robot_height", robot_height_, 1.723);
            node_handle_.param("/gaussian/max_seed_range", max_seed_range_, 50.0);
            node_handle_.param("/gaussian/max_seed_height", max_seed_height_, 3.0);

            this->initializePolarBinGrid();
            keep_ground_ = true;
        };
        void print_rosparam(ros::NodeHandle *nh){
            ROS_INFO("Sensor Height: %f", sensor_height_);
            ROS_INFO("Rmx: %f", rmax_);

            ROS_INFO("Max_Bin_Points: %d", max_bin_points_);
            ROS_INFO("Num_Seed_Points: %d", num_seed_points_);

            ROS_INFO("Num_Bin_a: %d", num_bins_a_);
            ROS_INFO("Num_Bin_l: %d", num_bins_l_);

            ROS_INFO("p_l: %f", p_l_);
            ROS_INFO("p_sf: %f", p_sf_);
            ROS_INFO("p_sn: %f", p_sn_);
            ROS_INFO("p_tmodel: %f", p_tmodel_);
            ROS_INFO("p_tdata: %f", p_tdata_);
            ROS_INFO("p_tg: %f", p_tg_);

            ROS_INFO("Robot Height: %f", robot_height_);
            ROS_INFO("max_seed_range: %f", max_seed_range_);
            ROS_INFO("max_seed_height: %f", max_seed_height_);
        }

        void estimate_ground(pcl::PointCloud<PointXYZILID> &cloudIn,
                             pcl::PointCloud<PointXYZILID> &cloudOut,
                             pcl::PointCloud<PointXYZILID> &cloudNonground,
                             double &time_taken);

        using PointCloud = typename pcl::Filter<PointT>::PointCloud;

        void setKeepGround(bool v)
        {
            this->keep_ground_ = v;
        }

        void setNegative(bool neg)
        {
            this->neg_ = neg;
        }
        void searchNonground(pcl::PointCloud<PointXYZILID> &cloudIn,
                             pcl::PointCloud<PointXYZILID> &cloudGround,
                             pcl::PointCloud<PointXYZILID> &cloudOut) ;

        void applyFilter(PointCloud& output); //override;

        void genPolarBinGrid();

        void initializePolarBinGrid();

        void sectorINSAC(int);

        Eigen::MatrixXd genGPModel(std::vector<SignalPoint>& ps1, std::vector<SignalPoint>& ps2, float sig_f, float p_l);

    private:
        ros::NodeHandle node_handle_;

        double rmax_;  // max radius of point to consider.

        int max_bin_points_;  // max number of points to consider per bin.
        int num_seed_points_;

        int num_bins_a_; // number of divided(by angle) bins. number_bins_divided_by_angle_
        int num_bins_l_; // number of divided(by range) bins. number_bins_divided_by_range_

        float p_l_;       // length parameter, how close points have to be in the GP model to correlate them
        float p_sf_;      // scaling on the whole covariance function
        float p_sn_;      // the expected noise for the mode
        float p_tmodel_;  // the required confidence required in order to consider
        float p_tdata_;   // scaled value that is required for a query point to be considered ground
        float p_tg_;      // ground height threshold

        double robot_height_; // Height the robot (m), used to distinguish "drivable" overhanging points
        double max_seed_range_;
        double max_seed_height_;

        double sensor_height_;

        PolarBinGrid     polar_bin_grid;
        std::vector<int> ground_indices;
        bool keep_ground_;
        bool neg_;
        bool org_;

    };

    double wrapTo360(double euler_angle)
    {
        if (euler_angle > 0)
        {
            return fmod(euler_angle, 360.0);
        }
        else
        {
            euler_angle += 360.0;
            return fmod(euler_angle, 360.0);
        }
    }

    bool compareSignalPoints(const SignalPoint& a, const SignalPoint& b)
    {
        return a.height < b.height;
    }

    template <typename PointT>
    void GaussianFloorSegmentation<PointT>::initializePolarBinGrid()
    {
        this->polar_bin_grid.ang_cells.clear();
        this->polar_bin_grid.ang_cells.resize(num_bins_a_);

        for (int i = 0; i < num_bins_a_; i++)
        {
            auto& ang_cell = this->polar_bin_grid.ang_cells[i];

            ang_cell.lin_cell.resize(num_bins_l_);
            ang_cell.sig_points.resize(num_bins_l_);
            ang_cell.range_height_signal.resize(num_bins_l_);

            for (int j = 0; j < num_bins_l_; j++)
            {
                ang_cell.lin_cell[j].prototype_index  = -1;
                ang_cell.range_height_signal[j].x     = NAN;
                ang_cell.range_height_signal[j].y     = NAN;
                ang_cell.lin_cell[j].cluster_assigned = -1;
            }
        }
    }

    template <typename PointT>
    void GaussianFloorSegmentation<PointT>::genPolarBinGrid()
    {
        this->initializePolarBinGrid();

        double     bsize_rad  = (double)((360.0) / num_bins_a_);
        double     bsize_lin  = (double)rmax_ / num_bins_l_;
        const auto num_points = this->input_->size();
        for (auto i = 0u; i < num_points; ++i)
        {
            const auto& cur_point = (*this->input_)[i];
            const auto& px        = cur_point.x;
            const auto& py        = cur_point.y;
            const auto& pz        = cur_point.z;

            if (sqrt(px * px + py * py + pz * pz) < rmax_)
            {
                double ph = (atan2(py, px)) * (180 / M_PI);  // in degrees
                ph        = wrapTo360(ph);

                // bin into sector
                unsigned int bind_rad = static_cast<unsigned int>(ph / bsize_rad);  // got the radial bin

                // get the linear bin
                float xy_dist = std::sqrt(px * px + py * py);

                unsigned int bind_lin = static_cast<unsigned int>(xy_dist / bsize_lin);  // got the radial bin

                this->polar_bin_grid.ang_cells[bind_rad].lin_cell[bind_lin].bin_indices.push_back(i);
                // add the point to the bin
                // check the prototype point
                auto& prototype_index = this->polar_bin_grid.ang_cells[bind_rad].lin_cell[bind_lin].prototype_index;
                if (prototype_index < 0 || pz < (*this->input_)[prototype_index].z)  // smallest by z
                {
                    prototype_index                                                          = i;
                    this->polar_bin_grid.ang_cells[bind_rad].range_height_signal[bind_lin].x = xy_dist;
                    this->polar_bin_grid.ang_cells[bind_rad].range_height_signal[bind_lin].y = pz;
                }
            }
        }
    }

    template <typename PointT>
    Eigen::MatrixXd GaussianFloorSegmentation<PointT>::genGPModel(std::vector<SignalPoint>& ps1,
                                                                  std::vector<SignalPoint>& ps2, float sig_f, float p_l)
    {
        size_t nP1 = ps1.size();
        size_t nP2 = ps2.size();

        Eigen::MatrixXd cmat(nP1, nP2);
        // cmat.resize(nP1, nP2);
        float coeff = (-1 / (2 * p_l * p_l));

        for (size_t i = 0; i < nP1; i++)
        {
            for (size_t j = 0; j < nP2; j++)
            {
                double diff = (ps1[i].range - ps2[j].range);
                cmat(i, j)  = sig_f * exp(coeff * (diff * diff));
            }
        }
        return cmat;
    }

    template <typename PointT>
    void GaussianFloorSegmentation<PointT>::sectorINSAC(int sector_index)
    {
        if (sector_index >= num_bins_a_)
        {
            return;
        }
        int num_filled = 0;

        // pull out the valid points from the sector
        auto& sig_points = this->polar_bin_grid.ang_cells[sector_index].sig_points;
        sig_points.clear();
        for (int i = 0; i < num_bins_l_; i++)
        {
            if (!std::isnan(this->polar_bin_grid.ang_cells[sector_index].range_height_signal[i].x) &&
                    this->polar_bin_grid.ang_cells[sector_index].lin_cell[i].bin_indices.size() > 5)
            {
                // bin has a valid point, and enough points to make a good
                // guess for a protopoint
                SignalPoint new_point;
                new_point.range  = this->polar_bin_grid.ang_cells[sector_index].range_height_signal[i].x;
                new_point.height = this->polar_bin_grid.ang_cells[sector_index].range_height_signal[i].y;
                new_point.index  = i;
                sig_points.push_back(new_point);
                num_filled++;
            }
        }
        // get the seed points.  Select the 3 lowest points.  Sort based on height values
        sort(sig_points.begin(), sig_points.end(), compareSignalPoints);
        // now that the z points are sorted by height, take the
        // this->params.num_seed_points worth as the seed
        size_t num_points = sig_points.size() < static_cast<size_t>(num_seed_points_) ?
                            sig_points.size() :
                            static_cast<size_t>(num_seed_points_);

        std::vector<SignalPoint> current_model;

        int  point_count      = 0;
        int  curr_idx         = 0;
        bool keep_going       = true;
        bool sufficient_model = true;

        while (true)
        {
            if (static_cast<size_t>(curr_idx) >= sig_points.size())  // overflow
            {
                break;
            }

            if (sig_points[curr_idx].range < max_seed_range_ &&
                fabs(sig_points[curr_idx].height) < max_seed_height_)
            {
                // close enough to robot and height make sense in robot locality
                sig_points[curr_idx].is_ground = true;
                current_model.push_back(sig_points[curr_idx]);
                sig_points.erase(sig_points.begin() + curr_idx);
                point_count++;
            }
            else
            {
                curr_idx++;
            }

            if (static_cast<size_t>(point_count) >= num_points)  // done
            {
                break;
            }
        }

        // check size
        if (current_model.size() < 2)  // not enough for model, all obs pts
        {
            keep_going       = false;
            sufficient_model = false;
        }

        // got the seedpoints, start theINSAC process cov matrices
        Eigen::MatrixXd C_XsX;
        Eigen::MatrixXd C_XX;
        Eigen::MatrixXd C_XsXs;
        Eigen::MatrixXd C_XXs;

        if (0 == sig_points.size())  // no points to insac, put the seed points in as ground
        {
            keep_going = false;
        }

        Eigen::MatrixXd temp;
        Eigen::MatrixXd f_s;
        Eigen::MatrixXd Vf_s;

        while (keep_going)
        {
            // generate the covariance matrices
            C_XsX  = genGPModel(sig_points, current_model, p_sf_, p_l_);
            C_XX   = genGPModel(current_model, current_model, p_sf_, p_l_);
            C_XsXs = genGPModel(sig_points, sig_points, p_sf_, p_l_);
            C_XXs  = C_XsX.transpose();

            // temporary calc
            Eigen::MatrixXd temp_calc1 = C_XX + (p_sn_ * Eigen::MatrixXd::Identity(C_XX.rows(), C_XX.cols()));
            Eigen::MatrixXd temp_calc2 = C_XsX * temp_calc1.inverse();

            // test the points against the current model
            Eigen::MatrixXd model_z(current_model.size(), 1);
            for (unsigned int i = 0; i < current_model.size(); ++i)
            {
                model_z(i, 0) = current_model[i].height;
            }

            f_s  = temp_calc2 * model_z;
            Vf_s = C_XsXs - temp_calc2 * C_XXs;

            if (0 == Vf_s.rows())
            {
                keep_going = false;
                std::cout << "WARNING BREAKING LOOP: VF_s does not exist" << std::endl;
                continue;
            }

            bool search_candidate_points = true;

            unsigned int k = 0;
            // test for inliers using INSAC algorithm
            const auto start_size = current_model.size();  // beginning size of the model set
            while (search_candidate_points)
            {
                double vf  = Vf_s(k, k);
                double met = (sig_points[k].height - f_s(k)) / (sqrt(p_sn_ + vf * vf));

                if (vf < p_tmodel_ && std::abs(met) < p_tdata_)
                {  // we have an inlier! add to model set
                    current_model.push_back(sig_points[k]);
                    // remove from sample set
                    sig_points.erase(sig_points.begin() + k);

                    // delete row from f_s
                    temp = f_s;
                    f_s.resize(f_s.rows() - 1, f_s.cols());
                    f_s.topRows(k)                      = temp.topRows(k);
                    f_s.bottomRows(temp.rows() - k - 1) = temp.bottomRows(temp.rows() - k - 1);

                    // delete row from Vf_s
                    temp = Vf_s;
                    Vf_s.resize(Vf_s.rows() - 1, Vf_s.cols());
                    Vf_s.topRows(k)                      = temp.topRows(k);
                    Vf_s.bottomRows(temp.rows() - k - 1) = temp.bottomRows(temp.rows() - k - 1);

                    // delete col from Vf_s
                    temp = Vf_s;
                    Vf_s.resize(Vf_s.rows(), Vf_s.cols() - 1);
                    Vf_s.leftCols(k)                    = temp.leftCols(k);
                    Vf_s.rightCols(temp.cols() - k - 1) = temp.rightCols(temp.cols() - k - 1);
                }
                else
                {
                    k++;
                }

                if (sig_points.size() == k)
                {
                    search_candidate_points = false;
                }
            }

            const auto end_size = current_model.size();  // end size of the model set
            if (start_size == end_size || 0 == sig_points.size())
            {
                keep_going = false;
            }
        }  // end INSAC

        for (int i = 0; i < (int)current_model.size(); i++)
        {
            int   currIdx  = current_model[i].index;
            auto& cur_cell = this->polar_bin_grid.ang_cells[sector_index].lin_cell[currIdx];

            // go through all the points in this cell and assign to ground/not ground
            for (const auto j : cur_cell.bin_indices)
            {
                const auto& cur_point = (*this->input_)[j];

                float h = std::abs(current_model[i].height - cur_point.z);
                if (h < p_tg_)  // z heights are close
                {
                    this->ground_indices.push_back(j);
                    cur_cell.ground_indices.push_back(j);
                }
            }
        }
    }

    template <typename PointT>
    void GaussianFloorSegmentation<PointT>::applyFilter(PointCloud& output)
    {
        // Do the work and fill the indices vectors
        this->genPolarBinGrid();
        for (int i = 0; i < num_bins_a_; ++i)
        {
            this->sectorINSAC(i);
        }
        // Copy the points the user wants
        std::vector<int> out_indices;
        out_indices = this->ground_indices;
        pcl::copyPointCloud(*this->input_, out_indices, output);
    }

    template <typename PointT>
    void GaussianFloorSegmentation<PointT>::estimate_ground(pcl::PointCloud<PointXYZILID> &cloudIn,
                                                            pcl::PointCloud<PointXYZILID> &cloudOut,
                                                            pcl::PointCloud<PointXYZILID> &cloudNonground,
                                                            double &time_taken)
    {
        pcl::PointIndicesPtr ground(new pcl::PointIndices);

        pcl::PointCloud<PointXYZILID>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZILID>);
        pcl::PointCloud<PointXYZILID>::Ptr cloud_nonground(new pcl::PointCloud<PointXYZILID>);

        pcl::PointCloud<PointXYZILID> nonground_points;
        pcl::PointCloud<PointXYZILID> ground_points;
        nonground_points.header = cloudIn.header;
        ground_points.header    = cloudIn.header;
        nonground_points.reserve(200000);
        ground_points.reserve(200000);

        // Create the filtering object
        pcl::GaussianFloorSegmentation<PointXYZILID> ground_segmentation(&node_handle_);

        auto start = chrono::high_resolution_clock::now();
        auto cloudInput = boost::make_shared<pcl::PointCloud<PointXYZILID>>(cloudIn);

        //search ground
        ground_segmentation.setInputCloud(cloudInput);
        ground_segmentation.setKeepGround(true);
        ground_segmentation.filter(*cloud_filtered);
        cloudOut = *cloud_filtered;

        auto end = chrono::high_resolution_clock::now();
        time_taken = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count()) / 1000000.0;

        //search non ground with kdtree flann
        ground_segmentation.searchNonground(cloudIn, cloudOut, cloudNonground);
    }

    template <typename PointT>
    void GaussianFloorSegmentation<PointT>::searchNonground(pcl::PointCloud<PointXYZILID> &cloudIn,
                                                            pcl::PointCloud<PointXYZILID> &cloudGround,
                                                            pcl::PointCloud<PointXYZILID> &cloudOut)
    {
        pcl::KdTreeFLANN<PointXYZILID> kdtree;
        std::vector<int> idxes_tmp;
        std::vector<int> idxes;
        std::vector<float> sqr_dists;

        int i;
        float th_dist = 0.001;

        auto cloudInput = boost::make_shared<pcl::PointCloud<PointXYZILID>>(cloudIn);
        kdtree.setInputCloud(cloudInput);

        for (const auto &basic_point: cloudGround.points) {
            idxes_tmp.clear();
            sqr_dists.clear();

            //using radiusSearch
            kdtree.radiusSearch(basic_point, th_dist, idxes_tmp, sqr_dists);
            for (auto i : idxes_tmp){
                idxes.push_back(i);
            }
            //using nearestKSearch
//            kdtree.nearestKSearch(basic_point, 5, idxes_tmp, sqr_dists);
//            for ( i = 0 ; i < idxes_tmp.size() ; i++ ) {
//                if ( sqr_dists[i] < th_dist ) {
//                    idxes.push_back(idxes_tmp[i]);
//                }
//            }
        }
        for ( i = 0 ; i < cloudIn.size() ; i++ ) {
            auto it = find(idxes.begin(),idxes.end(),i);
            if(it == idxes.end()){              //can't find index->non ground
                cloudOut.push_back(cloudIn.points[i]);
            }
        }

    }

}  // namespace pcl

#endif  //! GAUSSIAN_FLOOR_SEGMENTATION_H_
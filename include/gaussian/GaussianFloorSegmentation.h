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
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <vector>
#include "GaussianFloorSegmentationParams.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

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
    std::vector<int> obs_indices;
    std::vector<int> drv_indices;
    std::vector<int> ground_indices;

    int prototype_index;
    int cluster_assigned;

    Eigen::Vector3d obs_mean;
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
    using PointCloud = typename pcl::Filter<PointT>::PointCloud;

    void setKeepGround(bool v)
    {
        this->keep_ground_ = v;
    }

    void setKeepObstacle(bool v)
    {
        this->keep_obs_ = v;
    }

    void setKeepOverHanging(bool v)
    {
        this->keep_drv_ = v;
    }

    explicit GaussianFloorSegmentation(const GaussianFloorSegmentationParams& config);

    void applyFilter(PointCloud& output) override;

private:
    void genPolarBinGrid();

    void initializePolarBinGrid();

    void sectorINSAC(int);

    Eigen::MatrixXd genGPModel(std::vector<SignalPoint>& ps1, std::vector<SignalPoint>& ps2, float sig_f, float p_l);

private:
    GaussianFloorSegmentationParams params;

    PolarBinGrid     polar_bin_grid;
    std::vector<int> ground_indices;
    std::vector<int> obs_indices;
    std::vector<int> drv_indices;

    bool keep_ground_;
    bool keep_obs_;
    bool keep_drv_;
};

}  // namespace pcl

#endif  //! GAUSSIAN_FLOOR_SEGMENTATION_H_
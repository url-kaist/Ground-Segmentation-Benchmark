/**
 * @file GaussianFloorSegmentationParams.h
 * @author smallmunich (qqlpw@hotmail.com)
 * @brief Gaussian Process Cloud Segmentation.
 * @version 0.1
 * @date 2020-09-29
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef Gaussian_Floor_Segmentation_Params_H_
#define Gaussian_Floor_Segmentation_Params_H_

namespace pcl
{
struct GaussianFloorSegmentationParams
{
    // set default parameters
    GaussianFloorSegmentationParams()
    {
        rmax_            = 100;
        max_bin_points_  = 200;
        num_seed_points_ = 10;

        num_bins_a_ = 72;
        num_bins_l_ = 200;
        // GP model
        p_l_      = 30;
        p_sf_     = 1;
        p_sn_     = 0.3;
        p_tmodel_ = 5;
        p_tdata_  = 5;
        p_tg_     = 0.3;

        robot_height_    = 1.2;
        max_seed_range_  = 50;
        max_seed_height_ = 3;
    }

    GaussianFloorSegmentationParams(double rmax, int max_bin_points, int num_seed_points, int num_bins_a,
                                    int num_bins_l, float p_l, float p_sf, float p_sn, float p_tmodel, float p_tdata,
                                    float p_tg, double robot_height, double max_seed_range, double max_seed_height)
      : rmax_(rmax)
      , max_bin_points_(max_bin_points)
      , num_seed_points_(num_seed_points)
      , num_bins_a_(num_bins_a)
      , num_bins_l_(num_bins_l)
      , p_l_(p_l)
      , p_sf_(p_sf)
      , p_sn_(p_sn)
      , p_tmodel_(p_tmodel)
      , p_tdata_(p_tdata)
      , p_tg_(p_tg)
      , robot_height_(robot_height)
      , max_seed_range_(max_seed_range)
      , max_seed_height_(max_seed_height)
    {
    }

    double rmax_;  // max radius of point to consider.

    int max_bin_points_;  // max number of points to consider per bin.
    int num_seed_points_;

    int num_bins_a_; // number of devided(by angle) bins. number_bins_devided_by_angle_
    int num_bins_l_; // number of devided(by range) bins. number_bins_devided_by_range_

    float p_l_;       // length parameter, how close points have to be in the GP model to correlate them
    float p_sf_;      // scaling on the whole covariance function
    float p_sn_;      // the expected noise for the mode
    float p_tmodel_;  // the required confidence required in order to consider
    float p_tdata_;   // scaled value that is required for a query point to be considered ground
    float p_tg_;      // ground height threshold

    double robot_height_;  // Height the robot (m), used to distinguish "drivable" overhanging points
    double max_seed_range_;
    double max_seed_height_;
};

}  // namespace pcl

#endif  // WAVE_GROUNDSEGMENTATIONPARAMS_H
#include "GaussianFloorSegmentation.h"
#include "../../Ground-Segmentation-Benchmark/include/gaussian/GaussianFloorSegmentation.hpp"

#ifndef PCL_NO_PRECOMPILE
// Precompile our filter for common PCL point types
// See http://pointclouds.org/documentation/tutorials/writing_new_classes.php
#include <pcl/impl/instantiate.hpp>
PCL_INSTANTIATE(GaussianFloorSegmentation, PCL_XYZ_POINT_TYPES);
#endif  // PCL_NO_PRECOMPILE
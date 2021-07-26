#include "core/ndt_omp/include/pclomp/voxel_grid_covariance_omp.h"
#include "core/ndt_omp/include/pclomp/voxel_grid_covariance_omp_impl.hpp"

template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;
template class pclomp::VoxelGridCovariance<mapping::common::PointXYZIHIRBS>;

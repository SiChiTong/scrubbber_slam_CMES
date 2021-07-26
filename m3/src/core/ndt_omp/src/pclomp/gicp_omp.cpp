#include "core/ndt_omp/include/pclomp/gicp_omp.h"
#include "core/ndt_omp/include/pclomp/gicp_omp_impl.hpp"

template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::GeneralizedIterativeClosestPoint<mapping::common::PointXYZIHIRBS,
                                                        mapping::common::PointXYZIHIRBS>;

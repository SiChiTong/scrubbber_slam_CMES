#include "core/ndt_omp/include/pclomp/ndt_omp.h"
#include "core/ndt_omp/include/pclomp/ndt_omp_impl.hpp"
#include "common/mapping_point_types.h"

template class pcl::KdTreeFLANN<pcl::PointXYZ>;
template class pcl::KdTreeFLANN<pcl::PointXYZI>;
template class pcl::KdTreeFLANN<mapping::common::PointXYZIHIRBS>;

template class pcl::search::KdTree<pcl::PointXYZ>;
template class pcl::search::KdTree<pcl::PointXYZI>;
template class pcl::search::KdTree<mapping::common::PointXYZIHIRBS>;

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::NormalDistributionsTransform<mapping::common::PointXYZIHIRBS, mapping::common::PointXYZIHIRBS>;

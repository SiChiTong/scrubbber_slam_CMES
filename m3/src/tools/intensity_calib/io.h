#ifndef CALIBRATION_TOOLS_IO_H_
#define CALIBRATION_TOOLS_IO_H_

#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <string>

#include "common/mapping_point_types.h"
#include "tools/intensity_calib/common_struct.h"

namespace calibration {

Measurements LoadMeasurement(mapping::common::PointCloudXYZIHIRBS::Ptr& cloud, double voxel_size, double maximum_range);
// load beam mappings
void LoadMappings(const std::string& filename, BeamMappings& mappings);

bool SaveCloud(const std::string& filename, const mapping::common::PointCloudXYZIHIRBS& cloud);
// save beam mappings
void SaveMappings(const std::string& filename, const BeamMappings& mappings);

void SaveProbability(const std::string& filename, const BeamModel& beam_probs);

void SaveBeamInfo(const std::string& filename, const BeamInfo& beam_info);

inline int FindVoxel(pcl::VoxelGrid<mapping::common::PointXYZIHIRBS>& voxelgrid, float x, float y, float z) {
    return voxelgrid.getCentroidIndexAt(voxelgrid.getGridCoordinates(x, y, z));
}
}  // namespace calibration

#endif
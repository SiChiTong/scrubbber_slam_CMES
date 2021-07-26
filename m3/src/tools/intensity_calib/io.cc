#include "io.h"

namespace calibration {
// Cut the bull shit and assume its PointComplete
Measurements LoadMeasurement(mapping::common::PointCloudXYZIHIRBS::Ptr& cloud, double voxel_size,
                             double maximum_range) {
    // Put in bins
    mapping::common::PointCloudXYZIHIRBS::Ptr limited_cloud(new mapping::common::PointCloudXYZIHIRBS);
    for (auto& pt : cloud->points) {
        if (pt.range < maximum_range) {
            limited_cloud->push_back(pt);
        }
    }
    cloud.swap(limited_cloud);
    LOG(WARNING) << "remove points out of range";

    pcl::VoxelGrid<mapping::common::PointXYZIHIRBS> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    grid.setSaveLeafLayout(true);
    grid.setDownsampleAllData(false);
    mapping::common::PointCloudXYZIHIRBS::Ptr tmp_cloud(new mapping::common::PointCloudXYZIHIRBS);
    grid.filter(*tmp_cloud);

    pcl::io::savePCDFile("./data/cloud_to_calib.pcd", *tmp_cloud);

    // Now put in measurement
    Measurements results;
    for (const auto pt : cloud->points) {
        int a = static_cast<int>(pt.intensity);
        int b = static_cast<int>(pt.ring);
        int k = FindVoxel(grid, pt.x, pt.y, pt.z);
        if (a >= 100) {
            continue;
        }
        CHECK(k >= 0) << "Did not find corresponding voxel";
        CHECK(a < 257 && a >= 0) << "Measured intensity is wrong value";
        CHECK(b >= 0) << "Ring index is wrong";
        results.emplace_back(a, b, k);
    }
    LOG(INFO) << "Gathered measurements " << results.size();
    return results;
}

void LoadMappings(const std::string& filename, BeamMappings& mappings) {
    std::ifstream ifs(filename.c_str(), std::ios::in);
    if (!ifs) {
        LOG(ERROR) << "Failed to open mapping file: " << filename;
        return;
    }
    mappings = BeamMappings();
    CHECK(ifs.is_open());
    std::string line;
    getline(ifs, line);
    std::istringstream iss(line);
    int num_of_rings, size_of_mapping;
    iss >> num_of_rings;
    iss >> size_of_mapping;
    LOG(INFO) << " There are " << num_of_rings << " rings and correspondences " << size_of_mapping;

    for (int k = 0; k < num_of_rings; k++) {
        getline(ifs, line);
        std::istringstream iss(line);
        BeamMapping mapping(size_of_mapping);
        for (int i = 0; i < size_of_mapping; i++) {
            iss >> mapping(i);
        }
        mappings.push_back(mapping);
    }
}

bool SaveCloud(const std::string& filename, const mapping::common::PointCloudXYZIHIRBS& cloud) {
    LOG(INFO) << "Saving under " << filename;
    return pcl::io::savePCDFileBinary(filename, cloud) != -1;
}

// mapping io
void SaveMappings(const std::string& filename, const BeamMappings& mappings) {
    std::ofstream file(filename.c_str());
    if (file.is_open()) {
        file << mappings.size() << " " << mappings.at(0).cols() << "\n";
        int line = 0;
        for (const auto mapping : mappings) {
            file << mapping << '\n';
            line++;
        }
    }
}

void SaveProbability(const std::string& filename, const BeamModel& beam_probs) {
    std::ofstream file(filename.c_str());
    if (file.is_open()) {
        int line = 0;
        for (const auto beam : beam_probs) {
            file << beam.probability << '\n';
            line++;
        }
    }
}

void SaveBeamInfo(const std::string& filename, const BeamInfo& beam_info) {
    std::ofstream file(filename.c_str());
    if (file.is_open()) {
        int line = 0;
        for (const auto beam_intensity : beam_info) {
            for (const auto IntensityCount : beam_intensity) {
                file << std::get<1>(IntensityCount) << " ";
            }
            file << '\n';
            line++;
        }
    }
}

}  // end of namespace calibration

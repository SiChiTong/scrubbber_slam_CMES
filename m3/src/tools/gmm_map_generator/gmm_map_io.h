//
// Created by pengguoqi on 19-8-24.
//

#ifndef GMM_MAP_IO_H_
#define GMM_MAP_IO_H_

#include <dirent.h>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <fstream>
#include <limits>
#include <unordered_set>
#include <unordered_map>

#include "tools/gmm_map_generator/gmm_map_struct.h"
#include "io/db_io.h"

namespace mapping::tools {

class GmmBaseIO {
   public:
    explicit GmmBaseIO() = default;

    ~GmmBaseIO() { std::map<int, SE3>().swap(map_id_pose_); };

    int LoadRawMap(std::string filename, common::PointCloudXYZI::Ptr &cloud);

    int LoadRawMap(std::string filename);

    int HandlingPCD(const std::string &filename);

    int HandlingPCD(const std::string &filename, const TilesPointF &center, const float &size);

    int GetZ(const float r, int &rows, int &cols, std::vector<short int> &gs, const int &type,
             const std::string &binfile);

    void SetResolution(float resolution);

    int ReadBin(std::string filename, int rc, MapTa &mapa, MapTr &mapr);

    int ReadTilesBin(std::string filename, int rc, MapTa &mapa, MapTr &mapr);

    int FindFileInfolder(const char *dirname, const std::string &extendname, std::vector<std::string> &filenames);

    void Visualize(const std::string &filename, const double &resolution, const MapTa &mapa, const MapTr &mapr,
                   bool is_tiles = false, bool is_init = false);

    inline MapRange GetRange() { return map_range_; };

    inline void ClearHashRef() { cells_.clear(); };

    inline std::unordered_map<unsigned long long, std::vector<AR>> &GetHashRef() { return cells_; }

   private:
    bool GetPcd(std::string filename, common::PointCloudXYZI::Ptr &all, double minz, double maxz);

    void CutZ(common::PointCloudType::Ptr pcd, common::PointCloudXYZI::Ptr &out, double min, double max);

    void TransformPointCloud(common::PointCloudType::Ptr src, common::PointCloudXYZI::Ptr &out);

    void TransformPcd(common::PointCloudXYZI::Ptr &src, SE3 &pose);

    void TransformPcd2(common::PointCloudXYZI::Ptr &src, SE3 &pose);

    void AddPointsToHashCells(MapPointsf &ps);

    MapRange ComputeRange();

    MapPointsf VoxelFilter(const MapPointsf &points);

    std::bitset<3 * 32> IndexToKey(const Eigen::Array3i &index);

    Eigen::Array3i GetCellIndex(const MapPointf &point) const;

    int BinFile2EigenMatrix(std::string filename, MapT &map);

    MapRange map_range_;

    float resolution_{};

    //    std::map<int, DBPose> map_id_pose_;//mapping3.0
    std::map<int, SE3> map_id_pose_;
    std::unordered_map<unsigned long long, std::vector<AR>> cells_;
};

}  // namespace mapping::tools

#endif  // GMM_MAP_IO_H_

//
// Created by gaoxiang on 2020/8/10.
//

#ifndef MAPPING_FILE_IO_H
#define MAPPING_FILE_IO_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common/candidate.h"
#include "common/keyframe.h"
#include "common/merge_info.h"
#include "common/ndt_origin_data.h"
#include "common/num_type.h"
#include "common/origin_point_info.h"

namespace mapping::common {
struct KeyFrame;
struct VehicleCalibrationParam;
struct Trajectory;
}  // namespace mapping::common

namespace mapping::io {
class YAML_IO;
}

namespace mapping::io {

/**
 * 检查某个路径是否存在
 * @param file_path 路径名
 * @return true if exist
 */
bool PathExists(const std::string &file_path);

/**
 * 判断某路径是否为目录
 * @param path
 * @return
 */
bool IsDirectory(const std::string &path);

/// 查找文件
/// 如果filename能够在dir下找到，返回true，并在file_path中填入文件路径
bool FindFile(const std::string &dir, const std::string &filename, std::string &file_path);

/**
 * 若文件存在，则删除之
 * @param path
 * @return
 */
bool RemoveIfExist(const std::string &path);

/**
 * 保存Keyframes 各种 pose
 * @param save_path
 * @param keyframes
 */
bool SaveKeyframePose(const std::string &save_path, const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes);
bool SaveKeyframePose(const std::string &save_path,
                      const std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes);

/**
 * 保存子地图分割关键帧分布数据
 * @param save_path
 * @param splite_results
 */
bool SaveSpliteResults(const std::string &save_path, const std::map<IdType, std::map<IdType, SE3>> &splite_results);

/**
 * 读取Keyframes 各种 pose
 * @param load_path
 * @param keyframes
 */
bool LoadKeyframes(const std::string &load_path, std::vector<std::shared_ptr<common::KeyFrame>> &keyframes);
bool LoadKeyframes(const std::string &load_path,
                   std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes);
bool LoadKeyframes(const std::string &load_path, std::map<IdType, std::shared_ptr<common::KeyFrame>> &keyframes);

/**
 * 从map.db中获取地图原点信息
 * @param map_db_path
 * @param origin_info
 */
bool LoadOriginInfoByDB(const std::string &map_db_path, common::OriginPointInformation &origin_info);

/**
 * 保存地图合并信息
 * @param save_path
 * @param merge_info_vec
 */
bool SaveMergeInfo(const std::string &save_path, const common::MergeInfoVec &merge_info_vec);

/**
 * 读取地图合并信息
 * @param file_name
 * @param merge_info_vec
 */
bool LoadMergeInfo(const std::string &file_name, common::MergeInfoVec &merge_info_vec);

/**
 * 保存keyframes的degenracy score
 * @param save_path
 * @param keyframes
 * @return
 */
bool SaveDegeneracyScore(const std::string &save_path,
                         const std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes);

enum class SaveKeyframePathType {
    GPS_PATH = 0,
    DR_PATH,
    MATCHING_PATH,
    OPTI_PATH_STAGE_1,
    OPTI_PATH_STAGE_2,
};
/**
 * 保存单条关键帧轨迹，以供python绘图使用
 */
bool SaveKeyframeSinglePath(const std::string &path,
                            const std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes,
                            SaveKeyframePathType save_type);

/**
 * 将点云拼接成pcd然后保存，可以指定使用的pose
 * @param path 存储路径
 * @param db_path db文件路径
 * @param keyframes 关键帧数据
 * @param pose_type 使用哪一种pose
 * @param resolution 分辨率
 * @return
 */
bool SavePCDWithPose(const std::string &path, const std::string &db_path,
                     const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes, SaveKeyframePathType pose_type,
                     double resolution = 0.5);
bool SavePCDWithMapDB(const std::string &path, const std::string &db_path,
                      double resolution = 0.5); 
bool SavePCDWithPoseTest(const std::string &path, const std::string &db_path,
                         const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes,
                         SaveKeyframePathType pose_type, double delta_x, double delta_y, double resolution = 0.5);
bool SaveSinglePCDWithPose(const std::string &path, const std::string &db_path,
                           const std::shared_ptr<common::KeyFrame> &keyframe, SaveKeyframePathType pose_type);

/**
 * 存储回环信息
 * @param path loop.txt位置
 * @param loop_candidates 回环候选帧信息
 * @return
 */
bool SaveLoopCandidates(const std::string &path, const std::vector<common::LoopCandidate> &loop_candidates,
                        bool append = false);

bool SaveLoopCandidatesScore(const std::string &path, const std::vector<common::LoopCandidate> &loop_candidates,
                             const std::map<IdType, common::KFPtr> keyframes, bool remove);

/**
 * 读取回环信息
 * @param path
 * @param loop_candidates
 * @return
 */
bool LoadLoopCandidates(const std::string &path, std::vector<common::LoopCandidate> &loop_candidates);

bool LoadLoopCandidatesScore(const std::string &path, std::vector<common::NdtOriginData> &score_ndt);

/**
 * 存储轨迹与包的名称信息
 * @param trajectory_map
 * @return
 */
bool SaveTrajectoryBagName(const std::string &path,
                           const std::map<IdType, std::shared_ptr<common::Trajectory>> &trajectory_map);
/**
 * 读取轨迹和包名称信息
 * @param path
 * @param trajectory_map
 * @param only_valid_bags 仅读取验证包
 * @return
 *
 * 格式：轨迹id 包id 包名 是否为建图包
 * 默认存在trajectory_info.txt中
 */
bool LoadTrajectoryBagName(const std::string &path,
                           std::map<IdType, std::shared_ptr<common::Trajectory>> &trajectory_map,
                           bool only_validation_bags);

/// 保存GPS轨迹，用于计算直方图
bool SaveGpsError(const std::string &path, const std::vector<double> &gps_chi2);
/// res格式
bool SaveGpsError(const std::string &path, const std::vector<V6d> &gps_res);

struct FileNameInfo {
    /// 一个包的基本信息
    bool parse_success = false;  // 解析是否成功
    std::string bag_name;        // 包名
    bool has_part = false;       // 是否为分包
    int part_num = 0;            // 若是分包，分包id
    std::string bag_file_path;   // 包的路径
};

/**
 * 解析一个包名，得到基本信息
 * NOTE 包名是转换之后的，必须在ConvertData下面
 * @param name
 * @return
 */
FileNameInfo ParseBagName(const std::string &name);

double GetBagFileTime(const std::string &name);

bool KeyFramesPoints(std::string db_path, std::vector<std::shared_ptr<common::KeyFrame>> &pointcloud_vec);

/// 从YAML中读取
bool LoadVehicleParamsFromYAML(const io::YAML_IO &yaml, common::VehicleCalibrationParam &vehicle_params);

struct GpsInfo {
    GpsInfo(IdType _id, bool _fixed_inlier) {
        id = _id;
        fixed_inlier = _fixed_inlier;
    }
    IdType id = 0;  // keyframe的Id
    bool fixed_inlier = false;
};

bool LoadGpsInfo(const std::string &path, std::vector<GpsInfo> &gps_info);

}  // namespace mapping::io
#endif  // MAPPING_FILE_IO_H

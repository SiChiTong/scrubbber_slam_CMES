//
// Created by gaoxiang on 2020/9/28.
//

#ifndef MAPPING_CHECK_OUT_H
#define MAPPING_CHECK_OUT_H

#include "common/car_type.h"
#include "common/keyframe.h"
#include "common/trajectory.h"
#include "core/coordinate_transform/gps_trans.h"
#include "core/scan_context/scan_context.h"
#include "filter_ndt_score/filter_ndt_score.h"
#include "io/db_io.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"
#include "scan_builder/scan_builder.h"
// for gmm map
#include "tools/gmm_map_generator/gmm_map_generator.h"
#include "tools/gmm_map_generator/gmm_map_struct.h"
#include "tools/gmm_map_generator/idp_gmm_map_generator.h"
#include "tools/gmm_map_generator/lads_gmm_map_generator.h"
#include "tools/gmm_map_generator/tiles_gmm_map_generator.h"
#include "tools/map_splitter/map_splitter.h"

#include <rosbag/bag.h>

namespace mapping::io {
class DB_IO;
}

namespace mapping::pipeline {

/**
 * 检出
 * 需要做的事：
 * 1. 贴边轨迹转成track形式写入db
 * 2. 图片和功能点关联到最近的track pose
 * 3. scan context
 * 4. GMM
 * 5. 生成报告中用到的图片
 *
 * NOTE: 由于DB协议，所有计算的XYZ都要转换为lat lon才能写入DB
 */
class CheckOut : public PipelineContext {
   public:
    /// 给定配置文件
    CheckOut(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);
    ~CheckOut() override;

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

    /// 生成执行报告, 报告内容在report中
    /// 用verbose参数控制是否产生详细报告
    bool GenerateReport(std::string &report, bool verbose = true) override;

    /// 缓存中间结果
    bool Save() override;

    bool Load() override;

    /// 填写任务信息
    bool FillTaskInfo(TaskInfo &info) override;

    void SetInitialFiles(std::vector<core::ScanContextWithPose> &input) { scan_context_data_ = input; }

   private:
    /// 蜗小白checkout
    bool CheckOutForWXX();
    /// lads checkout
    bool CheckOutForLADS();

    // 解析数据包
    void ParseTrajectory(std::shared_ptr<common::Trajectory> trajectory);

    /// 向准出结果添加一个功能点
    void AddFunctionPoint(common::FunctionPoint fp);

    /// 将功能点写入DB
    void SaveFunctionPoints();

    /// 贴边轨迹
    void SaveTracks();

    /// 从ros message中解析图像信息
    bool ParseImageData(const rosbag::MessageInstance &m);

    /// 向DB写入原点信息
    void SaveOriginInfo();

    /// 生成相关图像
    void MakeFigures();

    bool SaveScanContext();

    /// 生成scan_context初始化文件
    bool ScanContextInitial();

    /// 生成scan_context初始化文件
    void SaveScanContext(const common::PointCloudXYZIT::ConstPtr &input, M4f &localizer);

    /// 子地图切割
    bool SpliteSubMap(); 

    ///保存GMM Map
    bool SaveGmmMap();  // from mapping3.0

    std::string local_data_path_;
    std::string local_db_path_;

    std::string report_;
    std::map<IdType, std::shared_ptr<common::Trajectory>> trajectory_map_;
    io::YAML_IO yaml_;

    common::OriginPointInformation origin_info_;               // 地图原点信息
    std::shared_ptr<core::GpsTransform> gps_trans_ = nullptr;  // GPS转换
    common::KFTrajType keyframes_;                             // 轨迹形式组织的关键帧
    std::map<double, common::KFPtr> keyframes_map_;            // 按时间排序的关键帧

    std::vector<common::FunctionPoint> function_points_;  // 功能点
    std::map<int, int> func_id_to_number;                 // 功能点id到number的映射

    int num_images_loaded_ = 0;            // 存入的图像数据
    std::vector<ImageMsgPtr> image_msgs_;  // 缓存图像消息
    std::shared_ptr<io::DB_IO> db_io_ = nullptr;

    TaskInfo task_info_;  // 任务信息

    std::vector<io::DB_IO::ScanContextData> scan_contexts_;
    std::vector<core::ScanContextWithPose> scan_context_data_;
    std::string scan_context_path_;
    std::string scan_file_path_;
    std::shared_ptr<ScanBuilder> scan_builder_;

    int gmm_map_ = 0;  //

    bool enable_map_splitter_ = false;

    common::CarType car_type_ = common::CarType::WXX;

    std::vector<common::NdtOriginData> ndt_data_;
};
}  // namespace mapping::pipeline

#endif  // MAPPING_CHECK_OUT_H

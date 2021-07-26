//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_OPTIMIZATION_S1_H
#define MAPPING_OPTIMIZATION_S1_H

#include "common/num_type.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

struct OptimizationStage1Impl;

/**
 * Optimization on stage 1 第一轮优化
 * 不带回环约束，使用GPS、DR、Matching计算粗略的位姿
 *
 * 相比于3.0，对不同gnss精度使用不同的噪声
 * 并且多了一步找优化初值的计算过程
 */
class OptimizationStage1 : public PipelineContext {
   public:
    explicit OptimizationStage1(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);

    ~OptimizationStage1() override;

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

   private:
    /// 计算优化问题的初始值
    void CalculateInitialValue();
    void CalculateInitialValueWithGps();
    void CalculateInitialValueWithoutGps();

    /// 建立优化问题
    void BuildProblem();

    /// 求解问题
    void Solve(bool fetch_results = false);

    /// 剔除错误点
    void RemoveOutliers();

    /// 将优化问题中变量设回初始值
    void ResetToInitialValue();

    // 添加顶点
    void AddVertex();

    // 添加各种边
    void AddGpsFactors();
    void AddMatchingFactors();
    void AddDRFactors();
    void AddGlobalRotationFactors();
    void AddHeightFactors();
    void AddLoopFactors();

    // 输出优化统计信息
    void CollectOptimizationStatistics();

    /// GPS好时，重新估计lidar与gnss外参数
    void CalibLidarGNSS();

    /// 无GPS环境时，若优化轨迹整体较远，那么就移到合适位置
    void MoveToOriginInGpsDenied();

    /// 输出结果
    void SaveResults();

    /// 如果轨迹GPS状态不好，则考虑对轨迹进行连接
    void ConnectTrajectories();

    SE3 GetPlanePose(const SE3& pose){
        SE3 plane_pose = pose;
        plane_pose.translation()[2] = 0;
        plane_pose.so3() = SO3::exp(V3d(0, 0, pose.so3().log()[2]));
        return plane_pose;
    }

    io::YAML_IO yaml_file_;
    std::string local_data_path_;
    std::string local_db_path_;

    std::unique_ptr<OptimizationStage1Impl> impl_;
};
}  // namespace mapping::pipeline

#endif  // MAPPING_OPTIMIZATION_S1_H

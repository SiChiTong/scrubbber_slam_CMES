//
// Created by gaoxiang on 19-7-23.
//

#ifndef FILTER_NDT_SCORE_H
#define FILTER_NDT_SCORE_H

#include "common/ndt_origin_data.h"
#include "common/std_headers.h"

namespace mapping {
namespace pipeline {

constexpr double MIN_THESHOLD = 1.8;
constexpr double MAX_THESHOLD = 4.2;
constexpr double THESHOLD_STEP = 0.3;
constexpr double DISTANCE_SUM_THESHOLD = 8;
constexpr double FILTER_NUM = 7;

class FilterNdtScore {
   public:
    explicit FilterNdtScore(){};

    ~FilterNdtScore(){};

    /// 设置ndt阈值原始数据
    void SetNdtScoreData(const std::vector<common::NdtOriginData> ndt_data);

    /// 获取优化后的ndt阈值
    std::vector<std::tuple<float, float, float>> GetNdtScoreFilter();

    /// 启动ndt阈值过滤器
    bool RunFilter();

    /// 生成错误报告信息
    bool GenerateReport(std::string &report, bool verbose = true);

   private:
    /// ndt阈值分级
    void AlignThesholdIndex();

    /// 通过编号和距离关系合并ndt阈值等级
    bool MergeThesholdByIndexAndDistance();

    ///  查找满足要求的最后一个ndt等级对应的ndt阈值标号
    void FindLastMinTheshold(int &theshold_index, int &min_theshold);

    /// 查找满足要求的首个ndt等级对应的ndt阈值标号
    void FindBeginMinTheshold(int &theshold_index, int &min_theshold);

    /// 更新输入ndt阈值信息中的标号信息
    void UpdateNdtData();

    /// 二次计算ndt等级对应的阈值起始标号和距离 保证ndt分段阈值的整齐性
    void ReProcessNdtData();

    /// 计算ndt等级对应的阈值起始标号和距离
    void PreProcessNdtData();

    /// 过滤尖刺段
    void FilterNdtData();

    /// ndt阈值的平均值滤波
    float GetAverageValue(const std::queue<float> &save_data);

    /// 过滤尖刺段
    void FilterLowerTrans();

    /// 空间过滤方法
    void FilterNdtTransBySpace();

    /// 曲率过滤方法
    void FilterNdtTransByCure();

    /// 优化分段值
    void FilterNdtByPredict();

    std::string report_;
    std::vector<common::NdtOriginData> ndt_data_;
    std::vector<std::tuple<int, int, float>> origin_index_;
    std::vector<std::tuple<int, int, float>> index_by_threshold_;
    std::vector<std::tuple<int, int, float>> merge_threshold_index_;  /// by index and distance_sum
    std::vector<std::tuple<float, float, float>> filter_threshold_;   /// xg, yg, theshold;
    std::vector<std::tuple<float, float, float>> merge_by_space_;
    std::vector<std::tuple<float, int, float>> index_by_cure_;
    float max_distance_sum_ = 0;
    std::vector<size_t> sum_suit_index_;
    size_t last_index_ = 0;
};

}  // namespace pipeline
}  // namespace mapping

#endif

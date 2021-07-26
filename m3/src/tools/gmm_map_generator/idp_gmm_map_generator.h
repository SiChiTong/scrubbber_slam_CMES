//
// Created by pengguoqi on 19-8-24.
//

#ifndef _IDP_GMM_MAP_GENERATOR_H_
#define _IDP_GMM_MAP_GENERATOR_H_

#include "common/mapping_point_types.h"
#include "tools/gmm_map_generator/gmm_map_generator.h"
#include "tools/gmm_map_generator/gmm_map_struct.h"

namespace mapping::tools {

class IDPGmmMapGenerator : public GmmMapGenerator {
   public:
    explicit IDPGmmMapGenerator(GmmMapParam &param) : gmm_map_param_(param){};

    ~IDPGmmMapGenerator();

    /// 初始化ｇｍｍ地图生成器
    int Init() override;

    /// 开始处理，
    int Start() override;

    /// 压缩gmm地图
    void ZipGmm() override;

    /// 生成执行报告, 报告内容在report中
    /// 用verbose参数控制是否产生详细报告
    bool GenerateReport(std::string &report, bool verbose = true) override;

   protected:
    int InitMap(const int cell_type);

    int LoadRawData();

    int GetMapParam();

    int TrainMap(const int cell_type);

    int CalculateMapBoundary();

    int CreateMap(const int cell_type);

    int CalculateMapDilutionRatio();

    void GmmTrainZ(int row, int col);

    void GmmTrainI(int row, int col);

    int WriteOffset();

    int WriteMapa();

    int WriteMapr();

    void SwitchMapParam(int param_type);

   protected:
    GmmMapParam gmm_map_param_;
    GmmMapParam complete_gmm_map_param_;

    GmmMapMv map_r_;
    GmmMapMv map_a_;
    GmmMapMc gmm_map_r_;
    GmmMapMc gmm_map_a_;

    std::thread line_a_, line_r_;
    std::string report_;

    cv::Ptr<cv::ml::EM> em_r_ = cv::ml::EM::create();
    cv::Ptr<cv::ml::EM> em_a_ = cv::ml::EM::create();

    common::PointCloudXYZI::Ptr point_cloud_ptr_;

    bool end_ = false;
};
}  // namespace mapping::tools

#endif  // _IDP_GMM_MAP_GENERATOR_H_

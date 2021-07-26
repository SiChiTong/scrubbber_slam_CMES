//
// Created by pengguoqi on 20-02-25.
//

#ifndef _LADS_GMM_MAP_GENERATOR_H_
#define _LADS_GMM_MAP_GENERATOR_H_

#include "tools/gmm_map_generator/gmm_map_generator.h"
#include "tools/gmm_map_generator/gmm_map_io.h"
#include "tools/gmm_map_generator/gmm_map_struct.h"

namespace mapping::tools {

class LADSGmmMapGenerator : public GmmMapGenerator {
   public:
    explicit LADSGmmMapGenerator(GmmMapParam& param) : gmm_map_param_(param){};

    virtual ~LADSGmmMapGenerator();

    /// 初始化ｇｍｍ地图生成器
    int Init() override;

    /// 开始处理，
    int Start() override;

    /// 压缩gmm地图
    void ZipGmm() override;

    /// 生成执行报告, 报告内容在report中
    /// 用verbose参数控制是否产生详细报告
    bool GenerateReport(std::string& report, bool verbose = true) override;

   protected:
    int LoadRawData();

    int Build();

    void EmTrain();

    bool Train1(std::vector<AR>& ars, Cell& cell);

    bool Train2(std::vector<AR>& ars, Cell& cell);

    bool Train3(std::vector<AR>& ars, Cell& cell);

    int WriteMap(const std::string& file);

    int WriteOffset();

    int ViewResult();

    int WriteGroud();

    int WriteBin(const std::string& filename, const void* data, const int& size);

   protected:
    GmmMapParam gmm_map_param_;

    std::shared_ptr<GmmBaseIO> gb_io_;
    std::string report_;

    std::unordered_map<unsigned long long, Cell> map_cells_;

    MapRange map_range_;

    float resolution_{};

    MapTa bina_;
    MapTr binr_;
    MapTa bina2_;
    MapTr binr2_;

    int size1_{};
    int size2_{};
	int invalid_num_{};
    int invalid_num_1_{};
    int invalid_num_2_{};
};
}  // namespace mapping::tools

#endif  // _LADS_GMM_MAP_GENERATOR_H_

//
// Created by pengguoqi on 20-03-23.
//

#ifndef _TILES_GMM_MAP_GENERATOR_H_
#define _TILES_GMM_MAP_GENERATOR_H_

#include "tools/gmm_map_generator/gmm_map_generator.h"
#include "tools/gmm_map_generator/gmm_map_io.h"
#include "tools/gmm_map_generator/gmm_map_struct.h"

namespace mapping::tools {

class TilesGmmMapGenerator : public GmmMapGenerator {
   public:
    explicit TilesGmmMapGenerator(GmmMapParam &param) : gmm_map_param_(param){};

    virtual ~TilesGmmMapGenerator();

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
    int LoadRawData();

    int BuildMap(const std::string &filepath);

    int Build(const TilesPointF &center, const float &size);

    int EmTrain();

    bool Train3(std::vector<AR> &ars, Cell &cell);

    int WriteMap(const std::string &file, const int &xol, const int &row);

    int WriteOffset();

    int ViewResult();

    int WriteEffectiveCell(const std::string &path);

    int WriteGroud();

    int WriteBin(const std::string &filename, const void *data, const int &size);

   protected:
    GmmMapParam gmm_map_param_;

    std::shared_ptr<GmmBaseIO> gb_io_;
    std::string report_;

    std::unordered_map<unsigned long long, Cell> map_cells_;

    MapRange map_range_;

    float resolution_ = 0.0;

    bool select_gmm_size_ = false;

    int size_ = 0;
    int debug_size_ = 0;
    int invalid_num_{};
};
}  // namespace mapping::tools

#endif  // _TILES_GMM_MAP_GENERATOR_H_

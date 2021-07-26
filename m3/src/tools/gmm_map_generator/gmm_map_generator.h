//
// Created by pengguoqi on 19-8-24.
//

#ifndef _GMM_MAP_GENERATOR_H_
#define _GMM_MAP_GENERATOR_H_

#include <cmath>
#include <condition_variable>
#include <thread>

#include "tools/gmm_map_generator/gmm_map_io.h"
#include "tools/gmm_map_generator/gmm_map_struct.h"

/// 由于opencv引用的问题，必须在这里include
/// @see
/// https://stackoverflow.com/questions/42504592/flann-util-serialization-h-class-stdunordered-mapunsigned-int-stdvectorun
#include "opencv2/opencv.hpp"

namespace mapping::tools {

class GmmMapGenerator {
   public:
    GmmMapGenerator(){};

    ~GmmMapGenerator(){};

    /// 初始化ｇｍｍ地图生成器
    virtual int Init() = 0;

    /// 开始处理，
    virtual int Start() = 0;

    /// 压缩gmm地图
    virtual void ZipGmm() = 0;

    /// 生成执行报告, 报告内容在report中
    /// 用verbose参数控制是否产生详细报告
    virtual bool GenerateReport(std::string &report, bool verbose = true) = 0;
};

}  // namespace mapping::tools

#endif  // _GMM_MAP_GENERATOR_H_

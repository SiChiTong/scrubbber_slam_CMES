//
// Created by pengguoqi on 19-7-16.
//
#ifndef XML_IO_H
#define XML_IO_H

#include "common/std_headers.h"
#include "common/vehicle_calib_param.h"

namespace tinyxml2 {
class XMLElement;
}

namespace mapping::io {

/// 读取xml文件的相关IO，用于解析velodyne参数
class XML_IO {
   public:
    XML_IO(const std::vector<std::string> &xml_path) : xml_path_(xml_path){};

    XML_IO(const std::vector<std::string> &conf_path, const std::vector<std::string> &velodyne_path)
        : conf_path_(conf_path), velodyne_path_(velodyne_path) {}

    XML_IO(const std::vector<std::string> &conf_path, const std::string &lidar_launch, const std::string &lidar_factory)
        : conf_path_(conf_path), lidar_launch_(lidar_launch), lidar_factory_(lidar_factory){};

    ~XML_IO(){};

    /// 设置xml文件路径
    inline void SetXmlFilePath(const std::vector<std::string> &xml_path) { xml_path_ = xml_path; };

    /// 解析xml文件
    int ParseXml(const std::string &xml_parse_check);

    int ParseXml();

    /// 获取xml文件中必要参数
    inline common::VehicleCalibrationParam GetParamForMapping() { return vehicle_calib_params_; };

   private:
    /// 判断输入的xml文件是否存在，且个数是否争取
    int CheckXmlVector(const std::vector<std::string> &xml_path, const std::string &check_name);

    int ParseVelodyneXml();
    int ParseHeSaiXml();
    int ParseSuTengXml();

    /// 获取xml中特定参数并转换成指定结构体
    void TransParamToStruct(tinyxml2::XMLElement *elem, const char *elemName);

    std::vector<std::string> xml_path_;
    std::vector<std::string> conf_path_;
    std::string lidar_launch_;
    std::vector<std::string> velodyne_path_;
    common::VehicleCalibrationParam vehicle_calib_params_;
    std::string lidar_factory_;  // 雷达生产厂家
};
}  // namespace mapping::io

#endif
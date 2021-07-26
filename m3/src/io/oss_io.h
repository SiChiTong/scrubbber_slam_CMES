//
// Created by gaoxiang on 2019/12/20.
//

#ifndef MAPPING_OSS_IO_H
#define MAPPING_OSS_IO_H

#include <alibabacloud/oss/OssClient.h>

namespace mapping {

namespace io {

struct OSSInfo {
    std::string access_key_id;
    std::string access_key_secret;
    std::string end_point;
    std::string bucket_name;
};

struct UploadConfig {
    std::string upload_data_type =
        "PreProcessingData";  // 上传数据类型: RawData/PreProcessingData/ProductData 建图主要使用PreProcessingData
    std::string file_upload_url;       // 上传数据网址: /Domestic/省/市/地区名称
    std::string version = "v1.0.0.0";  // 版本，四位编号，v1.0.0.0
};

struct DownLoadConfig {
    std::string upload_data_type =
        "PreProcessingData";  // 上传数据类型: RawData/PreProcessingData/ProductData 建图主要使用PreProcessingData
    std::string file_upload_url;       // 上传数据网址: /Domestic/省/市/地区名称
    std::string version = "v1.0.0.0";  // 版本，四位编号，v1.0.0.0
};

/**
 * 与阿里云OSS通信的IO，用于上报建图进度
 */
class OSS_IO {
   public:
    OSS_IO(const UploadConfig &config) : upload_config_(config) {
        default_oss_.access_key_id = "LTAI4Fg67DFaXM4PLpPe1NXZ";
        default_oss_.access_key_secret = "WdeT3xktMSFr1Z3FCrdcdJEskaFkjK";
        default_oss_.end_point = "oss-cn-beijing.aliyuncs.com";
        default_oss_.bucket_name = "official-bag-storage";
    }
    OSS_IO(const DownLoadConfig &config) : download_config_(config) {
        default_oss_.access_key_id = "LTAI4Fg67DFaXM4PLpPe1NXZ";
        default_oss_.access_key_secret = "WdeT3xktMSFr1Z3FCrdcdJEskaFkjK";
        default_oss_.end_point = "oss-cn-beijing.aliyuncs.com";
        default_oss_.bucket_name = "official-bag-storage";
    }

    ~OSS_IO() {}

    /**
     * 上传文件，阻塞
     * @param path_to_result 本地结果文件
     * @return 上传是否成功
     */
    bool upload(const std::string &path_to_result);

    bool download(const std::string &path_to_result);

   private:
    OSSInfo default_oss_;
    UploadConfig upload_config_;
    DownLoadConfig download_config_;
};

}  // namespace io

}  // namespace mapping

#endif  // MAPPING_OSS_IO_H

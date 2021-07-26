//
// Created by gaoxiang on 2019/12/20.
//

#include "io/oss_io.h"
#include <glog/logging.h>

using namespace AlibabaCloud::OSS;

namespace mapping {
namespace io {

bool OSS_IO::upload(const std::string &path_to_result) {
    std::string cmd = "cd " + path_to_result + " && zip -r mapping_result.zip .";
    system(cmd.c_str());

    InitializeSdk();
    ClientConfiguration conf;
    OssClient client(default_oss_.end_point, default_oss_.access_key_id, default_oss_.access_key_secret, conf);

    std::string full_name = upload_config_.upload_data_type + upload_config_.file_upload_url + upload_config_.version +
                            "/mapping_result.zip";
    LOG(INFO) << "upload full name: " << full_name;

    auto out = client.PutObject(default_oss_.bucket_name, full_name, path_to_result + "/mapping_result.zip");

    if (out.isSuccess() == false) {
        LOG(ERROR) << "Upload file " << path_to_result << "/mapping_result.zip failed";
        return false;
    } else {
        LOG(INFO) << "Upload success";
    }

    ShutdownSdk();
    return true;
}

bool OSS_IO::download(const std::string &path_to_result) {
    InitializeSdk();
    ClientConfiguration conf;
    OssClient client(default_oss_.end_point, default_oss_.access_key_id, default_oss_.access_key_secret, conf);

    std::string full_name = download_config_.upload_data_type + download_config_.file_upload_url +
                            download_config_.version + "/mapping_result.zip";
    LOG(INFO) << "download full name: " << full_name;

    auto out = client.GetObject(default_oss_.bucket_name, full_name, path_to_result + "/mapping_result.zip");

    if (out.isSuccess() == false) {
        LOG(ERROR) << "Download file " << full_name << " failed";
        return false;
    } else {
        LOG(INFO) << "Download success";
    }

    ShutdownSdk();
    return true;
}

}  // namespace io
}  // namespace mapping

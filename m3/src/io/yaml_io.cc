//
// Created by gaoxiang on 19-7-12.
//
#include <glog/logging.h>
#include <fstream>

#include "io/yaml_io.h"
#include "yaml-cpp/node/parse.h"
#include "yaml-cpp/yaml.h"

namespace mapping {
namespace io {

YAML_IO::YAML_IO(const std::string &path) {
    path_ = path;
    yaml_node_ = YAML::LoadFile(path);
    if (yaml_node_.IsNull()) {
        LOG(ERROR) << "Failed to open yaml: " << path;
    }
    is_opened_ = true;
}

YAML_IO::~YAML_IO() {}

bool YAML_IO::Save(const std::string &path) {
    if (path.empty()) {
        std::ofstream fout(path_);
        fout << yaml_node_;
    } else {
        std::ofstream fout(path);
        fout << yaml_node_;
    }
    return true;
}

}  // namespace io
}  // namespace mapping

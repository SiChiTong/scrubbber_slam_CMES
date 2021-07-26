//
// Created by gaoxiang on 19-7-12.
//

#ifndef MAPPING_YAML_IO_H
#define MAPPING_YAML_IO_H

#include <yaml-cpp/yaml.h>
#include <cassert>
#include <string>

namespace mapping::io {

/**
 * 读取yaml配置文件的相关IO
 * NOTE YAML_IO要求yaml文件必须存在
 * 而且读取的字段时也必须存在，否则抛出异常
 */
class YAML_IO {
   public:
    YAML_IO(const std::string &path);

    YAML_IO() {}

    ~YAML_IO();

    inline bool IsOpened() const { return is_opened_; }

    /// 保存文件，不指明路径时，覆盖原文件
    bool Save(const std::string &path = "");

    /// 获取类型为T的参数值
    template <typename T>
    T GetValue(const std::string &key) const {
        assert(is_opened_);
        return yaml_node_[key].as<T>();
    }

    /// 获取在NODE下的key值
    template <typename T>
    T GetValue(const std::string &node, const std::string &key) const {
        assert(is_opened_);
        return yaml_node_[node][key].as<T>();
    }

    /// 设定类型为T的参数值
    template <typename T>
    void SetValue(const std::string &key, const T &value) {
        yaml_node_[key] = value;
    }

    /// 设定NODE下的key值
    template <typename T>
    void SetValue(const std::string &node, const std::string &key, const T &value) {
        yaml_node_[node][key] = value;
    }

   private:
    std::string path_;
    bool is_opened_ = false;
    YAML::Node yaml_node_;
};

}  // namespace mapping::io

#endif  // MAPPING_YAML_IO_H

//
// Created by wangqi on 19-7-15.
//

#ifndef MAPPING_FILE_FILTER_H
#define MAPPING_FILE_FILTER_H

#include <sys/stat.h>

#include "common/origin_files.h"
#include "common/std_headers.h"

namespace mapping::pipeline {

/// 获得固定目录下原始数据文件
class FilesFilter {
   public:
    FilesFilter(std::string parent_path) : parent_path_(parent_path) {}

    void SetFolderPath(const std::string &path) { parent_path_ = path; }

    /// 搜索指定目录下文件
    int FilterParentFolder();

    /// 递归搜索指定目录下所有有效文件
    int FilterAllFolderUnderParent();

    /// 获得搜索结果
    common::OriginFiles GetFilteredFiles() { return origin_files_; }

    std::vector<std::string> GetPcdFilesFromFolder();

   private:
    inline int CheckDir(const std::string &path) {
        if ((path == "INVALID") || (path.empty())) {
            return 0;
        }

        struct stat info {};
        return stat(path.c_str(), &info) == 0;
    }

    void FilterFilesToFileStruct(const std::string &parent_path, const std::string &subfile);

    int SearchAllFolders(std::string &path);

   private:
    std::string parent_path_;
    common::OriginFiles origin_files_;
    std::vector<std::string> all_files_;
    std::vector<std::string> pcd_files_;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_FILE_FILTER_H
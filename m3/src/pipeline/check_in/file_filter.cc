//
// Created by wangqi on 19-7-15.
//

#include "pipeline/check_in/file_filter.h"
#include <dirent.h>
#include <cstring>

namespace mapping::pipeline {

int FilesFilter::FilterParentFolder() {
    origin_files_.ClearFiles();
    all_files_.clear();
    if (!CheckDir(parent_path_)) {
        return -1;
    }
    DIR *dir;
    dirent *pdir;
    dir = opendir(parent_path_.c_str());
    if (nullptr == dir) {
        return -1;
    }

    while ((pdir = readdir(dir))) {
        if (std::strcmp(pdir->d_name, ".") != 0 && std::strcmp(pdir->d_name, "..") != 0) {
            bool flag = false;
            for (uint i = 0; i < all_files_.size(); i++) {
                if (pdir->d_name == all_files_[i].c_str()) {
                    flag = true;
                }
            }

            if (!flag && pdir->d_type != 4) {
                all_files_.emplace_back(pdir->d_name);
                FilterFilesToFileStruct(parent_path_, pdir->d_name);
            }
        }
    }
    closedir(dir);
    return all_files_.size();
}

void FilesFilter::FilterFilesToFileStruct(const std::string &parent_path, const std::string &subfile) {
    std::string::size_type pos_bag = subfile.find(".bag");
    std::string::size_type pos_conv_bag = subfile.find("_V.bag");
    std::string::size_type pos_db = subfile.find(".db");
    std::string::size_type pos_pcd = subfile.find(".pcd");
    std::string::size_type pos_active = subfile.find(".active");
    std::string::size_type pos_calib = subfile.find("calibration.launch");
    std::string::size_type pos_vehicleparam = subfile.find("vehicleparams.launch");
    std::string::size_type pos_velodyne = subfile.find("velodyne.launch");
    std::string::size_type pos_conf = subfile.find(".conf");
    std::string file_path;
    if (parent_path.back() != '/') {
        file_path = parent_path + '/' + subfile;
    } else {
        file_path = parent_path + subfile;
    }

    if (pos_bag != std::string::npos && pos_active == std::string::npos) {
        if (pos_conv_bag == std::string::npos) {
            // origin data
            origin_files_.bag_files.push_back(file_path);
        } else {
            // converted data
            origin_files_.converted_bag_files.push_back(file_path);
        }
    }
    if (pos_db != std::string::npos) {
        origin_files_.dbs_files.push_back(file_path);
    }
    if (pos_pcd != std::string::npos) {
        origin_files_.pcd_files.push_back(file_path);
    }
    if (pos_calib != std::string::npos) {
        origin_files_.calib_params.push_back(file_path);
    }
    if (pos_vehicleparam != std::string::npos) {
        origin_files_.vehicle_params.push_back(file_path);
    }
    if (pos_velodyne != std::string::npos) {
        origin_files_.velodyne_params.push_back(file_path);
    }
    if (pos_conf != std::string::npos) {
        origin_files_.conf_params.push_back(file_path);
    }
}

int FilesFilter::FilterAllFolderUnderParent() {
    origin_files_.ClearFiles();
    all_files_.clear();
    int file_size = SearchAllFolders(parent_path_);
    return file_size;
}

int FilesFilter::SearchAllFolders(std::string &basepath) {
    if (!CheckDir(basepath)) return -1;
    DIR *dir;
    dirent *pdir;
    dir = opendir(basepath.c_str());
    if (nullptr == dir) return -1;
    while ((pdir = readdir(dir))) {
        if (std::strcmp(pdir->d_name, ".") != 0 && std::strcmp(pdir->d_name, "..") != 0) {
            if (pdir->d_type == 4) {
                std::string basepath_child;
                if (basepath.back() != '/') {
                    basepath_child = basepath + "/" + pdir->d_name;
                } else {
                    basepath_child = basepath + pdir->d_name;
                }
                SearchAllFolders(basepath_child);
            }
            bool flag = false;
            for (uint i = 0; i < all_files_.size(); i++) {
                if (pdir->d_name == all_files_[i].c_str()) {
                    flag = true;
                }
            }
            if (false == flag && pdir->d_type != 4) {
                all_files_.push_back(pdir->d_name);
                FilterFilesToFileStruct(basepath, pdir->d_name);
            }
        }
    }
    closedir(dir);
    return all_files_.size();
}

std::vector<std::string> FilesFilter::GetPcdFilesFromFolder() {
    FilterParentFolder();
    for (uint index = 0; index < all_files_.size(); index++) {
        std::string::size_type pos_pcd = all_files_[index].find(".pcd");
        std::string file_path;
        if (parent_path_.back() != '/') file_path = parent_path_ + '/' + all_files_[index];
        else
            file_path = parent_path_ + all_files_[index];
        if (pos_pcd != std::string::npos) {
            pcd_files_.push_back(file_path);
        }
    }
    return pcd_files_;
}

}  // namespace mapping::pipeline
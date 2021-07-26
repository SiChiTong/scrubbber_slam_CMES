//
// Created by idriver on 2020/10/15.
//
#include "scan_context.h"

namespace mapping::core {

ScanContext::ScanContext() {
    for (int k = 0; k < CONTEXT_COL; ++k) {
        for (int b = 0; b < CONTEXT_ROW; ++b) {
            context_[k][b] = 0;
        }
    }
}

void ScanContext::ConstructInitializationFile(const common::PointCloudXYZIT::ConstPtr &points_ptr,
                                              const SE3 &real_pose) {
    ConstructScanContext(points_ptr);
    context_and_pos_.loc_pos.x = real_pose.translation()[0];
    context_and_pos_.loc_pos.y = real_pose.translation()[1];
    context_and_pos_.loc_pos.z = real_pose.translation()[2];
    context_and_pos_.loc_pos.roll = real_pose.so3().log()[0];
    context_and_pos_.loc_pos.pitch = real_pose.so3().log()[1];
    context_and_pos_.loc_pos.yaw = real_pose.so3().log()[2];
    // context_and_pos_.loc_pos = real_pose;
    // LOG(INFO) << "context_and_pos_.loc_pos.z: "<<context_and_pos_.loc_pos.z;

    for (int k = 0; k < CONTEXT_COL; ++k) {
        for (int b = 0; b < CONTEXT_ROW; ++b) {
            context_and_pos_.context[k][b] = context_[k][b];
        }
    }

    //    SaveScanContextToFile(context_and_pos_);
}

void ScanContext::ConstructScanContext(const common::PointCloudXYZIT::ConstPtr &points_ptr) {
    for (int k = 0; k < CONTEXT_COL; ++k) {
        for (int b = 0; b < CONTEXT_ROW; ++b) {
            context_[k][b] = 0;
        }
    }

    uint row = 0, col = 0;
    for (uint i = 0; i < points_ptr->size(); ++i) {
        common::PointXYZIT single_point = points_ptr->points[i];
        GetColAndRow(single_point, col, row);
        // LOG(INFO) << "col: "<<col<<" "<<row<<" "<<single_point.z + 10<<" "<<context_[col][row];
        if (col >= CONTEXT_COL || row >= CONTEXT_ROW) {
            continue;
        }

        float height = single_point.z + 10;  /// (-10 ~ 25) => (0 ~ 35)
        if (context_[col][row] <= height && height > 0 && height < 100) context_[col][row] = height;
    }
    return;
}

void ScanContext::GetColAndRow(const common::PointXYZIT &single_point, uint& col, uint& row) {
    float distance = sqrt(single_point.x * single_point.x + single_point.y * single_point.y);
    row = uint(distance / resolution_row_);
    // col = uint(single_point.angle / resolution_col_);
    double angle = -atan2(single_point.y, single_point.x) * 180.0 / M_PI;
    while(angle < 0){
        angle += 360.0;
    }
    col = uint(angle / resolution_col_);
    return;
}

void ScanContext::SaveScanContextToFile(const ScanContextWithPose &context_and_pos) {
    std::string filename_f = sacncontext_path_;
    std::string filename_s = filename_f + "scancon" + "_" +
                             std::to_string((int)(context_and_pos.loc_pos.x)) + "_" +
                             std::to_string((int)(context_and_pos.loc_pos.y)) + ".bin";
    const char *filename = filename_s.data();
    FILE *fp;
    fp = fopen(filename, "wb");
    if (!fp) {
        LOG(ERROR) << "ERROR BUILD THE HISTOGRAM FILE!!!";
        return;
    }
    fwrite(&context_and_pos, sizeof(ScanContextWithPose), 1, fp);
    fclose(fp);
    return;
}

void ScanContext::SetInitializationPath(const std::string initialization_path) {
    sacncontext_path_ = initialization_path;
    if (sacncontext_path_.back() != '/') {
        sacncontext_path_ += '/';
    }

    return;
}

int ScanContext::FindSubFile(const std::string path) {
    if (path == "INVALID") {
        return 0;
    }
    DIR *dir;
    dirent *pdir;
    dir = opendir(path.c_str());
    if (NULL == dir) {
        return 0;
    }

    while ((pdir = readdir(dir))) {
        if (std::strcmp(pdir->d_name, ".") != 0 && std::strcmp(pdir->d_name, "..") != 0) {
            bool flag = false;
            for (uint i = 0; i < files_.size(); ++i) {
                if (pdir->d_name == files_[i].c_str()) {
                    flag = true;
                }
            }
            std::string subfile = pdir->d_name;
            std::string::size_type pos = subfile.find("scancon");
            if (false == flag && pos != std::string::npos) {
                files_.push_back(pdir->d_name);
            }
        }
    }
    closedir(dir);
    return files_.size();
}

}  // namespace mapping::core

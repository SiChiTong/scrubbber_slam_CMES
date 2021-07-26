//
// Created by gaoxiang on 2020/9/23.
//

#include "core/mem_control/mem_control.h"
#include "common/std_headers.h"
#include "io/db_io.h"

#include <glog/logging.h>

namespace mapping::core {

MemoryControl::MemoryControl(const std::string& local_db_path, int kf_limit, bool running_as_gem)
    : num_loaded_kf_limit_(kf_limit), running_as_gem_(running_as_gem), local_db_path_(local_db_path) {
    db_io_mapping_ = std::make_shared<io::DB_IO>(local_db_path + "map.db");
    db_io_validation_ = std::make_shared<io::DB_IO>(local_db_path + "val.db");
}

MemoryControl::~MemoryControl() {}

common::PointCloudType::Ptr MemoryControl::RequestCloud(common::KFPtr kf) {
    UL lock(mem_mutex_);

    if (kf->cloud_ == nullptr) {
        if (record_.size() > num_loaded_kf_limit_ * 1.2) {
            Clean();
        }

        // 未载入，新增一个载入记录
        bool load_succ = false;

        if (kf->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
            load_succ = db_io_mapping_->ReadSingleKF(kf->id_, kf, false);
        } else {
            load_succ = db_io_validation_->ReadSingleKF(kf->id_, kf, false);
        }

        if (load_succ) {
            KeyFrameCloudRecord rec(kf, load_time_count_++);
            record_.emplace_back(std::move(rec));
        } else {
            kf->cloud_.reset(new common::PointCloudType);
        }

    } else {
        // 查找并更新记录
        auto iter = std::find_if(record_.begin(), record_.end(),
                                 [&kf](const KeyFrameCloudRecord& iter) { return iter.kf_id == kf->id_; });
        if (iter == record_.end()) {
            LOG(ERROR) << "should not happen";
            KeyFrameCloudRecord rec(kf, load_time_count_++);
            record_.emplace_back(std::move(rec));

            return kf->cloud_;
        } else {
            iter->loaded_time = load_time_count_++;
        }
    }

    assert(kf->cloud_ != nullptr);
    return kf->cloud_;
}

void MemoryControl::Clean() {
    std::sort(record_.begin(), record_.end(), [](const KeyFrameCloudRecord& c1, const KeyFrameCloudRecord& c2) {
        return c1.loaded_time < c2.loaded_time;
    });

    /// 清空记录与点云
    int size_before_clean = record_.size();
    int num_to_remove = record_.size() - num_loaded_kf_limit_ * 0.8;
    for (int i = 0; i < num_to_remove; ++i) {
        record_[i].keyframe->UnloadCloud();
    }
    record_.erase(record_.begin(), record_.begin() + num_to_remove);

    // update record time
    if (record_.empty()) {
        return;
    }

    int st = record_.front().loaded_time;
    load_time_count_ -= st;

    for (auto& c : record_) {
        c.loaded_time -= st;
    }

    if (verbose_) {
        LOG(INFO) << "memory control: keyframe reduced from " << size_before_clean << " to " << record_.size();
    }
}

}  // namespace mapping::core

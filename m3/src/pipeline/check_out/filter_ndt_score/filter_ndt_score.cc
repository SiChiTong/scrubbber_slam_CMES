//
// Created by gaoxiang on 19-7-23.
//

#include <glog/logging.h>

#include "common/ndt_origin_data.h"
#include "pipeline/check_out/filter_ndt_score/filter_ndt_score.h"

namespace mapping {
namespace pipeline {

void FilterNdtScore::SetNdtScoreData(const std::vector<common::NdtOriginData> ndt_data) {
    ndt_data_ = ndt_data;
    if (ndt_data_.size() < 1) {
        LOG(ERROR) << "NO INPUT NDT DATA!";
    }
    FilterNdtData();
}

void FilterNdtScore::FilterNdtData() {
    std::vector<float> filter_ndt_trans;
    std::queue<float> save_data;
    if (ndt_data_.size() < FILTER_NUM) {
        LOG(ERROR) << "The size of ndt_data_ is less than 7!";
        report_ += "filter warning ndt score : the size of ndt_data_ is less than 7.\n";
    }
    for (size_t i = 0; i < ndt_data_.size(); ++i) {
        if (i < FILTER_NUM - 1) {
            save_data.push(ndt_data_[i].trans_probability);
        } else {
            save_data.push(ndt_data_[i].trans_probability);
            float theshold = GetAverageValue(save_data);
            save_data.pop();
            if (i == FILTER_NUM) {
                for (uint i = 0; i < (int)(FILTER_NUM / 2); ++i) {
                    filter_ndt_trans.push_back(theshold);
                }
            }
            filter_ndt_trans.push_back(theshold);
            if (i == (ndt_data_.size() - 1)) {
                for (uint i = ndt_data_.size() - (int)(FILTER_NUM / 2); i < ndt_data_.size(); ++i) {
                    filter_ndt_trans.push_back(theshold);
                }
            }
        }
    }
    if (ndt_data_.size() != filter_ndt_trans.size()) {
        LOG(ERROR) << "filter size is not corresponding!";
        report_ += "filter warning ndt score : filter size is not corresponding.\n";
    } else {
        for (uint i = 0; i < ndt_data_.size(); ++i) {
            ndt_data_[i].trans_probability = filter_ndt_trans[i];
        }
    }
}

float FilterNdtScore::GetAverageValue(const std::queue<float> &save_data) {
    std::queue<float> queue_data = save_data;
    float sum = 0;
    if (queue_data.size() == FILTER_NUM) {
        std::map<float, int> data_map;
        for (int i = 0; i < (int)save_data.size(); ++i) {
            data_map.insert(std::make_pair(queue_data.front(), i));
            queue_data.pop();
        }
        auto iter_begin = data_map.begin();
        auto iter_end = data_map.end();
        iter_begin++;
        iter_end--;
        for (; iter_begin != iter_end; ++iter_begin) {
            sum += iter_begin->first;
        }
        return (float)(sum / (save_data.size() - 2));
    }
    return 0;
}

std::vector<std::tuple<float, float, float>> FilterNdtScore::GetNdtScoreFilter() {
    if (ndt_data_.size() < 1) LOG(ERROR) << "NO NDT DATA!";
    ReProcessNdtData();
    for (size_t i = 0; i < ndt_data_.size(); i++) {
        filter_threshold_.push_back(std::make_tuple(ndt_data_[i].xg, ndt_data_[i].yg, ndt_data_[i].threshold_index));
    }
    FilterNdtTransBySpace();
    FilterNdtByPredict();
    return filter_threshold_;
}

void FilterNdtScore::FilterNdtByPredict() {
    std::map<float, int> tmp_theshold;
    std::map<float, int> tmp_theshold_1;
    std::map<float, float> theshold_c;
    for (size_t i = 0; i < ndt_data_.size(); i++) {
        tmp_theshold.insert(std::make_pair(ndt_data_[i].threshold_index, i));
    }
    if (tmp_theshold.size() < 1) return;
    int theshild_size = tmp_theshold.size();
    if (theshild_size == 1) {
        float origin_th = MIN_THESHOLD + ((ndt_data_[0].threshold_index) - 1.5) * THESHOLD_STEP;
        if (origin_th >= 2.7) {
            float guess_theshold = origin_th - 0.6;
            if (guess_theshold >= 2.7 && guess_theshold < 3.2) guess_theshold = 2.7;
            if (guess_theshold <= 2.2) guess_theshold = 2.2;
            theshold_c.insert(std::make_pair(origin_th, guess_theshold));
        } else if (origin_th < 2.7 && origin_th >= 2.5) {
            theshold_c.insert(std::make_pair(origin_th, 2.2));
        } else if (origin_th < 2.5 && origin_th >= 2.2) {
            theshold_c.insert(std::make_pair(origin_th, 1.9));
        } else {
            float guess_theshold = origin_th - 0.3;
            theshold_c.insert(std::make_pair(origin_th, guess_theshold));
        }
    } else {
        auto iter = tmp_theshold.begin();
        int sort_index = 0;
        for (; iter != tmp_theshold.end(); iter++) {
            tmp_theshold_1.insert(std::make_pair(iter->first, sort_index));
            sort_index++;
            float origin_th = MIN_THESHOLD + ((iter->first) - 1.5) * THESHOLD_STEP;
            if (origin_th >= 2.7) {
                float guess_theshold = origin_th - 0.6;
                if (guess_theshold >= 2.7 && guess_theshold < 3.2) guess_theshold = 2.7;
                if (guess_theshold <= 2.2) guess_theshold = 2.2;
                theshold_c.insert(std::make_pair(origin_th, guess_theshold));
            } else if (origin_th < 2.7 && origin_th >= 2.5) {
                theshold_c.insert(std::make_pair(origin_th, 2.2));
            } else if (origin_th < 2.5 && origin_th >= 2.2) {
                theshold_c.insert(std::make_pair(origin_th, 1.9));
            } else {
                float guess_theshold = origin_th - 0.3;
                theshold_c.insert(std::make_pair(origin_th, guess_theshold));
            }
        }
    }

    auto filter_theshold_tmp = filter_threshold_;
    filter_theshold_tmp.clear();
    for (size_t i = 0; i < filter_threshold_.size(); i++) {
        float filter_theshold_ori = MIN_THESHOLD + ((std::get<2>(filter_threshold_[i])) - 1.5) * THESHOLD_STEP;
        float suit_theshold = theshold_c.at(filter_theshold_ori);
        filter_theshold_tmp.push_back(
            std::make_tuple(std::get<0>(filter_threshold_[i]), std::get<1>(filter_threshold_[i]), suit_theshold));
    }
    filter_threshold_ = filter_theshold_tmp;
}

void FilterNdtScore::FilterNdtTransByCure() {
    std::map<float, int> tmp_theshold_;
    std::map<float, int> tmp_theshold_1;
    for (size_t i = 0; i < ndt_data_.size(); i++) {
        tmp_theshold_.insert(std::make_pair(ndt_data_[i].threshold_index, i));
    }
    auto iter = tmp_theshold_.begin();
    int sort_index = 0;
    for (; iter != tmp_theshold_.end(); iter++) {
        tmp_theshold_1.insert(std::make_pair(iter->first, sort_index));
        sort_index++;
    }

    for (size_t i = 10; i < filter_threshold_.size() - 10; i++) {
        float diffx = 0.0, diffy = 0.0, diff = 0.0;
        for (uint j = i - 10; j < i + 11; j++) {
            diffx += std::get<0>(filter_threshold_[j]);
            diffy += std::get<1>(filter_threshold_[j]);
        }
        diffx -= (21 * std::get<0>(filter_threshold_[i]));
        diffy -= (21 * std::get<1>(filter_threshold_[i]));
        diff = std::hypot(diffx, diffy);
        sort_index = tmp_theshold_1.at(std::get<2>(filter_threshold_[i]));
        if (i == 10) {
            for (uint num = 0; num < 10; num++) {
                index_by_cure_.push_back(std::make_tuple(std::get<2>(filter_threshold_[i]), sort_index, diff));
            }
        }
        index_by_cure_.push_back(std::make_tuple(std::get<2>(filter_threshold_[i]), sort_index, diff));
        if (i == (filter_threshold_.size() - 11)) {
            for (uint num = filter_threshold_.size() - 10; num < filter_threshold_.size() - 1; num++) {
                index_by_cure_.push_back(std::make_tuple(std::get<2>(filter_threshold_[i]), sort_index, diff));
            }
        }
    }
}

void FilterNdtScore::FilterNdtTransBySpace() {
    if (filter_threshold_.size() < 1) {
        LOG(ERROR) << "filter_theshold has no data to filter!";
        report_ += "filter warning ndt score : filter_theshold has no data to filter.\n";
    }
    auto filter_theshold_tmp = filter_threshold_;
    for (size_t i = 0; i < filter_threshold_.size(); i++) {
        for (uint j = 0; j < filter_threshold_.size(); j++) {
            double delta_x = fabs(std::get<0>(filter_threshold_[i]) - std::get<0>(filter_threshold_[j]));
            double delta_y = fabs(std::get<1>(filter_threshold_[i]) - std::get<1>(filter_threshold_[j]));
            double delta = std::hypot(delta_x, delta_y);
            if (delta < 15 && std::get<2>(filter_threshold_[i]) < std::get<2>(filter_threshold_[j])) {
                if (std::get<2>(filter_theshold_tmp[j]) > std::get<2>(filter_threshold_[i]))
                    filter_theshold_tmp[j] =
                        std::make_tuple(std::get<0>(filter_threshold_[j]), std::get<1>(filter_threshold_[j]),
                                        std::get<2>(filter_threshold_[i]));
            }
        }
    }

    filter_threshold_ = filter_theshold_tmp;
}

bool FilterNdtScore::RunFilter() {
    if (ndt_data_.size() < 1) {
        report_ += "filter ndt score error: no ndt data.\n";
        return false;
    }
    AlignThesholdIndex();
    return MergeThesholdByIndexAndDistance();
}

void FilterNdtScore::ReProcessNdtData() {
    PreProcessNdtData();
    if (index_by_threshold_.size() <= 1) return;
    if (std::get<2>(index_by_threshold_[0]) < DISTANCE_SUM_THESHOLD) {
        for (size_t i = 0; i <= (size_t)std::get<0>(index_by_threshold_[0]); i++) {
            ndt_data_[i].threshold_index = std::get<1>(index_by_threshold_[1]);
        }
    }
    for (size_t i = 1; i < index_by_threshold_.size() - 1; i++) {
        int theshold_index_pre = std::get<1>(index_by_threshold_[i - 1]);
        int theshold_index_aft = std::get<1>(index_by_threshold_[i + 1]);
        if (std::get<2>(index_by_threshold_[i]) < DISTANCE_SUM_THESHOLD * 1.5) {
            int min_index = theshold_index_pre < theshold_index_aft ? theshold_index_pre : theshold_index_aft;
            int index1 = std::get<0>(index_by_threshold_[i - 1]) + 1;
            int index2 = std::get<0>(index_by_threshold_[i]);
            for (int i = index1; i <= index2; i++) {
                ndt_data_[i].threshold_index = min_index;
            }
        }
    }
    if (std::get<2>(index_by_threshold_[index_by_threshold_.size() - 1]) < DISTANCE_SUM_THESHOLD) {
        for (size_t i = std::get<0>(index_by_threshold_[index_by_threshold_.size() - 2]) + 1; i < ndt_data_.size(); i++) {
            ndt_data_[i].threshold_index = std::get<1>(index_by_threshold_[index_by_threshold_.size() - 2]);
        }
    }
    FilterLowerTrans();
}

void FilterNdtScore::FilterLowerTrans() {
    float max_distance = 0;
    float current_theshold_index = 0;
    for (size_t i = 0; i < index_by_threshold_.size(); i++) {
        if (std::get<2>(index_by_threshold_[i]) > max_distance) {
            max_distance = std::get<2>(index_by_threshold_[i]);
            current_theshold_index = std::get<1>(index_by_threshold_[i]);
        }
    }
    for (size_t i = 0; i < index_by_threshold_.size(); i++) {
        uint index_begin = 0, index_end = 0;
        if (i == index_by_threshold_.size() - 1) {
            index_begin = std::get<0>(index_by_threshold_[i - 1]) + 1;
            index_end = ndt_data_.size();
        } else if (i == 0) {
            index_begin = 0;
            index_end = std::get<0>(index_by_threshold_[0]) + 1;
        } else {
            index_begin = std::get<0>(index_by_threshold_[i - 1]) + 1;
            index_end = std::get<0>(index_by_threshold_[i]) + 1;
        }
        if (std::get<1>(index_by_threshold_[i]) > current_theshold_index) {
            for (uint i = index_begin; i < index_end; i++) {
                ndt_data_[i].threshold_index = current_theshold_index;
            }
        }
    }
}

void FilterNdtScore::AlignThesholdIndex() {
    for (size_t i = 0; i < ndt_data_.size(); ++i) {
        float trans_pro = ndt_data_[i].trans_probability;
        if (trans_pro < MIN_THESHOLD) ndt_data_[i].threshold_index = 1;
        else if (trans_pro >= MAX_THESHOLD)
            ndt_data_[i].threshold_index = (int)((MAX_THESHOLD - MIN_THESHOLD) / THESHOLD_STEP + 2);
        else {
            ndt_data_[i].threshold_index = (int)((trans_pro - MIN_THESHOLD) / THESHOLD_STEP + 2);
        }

        if (i < 1) {
            ndt_data_[i].distance = 0;
        } else {
            ndt_data_[i].distance =
                std::sqrt((ndt_data_[i].xg - ndt_data_[i - 1].xg) * (ndt_data_[i].xg - ndt_data_[i - 1].xg) +
                          (ndt_data_[i].yg - ndt_data_[i - 1].yg) * (ndt_data_[i].yg - ndt_data_[i - 1].yg));
        }
    }
    PreProcessNdtData();
}

void FilterNdtScore::PreProcessNdtData() {
    if (origin_index_.size() > 0) {
        origin_index_.clear();
    }
    if (sum_suit_index_.size() > 0) {
        sum_suit_index_.clear();
    }
    if (index_by_threshold_.size() > 0) {
        index_by_threshold_.clear();
    }
    int current_theshold_index = 0;
    float distance_sum = 0;
    for (size_t i = 0; i < ndt_data_.size(); ++i) {
        origin_index_.push_back(std::make_tuple(i, ndt_data_[i].threshold_index, ndt_data_[i].distance));
        if (i < 1) {
            current_theshold_index = ndt_data_[i].threshold_index;
        } else {
            if (ndt_data_[i].threshold_index != current_theshold_index) {
                distance_sum = 0;
                if (index_by_threshold_.size() < 1 && i == 1) {
                    distance_sum = 0;
                } else if (index_by_threshold_.size() < 1 && i > 1) {
                    for (uint j = 0; j < i - 1; ++j) {
                        distance_sum += ndt_data_[j].distance;
                    }
                } else {
                    for (uint j = std::get<0>(index_by_threshold_.back()) + 1; j < i - 1; j++) {
                        distance_sum += ndt_data_[j].distance;
                    }
                }
                if (max_distance_sum_ < distance_sum) {
                    max_distance_sum_ = distance_sum;
                }
                index_by_threshold_.push_back(std::make_tuple(i - 1, current_theshold_index, distance_sum));
                current_theshold_index = ndt_data_[i].threshold_index;
                if (distance_sum > DISTANCE_SUM_THESHOLD) {
                    sum_suit_index_.push_back((index_by_threshold_.size() - 1));
                }
            }
        }
    }
}

bool FilterNdtScore::MergeThesholdByIndexAndDistance() {
    if (max_distance_sum_ < 3) {
        LOG(ERROR) << "Max distance is small than 3m, max is: " << max_distance_sum_;
    }
    if (sum_suit_index_.size() < 1) {
        LOG(ERROR) << "No suitable index to merge: " << sum_suit_index_.size();
        report_ += "filter ndt score error: no suitable index to merge.\n";
        return false;
    }
    if (sum_suit_index_[0] != 0) {
        int theshold_index = 100, min_theshold = 100;
        FindBeginMinTheshold(theshold_index, min_theshold);
        size_t near_index1 = std::get<0>(index_by_threshold_[sum_suit_index_[0] - 1]);
        if (near_index1 < 5) theshold_index = std::get<1>(index_by_threshold_[sum_suit_index_[0]]);
        for (size_t j = 0; j <= near_index1; j++) {
            if (theshold_index != 100) ndt_data_[j].threshold_index = theshold_index;
            else
                ndt_data_[j].threshold_index = min_theshold;
        }
        UpdateNdtData();
    } else if (sum_suit_index_[0] == 0) {
        UpdateNdtData();
    }
    if (sum_suit_index_.back() < (index_by_threshold_.size() - 1)) {
        int theshold_index = 100, min_theshold = 100;
        FindLastMinTheshold(theshold_index, min_theshold);
        size_t begin_index = std::get<0>(index_by_threshold_[sum_suit_index_.back()]) + 1;
        for (size_t j = begin_index; j < ndt_data_.size(); j++) {
            if (theshold_index != 100) ndt_data_[j].threshold_index = theshold_index;
            else
                ndt_data_[j].threshold_index = min_theshold;
        }
    }
    return true;
}

void FilterNdtScore::FindLastMinTheshold(int &theshold_index, int &min_theshold) {
    int near_index = sum_suit_index_.back() + 1;
    for (size_t j = near_index; j < index_by_threshold_.size(); j++) {
        int delta_index = std::get<1>(index_by_threshold_[j]) - std::get<1>(index_by_threshold_[j - 1]);
        if (std::get<1>(index_by_threshold_[j]) < min_theshold) min_theshold = std::get<1>(index_by_threshold_[j]);
        if (std::get<2>(index_by_threshold_[j]) > 0.5 && delta_index > 5 &&
            std::get<1>(index_by_threshold_[j]) < theshold_index) {
            theshold_index = std::get<1>(index_by_threshold_[j]);
        }
    }
}

void FilterNdtScore::FindBeginMinTheshold(int &theshold_index, int &min_theshold) {
    size_t near_index = sum_suit_index_[0];
    for (size_t j = 1; j < near_index; j++) {
        int delta_index = std::get<1>(index_by_threshold_[j]) - std::get<1>(index_by_threshold_[j - 1]);
        if (std::get<1>(index_by_threshold_[j]) < min_theshold) min_theshold = std::get<1>(index_by_threshold_[j]);
        if (std::get<2>(index_by_threshold_[j]) > 0.5 && delta_index > 10 &&
            std::get<1>(index_by_threshold_[j]) < theshold_index) {
            theshold_index = std::get<1>(index_by_threshold_[j]);
        }
    }
}

void FilterNdtScore::UpdateNdtData() {
    last_index_ = sum_suit_index_[0];
    for (size_t i = 1; i < sum_suit_index_.size(); i++) {
        if ((sum_suit_index_[i] - last_index_) != 1) {
            float theshold1 = std::get<1>(index_by_threshold_[last_index_]);
            float theshold2 = std::get<1>(index_by_threshold_[sum_suit_index_[i]]);
            size_t origin_index_1 = std::get<0>(index_by_threshold_[last_index_]);
            size_t origin_index_2 = std::get<0>(index_by_threshold_[sum_suit_index_[i] - 1]);
            if (theshold1 < theshold2) {
                for (size_t j = origin_index_1 + 1; j <= origin_index_2; j++) {
                    ndt_data_[j].threshold_index = theshold1;
                }
            } else {
                for (size_t j = origin_index_1 + 1; j <= origin_index_2; j++) {
                    ndt_data_[j].threshold_index = theshold2;
                }
            }
        }
        last_index_ = sum_suit_index_[i];
    }
}

bool FilterNdtScore::GenerateReport(std::string &report, bool verbose) {
    report = report_;
    return true;
}

}  // namespace pipeline
}  // namespace mapping
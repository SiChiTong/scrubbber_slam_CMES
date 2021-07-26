//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_COMMON_FUNC_H
#define MAPPING_COMMON_FUNC_H

#include <sstream>
#include <string>
#include <vector>

namespace mapping::common {

/// 分割字符串
inline void SplitString(const std::string &s, char delim, std::vector<std::string> &result) {
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        result.push_back(item);
    }
}

/// UTC时间转Unix时间
inline double Utc2Unix(int utc_w, double utc_s) {
    double UUDT = 315964800 - 18;
    double WEEKS = 7 * 24 * 3600;
    double time_scale = 2.5e-4;
    double utcs = utc_s * time_scale;
    return UUDT + utc_w * WEEKS + utcs;
}

}  // namespace mapping::common

#endif  // MAPPING_COMMON_FUNC_H

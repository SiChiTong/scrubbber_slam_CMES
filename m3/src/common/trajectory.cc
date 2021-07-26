//
// Created by gaoxiang on 2020/8/21.
//

#include "trajectory.h"

namespace mapping::common {

Trajectory::Trajectory(IdType id, const std::string& bag_name) {
    this->trajectory_id = id;
    this->bag_name = bag_name;
    // this->collected_time = BagDate2Time(time);
}

Trajectory::Trajectory() {}

Trajectory::TimeT BagDate2Time(const std::string& bag_time) {
    assert(bag_time.size() == 14);

    tm t;
    t.tm_year = std::stoi(bag_time.substr(0, 4));
    t.tm_mon = std::stoi(bag_time.substr(4, 6));
    t.tm_mday = std::stoi(bag_time.substr(6, 8));
    t.tm_hour = std::stoi(bag_time.substr(8, 10));
    t.tm_min = std::stoi(bag_time.substr(10, 12));
    t.tm_sec = std::stoi(bag_time.substr(12, 14));
    return t;
}

}  // namespace mapping::common
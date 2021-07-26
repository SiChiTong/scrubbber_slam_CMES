//
// Created by gaoxiang on 19-9-2.
//

#include "pipeline/pipeline_context.h"

#include <glog/logging.h>
#include <fstream>

namespace mapping::pipeline {

using std::endl;

void TaskInfo::Save(std::ofstream &out) {
    out << map_name << " " << time_usage << " " << valid_bags << " " << total_bag_size << " " << length << " " << area
        << " " << success << " " << total_keyframes << " " << checkout_passed << " " << checkin_passed << " " << endl;

    char loc_date_s[200] = {0};
    sprintf(loc_date_s, "%d%02d%02d-%02d:%02d:%02d", 1900 + start_time.tm_year, 1 + start_time.tm_mon,
            start_time.tm_mday, start_time.tm_hour, start_time.tm_min, start_time.tm_sec);
    out << loc_date_s << endl;

    char loc_date_e[200] = {0};
    sprintf(loc_date_e, "%d%02d%02d-%02d:%02d:%02d", 1900 + end_time.tm_year, 1 + end_time.tm_mon, end_time.tm_mday,
            end_time.tm_hour, end_time.tm_min, end_time.tm_sec);
    out << loc_date_e << endl;
}

void TaskInfo::Load(std::ifstream &in) {
    in >> map_name >> time_usage >> valid_bags >> total_bag_size >> length >> area >> success >> total_keyframes >>
        checkout_passed >> checkin_passed;
    char buff[200];
    in.getline(buff, 200);
    LOG(INFO) << std::string(buff);
    int year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
    if (sscanf(buff, "%d%02d%02d-%02d:%02d:%02d", &year, &month, &day, &hour, &min, &sec) >= 0) {
        start_time.tm_year = 1900 + year;
        start_time.tm_mon = month;
        start_time.tm_mday = day;
        start_time.tm_hour = hour;
        start_time.tm_min = min;
        start_time.tm_sec = sec;
    }

    in.getline(buff, 200);
    if (sscanf(buff, "%d%02d%02d-%02d:%02d:%02d", &year, &month, &day, &hour, &min, &sec) >= 0) {
        end_time.tm_year = 1900 + year;
        end_time.tm_mon = month;
        end_time.tm_mday = day;
        end_time.tm_hour = hour;
        end_time.tm_min = min;
        end_time.tm_sec = sec;
    }
    LOG(INFO) << "Load map name " << map_name << " with pcd url: " << pcd_url;
}

void PipelineContext::LogAndReport(const std::string &log) {
    LOG(INFO) << log;
    report_ += log + "\n";
}

}  // namespace mapping::pipeline

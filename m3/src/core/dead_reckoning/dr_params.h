//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_DR_PARAMS_H
#define MAPPING_DR_PARAMS_H

#include "io/yaml_io.h"

namespace mapping::core {

struct DrParams {
    void LoadFromYAML(const io::YAML_IO &yaml_file) {
        ratio_left = yaml_file.GetValue<double>("dr_params", "ratio_left");
        ratio_right = yaml_file.GetValue<double>("dr_params", "ratio_right");
        static_gyro_var = yaml_file.GetValue<double>("dr_params", "static_gyro_var");
        static_acc_var = yaml_file.GetValue<double>("dr_params", "static_acc_var");
        frequence = yaml_file.GetValue<double>("dr_params", "frequence");
        out_file_path = yaml_file.GetValue<std::string>("dr_params", "out_file_path");
    }

    double ratio_left = 1.0;
    double ratio_right = 1.0;
    double static_gyro_var = 0.02;
    double static_acc_var = 0.05;
    float frequence = 10;
    std::string out_file_path;
};

}  // namespace mapping::core

#endif  // MAPPING_DR_PARAMS_H

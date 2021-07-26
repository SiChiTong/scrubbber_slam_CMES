//
// Created by wangqi on 19-7-21.
//

#ifndef MAPPING_NDT_MATCHING_PARAMS_H
#define MAPPING_NDT_MATCHING_PARAMS_H

#include "io/yaml_io.h"

namespace mapping::core {

struct NdtMatchingParams {
    void LoadFromYAML(const io::YAML_IO &yaml) {
        resolution = yaml.GetValue<double>("ndt_matching_params", "resolution");
        step_size = yaml.GetValue<double>("ndt_matching_params", "step_size");
        transformation_epsilon =
            yaml.GetValue<double>("ndt_matching_params", "transformation_epsilon");
        maximum_iterations = yaml.GetValue<double>("ndt_matching_params", "maximum_iterations");
        stable_theshold = yaml.GetValue<double>("ndt_matching_params", "maximum_iterations");
        use_omp = yaml.GetValue<bool>("ndt_matching_params", "use_omp");
        degeneracy_theshold = yaml.GetValue<double>("ndt_matching_params", "degeneracy_theshold");
    }

    double resolution = 0;
    double step_size = 0;
    double transformation_epsilon = 0;
    double maximum_iterations = 0;
    double stable_theshold = 0;
    double degeneracy_theshold = 100;
    bool use_omp = false;
};

}  // namespace mapping::core

#endif  // MAPPING_NDT_MATCHING_PARAMS_H

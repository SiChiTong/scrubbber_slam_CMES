//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_OPTIMIZATION_PARAMS_H
#define MAPPING_OPTIMIZATION_PARAMS_H

#include "io/yaml_io.h"

namespace mapping::pipeline {

struct OptimizationParams {
    void LoadFromYAML(const io::YAML_IO &yaml) {
        matching_weight = yaml.GetValue<double>("optimization_params", "matching_weight");
        gps_weight = yaml.GetValue<double>("optimization_params", "gps_weight");
        loop_weight = yaml.GetValue<double>("optimization_params", "loop_weight");
        dr_weight = yaml.GetValue<double>("optimization_params", "dr_weight");
        dr_yaw_weight = yaml.GetValue<double>("optimization_params", "dr_yaw_weight");
        matching_chi2_th = yaml.GetValue<double>("optimization_params", "matching_chi2_th");
        dr_chi2_th = yaml.GetValue<double>("optimization_params", "dr_chi2_th");
        gps_chi2_th = yaml.GetValue<double>("optimization_params", "gps_chi2_th");
        loop_chi2_th = yaml.GetValue<double>("optimization_params", "loop_chi2_th");
        gps_height_noise_ratio = yaml.GetValue<double>("optimization_params", "gps_height_noise_ratio");
        loop_type = yaml.GetValue<double>("optimization_params", "loop_type");

        dr_continous_num = yaml.GetValue<double>("optimization_params", "dr_continous_num");
        lidar_continous_num = yaml.GetValue<double>("optimization_params", "lidar_continous_num");
        set_height_zero = yaml.GetValue<bool>("feature_matching_params", "set_height_zero");
        gps_status_set = yaml.GetValue<int>("gps_status");

        with_height = yaml.GetValue<bool>("optimization_params", "with_height");
        with_global_rotation = yaml.GetValue<bool>("optimization_params", "with_global_rotation");

        use_rk_dr = yaml.GetValue<bool>("optimization_params", "use_rk_dr");
        use_rk_matching = yaml.GetValue<bool>("optimization_params", "use_rk_matching");
    }

    double matching_weight{};             // lidar factor æé
    double gps_weight{};                  // GPSæé
    double loop_weight{};                 // åç¯æé
    double dr_weight{};                   // DR æé
    double dr_yaw_weight{};               // DR YAWæé
    double matching_chi2_th{};            // matchingéå¼
    double dr_chi2_th{};                  // dr éå¼
    double gps_chi2_th{};                 // gps éå¼
    double loop_chi2_th{};                // loop éå¼
    double gps_height_noise_ratio = 5.0;  // GPSé«åº¦å¼åªå£°åæ°
    int dr_continous_num = 5;             // dr çº¦æè¿ç»­å¸§æ°
    int lidar_continous_num = 5;          // lidar çº¦æè¿ç»­å¸§æ°
    bool set_height_zero = false;         // æ¯å¦éå¶é«åº¦ä¸ºé¶
    int gps_status_set = 3;               // GPS ç¶æ
    bool with_height = true;              // ä½¿ç¨å±çº§åé«åº¦çº¦æ
    bool with_global_rotation = false;    // ä½¿ç¨å¨å±æè½¬çº¦æï¼å¯è½å¯¼è´é®é¢ï¼
    bool use_rk_matching = true;          // matchingä¸­ä½¿ç¨robust kernel
    bool use_rk_dr = true;                // drä¸­ä½¿ç¨robust kernel
    int loop_type{};
};

}  // namespace mapping::pipeline
#endif  // MAPPING_OPTIMIZATION_PARAMS_H

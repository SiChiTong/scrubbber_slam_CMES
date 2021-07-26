//
// Created by pengguoqi on 19-7-16.
//

#include "xml_io.h"
#include "param_config_io.h"
#include "tinyxml2.h"

#include <glog/logging.h>

using namespace tinyxml2;
namespace mapping::io {

int XML_IO::ParseXml(const std::string &xml_parse_check) {
    std::string xml_parse_path;
    int parse_index = CheckXmlVector(xml_path_, xml_parse_check);
    if (parse_index < 0) {
        return -1;
    }

    if (parse_index >= (int)xml_path_.size()) {
        return -2;
    } else {
        xml_parse_path = xml_path_[parse_index];
    }
    XMLDocument doc;
    if (XML_SUCCESS == doc.LoadFile(xml_parse_path.c_str())) {
        XMLElement *root = doc.FirstChildElement();
        for (XMLElement *elem = root->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement()) {
            const char *elemName = elem->Attribute("name");
            TransParamToStruct(elem, elemName);
        }
        doc.SaveFile(xml_parse_path.c_str());
    } else {
        return -3;
    }
    return 1;
}

void XML_IO::TransParamToStruct(XMLElement *elem, const char *elemName) {
    if (strcmp(elemName, "perception_lidar_x_offset_top_center") == 0) {
        vehicle_calib_params_.perception_lidar_x_offset_top_center = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "perception_lidar_y_offset_top_center") == 0) {
        vehicle_calib_params_.perception_lidar_y_offset_top_center = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "perception_lidar_z_offset_top_center") == 0) {
        vehicle_calib_params_.perception_lidar_z_offset_top_center = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "perception_lidar_roll_top_center") == 0) {
        vehicle_calib_params_.perception_lidar_roll_top_center = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "perception_lidar_pitch_top_center") == 0) {
        vehicle_calib_params_.perception_lidar_pitch_top_center = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "perception_lidar_yaw_top_center") == 0) {
        vehicle_calib_params_.perception_lidar_yaw_top_center = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "odom_ratio_left") == 0) {
        vehicle_calib_params_.odom_ratio_left = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "odom_ratio_right") == 0) {
        vehicle_calib_params_.odom_ratio_right = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "antenna_x") == 0) {
        vehicle_calib_params_.antenna_x = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "antenna_y") == 0) {
        vehicle_calib_params_.antenna_y = atof(elem->Attribute("value"));
    }
    if (strcmp(elemName, "antenna_angle") == 0) {
        vehicle_calib_params_.antenna_angle = atof(elem->Attribute("value"));
    }
    vehicle_calib_params_.lidar_type = 16;
    return;
}

int XML_IO::CheckXmlVector(const std::vector<std::string> &xml_path, const std::string &check_name) {
    if (xml_path.size() < 1) {
        return -1;
    } else {
        int calib_file_size = 0;
        uint calib_index = 0;
        for (uint i = 0; i < xml_path.size(); i++) {
            std::string::size_type pos_calib = xml_path[i].find(check_name);
            if (pos_calib != std::string::npos) {
                calib_file_size++;
                calib_index = i;
            }
        }
        if (calib_file_size == 0) return -1;
        else if (calib_file_size > 1)
            return -2;
        else if (calib_file_size == 1)
            return calib_index;
        else
            return -1;
    }
}

int XML_IO::ParseXml() {
    if (conf_path_.size() < 2) return -1;
    int localization_conf = -1;
    int vehicle_sensor_colibration_conf = -1;



    /// NOTE @pengguoqi 这里拼错了啊大哥
    for (uint i = 0; i < conf_path_.size(); i++) {
        if (conf_path_[i].find("localization.conf") != std::string::npos) {
            localization_conf = i;
        } else if (conf_path_[i].find("vehicle_sensor_calibration.conf") != std::string::npos) {
            vehicle_sensor_colibration_conf = i;
        } else {
            LOG(ERROR) << "cannot determine this conf: " << conf_path_[i];
        }
    }

    if (-1 == vehicle_sensor_colibration_conf || -1 == localization_conf) {
        LOG(ERROR) << "cannot find localization.conf or vehicle_sensor_calibration.conf";
        return -2;
    }

    std::string is_irad = lidar_launch_.substr(lidar_launch_.find('.') + 1, lidar_launch_.length() - 1);
    ParamConfigManager param_config_manager;
    param_config_manager.Reset();

    ModelConfig *vehicle_sensor_colibration = new ModelConfig();
    if (param_config_manager.ModuleConfigLoad(conf_path_.at(vehicle_sensor_colibration_conf)) == false) {
        return -3;
    }

    param_config_manager.GetModelConfig("sensor_calibration", &vehicle_sensor_colibration);
    vehicle_sensor_colibration->GetValue("lidar_1_xoffset",
                                         &vehicle_calib_params_.perception_lidar_x_offset_top_center);
    vehicle_sensor_colibration->GetValue("lidar_1_yoffset",
                                         &vehicle_calib_params_.perception_lidar_y_offset_top_center);
    vehicle_sensor_colibration->GetValue("lidar_1_zoffset",
                                         &vehicle_calib_params_.perception_lidar_z_offset_top_center);
    vehicle_sensor_colibration->GetValue("lidar_1_roll", &vehicle_calib_params_.perception_lidar_roll_top_center);
    vehicle_sensor_colibration->GetValue("lidar_1_pitch", &vehicle_calib_params_.perception_lidar_pitch_top_center);
    vehicle_sensor_colibration->GetValue("lidar_1_yaw", &vehicle_calib_params_.perception_lidar_yaw_top_center);
    
    if (is_irad == "irad" && lidar_factory_ == "suteng") {
        LOG(INFO) << "parse irad lidar driver";
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_xoffset", &vehicle_calib_params_.perception_lidar_x_offset_top_center);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_yoffset", &vehicle_calib_params_.perception_lidar_y_offset_top_center);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_z_ground_offset", &vehicle_calib_params_.perception_lidar_z_offset_top_center);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_roll", &vehicle_calib_params_.perception_lidar_roll_top_center);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_pitch", &vehicle_calib_params_.perception_lidar_pitch_top_center);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_yaw", &vehicle_calib_params_.perception_lidar_yaw_top_center);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_prerotaxis_0", &vehicle_calib_params_.pre_rot_axis_0);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_prerotaxis_1", &vehicle_calib_params_.pre_rot_axis_1);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_prerotaxis_2", &vehicle_calib_params_.pre_rot_axis_2);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_prerotdegree_0", &vehicle_calib_params_.pre_rot_degree_0);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_prerotdegree_1", &vehicle_calib_params_.pre_rot_degree_1);
        vehicle_sensor_colibration->GetValue("rs_lidar_driver_top_prerotdegree_2", &vehicle_calib_params_.pre_rot_degree_2);
    }

    delete vehicle_sensor_colibration;


    param_config_manager.Reset();
    ModelConfig *localization_config = new ModelConfig();
    if (param_config_manager.ModuleConfigLoad(conf_path_.at(localization_conf)) == false) {
        return -4;
    }

    param_config_manager.GetModelConfig("navbox", &localization_config);
    localization_config->GetValue("odom_factor_left", &vehicle_calib_params_.odom_ratio_left);
    localization_config->GetValue("odom_factor_right", &vehicle_calib_params_.odom_ratio_right);
    localization_config->GetValue("gnss_offset_x", &vehicle_calib_params_.antenna_x);
    localization_config->GetValue("gnss_offset_y", &vehicle_calib_params_.antenna_y);
    localization_config->GetValue("heading_install_deviation", &vehicle_calib_params_.antenna_angle);
    localization_config->GetValue("max_gyro_var", &vehicle_calib_params_.static_gyro_var);
    localization_config->GetValue("max_acce_var", &vehicle_calib_params_.static_acc_var);
    // vehicle_calib_params_.odom_ratio_left = 1.0;
    // vehicle_calib_params_.odom_ratio_right = 1.0;
    // vehicle_calib_params_.antenna_x = 0.0;
    // vehicle_calib_params_.antenna_y = 0.0;
    // vehicle_calib_params_.antenna_angle = 0.0;

    if (lidar_factory_ == "velodyne") {
        return ParseVelodyneXml();
    } else if (lidar_factory_ == "suteng") {
        vehicle_calib_params_.lidar_type = 80;
        return 0;
    } else if (lidar_factory_ == "hesai") {
        return ParseHeSaiXml();
    } else {
        vehicle_calib_params_.lidar_type = 16;
        return -6;
    }
}

int XML_IO::ParseVelodyneXml() {
    XMLDocument doc;
    if (XML_SUCCESS == doc.LoadFile(lidar_launch_.c_str())) {
        XMLElement *root = doc.FirstChildElement();
        for (XMLElement *elem = root->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement()) {
            const char *elemName = elem->Attribute("name");
            if (strcmp(elemName, "velodyne_pointcloud_1/model") == 0) {
                std::string model = elem->Attribute("value");
                if (model == "VLP16") vehicle_calib_params_.lidar_type = 16;
                else if (model == "VLP32")
                    vehicle_calib_params_.lidar_type = 32;
                else if (model == "VLP32C")
                    vehicle_calib_params_.lidar_type = 32;
                else if (model == "64E_S2" || model == "64E_S3S" ||
                         model == "64E_S3D_STRONGEST" || model == "HDL64E" ||
                         model == "64E_S3D_LAST" || model == "64E_S3D_DUAL")
                    vehicle_calib_params_.lidar_type = 64;
                else {
                    return -5;
                }
            }
        }
        doc.SaveFile(lidar_launch_.c_str());
    }
    return 0;
}

/*
40P <------> lidar_type = "Pandar40P"
64  <------> lidar_type = "Pandar64"    (默认)
20A <------> lidar_type = "Pandar20A"
20B <------> lidar_type = "Pandar20B"
QT <------> lidar_type = "PandarQT"
40M <------> lidar_type = "Pandar40M"
32 <------> lidar_type = "PandarXT-32"
16 <------> lidar_type = "PandarXT-16"
*/

int XML_IO::ParseHeSaiXml() {
    XMLDocument doc;
    if (XML_SUCCESS == doc.LoadFile(lidar_launch_.c_str())) {
        XMLElement *root = doc.FirstChildElement();
        for (XMLElement *elem = root->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement()) {
            const char *elemName = elem->Attribute("name");
            if (strcmp(elemName, "lidar_type") == 0) {
                std::string model = elem->Attribute("default");
                if (model == "Pandar40P" || model == "Pandar40M") vehicle_calib_params_.lidar_type = 40;
                else if (model == "Pandar64")
                    vehicle_calib_params_.lidar_type = 64;
                else if (model == "Pandar20A" || model == "Pandar20B")
                    vehicle_calib_params_.lidar_type = 20;
                else if (model == "PandarXT-32")
                    vehicle_calib_params_.lidar_type = 32;
                else if (model == "PandarXT-16")
                    vehicle_calib_params_.lidar_type = 16;
                else if (model == "PandarQT")
                    vehicle_calib_params_.lidar_type = 16;  // QT
                else if (model == "")
                    vehicle_calib_params_.lidar_type = 64;  // QT
                else {
                    return -5;
                }
            }
        }
        doc.SaveFile(lidar_launch_.c_str());
    }
    return 0;
}

}  // namespace mapping::io
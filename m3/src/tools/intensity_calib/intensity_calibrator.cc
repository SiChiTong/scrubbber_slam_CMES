#include "intensity_calibrator.h"
#include "tools/save_file/save_file.h"

// a flaw of c++11, fixed in c++17
constexpr double calibration::IntensityCalibrator::uniform_dist_;

namespace calibration {

using namespace mapping;

void IntensityCalibrator::Calibration(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes) {
    mapping::common::PointCloudXYZIHIRBS::Ptr cloud(new mapping::common::PointCloudXYZIHIRBS);
    mapping::common::PointCloudXYZIHIRBS::Ptr filtered_cloud(new mapping::common::PointCloudXYZIHIRBS);
    for (auto it = keyframes->begin(); it != keyframes->end(); it++) {
        if (it->first - keyframes->begin()->first > params_.useful_frames_num) continue;
        mapping::common::PointCloudXYZIHIRBS::Ptr key_frame_points(new mapping::common::PointCloudXYZIHIRBS);
        mapping::common::PointCloudXYZIHIRBS::Ptr global_points(new mapping::common::PointCloudXYZIHIRBS);
        std::vector<mapping::common::PacketsMsgPtr> three_packets;
        three_packets.push_back(it->second.packets_ptr_vec[0]);
        three_packets.push_back(it->second.packets_ptr_vec[1]);
        three_packets.push_back(it->second.packets_ptr_vec[2]);
        perception_interface_.SetDrPose(it->second.poses_buffer);
        perception_interface_.PointConvertAndHeightProcess(three_packets, key_frame_points);
        pcl::transformPointCloud(*key_frame_points, *global_points, it->second.optimized_pose);
        *cloud += *global_points;
    }
    // self-defined filter
    int select_index = 0;
    int inner_index = 0;

    // 以16为循环，每5个16里取1个
    for (auto& pt : cloud->points) {
        inner_index++;
        if (inner_index > 16) {
            select_index++;
            inner_index -= 16;
            if (select_index > 5) {
                select_index -= 5;
            }
        }
        if (select_index == 1) {
            filtered_cloud->push_back(pt);
        }
    }
    pcl::io::savePCDFile("./data/before_calib.pcd", *filtered_cloud);
    InitModel(filtered_cloud);
    Run();

    Correction(filtered_cloud);
    SaveCloud("./data/intensity_calibrated_cloud", *filtered_cloud);
    LOG(INFO) << "Intensity calibration done!";
}

void IntensityCalibrator::Calibration32(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes) {
    mapping::common::PointCloudXYZIHIRBS::Ptr cloud(new mapping::common::PointCloudXYZIHIRBS);
    mapping::common::PointCloudXYZIHIRBS::Ptr filtered_cloud(new mapping::common::PointCloudXYZIHIRBS);

    for (auto it = keyframes->begin(); it != keyframes->end(); it++) {
        if (it->first - keyframes->begin()->first > params_.useful_frames_num) {
            continue;
        }

        mapping::common::PointCloudXYZIHIRBS::Ptr key_frame_points(new mapping::common::PointCloudXYZIHIRBS);
        mapping::common::PointCloudXYZIHIRBS::Ptr global_points(new mapping::common::PointCloudXYZIHIRBS);

        perception_interface_.SetDrPose(it->second.poses_buffer);
        perception_interface_.ProcessScan32(it->second.packets_ptr_vec[1], key_frame_points);

        pcl::transformPointCloud(*key_frame_points, *global_points, it->second.optimized_pose);
        *cloud += *global_points;
    }

    // self-defined filter
    int select_index = 0;
    int inner_index = 0;

    // NOTE 这边改成32
    int min_intensity = 999, max_intensity = 0;
    for (auto& pt : cloud->points) {
        inner_index++;
        if (inner_index > 32) {
            select_index++;
            inner_index -= 32;
            if (select_index > 5) {
                select_index -= 5;
            }
        }
        if (pt.intensity > 99) {
            continue;
        }

        if (select_index == 1) {
            filtered_cloud->push_back(pt);
        }

        if (pt.intensity > max_intensity) {
            max_intensity = pt.intensity;
        }
        if (pt.intensity < min_intensity) {
            min_intensity = pt.intensity;
        }
    }

    LOG(INFO) << "intensity min: " << min_intensity << ", max: " << max_intensity;

    pcl::io::savePCDFile("./data/before_calib.pcd", *filtered_cloud);
    InitModel(filtered_cloud);
    Run();
    Correction(filtered_cloud);
    SaveCloud("./data/intensity_calibrated_cloud", *filtered_cloud);
    LOG(INFO) << "Intensity calibration done!";
}

void IntensityCalibrator::Calibration(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes,
                                      mapping::tools::ReadFile* read_file) {
    mapping::common::PointCloudXYZIHIRBS::Ptr cloud(new mapping::common::PointCloudXYZIHIRBS);
    mapping::common::PointCloudXYZIHIRBS::Ptr filtered_cloud(new mapping::common::PointCloudXYZIHIRBS);
    for (auto it = keyframes->begin(); it != keyframes->end(); it++) {
        if (it->first - keyframes->begin()->first > params_.useful_frames_num) continue;
        // load points from read_file
        std::shared_ptr<std::vector<common::PointCloudType::Ptr>> kf_cloud(
            new std::vector<common::PointCloudType::Ptr>);
        read_file->KeyFramesPointsByID(save_file_path_ + "/map.db", {it->second.id_}, kf_cloud);

        if (kf_cloud == nullptr || kf_cloud->empty()) {
            continue;
        }

        auto key_frame_points = (*kf_cloud)[0];
        mapping::common::PointCloudXYZIHIRBS::Ptr global_points(new mapping::common::PointCloudXYZIHIRBS);
        pcl::transformPointCloud(*key_frame_points, *global_points, it->second.optimized_pose);
        *cloud += *global_points;
    }

    // self-defined filter
    int select_index = 0;
    int inner_index = 0;

    // 以16为循环，每5个16里取1个
    for (auto& pt : cloud->points) {
        inner_index++;
        if (inner_index > 16) {
            select_index++;
            inner_index -= 16;
            if (select_index > 5) {
                select_index -= 5;
            }
        }

        if (pt.intensity > 99) {
            continue;
        }

        if (select_index == 1) {
            filtered_cloud->push_back(pt);
        }
    }
    pcl::io::savePCDFile("./data/before_calib.pcd", *filtered_cloud);
    InitModel(filtered_cloud);
    Run();

    Correction(filtered_cloud);
    SaveCloud("./data/intensity_calibrated_cloud", *filtered_cloud);
    LOG(INFO) << "Intensity calibration done!";
}

void IntensityCalibrator::Calibration32(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes,
                                        mapping::tools::ReadFile* read_file) {
    mapping::common::PointCloudXYZIHIRBS::Ptr cloud(new mapping::common::PointCloudXYZIHIRBS);
    mapping::common::PointCloudXYZIHIRBS::Ptr filtered_cloud(new mapping::common::PointCloudXYZIHIRBS);

    for (auto it = keyframes->begin(); it != keyframes->end(); it++) {
        if (it->first - keyframes->begin()->first > params_.useful_frames_num) {
            continue;
        }

        // load points from read_file
        std::shared_ptr<std::vector<common::PointCloudType::Ptr>> kf_cloud(
            new std::vector<common::PointCloudType::Ptr>);
        read_file->KeyFramesPointsByID(save_file_path_ + "/map.db", {it->second.id_}, kf_cloud);

        if (kf_cloud == nullptr || kf_cloud->empty()) {
            continue;
        }

        auto key_frame_points = (*kf_cloud)[0];
        mapping::common::PointCloudXYZIHIRBS::Ptr global_points(new mapping::common::PointCloudXYZIHIRBS);
        pcl::transformPointCloud(*key_frame_points, *global_points, it->second.optimized_pose);
        *cloud += *global_points;
    }

    // self-defined filter
    int select_index = 0;
    int inner_index = 0;

    // NOTE 这边改成32
    int min_intensity = 999, max_intensity = 0;
    for (auto& pt : cloud->points) {
        inner_index++;
        if (inner_index > 32) {
            select_index++;
            inner_index -= 32;
            if (select_index > 5) {
                select_index -= 5;
            }
        }
        if (pt.intensity > 99) {
            continue;
        }

        if (select_index == 1) {
            filtered_cloud->push_back(pt);
        }

        if (pt.intensity > max_intensity) {
            max_intensity = pt.intensity;
        }
        if (pt.intensity < min_intensity) {
            min_intensity = pt.intensity;
        }
    }

    LOG(INFO) << "intensity min: " << min_intensity << ", max: " << max_intensity;

    pcl::io::savePCDFile("./data/before_calib.pcd", *filtered_cloud);
    InitModel(filtered_cloud);
    Run();
    Correction(filtered_cloud);
    SaveCloud("./data/intensity_calibrated_cloud", *filtered_cloud);
    LOG(INFO) << "Intensity calibration done!";
}

void IntensityCalibrator::Correction(mapping::common::PointCloudXYZIHIRBS::Ptr& cloud) {
    for (uint i = 0; i < cloud->points.size(); i++) {
        if (cloud->points[i].range < params_.maximum_range && cloud->points[i].intensity < 100) {
            CHECK(cloud->points[i].ring < beam_mapping_final_.size());
            CHECK(beam_mapping_final_.at(cloud->points[i].ring).cols() > cloud->points[i].intensity);
            if (params_.if_run_correction) {
                cloud->points[i].intensity_vi =
                    static_cast<int>(beam_mapping_final_.at(cloud->points[i].ring)(cloud->points[i].intensity));
            }
        }
        cloud->points[i].intensity_vi =
            std::min((int)(intensity_coeff_ * cloud->points[i].intensity_vi), params_.maximum_intensity_vi);
    }
}

void IntensityCalibrator::Run() {
    int step = 0;
    LOG(INFO) << "start calibration!";
    while (Expectation() > params_.convergence && step < params_.iterate_step) {
        double error = Maximization();
        ++step;
        LOG(INFO) << "Iteration " << step << " Has M step error " << error;
    }
    LOG(WARNING) << " save file path  = " << save_file_path_;
    if (params_.if_save_calib_info) {
        SaveProbability(save_file_path_ + "beam_model.txt", beam_model_);
        SaveBeamInfo(save_file_path_ + "beam_info.txt", beam_info_);
    }

    for (const auto& beam : beam_model_) {
        beam_mapping_final_.emplace_back(beam.GetMapping());
    }
    if (params_.if_save_calib_info) {
        SaveMappings(save_file_path_ + "beam_mappings.txt", beam_mapping_final_);
    }
}

double IntensityCalibrator::Expectation() {
    // LOG(INFO) << "RUNNING E STEP " ;
    auto buffered = cell_model_;
    for (auto& cell : cell_model_) cell.setZero();
    for (auto& variance : cell_info_) variance = std::make_tuple(0, 0.0, 0.0);
    for (const auto& m : measurements_) {
        CHECK(m.k < cell_model_.size()) << " Index too high";
        cell_model_.at(m.k) += beam_model_.at(m.b).atLog(m.a);
        AddSample(cell_info_.at(m.k), m);
    }

    for (auto& cell : cell_model_) {
        auto val = cell.maxCoeff();
        for (uint i = 0; i < MAX_REMITTANCE_READING; i++) {
            if (std::isnan(cell(i))) {
                cell(i) = params_.epsilon;
            } else if (cell(i) - val >= std::log(params_.precision) - std::log(MAX_REMITTANCE_READING)) {
                cell(i) = std::exp(cell(i) - val);
            } else {
                cell(i) = params_.epsilon;
            }
        }
    }

    double diff = 0.0;
    for (int i = 0; i < cell_model_.size(); i++) {
        cell_model_.at(i) /= cell_model_.at(i).sum();  // Average
        double error = (cell_model_.at(i) - buffered.at(i)).norm();
        if (std::isnan(error)) {
            LOG(WARNING) << " error is nan value!!!, i = ";
            diff = 0.01;
            // throw std::runtime_error(" Illegal value");
        } else {
            diff += error;
        }
    }
    double averaged_error = diff / cell_model_.size();
    // LOG(WARNING) << "Current E STEP averaged error" << averaged_error;
    return averaged_error;
}

void IntensityCalibrator::AddSample(CellVariance& variance, const Measurement& m) {
    int temp_count = std::get<0>(variance);
    ++temp_count;
    float temp_average = std::get<1>(variance);
    float temp_variance = std::get<2>(variance);
    float delta = m.a - temp_average;
    temp_average += delta / temp_count;
    float delta2 = m.a - temp_average;
    temp_variance = ((temp_count - 1) * temp_variance + delta * delta2) / temp_count;
    variance = std::make_tuple(temp_count, temp_average, temp_variance);
}

double IntensityCalibrator::Maximization() {
    // LOG(INFO) << "RUNNING M STEP " ;
    auto buffered = beam_model_;
    BeamCountings countings(beam_model_.size());
    beam_info_.resize(beam_model_.size());
    for (auto& counting : countings) {
        counting = BeamCounting(MAX_REMITTANCE_READING, MAX_REMITTANCE_READING);
        counting.setZero();
    }
    for (auto& beam : beam_info_) {
        beam.resize(MAX_REMITTANCE_READING);
    }

    for (const auto& m : measurements_) {
        auto temp_info = cell_info_.at(m.k);
        if (std::get<0>(temp_info) >= params_.minimum_points_in_cell) {
            countings.at(m.b).col(m.a) += cell_model_.at(m.k);
            auto& beam = beam_info_.at(m.b);
            beam[m.a] = std::make_tuple(m.a, std::get<1>(beam[m.a]) + 1);
        }
    }
    std::vector<std::unordered_set<int>> results = HandleBeams(countings, beam_info_);

    double diff = 0;
    for (uint i = 0; i < countings.size(); i++) {
        CHECK_EQ(countings.at(i).rows(), MAX_REMITTANCE_READING);
        beam_model_.at(i) = BeamProbability(countings.at(i), params_.beam_probability_type);
        if (i < params_.corrected_beam_num) {
            for (uint k = 0; k < 100; k++) {
                if (k == 0 || k == 1 || k == 2 || results[i].find(k) == results[i].end()) {
                    beam_model_.at(i).probability.col(k) = buffered.at(i).probability.col(k);
                }
            }
        } else {
            for (uint k = 0; k < 100; k++) {
                beam_model_.at(i).probability.col(k) = buffered.at(i).probability.col(k);
            }
        }
        beam_model_.at(i).Normalize(params_.beam_probability_type);
        diff += (beam_model_.at(i).probability - buffered.at(i).probability).norm();
    }
    double res = diff / countings.size();
    return res;
}

std::vector<std::unordered_set<int>> IntensityCalibrator::HandleBeams(BeamCountings& countings,
                                                                      const BeamInfo& beam_info) {
    // beam intensity info
    std::vector<std::unordered_set<int>> intensity_sets;
    for (uint j = 0; j < beam_info.size(); j++) {
        auto beam = beam_info.at(j);
        if (params_.if_output_info) {
            for (uint u = 0; u < MAX_REMITTANCE_READING; u++) {
                LOG(INFO) << "beam id = " << j << ", intensity value = " << u << ", count = " << std::get<1>(beam[u]);
            }
        }
        std::sort(beam.begin(), beam.end(),
                  [](std::tuple<int, int> a, std::tuple<int, int> b) { return std::get<1>(a) > std::get<1>(b); });
        // LOG(INFO)<<"beam id = "<<j<<", top "<<params_.corrected_intensity_num<<" intensity measurements is: ";
        std::unordered_set<int> intensity_set;
        for (uint k = 0; k < params_.corrected_intensity_num; k++) {
            if (params_.if_output_info)
                LOG(INFO) << " intensity = " << std::get<0>(beam[k]) << ", count = " << std::get<1>(beam[k]);
            intensity_set.emplace(std::get<0>(beam[k]));
        }
        intensity_sets.push_back(intensity_set);
        intensity_set.clear();
    }
    return intensity_sets;
}

void IntensityCalibrator::LoadCloud(const std::string& filename) {
    mapping::common::PointCloudXYZIHIRBS::Ptr cloud(new mapping::common::PointCloudXYZIHIRBS);
    pcl::io::loadPCDFile(filename, *cloud);
    InitModel(cloud);
}

void IntensityCalibrator::InitModel(mapping::common::PointCloudXYZIHIRBS::Ptr cloud) {
    filtered_cloud_.reset(new mapping::common::PointCloudXYZIHIRBS);
    for (const auto& pt : cloud->points) {
        if (pt.range <= params_.maximum_range) {
            filtered_cloud_->push_back(pt);
        }
    }
    CalculateCoeff(filtered_cloud_);
    LOG(WARNING) << "intensity coeff = " << intensity_coeff_;
    LOG(INFO) << " From original points " << cloud->size() << " now filtered " << filtered_cloud_->size();

    measurements_ = LoadMeasurement(filtered_cloud_, params_.voxel_size, params_.maximum_range);
    LOG(INFO) << "Loaded measurements. Downsampled size of voxel " << filtered_cloud_->size();

    cell_model_.resize(filtered_cloud_->size());
    cell_info_.resize(filtered_cloud_->size());

    // Init cell
    for (auto& cell_prob : cell_model_) {
        cell_prob = Eigen::Matrix<double, MAX_REMITTANCE_READING, 1>::Ones() * uniform_dist_;
    }
    LOG(INFO) << "Initialized Cell Probability as uniform " << cell_model_.size();

    // Init beam
    int num_of_rings = 0;
    for (const auto& m : measurements_) {
        num_of_rings = std::max(num_of_rings, m.b);
    }

    num_of_rings++;
    LOG(INFO) << "Number of beams " << num_of_rings;

    for (int beam_i = 0; beam_i < num_of_rings; beam_i++) {
        beam_model_.emplace_back(MAX_REMITTANCE_READING, params_.std_var, params_.epsilon);
    }
    LOG(INFO) << "Initialization completed";
}

void IntensityCalibrator::CalculateCoeff(mapping::common::PointCloudXYZIHIRBS::Ptr input) {
    // CHECK_GT(input->points.size(), 500);
    if (input->points.size() < 500) {
        LOG(WARNING) << "points size is too small";
        intensity_coeff_ = 2.0;
    }
    double averaged_intensity = 0;
    int points_num = 0;
    for (const auto& pt : input->points) {
        if (pt.z > (-0.2) && pt.z < (0.2) && points_num < 500) {
            averaged_intensity = (averaged_intensity * points_num + pt.intensity) / (points_num + 1);
            points_num++;
        }
    }
    // CHECK_GT(points_num, 499);
    // CHECK_GT(averaged_intensity, 2.5);
    if (points_num < 499 || averaged_intensity < 3.0) {
        intensity_coeff_ = 2.0;
    } else {
        if (averaged_intensity > 30) {
            intensity_coeff_ = 1.0;
        } else {
            intensity_coeff_ = 4.0 - 0.1 * averaged_intensity;
        }
    }
    LOG(INFO) << "intensity coeff = " << intensity_coeff_;
}

}  // namespace calibration

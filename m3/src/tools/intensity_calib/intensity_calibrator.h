#ifndef INTENSITY_CALIBRATOR_H_
#define INTENSITY_CALIBRATOR_H_

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

#include "common/common.h"
#include "tools/intensity_calib/common_struct.h"
#include "tools/intensity_calib/io.h"
#include "tools/perception_interface/interface.h"

/// 实际最大反射率值
static constexpr int MAX_REMITTANCE_READING = 100;

// forward declare
namespace mapping {
namespace tools {
class ReadFile;
}
}  // namespace mapping

namespace calibration {

/**
 * 激光反射率标定
 *
 * Velodyne自身反射率为0-100（漫反射率），物体实际的反射率也为0-100, 未标定时，二者之间存在差异
 * 对于任意一条线，测量到的读数与实际反射率之间的映射称为beam mapping，为线数 x 100的矩阵
 */
class IntensityCalibrator {
   public:
    // constructor
    IntensityCalibrator() {}
    IntensityCalibrator(const mapping::io::YAML_IO& yaml) : perception_interface_(yaml) {
        params_.convergence = yaml.GetValue<double>("intensity_calib", "convergence");
        params_.precision = yaml.GetValue<double>("intensity_calib", "precision");
        params_.voxel_size = yaml.GetValue<double>("intensity_calib", "voxel_size");
        params_.std_var = yaml.GetValue<double>("intensity_calib", "std_var");
        params_.epsilon = yaml.GetValue<double>("intensity_calib", "epsilon");
        params_.maximum_range = yaml.GetValue<double>("intensity_calib", "maximum_range");
        params_.iterate_step = yaml.GetValue<int>("intensity_calib", "iterate_step");
        params_.corrected_beam_num = yaml.GetValue<int>("intensity_calib", "corrected_beam_num");
        params_.corrected_intensity_num = yaml.GetValue<int>("intensity_calib", "corrected_intensity_num");
        params_.minimum_points_in_cell = yaml.GetValue<int>("intensity_calib", "minimum_points_in_cell");
        params_.maximum_intensity_vi = yaml.GetValue<int>("intensity_calib", "maximum_intensity_vi");
        params_.beam_probability_type = yaml.GetValue<int>("intensity_calib", "beam_probability_type");
        params_.useful_frames_num = yaml.GetValue<int>("intensity_calib", "useful_frames_num");
        params_.if_run_EM = yaml.GetValue<bool>("intensity_calib", "if_run_EM");
        params_.if_output_info = yaml.GetValue<bool>("intensity_calib", "if_output_info");
        params_.if_run_correction = yaml.GetValue<bool>("intensity_calib", "if_run_correction");
        params_.if_save_corrected_pcd = yaml.GetValue<bool>("intensity_calib", "if_save_corrected_pcd");
        params_.if_save_calib_info = yaml.GetValue<bool>("intensity_calib", "if_save_calib_info");
        save_file_path_ = yaml.GetValue<std::string>("data_fetching", "local_data_path");

        intensity_coeff_ = 1.0;
        filtered_cloud_.reset(new mapping::common::PointCloudXYZIHIRBS);
    }
    // destructor
    ~IntensityCalibrator() {}

    // Iterative calculation of E-step and M-step
    // After convergence, save results

    // 这是在data_preparing阶段使用的
    void Calibration(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes);
    void Calibration32(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes);

    // 这是在后端优化完成时使用的
    void Calibration(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes,
                     mapping::tools::ReadFile* read_file);
    void Calibration32(std::shared_ptr<std::map<int, mapping::common::KeyFrame>> keyframes,
                       mapping::tools::ReadFile* read_file);

    // 修正一个点云
    void Correction(mapping::common::PointCloudXYZIHIRBS::Ptr& cloud);

   private:
    void Run();
    // Load point cloud
    void LoadCloud(const std::string& filename);

    void InitModel(mapping::common::PointCloudXYZIHIRBS::Ptr cloud);

    double GetCoeff() { return intensity_coeff_; }

    // E-step
    double Expectation();
    // M-step
    double Maximization();

    std::vector<std::unordered_set<int>> HandleBeams(BeamCountings& countings, const BeamInfo& beam_info);

    void AddSample(CellVariance& variance, const Measurement& m);

    void CalculateCoeff(mapping::common::PointCloudXYZIHIRBS::Ptr input);

    static constexpr double uniform_dist_ = 1.0 / MAX_REMITTANCE_READING;

    // params
    IntensityCalibParams params_;
    std::string save_file_path_;

    // data and results
    BeamModel beam_model_;
    BeamMappings beam_mapping_final_;
    CellModel cell_model_;
    CellInfo cell_info_;
    BeamInfo beam_info_;
    Measurements measurements_;

    mapping::PerceptionInterface perception_interface_;

    mapping::common::PointCloudXYZIHIRBS::Ptr filtered_cloud_;

    double intensity_coeff_;
};

}  // namespace calibration

#endif
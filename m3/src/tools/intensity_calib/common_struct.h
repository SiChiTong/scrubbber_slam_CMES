#define PCL_NO_PRECOMPILE
#ifndef COMMON_STRUCT_H_
#define COMMON_STRUCT_H_

#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>
#include <string>

#include <glog/logging.h>

namespace calibration {

struct PDF {
    // constructor
    PDF() : mu(0.0), var(1.0) {}
    PDF(const double& mu, const double& var) : mu(mu), var(var) {}

    // Raw density
    double at(const double& x) { return std::exp(-0.5 * (x - mu) * (x - mu) / var / var); }
    double mu, var;
};

struct IntensityCalibParams {
    double convergence;
    double precision;
    double voxel_size;
    double std_var;
    double epsilon;
    double maximum_range;
    int iterate_step;
    int corrected_beam_num;
    int corrected_intensity_num;
    int minimum_points_in_cell;
    int maximum_intensity_vi;
    int beam_probability_type;
    int useful_frames_num;
    bool if_run_EM;
    bool if_output_info;
    bool if_run_correction;
    bool if_save_corrected_pcd;
    bool if_save_calib_info;
};

struct IntensityCalibDirs {
    std::string original_pcd_dir;
    std::string save_pcd_dir;
    std::string calib_file_dir;
    std::string prob_path;
    std::string mappings_path;
    std::string beam_info_path;
};

// core data structure
/**@ intensity distribution of each cell @**/
using CellProbability = Eigen::VectorXd;
/**@
 * accumulated counting without normalization for each beam based on the intensity distribution of each cell
 * row data used to update beam model
 * @**/
using BeamCounting = Eigen::MatrixXd;

/**@ beam map, for each measured value ,there is a response value, which means maximum likehood.@**/
using BeamMapping = Eigen::RowVectorXi;

// Columns is Measured Value, 0 till 100
// Rows is Distribution, 0 till 100
struct BeamProbability {
    /**@ epsilon is important, preventing from -nan in log calculation @**/
    BeamProbability(int size, double var = 1.0, double epsilon = 0.1, int type = 0) {
        probability = Eigen::MatrixXd(size, size);
        for (int i = 0; i < size; i++) {
            PDF pdf(static_cast<double>(i), var);
            for (int j = 0; j < size; j++) {
                probability(i, j) = pdf.at(static_cast<double>(j)) + epsilon;
            }
        }
        Normalize(type);
        log_probability = probability.array().log().matrix();
    }

    BeamProbability(BeamCounting input, int type = 0) {
        probability = input;
        Normalize(type);
        log_probability = probability.array().log().matrix();
    }

    BeamMapping GetMapping() const {
        BeamMapping result(probability.rows());
        int useful_num = 0;
        for (uint i = 0; i < probability.rows(); i++) {
            auto val = probability.col(i).maxCoeff(&result(i));
            /** Be careful! **/
            if (val < 0.5 || std::fabs(result(i) - i) > 5) {
                result(i) = i;
            } else {
                useful_num++;
            }
        }
        return result;
    }

    void Normalize(int type) {
        CHECK(probability.rows() > 0);
        if (0 == type) {
            for (uint i = 0; i < 100; i++) {
                double temp_sum = probability.col(i).sum();
                if (temp_sum < 1) {
                    PDF pdf(static_cast<double>(i), 1);
                    for (uint j = 0; j < 100; j++) {
                        probability(j, i) = pdf.at(static_cast<double>(j)) + 0.001;
                    }
                }
            }
            Eigen::RowVectorXd col_wise_sum = probability.colwise().sum();
            probability.array().rowwise() /= col_wise_sum.array();
            CHECK_NEAR(probability.col(0).sum(), 1.0, 1e-4);
        } else {
            Eigen::VectorXd row_wise_sum = probability.rowwise().sum();
            probability.array().colwise() /= row_wise_sum.array();
            CHECK_NEAR(probability.row(0).sum(), 1.0, 1e-4);
        }
    }

    CellProbability atLog(int measured) { return log_probability.col(measured); }

    Eigen::MatrixXd probability, log_probability;
};

/**@ probability distribution for all beams @**/
using BeamModel = std::vector<BeamProbability>;
/**@ map of real value and measured value for all beams @**/
using BeamMappings = std::vector<BeamMapping>;
/**@ a set of beam counting@**/
using BeamCountings = std::vector<BeamCounting>;

/**@
 * using a combination of measured intensity, ring id and index to represent measurement
 * index is calculated by voxel grid in PCL
 * @**/
struct Measurement {
    // constructor
    Measurement(int a, int b, int k) : a(a), b(b), k(k) {}
    // measured, beam, index
    int a, b, k;
};
/**@ all measurements @**/
using Measurements = std::vector<Measurement>;
/**@ fill all measurements into corresponding cells @**/
using CellModel = std::vector<CellProbability>;

/**@ intermidiate data for analysis @**/

/**@ count, average, variance for each cell @**/
using CellVariance = std::tuple<int, float, float>;
using CellInfo = std::vector<CellVariance>;
/**@ intensity value, count @**/
using IntensityCount = std::tuple<int, int>;
/**@ intensity count for each beam @**/
using BeamIntensity = std::vector<IntensityCount>;
using BeamInfo = std::vector<BeamIntensity>;
}  // namespace calibration

#endif
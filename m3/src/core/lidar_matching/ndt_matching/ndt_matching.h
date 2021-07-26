// Created by wangqi on 19-7-21.
//
#ifndef MAPPING_NDT_MATCHING_H
#define MAPPING_NDT_MATCHING_H

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "core/lidar_matching/lidar_matching.h"
#include "core/lidar_matching/ndt_matching/ndt_matching_params.h"
#include "core/ndt_omp/include/pclomp/ndt_omp.h"

#include <pcl/registration/ndt.h>

namespace mapping::core {

class NdtMatching : public LidarMatching {
   public:
    struct NdtMatchingInfo {
        NdtMatchingInfo(SE3 &p, double trans, double fit, double degeneracy, int iter)
            : pose(p), trans_probability(trans), fitness_score(fit), degeneracy_eigen_value(degeneracy), iteration(iter) {}
        SE3 pose;
        double trans_probability;
        double fitness_score;
        double degeneracy_eigen_value;
        int iteration;
    };

    explicit NdtMatching(const NdtMatchingParams &params);

    ~NdtMatching() override{};

   public:
    void SetFirstFixedPose(const SE3 &first_pose) override;

    void SetInputCloud(const common::PointCloudType::Ptr &input, const int kf_id) override;

    void SetStartID(int frame_id) override { start_id_ = frame_id; }

    void SetPredictPose(const SE3 &delta_pose) override {
        dr_delta_pose_ = delta_pose;
        guess_type_ = "DR_PREDICTION";
    }

    void Run() override;

    std::map<int, SE3> GetMatchingPoses() override;

    std::map<int, V6d> GetMatchingNoise() override;

    std::map<int, float> GetDegeneracyEigenValue() override;

    void SetNdtMatchingInputSource(common::PointCloudType::Ptr &input);

    void SetNdtMatchingInputTarget(common::PointCloudType::Ptr &input);

    void NdtAlign(SE3 &input_guess);

    SE3 GetAlignMatrix() const { return align_matrix_; }

    float GetAlignTransProbability() const { return trans_probability_; }

    float GetAlignFitness() const { return fitness_score_; }

    float GetLambdaMin() const { return lambda_min_; }

    float GetLambdaMax() const { return lambda_max_; }

    int GetNdtIteration() const { return iteration_; }

   private:
    void SetGuessPose();

    void RunNdtMatching(common::PointCloudType::Ptr points_target, common::PointCloudType::Ptr points_source,
                        SE3 &guess_trans);

    void Initialization(const SE3 &input);

    void ClearPointsCloud();

    void ComputeMatchingResult();

   private:
    NdtMatchingParams params_;

    bool use_omp_ = false;
    pcl::NormalDistributionsTransform<common::PointType, common::PointType> ndt_matcher_;
    pclomp::NormalDistributionsTransform<common::PointType, common::PointType> ndt_omp_matcher_;

    common::PointCloudType::Ptr target_input_ = boost::make_shared<common::PointCloudType>();
    common::PointCloudType::Ptr source_input_ = boost::make_shared<common::PointCloudType>();
    common::PointCloudType::Ptr local_map_ptr_ = boost::make_shared<common::PointCloudType>();

    SE3 previous_pose_;
    SE3 current_pose_;
    SE3 guess_pose_;
    SE3 dr_delta_pose_;
    SE3 delta_pose_;
    std::string guess_type_ = "MOTION_PREDICTION";

    double trans_probability_ = 0;
    double fitness_score_ = 1000;
    float lambda_min_;
    float lambda_max_;
    int iteration_ = 0;
    SE3 align_matrix_;

    int frame_id_ = -1;
    int start_id_ = -1;

    std::deque<common::PointCloudType::Ptr> local_map_deque_;
    unsigned int local_map_deque_size_ = 50;

    std::vector<NdtMatchingInfo> matching_info_;
    std::vector<float> noises_vec_;

    std::map<int, SE3> matching_poses_;
    std::map<int, V6d> matching_noises_;

    std::map<int, float> degeneracy_eigen_value_;
};
}  // namespace mapping::core

#endif  // MAPPING_NDT_MATCHING_H

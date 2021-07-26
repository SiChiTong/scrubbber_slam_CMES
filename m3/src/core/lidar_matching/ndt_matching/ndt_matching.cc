//
// Created by idriver on 19-7-21.
//

#include "core/lidar_matching/ndt_matching/ndt_matching.h"
#include <glog/logging.h>
#include <pcl/registration/impl/ndt.hpp>

namespace mapping::core {

using namespace mapping::common;

NdtMatching::NdtMatching(const NdtMatchingParams &params) : params_(params) {
    use_omp_ = params_.use_omp;
    if (use_omp_) {
        ndt_omp_matcher_.setResolution(params_.resolution);
        ndt_omp_matcher_.setStepSize(params_.step_size);
        ndt_omp_matcher_.setTransformationEpsilon(params_.transformation_epsilon);
        ndt_omp_matcher_.setMaximumIterations(params_.maximum_iterations);
        ndt_omp_matcher_.setNeighborhoodSearchMethod(pclomp::NeighborSearchMethod::DIRECT7);
    } else {
        ndt_matcher_.setResolution(params_.resolution);
        ndt_matcher_.setStepSize(params_.step_size);
        ndt_matcher_.setTransformationEpsilon(params_.transformation_epsilon);
        ndt_matcher_.setMaximumIterations(params_.maximum_iterations);
    }

    frame_id_ = -1;
    start_id_ = -1;
}

void NdtMatching::Initialization(const SE3 &input) {
    previous_pose_ = input;
    current_pose_ = input;
    guess_pose_ = input;
    delta_pose_ = SE3();
}

void NdtMatching::SetFirstFixedPose(const SE3 &first_pose) { Initialization(first_pose); }

void NdtMatching::SetInputCloud(const PointCloudType::Ptr &input, const int kf_id) {
    PointCloudType::Ptr input_posterior = boost::make_shared<PointCloudType>();
    frame_id_ = kf_id;
    if (input->size() < 30) {
        LOG(ERROR) << "input PointCloud is empty!!!";
        return;
    }

    if (start_id_ == frame_id_) {
        pcl::transformPointCloud(*input, *input_posterior, guess_pose_.matrix().cast<float>());
        target_input_ = input_posterior;
        *source_input_ = *input;
    } else {
        target_input_ = local_map_ptr_;
        *source_input_ = *input;
    }
}

void NdtMatching::SetGuessPose() {
    if (guess_type_ == "DR_PREDICTION") {
        guess_pose_ = previous_pose_ * dr_delta_pose_;
    } else if (guess_type_ == "MOTION_PREDICTION") {
        guess_pose_ = previous_pose_ * delta_pose_;
    } else
        guess_pose_ = previous_pose_ * delta_pose_;
}

void NdtMatching::SetNdtMatchingInputSource(PointCloudType::Ptr &input) {
    if (use_omp_) {
        ndt_omp_matcher_.setInputSource(input);
    } else {
        ndt_matcher_.setInputSource(input);
    }
}

void NdtMatching::SetNdtMatchingInputTarget(PointCloudType::Ptr &input) {
    if (use_omp_) {
        ndt_omp_matcher_.setInputTarget(input);
    } else {
        ndt_matcher_.setInputTarget(input);
    }
}

void NdtMatching::NdtAlign(SE3 &input_guess) {
    PointCloudType::Ptr output_points_from_source = boost::make_shared<PointCloudType>();
    if (use_omp_) {
        ndt_omp_matcher_.align(*output_points_from_source, input_guess.matrix().cast<float>());
        iteration_ = ndt_omp_matcher_.getFinalNumIteration();
        fitness_score_ = ndt_omp_matcher_.getFitnessScore();
        trans_probability_ = ndt_omp_matcher_.getTransformationProbability();
        M4d m = ndt_omp_matcher_.getFinalTransformation().cast<double>();
        Quat q(m.block<3, 3>(0, 0));
        q.normalize();
        SE3 output_matrix(q, m.block<3, 1>(0, 3));

        lambda_min_ = ndt_omp_matcher_.lambda5();
        lambda_max_ = ndt_omp_matcher_.lambda0();
        align_matrix_ = output_matrix;
    } else {
        ndt_matcher_.align(*output_points_from_source, input_guess.matrix().cast<float>());
        iteration_ = ndt_matcher_.getFinalNumIteration();
        fitness_score_ = ndt_matcher_.getFitnessScore();
        trans_probability_ = ndt_matcher_.getTransformationProbability();
        M4d m = (ndt_matcher_.getFinalTransformation().cast<double>());
        Quat q(m.block<3, 3>(0, 0));
        q.normalize();
        SE3 output_matrix(q, m.block<3, 1>(0, 3));

        // lambda_min_ = ndt_matcher_.lambda5();
        // lambda_max_ = ndt_matcher_.lambda0();
        align_matrix_ = output_matrix;
    }
}

void NdtMatching::RunNdtMatching(PointCloudType::Ptr points_target, PointCloudType::Ptr points_source,
                                 SE3 &guess_trans) {
    PointCloudType::Ptr target_cloud = boost::make_shared<PointCloudType>();
    PointCloudType::Ptr source_cloud = boost::make_shared<PointCloudType>();

    pcl::VoxelGrid<common::PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.1, 0.1, 0.1);

    voxel_grid_filter.setInputCloud(points_target);
    voxel_grid_filter.filter(*target_cloud);
    voxel_grid_filter.setInputCloud(points_source);
    voxel_grid_filter.filter(*source_cloud);

    SetNdtMatchingInputSource(source_cloud);
    SetNdtMatchingInputTarget(target_cloud);
    NdtAlign(guess_trans);
}

std::map<int, SE3> NdtMatching::GetMatchingPoses() {
    for (size_t i = 0; i < matching_info_.size(); ++i) {
        matching_poses_.insert({i + start_id_, matching_info_[i].pose});
    }
    return matching_poses_;
}

std::map<int, V6d> NdtMatching::GetMatchingNoise() {
    size_t num_poses = noises_vec_.size();
    for (size_t i = 0; i < num_poses; ++i) {
        V6d noise;
        float noise_factor = 1.0 / noises_vec_.at(i) * 0.3;
        noise << noise_factor * 0.05, noise_factor * 0.05, noise_factor * 0.05, noise_factor * 0.008,
            noise_factor * 0.008, noise_factor * 0.008, matching_noises_.emplace(start_id_ + i, noise);
    }
    return matching_noises_;
}

std::map<int, float> NdtMatching::GetDegeneracyEigenValue() {
    for (size_t i = 0; i < matching_info_.size(); ++i) {
        degeneracy_eigen_value_.insert({i + 1, matching_info_[i].degeneracy_eigen_value});
    }
    return degeneracy_eigen_value_;
}

void NdtMatching::ComputeMatchingResult() {
    PointCloudType::Ptr pose_points = boost::make_shared<PointCloudType>();
    current_pose_ = align_matrix_;
    float degeneracy_eigen_value = 200;
    LOG(INFO) << "lambda_max_ : " << lambda_max_ << ", lambda_min_ : " << lambda_min_;
    if ((lambda_max_ / lambda_min_) > params_.degeneracy_theshold) {
        degeneracy_eigen_value = 100;
        LOG(INFO) << "----degenerate----";
    } else {
        degeneracy_eigen_value = 200;
    }
    pcl::transformPointCloud(*source_input_, *pose_points, current_pose_.matrix().cast<float>());
    matching_info_.emplace_back(current_pose_, trans_probability_, fitness_score_, degeneracy_eigen_value, iteration_);
    noises_vec_.push_back(trans_probability_);

    delta_pose_ = previous_pose_.inverse() * current_pose_;
    previous_pose_ = current_pose_;

    ClearPointsCloud();
    if (local_map_deque_.size() < local_map_deque_size_) {
        local_map_deque_.push_back(pose_points);
    }
    LOG(INFO) << "current_pose : " << current_pose_.matrix()(0, 3) << ", " 
              << current_pose_.matrix()(1, 3) << ", " << current_pose_.matrix()(2, 3);
    for (auto &lm : local_map_deque_) {
        *local_map_ptr_ += *lm;
    }
    if (local_map_deque_size_ == local_map_deque_.size()) {
        local_map_deque_.pop_front();
    }
}

void NdtMatching::ClearPointsCloud() {
    target_input_->clear();
    source_input_->clear();
    local_map_ptr_->clear();
}

void NdtMatching::Run() {
    SetGuessPose();
    RunNdtMatching(target_input_, source_input_, guess_pose_);
    ComputeMatchingResult();
}

}  // namespace mapping::core

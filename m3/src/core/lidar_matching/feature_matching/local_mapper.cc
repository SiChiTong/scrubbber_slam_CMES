#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <glog/logging.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/mapping_math.h"
#include "core/lidar_matching/feature_matching/local_mapper.h"
#include "core/lidar_matching/feature_matching/local_mapper_impl.h"

using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;  // 每个误差项优化变量维度为6，误差值维度为6
using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;  // 线性求解器类型

namespace mapping::core {

using namespace mapping::common;

LocalMapper::LocalMapper(FeatureMatchingParams &params) : impl_(new LocalMapperImpl) {
    impl_->params_ = params;
    Initialization();
}

LocalMapper::~LocalMapper() {}

void LocalMapper::Initialization() {
    for (int i = 0; i < 6; ++i) {
        impl_->transform_last_[i] = 0;
        impl_->transform_sum_[i] = 0;
        impl_->transform_incre_[i] = 0;
        impl_->transform_tobe_mapped_[i] = 0;
        impl_->transform_bef_mapped_[i] = 0;
        impl_->transform_aft_mapped_[i] = 0;
    }

    impl_->if_set_height_zero_ = false;

    impl_->latest_keyframe_pose_ = SE3();
    impl_->dr_pose_ = SE3();

    impl_->laser_cloud_corner_last_.reset(new PointCloudType());
    impl_->laser_cloud_surf_last_.reset(new PointCloudType());
    impl_->laser_outlier_cloud_.reset(new PointCloudType());
    impl_->laser_cloud_surf_total_last_.reset(new PointCloudType());

    impl_->laser_cloud_corner_last_DS_.reset(new PointCloudType());
    impl_->laser_cloud_surf_last_DS_.reset(new PointCloudType());
    impl_->laser_cloud_outlier_last_DS_.reset(new PointCloudType());
    impl_->laser_cloud_surf_total_last_DS_.reset(new PointCloudType());

    impl_->laser_cloud_corner_from_map_.reset(new PointCloudType());
    impl_->laser_cloud_surf_from_map_.reset(new PointCloudType());
    impl_->laser_cloud_corner_from_map_DS_.reset(new PointCloudType());
    impl_->laser_cloud_surf_from_map_DS_.reset(new PointCloudType());

    impl_->laser_cloud_corner_from_map_DS_num_ = 0;
    impl_->laser_cloud_surf_from_map_DS_num_ = 0;
    impl_->laser_cloud_corner_last_DS_num_ = 0;
    impl_->laser_cloud_surf_total_last_DS_num_ = 0;

    impl_->latest_frame_ID_ = 0;

    impl_->cloud_keyposes_3D_.reset(new PointCloudType());
    impl_->cloud_keyposes_6D_.reset(new pcl::PointCloud<PointTypePose>());
    impl_->surrounding_keyposes_.reset(new PointCloudType());
    impl_->surrounding_keyposes_DS_.reset(new PointCloudType());

    impl_->kdtree_corner_from_map_.reset(new pcl::KdTreeFLANN<common::PointType>());
    impl_->kdtree_surf_from_map_.reset(new pcl::KdTreeFLANN<common::PointType>());
    impl_->kdtree_surrounding_keyposes_.reset(new pcl::KdTreeFLANN<common::PointType>());

    impl_->downsize_filter_corner_.setLeafSize(0.2, 0.2, 0.2);
    impl_->downsize_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
    impl_->downsize_filter_outlier_.setLeafSize(0.4, 0.4, 0.4);
    impl_->downsize_filter_surrounding_keyposes_.setLeafSize(1.0, 1.0, 1.0);

    impl_->laser_cloud_ori_.reset(new PointCloudType());
    impl_->coeff_sel_.reset(new PointCloudType());
    impl_->correspond_dis_.reset(new PointCloudType());

    impl_->fitness_score_ = 0;

    impl_->matA0_ = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
    impl_->matB0_ = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
    impl_->matX0_ = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

    impl_->matA1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    impl_->matD1_ = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    impl_->matV1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

    impl_->is_degenerate_ = false;
    impl_->matP_ = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    impl_->keyframe_delta_th_ = impl_->params_.keyframe_delta_th;

    impl_->keyframe_id_ = -1;
    impl_->previous_id_ = -1;
    impl_->start_keyframe_id_ = -1;
    impl_->trajectory_id_ = -1;

    impl_->latest_corner_keyframe_cloud_.reset(new PointCloudType());
    impl_->latest_surf_keyframe_cloud_.reset(new PointCloudType());
    impl_->latest_surf_keyframe_cloud_DS_.reset(new PointCloudType());
    impl_->near_history_surf_keyframe_cloud_.reset(new PointCloudType());
    impl_->near_history_surf_keyframe_cloud_DS_.reset(new PointCloudType());

    impl_->kdtree_history_keyposes_.reset(new pcl::KdTreeFLANN<common::PointType>());

    impl_->downsize_filter_history_keyframes_.setLeafSize(0.2, 0.2, 0.2);

    impl_->loop_closure_enable_ = impl_->params_.loop_closure_enable;
    impl_->potential_loop_flag_ = false;
    impl_->a_loop_is_closed_ = false;

    impl_->loop_current_processing_time_ = 0;
    impl_->loop_last_processing_time_ = 0;

    impl_->solver_ = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    impl_->optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    impl_->optimizer_->setAlgorithm(impl_->solver_);
}

void LocalMapper::SetFirstFixedPose(const SE3 &first_pose) {
    common::PoseRPY pose = SE3ToRollPitchYaw(first_pose);

    if (impl_->if_set_height_zero_) {
        impl_->transform_aft_mapped_[0] = 0;
        impl_->transform_aft_mapped_[2] = 0;
        impl_->transform_aft_mapped_[4] = 0;
    } else {
        impl_->transform_aft_mapped_[0] = pose.pitch;
        impl_->transform_aft_mapped_[2] = pose.roll;
        impl_->transform_aft_mapped_[4] = pose.z;
    }

    impl_->transform_aft_mapped_[1] = pose.yaw;
    impl_->transform_aft_mapped_[3] = pose.y;
    impl_->transform_aft_mapped_[5] = pose.x;
}

void LocalMapper::TransformAssociateToMap() {
    float x1 = cos(impl_->transform_sum_[1]) * (impl_->transform_bef_mapped_[3] - impl_->transform_sum_[3]) -
               sin(impl_->transform_sum_[1]) * (impl_->transform_bef_mapped_[5] - impl_->transform_sum_[5]);
    float y1 = impl_->transform_bef_mapped_[4] - impl_->transform_sum_[4];
    float z1 = sin(impl_->transform_sum_[1]) * (impl_->transform_bef_mapped_[3] - impl_->transform_sum_[3]) +
               cos(impl_->transform_sum_[1]) * (impl_->transform_bef_mapped_[5] - impl_->transform_sum_[5]);

    float x2 = x1;
    float y2 = cos(impl_->transform_sum_[0]) * y1 + sin(impl_->transform_sum_[0]) * z1;
    float z2 = -sin(impl_->transform_sum_[0]) * y1 + cos(impl_->transform_sum_[0]) * z1;

    impl_->transform_incre_[3] = cos(impl_->transform_sum_[2]) * x2 + sin(impl_->transform_sum_[2]) * y2;
    impl_->transform_incre_[4] = -sin(impl_->transform_sum_[2]) * x2 + cos(impl_->transform_sum_[2]) * y2;
    impl_->transform_incre_[5] = z2;

    float sbcx = sin(impl_->transform_sum_[0]);
    float cbcx = cos(impl_->transform_sum_[0]);
    float sbcy = sin(impl_->transform_sum_[1]);
    float cbcy = cos(impl_->transform_sum_[1]);
    float sbcz = sin(impl_->transform_sum_[2]);
    float cbcz = cos(impl_->transform_sum_[2]);

    float sblx = sin(impl_->transform_bef_mapped_[0]);
    float cblx = cos(impl_->transform_bef_mapped_[0]);
    float sbly = sin(impl_->transform_bef_mapped_[1]);
    float cbly = cos(impl_->transform_bef_mapped_[1]);
    float sblz = sin(impl_->transform_bef_mapped_[2]);
    float cblz = cos(impl_->transform_bef_mapped_[2]);

    float salx = sin(impl_->transform_aft_mapped_[0]);
    float calx = cos(impl_->transform_aft_mapped_[0]);
    float saly = sin(impl_->transform_aft_mapped_[1]);
    float caly = cos(impl_->transform_aft_mapped_[1]);
    float salz = sin(impl_->transform_aft_mapped_[2]);
    float calz = cos(impl_->transform_aft_mapped_[2]);

    float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) -
                cbcx * sbcy *
                    (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                     calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) -
                cbcx * cbcy *
                    (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                     calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
    impl_->transform_tobe_mapped_[0] = -asin(srx);

    float srycrx =
        sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) +
                calx * saly * sblx) -
        cbcx * cbcy *
            ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) +
             (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) +
        cbcx * sbcy *
            ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) +
             (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
    float crycrx =
        sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) +
                calx * caly * sblx) +
        cbcx * cbcy *
            ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) +
             (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) -
        cbcx * sbcy *
            ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) +
             (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);

    impl_->transform_tobe_mapped_[1] =
        atan2(srycrx / cos(impl_->transform_tobe_mapped_[0]), crycrx / cos(impl_->transform_tobe_mapped_[0]));

    float srzcrx =
        (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                                              calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) -
        (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                                              calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) +
        cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    float crzcrx =
        (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                                              calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) -
        (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                                              calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) +
        cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
    impl_->transform_tobe_mapped_[2] =
        atan2(srzcrx / cos(impl_->transform_tobe_mapped_[0]), crzcrx / cos(impl_->transform_tobe_mapped_[0]));

    x1 = cos(impl_->transform_tobe_mapped_[2]) * impl_->transform_incre_[3] -
         sin(impl_->transform_tobe_mapped_[2]) * impl_->transform_incre_[4];
    y1 = sin(impl_->transform_tobe_mapped_[2]) * impl_->transform_incre_[3] +
         cos(impl_->transform_tobe_mapped_[2]) * impl_->transform_incre_[4];
    z1 = impl_->transform_incre_[5];

    x2 = x1;
    y2 = cos(impl_->transform_tobe_mapped_[0]) * y1 - sin(impl_->transform_tobe_mapped_[0]) * z1;
    z2 = sin(impl_->transform_tobe_mapped_[0]) * y1 + cos(impl_->transform_tobe_mapped_[0]) * z1;

    impl_->transform_tobe_mapped_[3] = impl_->transform_aft_mapped_[3] - (cos(impl_->transform_tobe_mapped_[1]) * x2 +
                                                                          sin(impl_->transform_tobe_mapped_[1]) * z2);
    impl_->transform_tobe_mapped_[4] = impl_->transform_aft_mapped_[4] - y2;
    impl_->transform_tobe_mapped_[5] = impl_->transform_aft_mapped_[5] - (-sin(impl_->transform_tobe_mapped_[1]) * x2 +
                                                                          cos(impl_->transform_tobe_mapped_[1]) * z2);
}

void LocalMapper::TransformUpdate() {
    for (int i = 0; i < 6; i++) {
        impl_->transform_bef_mapped_[i] = impl_->transform_sum_[i];
        impl_->transform_aft_mapped_[i] = impl_->transform_tobe_mapped_[i];
    }

    if (impl_->if_set_height_zero_) {
        impl_->transform_bef_mapped_[0] = 0;
        impl_->transform_bef_mapped_[2] = 0;
        impl_->transform_bef_mapped_[4] = 0;
        impl_->transform_aft_mapped_[0] = 0;
        impl_->transform_aft_mapped_[2] = 0;
        impl_->transform_aft_mapped_[4] = 0;
    }
}

void LocalMapper::UpdatePointAssociateToMapSinCos() {
    impl_->cRoll_ = cos(impl_->transform_tobe_mapped_[0]);
    impl_->sRoll_ = sin(impl_->transform_tobe_mapped_[0]);

    impl_->cPitch_ = cos(impl_->transform_tobe_mapped_[1]);
    impl_->sPitch_ = sin(impl_->transform_tobe_mapped_[1]);

    impl_->cYaw_ = cos(impl_->transform_tobe_mapped_[2]);
    impl_->sYaw_ = sin(impl_->transform_tobe_mapped_[2]);

    impl_->tX_ = impl_->transform_tobe_mapped_[3];
    impl_->tY_ = impl_->transform_tobe_mapped_[4];
    impl_->tZ_ = impl_->transform_tobe_mapped_[5];
}

void LocalMapper::PointAssociateToMap(common::PointType const *const pi, common::PointType *const po) {
    float x1 = impl_->cYaw_ * pi->x - impl_->sYaw_ * pi->y;
    float y1 = impl_->sYaw_ * pi->x + impl_->cYaw_ * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = impl_->cRoll_ * y1 - impl_->sRoll_ * z1;
    float z2 = impl_->sRoll_ * y1 + impl_->cRoll_ * z1;

    po->x = impl_->cPitch_ * x2 + impl_->sPitch_ * z2 + impl_->tX_;
    po->y = y2 + impl_->tY_;
    po->z = -impl_->sPitch_ * x2 + impl_->cPitch_ * z2 + impl_->tZ_;
    po->range = pi->range;
    po->intensity = pi->intensity;
}

void LocalMapper::UpdateTransformPointCloudSinCos(PointTypePose *tIn) {
    impl_->ctRoll_ = cos(tIn->roll);
    impl_->stRoll_ = sin(tIn->roll);

    impl_->ctPitch_ = cos(tIn->pitch);
    impl_->stPitch_ = sin(tIn->pitch);

    impl_->ctYaw_ = cos(tIn->yaw);
    impl_->stYaw_ = sin(tIn->yaw);

    impl_->tInX_ = tIn->x;
    impl_->tInY_ = tIn->y;
    impl_->tInZ_ = tIn->z;
}

PointCloudType::Ptr LocalMapper::TransformPointCloud(PointCloudType::Ptr cloudIn) {
    PointCloudType::Ptr cloudOut(new PointCloudType());

    common::PointType *pointFrom;
    common::PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
        pointFrom = &cloudIn->points[i];
        float x1 = impl_->ctYaw_ * pointFrom->x - impl_->stYaw_ * pointFrom->y;
        float y1 = impl_->stYaw_ * pointFrom->x + impl_->ctYaw_ * pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = impl_->ctRoll_ * y1 - impl_->stRoll_ * z1;
        float z2 = impl_->stRoll_ * y1 + impl_->ctRoll_ * z1;

        pointTo.x = impl_->ctPitch_ * x2 + impl_->stPitch_ * z2 + impl_->tInX_;
        pointTo.y = y2 + impl_->tInY_;
        pointTo.z = -impl_->stPitch_ * x2 + impl_->ctPitch_ * z2 + impl_->tInZ_;
        pointTo.range = pointFrom->range;
        pointTo.intensity = pointFrom->intensity;
        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

PointCloudType::Ptr LocalMapper::TransformPointCloud(PointCloudType::Ptr cloudIn, PointTypePose *transformIn) {
    PointCloudType::Ptr cloudOut(new PointCloudType());

    common::PointType *pointFrom;
    common::PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
        pointFrom = &cloudIn->points[i];
        float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
        float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw) * pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
        float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

        pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
        pointTo.y = y2 + transformIn->y;
        pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
        pointTo.range = pointFrom->range;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }
    return cloudOut;
}

void LocalMapper::ExtractSurroundingKeyFrames() {
    if (impl_->cloud_keyposes_3D_->points.empty() == true) return;

    if (impl_->recent_corner_cloud_keyframes_.size() < 50) {
        impl_->recent_corner_cloud_keyframes_.clear();
        impl_->recent_surf_cloud_keyframes_.clear();
        impl_->recent_outlier_cloud_keyframes_.clear();
        int numPoses = impl_->cloud_keyposes_3D_->points.size();
        for (int i = numPoses - 1; i >= 0; --i) {
            int this_key_index = (int)impl_->cloud_keyposes_3D_->points[i].range;
            PointTypePose this_transformation = impl_->cloud_keyposes_6D_->points[this_key_index];
            UpdateTransformPointCloudSinCos(&this_transformation);
            impl_->recent_corner_cloud_keyframes_.push_front(
                TransformPointCloud(impl_->corner_cloud_keyFrames_[this_key_index]));
            impl_->recent_surf_cloud_keyframes_.push_front(
                TransformPointCloud(impl_->surf_cloud_keyFrames_[this_key_index]));
            impl_->recent_outlier_cloud_keyframes_.push_front(
                TransformPointCloud(impl_->outlier_cloud_keyFrames_[this_key_index]));
            if (impl_->recent_corner_cloud_keyframes_.size() >= 50) break;
        }

    } else {
        if ((size_t)impl_->latest_frame_ID_ != impl_->cloud_keyposes_3D_->points.size() - 1) {
            impl_->recent_corner_cloud_keyframes_.pop_front();
            impl_->recent_surf_cloud_keyframes_.pop_front();
            impl_->recent_outlier_cloud_keyframes_.pop_front();
            impl_->latest_frame_ID_ = impl_->cloud_keyposes_3D_->points.size() - 1;
            PointTypePose this_transformation = impl_->cloud_keyposes_6D_->points[impl_->latest_frame_ID_];
            UpdateTransformPointCloudSinCos(&this_transformation);
            impl_->recent_corner_cloud_keyframes_.push_back(
                TransformPointCloud(impl_->corner_cloud_keyFrames_[impl_->latest_frame_ID_]));
            impl_->recent_surf_cloud_keyframes_.push_back(
                TransformPointCloud(impl_->surf_cloud_keyFrames_[impl_->latest_frame_ID_]));
            impl_->recent_outlier_cloud_keyframes_.push_back(
                TransformPointCloud(impl_->outlier_cloud_keyFrames_[impl_->latest_frame_ID_]));
        }
    }

    for (size_t i = 0; i < impl_->recent_corner_cloud_keyframes_.size(); ++i) {
        *impl_->laser_cloud_corner_from_map_ += *impl_->recent_corner_cloud_keyframes_[i];
        *impl_->laser_cloud_surf_from_map_ += *impl_->recent_surf_cloud_keyframes_[i];
        *impl_->laser_cloud_surf_from_map_ += *impl_->recent_outlier_cloud_keyframes_[i];
    }

    impl_->downsize_filter_corner_.setInputCloud(impl_->laser_cloud_corner_from_map_);
    impl_->downsize_filter_corner_.filter(*impl_->laser_cloud_corner_from_map_DS_);
    impl_->laser_cloud_corner_from_map_DS_num_ = impl_->laser_cloud_corner_from_map_DS_->points.size();

    impl_->downsize_filter_surf_.setInputCloud(impl_->laser_cloud_surf_from_map_);
    impl_->downsize_filter_surf_.filter(*impl_->laser_cloud_surf_from_map_DS_);
    impl_->laser_cloud_surf_from_map_DS_num_ = impl_->laser_cloud_surf_from_map_DS_->points.size();
}

void LocalMapper::DownsampleCurrentScan() {
    impl_->laser_cloud_corner_last_DS_->clear();
    impl_->downsize_filter_corner_.setInputCloud(impl_->laser_cloud_corner_last_);
    impl_->downsize_filter_corner_.filter(*impl_->laser_cloud_corner_last_DS_);
    impl_->laser_cloud_corner_last_DS_num_ = impl_->laser_cloud_corner_last_DS_->points.size();

    impl_->laser_cloud_surf_last_DS_->clear();
    impl_->downsize_filter_surf_.setInputCloud(impl_->laser_cloud_surf_last_);
    impl_->downsize_filter_surf_.filter(*impl_->laser_cloud_surf_last_DS_);

    impl_->laser_cloud_outlier_last_DS_->clear();
    impl_->downsize_filter_outlier_.setInputCloud(impl_->laser_outlier_cloud_);
    impl_->downsize_filter_outlier_.filter(*impl_->laser_cloud_outlier_last_DS_);

    impl_->laser_cloud_surf_total_last_->clear();
    impl_->laser_cloud_surf_total_last_DS_->clear();
    *impl_->laser_cloud_surf_total_last_ += *impl_->laser_cloud_surf_last_DS_;
    *impl_->laser_cloud_surf_total_last_ += *impl_->laser_cloud_outlier_last_DS_;

    impl_->downsize_filter_surf_.setInputCloud(impl_->laser_cloud_surf_total_last_);
    impl_->downsize_filter_surf_.filter(*impl_->laser_cloud_surf_total_last_DS_);
    impl_->laser_cloud_surf_total_last_DS_num_ = impl_->laser_cloud_surf_total_last_DS_->points.size();
}

void LocalMapper::CornerOptimization() {
    UpdatePointAssociateToMapSinCos();
    for (int i = 0; i < impl_->laser_cloud_corner_last_DS_num_; i++) {
        impl_->point_ori_ = impl_->laser_cloud_corner_last_DS_->points[i];
        PointAssociateToMap(&impl_->point_ori_, &impl_->point_sel_);
        impl_->kdtree_corner_from_map_->nearestKSearch(impl_->point_sel_, 5, impl_->point_search_index_,
                                                       impl_->point_search_dis_);
        if (impl_->point_search_dis_[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].x;
                cy += impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].y;
                cz += impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].z;
            }
            cx /= 5;
            cy /= 5;
            cz /= 5;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                float ax = impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].x - cx;
                float ay = impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].y - cy;
                float az = impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].z - cz;

                a11 += ax * ax;
                a12 += ax * ay;
                a13 += ax * az;
                a22 += ay * ay;
                a23 += ay * az;
                a33 += az * az;
            }
            a11 /= 5;
            a12 /= 5;
            a13 /= 5;
            a22 /= 5;
            a23 /= 5;
            a33 /= 5;

            impl_->matA1_.at<float>(0, 0) = a11;
            impl_->matA1_.at<float>(0, 1) = a12;
            impl_->matA1_.at<float>(0, 2) = a13;
            impl_->matA1_.at<float>(1, 0) = a12;
            impl_->matA1_.at<float>(1, 1) = a22;
            impl_->matA1_.at<float>(1, 2) = a23;
            impl_->matA1_.at<float>(2, 0) = a13;
            impl_->matA1_.at<float>(2, 1) = a23;
            impl_->matA1_.at<float>(2, 2) = a33;

            cv::eigen(impl_->matA1_, impl_->matD1_, impl_->matV1_);

            if (impl_->matD1_.at<float>(0, 0) > 3 * impl_->matD1_.at<float>(0, 1)) {
                float x0 = impl_->point_sel_.x;
                float y0 = impl_->point_sel_.y;
                float z0 = impl_->point_sel_.z;
                float x1 = cx + 0.1 * impl_->matV1_.at<float>(0, 0);
                float y1 = cy + 0.1 * impl_->matV1_.at<float>(0, 1);
                float z1 = cz + 0.1 * impl_->matV1_.at<float>(0, 2);
                float x2 = cx - 0.1 * impl_->matV1_.at<float>(0, 0);
                float y2 = cy - 0.1 * impl_->matV1_.at<float>(0, 1);
                float z2 = cz - 0.1 * impl_->matV1_.at<float>(0, 2);

                float a012 = sqrt(
                    ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                    ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                    ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                            (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                           a012 / l12;

                float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                             (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                           a012 / l12;

                float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                             (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                           a012 / l12;

                float ld2 = a012 / l12;

                float s = 1 - 0.9 * fabs(ld2);

                impl_->coeff_.x = s * la;
                impl_->coeff_.y = s * lb;
                impl_->coeff_.z = s * lc;
                impl_->coeff_.range = s * ld2;
                impl_->coeff_.intensity = s * ld2;
                if (s > 0.1) {
                    impl_->laser_cloud_ori_->push_back(impl_->point_ori_);
                    impl_->coeff_sel_->push_back(impl_->coeff_);
                }

                common::PointType point_dist;
                point_dist.range = ld2;
                point_dist.intensity = ld2;
                impl_->correspond_dis_->push_back(point_dist);
            }
        }
    }
}

void LocalMapper::SurfOptimization() {
    UpdatePointAssociateToMapSinCos();
    for (int i = 0; i < impl_->laser_cloud_surf_total_last_DS_num_; i++) {
        impl_->point_ori_ = impl_->laser_cloud_surf_total_last_DS_->points[i];
        PointAssociateToMap(&impl_->point_ori_, &impl_->point_sel_);
        impl_->kdtree_surf_from_map_->nearestKSearch(impl_->point_sel_, 5, impl_->point_search_index_,
                                                     impl_->point_search_dis_);

        if (impl_->point_search_dis_[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                impl_->matA0_.at<float>(j, 0) =
                    impl_->laser_cloud_surf_from_map_DS_->points[impl_->point_search_index_[j]].x;
                impl_->matA0_.at<float>(j, 1) =
                    impl_->laser_cloud_surf_from_map_DS_->points[impl_->point_search_index_[j]].y;
                impl_->matA0_.at<float>(j, 2) =
                    impl_->laser_cloud_surf_from_map_DS_->points[impl_->point_search_index_[j]].z;
            }
            cv::solve(impl_->matA0_, impl_->matB0_, impl_->matX0_, cv::DECOMP_QR);

            float pa = impl_->matX0_.at<float>(0, 0);
            float pb = impl_->matX0_.at<float>(1, 0);
            float pc = impl_->matX0_.at<float>(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool plane_valid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * impl_->laser_cloud_surf_from_map_DS_->points[impl_->point_search_index_[j]].x +
                         pb * impl_->laser_cloud_surf_from_map_DS_->points[impl_->point_search_index_[j]].y +
                         pc * impl_->laser_cloud_surf_from_map_DS_->points[impl_->point_search_index_[j]].z + pd) >
                    0.2) {
                    plane_valid = false;
                    break;
                }
            }

            if (plane_valid) {
                float pd2 = pa * impl_->point_sel_.x + pb * impl_->point_sel_.y + pc * impl_->point_sel_.z + pd;

                float s = 1 - 0.9 * fabs(pd2) /
                                  sqrt(sqrt(impl_->point_sel_.x * impl_->point_sel_.x +
                                            impl_->point_sel_.y * impl_->point_sel_.y +
                                            impl_->point_sel_.z * impl_->point_sel_.z));

                impl_->coeff_.x = s * pa;
                impl_->coeff_.y = s * pb;
                impl_->coeff_.z = s * pc;
                impl_->coeff_.range = s * pd2;
                impl_->coeff_.intensity = s * pd2;

                if (s > 0.1) {
                    impl_->laser_cloud_ori_->push_back(impl_->point_ori_);
                    impl_->coeff_sel_->push_back(impl_->coeff_);
                }

                common::PointType point_dist;
                point_dist.range = pd2;
                point_dist.intensity = pd2;
                impl_->correspond_dis_->push_back(point_dist);
            }
        }
    }
}

bool LocalMapper::LMOptimization(int iter_count) {
    float srx = sin(impl_->transform_tobe_mapped_[0]);
    float crx = cos(impl_->transform_tobe_mapped_[0]);
    float sry = sin(impl_->transform_tobe_mapped_[1]);
    float cry = cos(impl_->transform_tobe_mapped_[1]);
    float srz = sin(impl_->transform_tobe_mapped_[2]);
    float crz = cos(impl_->transform_tobe_mapped_[2]);

    int laser_cloud_sel_num = impl_->laser_cloud_ori_->points.size();
    if (laser_cloud_sel_num < 50) {
        LOG(WARNING) << "impl_->laser_cloud_sel_num < 50.";
        if (iter_count == 0) {
            impl_->matching_degenerate_.push_back(true);
            impl_->degeneracy_eigenvalue_.emplace(impl_->cloud_keyposes_3D_->points.size(), 0);
        }
        return false;
    }

    cv::Mat matA(laser_cloud_sel_num, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laser_cloud_sel_num, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laser_cloud_sel_num, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    for (int i = 0; i < laser_cloud_sel_num; i++) {
        impl_->point_ori_ = impl_->laser_cloud_ori_->points[i];
        impl_->coeff_ = impl_->coeff_sel_->points[i];

        float arx = (crx * sry * srz * impl_->point_ori_.x + crx * crz * sry * impl_->point_ori_.y -
                     srx * sry * impl_->point_ori_.z) *
                        impl_->coeff_.x +
                    (-srx * srz * impl_->point_ori_.x - crz * srx * impl_->point_ori_.y - crx * impl_->point_ori_.z) *
                        impl_->coeff_.y +
                    (crx * cry * srz * impl_->point_ori_.x + crx * cry * crz * impl_->point_ori_.y -
                     cry * srx * impl_->point_ori_.z) *
                        impl_->coeff_.z;

        float ary = ((cry * srx * srz - crz * sry) * impl_->point_ori_.x +
                     (sry * srz + cry * crz * srx) * impl_->point_ori_.y + crx * cry * impl_->point_ori_.z) *
                        impl_->coeff_.x +
                    ((-cry * crz - srx * sry * srz) * impl_->point_ori_.x +
                     (cry * srz - crz * srx * sry) * impl_->point_ori_.y - crx * sry * impl_->point_ori_.z) *
                        impl_->coeff_.z;

        float arz = ((crz * srx * sry - cry * srz) * impl_->point_ori_.x +
                     (-cry * crz - srx * sry * srz) * impl_->point_ori_.y) *
                        impl_->coeff_.x +
                    (crx * crz * impl_->point_ori_.x - crx * srz * impl_->point_ori_.y) * impl_->coeff_.y +
                    ((sry * srz + cry * crz * srx) * impl_->point_ori_.x +
                     (crz * sry - cry * srx * srz) * impl_->point_ori_.y) *
                        impl_->coeff_.z;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arz;
        matA.at<float>(i, 3) = impl_->coeff_.x;
        matA.at<float>(i, 4) = impl_->coeff_.y;
        matA.at<float>(i, 5) = impl_->coeff_.z;
        matB.at<float>(i, 0) = -impl_->coeff_.range;
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iter_count == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        impl_->is_degenerate_ = false;
        float eign_th[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < eign_th[i]) {
                for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                impl_->is_degenerate_ = true;
            } else {
                break;
            }
        }
        impl_->matP_ = matV.inv() * matV2;

        impl_->latest_matching_degenerate_ = impl_->is_degenerate_;
        impl_->matching_degenerate_.push_back(impl_->is_degenerate_);
        impl_->degeneracy_eigenvalue_.emplace(impl_->cloud_keyposes_3D_->points.size(), matE.at<float>(0, 5));

        if (impl_->is_degenerate_) {
            LOG(INFO) << "degenerate detected at kf " << impl_->cloud_keyposes_3D_->points.size();
        }
    }

    if (impl_->is_degenerate_) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = impl_->matP_ * matX2;
    }

    impl_->transform_tobe_mapped_[0] += matX.at<float>(0, 0);
    impl_->transform_tobe_mapped_[1] += matX.at<float>(1, 0);
    impl_->transform_tobe_mapped_[2] += matX.at<float>(2, 0);
    impl_->transform_tobe_mapped_[3] += matX.at<float>(3, 0);
    impl_->transform_tobe_mapped_[4] += matX.at<float>(4, 0);
    impl_->transform_tobe_mapped_[5] += matX.at<float>(5, 0);

    float delta_R = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) + pow(rad2deg(matX.at<float>(1, 0)), 2) +
                         pow(rad2deg(matX.at<float>(2, 0)), 2));
    float delta_T = sqrt(pow(matX.at<float>(3, 0) * 100, 2) + pow(matX.at<float>(4, 0) * 100, 2) +
                         pow(matX.at<float>(5, 0) * 100, 2));

    if (delta_R < 0.05 && delta_T < 0.05) {
        return true;
    }
    return false;
}

void LocalMapper::Scan2MapOptimization() {
    if (impl_->laser_cloud_corner_from_map_DS_num_ > 10 && impl_->laser_cloud_surf_from_map_DS_num_ > 100) {
        impl_->kdtree_corner_from_map_->setInputCloud(impl_->laser_cloud_corner_from_map_DS_);
        impl_->kdtree_surf_from_map_->setInputCloud(impl_->laser_cloud_surf_from_map_DS_);

        for (int iter_count = 0; iter_count < 10; iter_count++) {
            impl_->laser_cloud_ori_->clear();
            impl_->coeff_sel_->clear();
            impl_->correspond_dis_->clear();

            CornerOptimization();
            SurfOptimization();

            if (LMOptimization(iter_count) == true) break;
        }

        impl_->fitness_score_ = 0;
        size_t correspond_num = impl_->correspond_dis_->points.size();
        for (size_t i = 0; i < correspond_num; i++) {
            impl_->fitness_score_ += impl_->correspond_dis_->points[i].range;
        }
        if (correspond_num != 0) impl_->fitness_score_ /= correspond_num;

        TransformUpdate();
    } else {
        impl_->matching_degenerate_.push_back(false);
    }
}

void LocalMapper::SaveKeyFrames() {
    impl_->current_robot_pos_point_.x = impl_->transform_aft_mapped_[3];
    impl_->current_robot_pos_point_.y = impl_->transform_aft_mapped_[4];
    impl_->current_robot_pos_point_.z = impl_->transform_aft_mapped_[5];

    if (impl_->cloud_keyposes_3D_->points.empty()) {
        for (int i = 0; i < 6; ++i) impl_->transform_last_[i] = impl_->transform_tobe_mapped_[i];
    }

    if (impl_->loop_closure_enable_) {
        double x = impl_->transform_aft_mapped_[5];
        double y = impl_->transform_aft_mapped_[3];
        double z = impl_->transform_aft_mapped_[4];
        double roll = impl_->transform_aft_mapped_[2];
        double pitch = impl_->transform_aft_mapped_[0];
        double yaw = impl_->transform_aft_mapped_[1];
        common::PoseRPY pose(x, y, z, roll, pitch, yaw);
        SE3 current_pose = XYZRPYToSE3(pose);
        impl_->latest_keyframe_pose_ = current_pose;

        impl_->vertex_ID_ = impl_->cloud_keyposes_3D_->points.size();
        if (impl_->vertex_ID_ == 0) {
            AddVertex(current_pose, impl_->vertex_ID_, true);
        } else {
            AddVertex(current_pose, impl_->vertex_ID_);
        }

        if (impl_->vertex_ID_ > 0) {
            double x = impl_->transform_last_[5];
            double y = impl_->transform_last_[3];
            double z = impl_->transform_last_[4];
            double roll = impl_->transform_last_[2];
            double pitch = impl_->transform_last_[0];
            double yaw = impl_->transform_last_[1];

            SE3 previous_pose = XYZRPYToSE3({x, y, z, roll, pitch, yaw});
            SE3 delta_pose = previous_pose.inverse() * current_pose;

            if (impl_->if_set_height_zero_) {
                common::PoseRPY p = SE3ToRollPitchYaw(delta_pose);
                p.z = 0;
                p.pitch = 0;  // pitch
                p.roll = 0;   // roll
                delta_pose = XYZRPYToSE3(p);
            }

            Eigen::Matrix<float, 6, 6> covariance = Eigen::Matrix<float, 6, 6>::Identity();
            for (int i = 0; i < 6; i++) {
                covariance(i, i) = 1.0;
            }

            AddEdge(impl_->vertex_ID_ - 1, impl_->vertex_ID_, delta_pose, covariance);
        }
    }

    common::PointType this_pose_3D;
    PointTypePose this_pose_6D;

    this_pose_3D.x = impl_->transform_aft_mapped_[3];
    this_pose_3D.y = impl_->transform_aft_mapped_[4];
    this_pose_3D.z = impl_->transform_aft_mapped_[5];
    this_pose_3D.range = impl_->cloud_keyposes_3D_->points.size();
    this_pose_6D.intensity = this_pose_3D.range;
    impl_->cloud_keyposes_3D_->push_back(this_pose_3D);

    this_pose_6D.x = this_pose_3D.x;
    this_pose_6D.y = this_pose_3D.y;
    this_pose_6D.z = this_pose_3D.z;
    this_pose_6D.roll = impl_->transform_aft_mapped_[0];
    this_pose_6D.pitch = impl_->transform_aft_mapped_[1];
    this_pose_6D.yaw = impl_->transform_aft_mapped_[2];
    this_pose_6D.time = impl_->loop_current_processing_time_;
    impl_->cloud_keyposes_6D_->push_back(this_pose_6D);

    if (impl_->cloud_keyposes_3D_->points.size() > 1) {
        for (int i = 0; i < 6; ++i) {
            impl_->transform_last_[i] = impl_->transform_aft_mapped_[i];
            impl_->transform_tobe_mapped_[i] = impl_->transform_aft_mapped_[i];
        }
    }

    PointCloudType::Ptr this_corner_keyframe(new PointCloudType());
    PointCloudType::Ptr this_surf_keyframe(new PointCloudType());
    PointCloudType::Ptr this_outlier_keyFrame(new PointCloudType());

    pcl::copyPointCloud(*impl_->laser_cloud_corner_last_DS_, *this_corner_keyframe);
    pcl::copyPointCloud(*impl_->laser_cloud_surf_last_DS_, *this_surf_keyframe);
    pcl::copyPointCloud(*impl_->laser_cloud_outlier_last_DS_, *this_outlier_keyFrame);

    impl_->corner_cloud_keyFrames_.push_back(this_corner_keyframe);
    impl_->surf_cloud_keyFrames_.push_back(this_surf_keyframe);
    impl_->outlier_cloud_keyFrames_.push_back(this_outlier_keyFrame);
}

void LocalMapper::ClearCloud() {
    impl_->laser_cloud_corner_from_map_->clear();
    impl_->laser_cloud_surf_from_map_->clear();
    impl_->laser_cloud_corner_from_map_DS_->clear();
    impl_->laser_cloud_surf_from_map_DS_->clear();
}

Aff3f LocalMapper::pclPointToAffine3fCameraToLidar(PointTypePose thisPoint) {
    return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll,
                                  thisPoint.pitch);
}

SE3 LocalMapper::pclPointToEigen(PointTypePose thisPoint) {
    double x = thisPoint.z;
    double y = thisPoint.x;
    double z = thisPoint.y;
    double roll = thisPoint.yaw;
    double pitch = thisPoint.roll;
    double yaw = thisPoint.pitch;

    return XYZRPYToSE3({x, y, z, roll, pitch, yaw});
}

bool LocalMapper::DetectLoop() {
    impl_->latest_surf_keyframe_cloud_->clear();
    impl_->near_history_surf_keyframe_cloud_->clear();
    impl_->near_history_surf_keyframe_cloud_DS_->clear();

    std::vector<int> point_search_index_loop;
    std::vector<float> point_search_sq_dis_loop;
    impl_->kdtree_history_keyposes_->setInputCloud(impl_->cloud_keyposes_3D_);
    impl_->kdtree_history_keyposes_->radiusSearch(impl_->current_robot_pos_point_,
                                                  impl_->params_.loop_near_search_dist_scope, point_search_index_loop,
                                                  point_search_sq_dis_loop, 0);

    impl_->closest_history_frame_ID_ = -1;
    for (size_t i = 0; i < point_search_index_loop.size(); ++i) {
        int id = point_search_index_loop[i];
        if (abs(impl_->cloud_keyposes_6D_->points[id].time - impl_->loop_current_processing_time_) >
            impl_->params_.loop_near_search_time_interval) {
            impl_->closest_history_frame_ID_ = id;
            break;
        }
    }
    if (impl_->closest_history_frame_ID_ == -1) {
        return false;
    }

    impl_->latest_frame_ID_loop_cloure_ = impl_->cloud_keyposes_3D_->points.size() - 1;
    *impl_->latest_surf_keyframe_cloud_ +=
        *TransformPointCloud(impl_->corner_cloud_keyFrames_[impl_->latest_frame_ID_loop_cloure_],
                             &impl_->cloud_keyposes_6D_->points[impl_->latest_frame_ID_loop_cloure_]);
    *impl_->latest_surf_keyframe_cloud_ +=
        *TransformPointCloud(impl_->surf_cloud_keyFrames_[impl_->latest_frame_ID_loop_cloure_],
                             &impl_->cloud_keyposes_6D_->points[impl_->latest_frame_ID_loop_cloure_]);

    PointCloudType::Ptr hahaCloud(new PointCloudType());
    int cloud_size = impl_->latest_surf_keyframe_cloud_->points.size();
    for (int i = 0; i < cloud_size; ++i) {
        if ((int)impl_->latest_surf_keyframe_cloud_->points[i].range >= 0) {
            hahaCloud->push_back(impl_->latest_surf_keyframe_cloud_->points[i]);
        }
    }
    impl_->latest_surf_keyframe_cloud_->clear();
    *impl_->latest_surf_keyframe_cloud_ = *hahaCloud;

    for (int j = -150; j <= 150; ++j) {
        if (impl_->closest_history_frame_ID_ + j < 0 ||
            impl_->closest_history_frame_ID_ + j > impl_->latest_frame_ID_loop_cloure_)
            continue;
        if (poseDistance(impl_->cloud_keyposes_6D_->points[impl_->closest_history_frame_ID_ + j],
                         impl_->cloud_keyposes_6D_->points[impl_->closest_history_frame_ID_]) < 10.0) {
            *impl_->near_history_surf_keyframe_cloud_ +=
                *TransformPointCloud(impl_->corner_cloud_keyFrames_[impl_->closest_history_frame_ID_ + j],
                                     &impl_->cloud_keyposes_6D_->points[impl_->closest_history_frame_ID_ + j]);
            *impl_->near_history_surf_keyframe_cloud_ +=
                *TransformPointCloud(impl_->surf_cloud_keyFrames_[impl_->closest_history_frame_ID_ + j],
                                     &impl_->cloud_keyposes_6D_->points[impl_->closest_history_frame_ID_ + j]);
        }
    }

    impl_->downsize_filter_history_keyframes_.setInputCloud(impl_->near_history_surf_keyframe_cloud_);
    impl_->downsize_filter_history_keyframes_.filter(*impl_->near_history_surf_keyframe_cloud_DS_);

    return true;
}

void LocalMapper::PerformLoopClosure() {
    if (impl_->cloud_keyposes_3D_->points.empty()) {
        impl_->loop_last_processing_time_ = impl_->loop_current_processing_time_;
        return;
    }

    if (impl_->loop_current_processing_time_ - impl_->loop_last_processing_time_ >=
        impl_->params_.loop_process_time_interval) {
        if (impl_->potential_loop_flag_ == false) {
            if (DetectLoop() == true) {
                impl_->potential_loop_flag_ = true;
            }
            if (impl_->potential_loop_flag_ == false) return;
        }
        impl_->potential_loop_flag_ = false;

        pcl::IterativeClosestPoint<common::PointType, common::PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(30);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(impl_->latest_surf_keyframe_cloud_);
        icp.setInputTarget(impl_->near_history_surf_keyframe_cloud_DS_);
        PointCloudType::Ptr unused_result(new PointCloudType());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > impl_->params_.loop_fitness_score_th) {
            return;
        }

        float x, y, z, roll, pitch, yaw;
        Aff3f correction_camera_frame;
        correction_camera_frame = icp.getFinalTransformation();
        pcl::getTranslationAndEulerAngles(correction_camera_frame, x, y, z, roll, pitch, yaw);
        Aff3f correction_lidar_frame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
        Aff3f t_wrong =
            pclPointToAffine3fCameraToLidar(impl_->cloud_keyposes_6D_->points[impl_->latest_frame_ID_loop_cloure_]);
        Aff3d t = (correction_lidar_frame * t_wrong).cast<double>();
        Quat q(t.rotation());
        q.normalize();
        SE3 pose_from(q, t.translation());
        SE3 pose_to = pclPointToEigen(impl_->cloud_keyposes_6D_->points[impl_->closest_history_frame_ID_]);
        SE3 transform_btw = pose_to.inverse() * pose_from;  // latest, matched

        auto pose = SE3ToRollPitchYaw(transform_btw);

        pose.z = 0;
        pose.roll = 0;
        pose.pitch = 0;
        transform_btw = XYZRPYToSE3(pose);

        float noiseScore = icp.getFitnessScore();
        Eigen::Matrix<float, 6, 6> covariance = Eigen::Matrix<float, 6, 6>::Identity();
        for (int i = 0; i < 6; i++) {
            covariance(i, i) = noiseScore * 1e-4;
        }

        AddEdge(impl_->closest_history_frame_ID_, impl_->latest_frame_ID_loop_cloure_, transform_btw, covariance);

        impl_->optimizer_->initializeOptimization();
        impl_->optimizer_->optimize(10);

        impl_->a_loop_is_closed_ = true;

        impl_->loop_last_processing_time_ = impl_->loop_current_processing_time_;
    }
}

void LocalMapper::CorrectPose() {
    if (impl_->a_loop_is_closed_ == true) {
        impl_->recent_corner_cloud_keyframes_.clear();
        impl_->recent_surf_cloud_keyframes_.clear();
        impl_->recent_outlier_cloud_keyframes_.clear();

        int num_poses = impl_->cloud_keyposes_3D_->points.size();
        for (int i = 0; i < num_poses; ++i) {
            g2o::VertexSE3 *each_vertex = dynamic_cast<g2o::VertexSE3 *>(impl_->optimizer_->vertex(i));
            Eigen::Isometry3d vertex_estimate = each_vertex->estimate();

            SE3 estimate_pose(vertex_estimate.matrix());
            impl_->cloud_keyposes_3D_->points[i].x = estimate_pose.translation().y();
            impl_->cloud_keyposes_3D_->points[i].y = estimate_pose.translation().z();
            impl_->cloud_keyposes_3D_->points[i].z = estimate_pose.translation().x();

            impl_->cloud_keyposes_6D_->points[i].x = impl_->cloud_keyposes_3D_->points[i].x;
            impl_->cloud_keyposes_6D_->points[i].y = impl_->cloud_keyposes_3D_->points[i].y;
            impl_->cloud_keyposes_6D_->points[i].z = impl_->cloud_keyposes_3D_->points[i].z;

            auto pose = SE3ToRollPitchYaw(estimate_pose);
            impl_->cloud_keyposes_6D_->points[i].roll = pose.pitch;
            impl_->cloud_keyposes_6D_->points[i].pitch = pose.yaw;
            impl_->cloud_keyposes_6D_->points[i].yaw = pose.roll;
        }

        if (num_poses > 0) {
            g2o::VertexSE3 *latest_vertex = dynamic_cast<g2o::VertexSE3 *>(impl_->optimizer_->vertex(num_poses - 1));
            SE3 latest_estimate_pose(latest_vertex->estimate().matrix());
            auto pose = SE3ToRollPitchYaw(latest_estimate_pose);

            impl_->transform_aft_mapped_[0] = pose.pitch;
            impl_->transform_aft_mapped_[1] = pose.yaw;
            impl_->transform_aft_mapped_[2] = pose.roll;
            impl_->transform_aft_mapped_[3] = pose.y;
            impl_->transform_aft_mapped_[4] = pose.z;
            impl_->transform_aft_mapped_[5] = pose.x;

            for (int i = 0; i < 6; ++i) {
                impl_->transform_last_[i] = impl_->transform_aft_mapped_[i];
                impl_->transform_tobe_mapped_[i] = impl_->transform_aft_mapped_[i];
            }
        }

        impl_->a_loop_is_closed_ = false;
    }
}

std::map<int, SE3> LocalMapper::GetMatchingPoses() {
    int num_poses = impl_->cloud_keyposes_3D_->points.size();
    for (int i = 0; i < num_poses; ++i) {
        double x = impl_->cloud_keyposes_6D_->points[i].z;
        double y = impl_->cloud_keyposes_6D_->points[i].x;
        double z = impl_->cloud_keyposes_6D_->points[i].y;
        double roll = impl_->cloud_keyposes_6D_->points[i].yaw;
        double pitch = impl_->cloud_keyposes_6D_->points[i].roll;
        double yaw = impl_->cloud_keyposes_6D_->points[i].pitch;
        SE3 each_pose = XYZRPYToSE3({x, y, z, roll, pitch, yaw});
        impl_->matching_poses_.emplace(impl_->start_keyframe_id_ + i, each_pose);
    }
    return impl_->matching_poses_;
}

std::map<int, V6d> LocalMapper::GetMatchingNoise() {
    V6d noise, large_noise;
    noise << 0.05, 0.05, 0.05, 0.008, 0.008, 0.008;
    large_noise << 0.05 * 100, 0.05 * 100, 0.05 * 100, 0.008 * 100, 0.008 * 100, 0.008 * 100;

    int num_poses = impl_->cloud_keyposes_3D_->points.size();

    if (((size_t)(num_poses - 1)) != impl_->matching_degenerate_.size()) {
        LOG(ERROR) << "matching_degenerate_ size is wrong.: " << num_poses - 1
                   << " != " << impl_->matching_degenerate_.size();
    }

    impl_->matching_noises_.emplace(impl_->start_keyframe_id_, noise);
    for (int i = 1; i < num_poses; ++i) {
        if (impl_->matching_degenerate_.at(i - 1)) {
            impl_->matching_noises_.emplace(impl_->start_keyframe_id_ + i, large_noise);
        } else {
            impl_->matching_noises_.emplace(impl_->start_keyframe_id_ + i, noise);
        }
    }
    return impl_->matching_noises_;
}

void LocalMapper::AddVertex(SE3 &input_pose, int &vertex_id, bool fixed_pose) {
    g2o::VertexSE3 *temp_vertex = new g2o::VertexSE3();
    temp_vertex->setId(vertex_id);
    temp_vertex->setEstimate(Eigen::Isometry3d(input_pose.matrix()));
    if (fixed_pose) {
        temp_vertex->setFixed(true);
    }
    impl_->optimizer_->addVertex(temp_vertex);
}

void LocalMapper::AddEdge(int first_id, int second_id, SE3 delta_pose, M6f covariance) {
    if (first_id < 0 || second_id < 0) {
        LOG(WARNING) << "Edge can not be build because of the leak of Vertex!";
        return;
    }

    g2o::EdgeSE3 *temp_edge = new g2o::EdgeSE3();
    temp_edge->setId(++impl_->edge_ID_);
    temp_edge->setVertex(0, impl_->optimizer_->vertex(first_id));
    temp_edge->setVertex(1, impl_->optimizer_->vertex(second_id));
    auto weight = covariance.cast<double>();
    temp_edge->setInformation(weight);
    temp_edge->setMeasurement(Eigen::Isometry3d(delta_pose.matrix()));
    impl_->optimizer_->addEdge(temp_edge);
}

void LocalMapper::SetTrajectoryID(int id) { impl_->trajectory_id_ = id; }

void LocalMapper::SetCornerPointLast(const common::PointCloudType::Ptr &laser_cloud_corner_last) {
    impl_->laser_cloud_corner_last_ = laser_cloud_corner_last;
    impl_->loop_current_processing_time_ =
        pcl_conversions::fromPCL(impl_->laser_cloud_corner_last_->header.stamp).toSec();
}

void LocalMapper::SetSurfPointLast(const common::PointCloudType::Ptr &laser_cloud_surf_last) {
    impl_->laser_cloud_surf_last_ = laser_cloud_surf_last;
}

void LocalMapper::SetOutlierCloud(const common::PointCloudType::Ptr &outlier_cloud) {
    impl_->laser_outlier_cloud_ = outlier_cloud;
}

void LocalMapper::SetCurrentTransform(const float *transform_sum) {
    for (int i = 0; i < 6; i++) {
        impl_->transform_sum_[i] = transform_sum[i];
    }
    impl_->transform_sum_[4] = 0;
}

void LocalMapper::SetKeyFrameID(int id) { impl_->keyframe_id_ = id; }

void LocalMapper::SetPreviousFrameID(int id) { impl_->previous_id_ = id; }

void LocalMapper::SetStartID(int id) { impl_->start_keyframe_id_ = id; }

void LocalMapper::SetSameHeight(bool flag) { impl_->if_set_height_zero_ = flag; }

void LocalMapper::SetDrPose(const SE3 &dr_pose) { impl_->dr_pose_ = dr_pose; }

SE3 LocalMapper::GetLatestKeyFramePose() { return impl_->latest_keyframe_pose_; }

double LocalMapper::GetFitnessScore() { return impl_->fitness_score_; }

std::map<int, float> LocalMapper::GetDegeneracyEigenValue() { return impl_->degeneracy_eigenvalue_; }

bool LocalMapper::GetLatestDegeneracy() { return impl_->latest_matching_degenerate_; }

void LocalMapper::Run() {
    TransformAssociateToMap();

    ExtractSurroundingKeyFrames();

    DownsampleCurrentScan();

    Scan2MapOptimization();

    SaveKeyFrames();

    if (impl_->loop_closure_enable_) {
        PerformLoopClosure();
        CorrectPose();
    }

    ClearCloud();
}

}  // namespace mapping::core

#include "core/lidar_matching/feature_matching/feature_tracker.h"
#include "common/mapping_math.h"

namespace mapping::core {

using namespace mapping::common;

FeatureTracker::FeatureTracker(FeatureMatchingParams &params) : params_(params) { Initialization(); }

FeatureTracker::~FeatureTracker() {}

void FeatureTracker::Initialization() {
    corner_points_sharp_.reset(new PointCloudType());
    corner_points_less_sharp_.reset(new PointCloudType());
    surf_points_flat_.reset(new PointCloudType());
    surf_points_less_flat_.reset(new PointCloudType());

    laser_cloud_corner_last_.reset(new PointCloudType());
    laser_cloud_surf_last_.reset(new PointCloudType());
    outlier_cloud_.reset(new PointCloudType());

    laser_cloud_ori_.reset(new PointCloudType());
    coeff_sel_.reset(new PointCloudType());

    kdtree_corner_last_.reset(new pcl::KdTreeFLANN<common::PointType>());
    kdtree_surf_last_.reset(new pcl::KdTreeFLANN<common::PointType>());

    point_search_corner_index1_.resize(16 * 1800);
    point_search_corner_index2_.resize(16 * 1800);
    point_search_surf_index1_.resize(16 * 1800);
    point_search_surf_index2_.resize(16 * 1800);
    point_search_surf_index3_.resize(16 * 1800);

    system_inited_LM_ = false;
    is_degenerate_ = false;

    for (int i = 0; i < 6; ++i) {
        transform_current_[i] = 0;
        transform_sum_[i] = 0;
    }
}

void FeatureTracker::SetPredictPose(const SE3 &predict_pose) {
    common::PoseRPY pose = SE3ToRollPitchYaw(predict_pose.inverse());

    transform_current_[0] = pose.pitch;  // pitch
    transform_current_[1] = pose.yaw;    // yaw
    transform_current_[2] = pose.roll;   // roll
    transform_current_[3] = pose.y;
    transform_current_[4] = pose.z;
    transform_current_[5] = pose.x;
}

void FeatureTracker::TransformToStart(common::PointType const *const pi, common::PointType *const po) {
    float s = 10 * (pi->range - int(pi->range));
    s = 0;
    float rx = s * transform_current_[0];
    float ry = s * transform_current_[1];
    float rz = s * transform_current_[2];
    float tx = s * transform_current_[3];
    float ty = s * transform_current_[4];
    float tz = s * transform_current_[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->range = pi->range;
    po->intensity = pi->intensity;
}

void FeatureTracker::TransformToEnd(common::PointType const *const pi, common::PointType *const po) {
    float s = 10 * (pi->range - int(pi->range));
    s = 1;
    float rx = s * transform_current_[0];
    float ry = s * transform_current_[1];
    float rz = s * transform_current_[2];
    float tx = s * transform_current_[3];
    float ty = s * transform_current_[4];
    float tz = s * transform_current_[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    float x3 = cos(ry) * x2 - sin(ry) * z2;
    float y3 = y2;
    float z3 = sin(ry) * x2 + cos(ry) * z2;

    rx = transform_current_[0];
    ry = transform_current_[1];
    rz = transform_current_[2];
    tx = transform_current_[3];
    ty = transform_current_[4];
    tz = transform_current_[5];

    float x4 = cos(ry) * x3 + sin(ry) * z3;
    float y4 = y3;
    float z4 = -sin(ry) * x3 + cos(ry) * z3;

    float x5 = x4;
    float y5 = cos(rx) * y4 - sin(rx) * z4;
    float z5 = sin(rx) * y4 + cos(rx) * z4;

    float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
    float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
    float z6 = z5 + tz;

    float x7 = 1 * (x6 - 0) - 0 * (y6 - 0);
    float y7 = 0 * (x6 - 0) + 1 * (y6 - 0);
    float z7 = z6 - 0;

    float x8 = x7;
    float y8 = 1 * y7 - 0 * z7;
    float z8 = 0 * y7 + 1 * z7;

    float x9 = 1 * x8 + 0 * z8;
    float y9 = y8;
    float z9 = -0 * x8 + 1 * z8;

    float x10 = cos(0) * x9 - sin(0) * z9;
    float y10 = y9;
    float z10 = sin(0) * x9 + cos(0) * z9;

    float x11 = x10;
    float y11 = cos(0) * y10 + sin(0) * z10;
    float z11 = -sin(0) * y10 + cos(0) * z10;

    po->x = cos(0) * x11 + sin(0) * y11;
    po->y = -sin(0) * x11 + cos(0) * y11;
    po->z = z11;
    po->range = int(pi->range);
    po->intensity = int(pi->intensity);
}

void FeatureTracker::AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, float &ox,
                                        float &oy, float &oz) {
    float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) - cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
    ox = -asin(srx);

    float srycrx = sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) +
                   cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) +
                   cos(lx) * cos(ly) * cos(cx) * sin(cy);
    float crycrx = cos(lx) * cos(ly) * cos(cx) * cos(cy) -
                   cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) -
                   sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
    oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

    float srzcrx = sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) +
                   cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) +
                   cos(lx) * cos(cx) * cos(cz) * sin(lz);
    float crzcrx = cos(lx) * cos(lz) * cos(cx) * cos(cz) -
                   cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) -
                   sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
    oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void FeatureTracker::FindCorrespondingCornerFeatures(int iter_count) {
    int corner_points_sharp_num = corner_points_sharp_->points.size();

    for (int i = 0; i < corner_points_sharp_num; i++) {
        TransformToStart(&corner_points_sharp_->points[i], &point_sel_);

        if (iter_count % 5 == 0) {
            kdtree_corner_last_->nearestKSearch(point_sel_, 1, point_search_index_, point_search_dis_);
            int closest_point_index = -1, min_point_index2 = -1;

            if (point_search_dis_[0] < params_.nearest_feature_search_dist) {
                closest_point_index = point_search_index_[0];

                int closest_point_scan = int(laser_cloud_corner_last_->points[closest_point_index].range);
                float point_sq_dis;
                float min_point_sq_dis2 = params_.nearest_feature_search_dist;
                for (int j = closest_point_index + 1; j < corner_points_sharp_num; j++) {
                    if (int(laser_cloud_corner_last_->points[j].range) > closest_point_scan + 2.5) {
                        break;
                    }

                    point_sq_dis = (laser_cloud_corner_last_->points[j].x - point_sel_.x) *
                                       (laser_cloud_corner_last_->points[j].x - point_sel_.x) +
                                   (laser_cloud_corner_last_->points[j].y - point_sel_.y) *
                                       (laser_cloud_corner_last_->points[j].y - point_sel_.y) +
                                   (laser_cloud_corner_last_->points[j].z - point_sel_.z) *
                                       (laser_cloud_corner_last_->points[j].z - point_sel_.z);

                    if (int(laser_cloud_corner_last_->points[j].range) > closest_point_scan) {
                        if (point_sq_dis < min_point_sq_dis2) {
                            min_point_sq_dis2 = point_sq_dis;
                            min_point_index2 = j;
                        }
                    }
                }
                for (int j = closest_point_index - 1; j >= 0; j--) {
                    if (int(laser_cloud_corner_last_->points[j].range) < closest_point_scan - 2.5) {
                        break;
                    }

                    point_sq_dis = (laser_cloud_corner_last_->points[j].x - point_sel_.x) *
                                       (laser_cloud_corner_last_->points[j].x - point_sel_.x) +
                                   (laser_cloud_corner_last_->points[j].y - point_sel_.y) *
                                       (laser_cloud_corner_last_->points[j].y - point_sel_.y) +
                                   (laser_cloud_corner_last_->points[j].z - point_sel_.z) *
                                       (laser_cloud_corner_last_->points[j].z - point_sel_.z);

                    if (int(laser_cloud_corner_last_->points[j].range) < closest_point_scan) {
                        if (point_sq_dis < min_point_sq_dis2) {
                            min_point_sq_dis2 = point_sq_dis;
                            min_point_index2 = j;
                        }
                    }
                }
            }

            point_search_corner_index1_[i] = closest_point_index;
            point_search_corner_index2_[i] = min_point_index2;
        }

        if (point_search_corner_index2_[i] >= 0) {
            tripod1_ = laser_cloud_corner_last_->points[point_search_corner_index1_[i]];
            tripod2_ = laser_cloud_corner_last_->points[point_search_corner_index2_[i]];

            float x0 = point_sel_.x;
            float y0 = point_sel_.y;
            float z0 = point_sel_.z;
            float x1 = tripod1_.x;
            float y1 = tripod1_.y;
            float z1 = tripod1_.z;
            float x2 = tripod2_.x;
            float y2 = tripod2_.y;
            float z2 = tripod2_.z;

            float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
            float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
            float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

            float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

            float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

            float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

            float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

            float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

            float ld2 = a012 / l12;

            float s = 1;
            if (iter_count >= 5) {
                s = 1 - 1.8 * fabs(ld2);
            }

            if (s > 0.1 && ld2 != 0) {
                coeff_.x = s * la;
                coeff_.y = s * lb;
                coeff_.z = s * lc;
                coeff_.range = s * ld2;
                coeff_.intensity = s * ld2;
                laser_cloud_ori_->push_back(corner_points_sharp_->points[i]);
                coeff_sel_->push_back(coeff_);
            }
        }
    }
}

void FeatureTracker::FindCorrespondingSurfFeatures(int iter_count) {
    int surf_points_flat_num = surf_points_flat_->points.size();

    for (int i = 0; i < surf_points_flat_num; i++) {
        TransformToStart(&surf_points_flat_->points[i], &point_sel_);

        if (iter_count % 5 == 0) {
            kdtree_surf_last_->nearestKSearch(point_sel_, 1, point_search_index_, point_search_dis_);
            int closest_point_index = -1, min_point_index2 = -1, min_point_index3 = -1;

            if (point_search_dis_[0] < params_.nearest_feature_search_dist) {
                closest_point_index = point_search_index_[0];
                int closest_point_scan = int(laser_cloud_surf_last_->points[closest_point_index].range);
                float point_sq_dis;
                float min_point_sq_dis2 = params_.nearest_feature_search_dist;
                float min_point_sq_dis3 = params_.nearest_feature_search_dist;
                for (int j = closest_point_index + 1; j < surf_points_flat_num; j++) {
                    if (int(laser_cloud_surf_last_->points[j].range) > closest_point_scan + 2.5) {
                        break;
                    }

                    point_sq_dis = (laser_cloud_surf_last_->points[j].x - point_sel_.x) *
                                       (laser_cloud_surf_last_->points[j].x - point_sel_.x) +
                                   (laser_cloud_surf_last_->points[j].y - point_sel_.y) *
                                       (laser_cloud_surf_last_->points[j].y - point_sel_.y) +
                                   (laser_cloud_surf_last_->points[j].z - point_sel_.z) *
                                       (laser_cloud_surf_last_->points[j].z - point_sel_.z);

                    if (int(laser_cloud_surf_last_->points[j].range) <= closest_point_scan) {
                        if (point_sq_dis < min_point_sq_dis2) {
                            min_point_sq_dis2 = point_sq_dis;
                            min_point_index2 = j;
                        }
                    } else {
                        if (point_sq_dis < min_point_sq_dis3) {
                            min_point_sq_dis3 = point_sq_dis;
                            min_point_index3 = j;
                        }
                    }
                }

                for (int j = closest_point_index - 1; j >= 0; j--) {
                    if (int(laser_cloud_surf_last_->points[j].range) < closest_point_scan - 2.5) {
                        break;
                    }

                    point_sq_dis = (laser_cloud_surf_last_->points[j].x - point_sel_.x) *
                                       (laser_cloud_surf_last_->points[j].x - point_sel_.x) +
                                   (laser_cloud_surf_last_->points[j].y - point_sel_.y) *
                                       (laser_cloud_surf_last_->points[j].y - point_sel_.y) +
                                   (laser_cloud_surf_last_->points[j].z - point_sel_.z) *
                                       (laser_cloud_surf_last_->points[j].z - point_sel_.z);
                    if (int(laser_cloud_surf_last_->points[j].range) >= closest_point_scan) {
                        if (point_sq_dis < min_point_sq_dis2) {
                            min_point_sq_dis2 = point_sq_dis;
                            min_point_index2 = j;
                        }
                    } else {
                        if (point_sq_dis < min_point_sq_dis3) {
                            min_point_sq_dis3 = point_sq_dis;
                            min_point_index3 = j;
                        }
                    }
                }
            }

            point_search_surf_index1_[i] = closest_point_index;
            point_search_surf_index2_[i] = min_point_index2;
            point_search_surf_index3_[i] = min_point_index3;
        }

        if (point_search_surf_index2_[i] >= 0 && point_search_surf_index3_[i] >= 0) {
            tripod1_ = laser_cloud_surf_last_->points[point_search_surf_index1_[i]];
            tripod2_ = laser_cloud_surf_last_->points[point_search_surf_index2_[i]];
            tripod3_ = laser_cloud_surf_last_->points[point_search_surf_index3_[i]];

            float pa = (tripod2_.y - tripod1_.y) * (tripod3_.z - tripod1_.z) -
                       (tripod3_.y - tripod1_.y) * (tripod2_.z - tripod1_.z);
            float pb = (tripod2_.z - tripod1_.z) * (tripod3_.x - tripod1_.x) -
                       (tripod3_.z - tripod1_.z) * (tripod2_.x - tripod1_.x);
            float pc = (tripod2_.x - tripod1_.x) * (tripod3_.y - tripod1_.y) -
                       (tripod3_.x - tripod1_.x) * (tripod2_.y - tripod1_.y);
            float pd = -(pa * tripod1_.x + pb * tripod1_.y + pc * tripod1_.z);

            float ps = sqrt(pa * pa + pb * pb + pc * pc);

            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            float pd2 = pa * point_sel_.x + pb * point_sel_.y + pc * point_sel_.z + pd;

            float s = 1;
            if (iter_count >= 5) {
                s = 1 - 1.8 * fabs(pd2) /
                            sqrt(sqrt(point_sel_.x * point_sel_.x + point_sel_.y * point_sel_.y +
                                      point_sel_.z * point_sel_.z));
            }

            if (s > 0.1 && pd2 != 0) {
                coeff_.x = s * pa;
                coeff_.y = s * pb;
                coeff_.z = s * pc;
                coeff_.range = s * pd2;
                coeff_.intensity = s * pd2;
                laser_cloud_ori_->push_back(surf_points_flat_->points[i]);
                coeff_sel_->push_back(coeff_);
            }
        }
    }
}

bool FeatureTracker::CalculateTransformationSurf(int iter_count) {
    int point_sel_num = laser_cloud_ori_->points.size();

    cv::Mat matA(point_sel_num, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, point_sel_num, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(point_sel_num, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transform_current_[0]);
    float crx = cos(transform_current_[0]);
    float sry = sin(transform_current_[1]);
    float cry = cos(transform_current_[1]);
    float srz = sin(transform_current_[2]);
    float crz = cos(transform_current_[2]);
    float tx = transform_current_[3];
    float ty = transform_current_[4];
    float tz = transform_current_[5];

    float a1 = crx * sry * srz;
    float a2 = crx * crz * sry;
    float a3 = srx * sry;
    float a4 = tx * a1 - ty * a2 - tz * a3;
    float a5 = srx * srz;
    float a6 = crz * srx;
    float a7 = ty * a6 - tz * crx - tx * a5;
    float a8 = crx * cry * srz;
    float a9 = crx * cry * crz;
    float a10 = cry * srx;
    float a11 = tz * a10 + ty * a9 - tx * a8;

    float b1 = -crz * sry - cry * srx * srz;
    float b2 = cry * crz * srx - sry * srz;
    float b5 = cry * crz - srx * sry * srz;
    float b6 = cry * srz + crz * srx * sry;

    float c1 = -b6;
    float c2 = b5;
    float c3 = tx * b6 - ty * b5;
    float c4 = -crx * crz;
    float c5 = crx * srz;
    float c6 = ty * c5 + tx * -c4;
    float c7 = b2;
    float c8 = -b1;
    float c9 = tx * -b2 - ty * -b1;

    for (int i = 0; i < point_sel_num; i++) {
        point_ori_ = laser_cloud_ori_->points[i];
        coeff_ = coeff_sel_->points[i];

        float arx = (-a1 * point_ori_.x + a2 * point_ori_.y + a3 * point_ori_.z + a4) * coeff_.x +
                    (a5 * point_ori_.x - a6 * point_ori_.y + crx * point_ori_.z + a7) * coeff_.y +
                    (a8 * point_ori_.x - a9 * point_ori_.y - a10 * point_ori_.z + a11) * coeff_.z;

        float arz = (c1 * point_ori_.x + c2 * point_ori_.y + c3) * coeff_.x +
                    (c4 * point_ori_.x - c5 * point_ori_.y + c6) * coeff_.y +
                    (c7 * point_ori_.x + c8 * point_ori_.y + c9) * coeff_.z;

        float aty = -b6 * coeff_.x + c4 * coeff_.y + b2 * coeff_.z;

        float d2 = coeff_.range;
        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = arz;
        matA.at<float>(i, 2) = aty;
        matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iter_count == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        is_degenerate_ = false;
        float eign_th[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eign_th[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                is_degenerate_ = true;
            } else {
                break;
            }
        }
        matP_ = matV.inv() * matV2;
    }

    if (is_degenerate_) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP_ * matX2;
    }

    transform_current_[0] += matX.at<float>(0, 0);
    transform_current_[2] += matX.at<float>(1, 0);
    transform_current_[4] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++) {
        if (std::isnan(transform_current_[i])) transform_current_[i] = 0;
    }

    float delta_R = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) + pow(rad2deg(matX.at<float>(1, 0)), 2));
    float delta_T = sqrt(pow(matX.at<float>(2, 0) * 100, 2));

    if (delta_R < 0.1 && delta_T < 0.1) {
        return false;
    }
    return true;
}

bool FeatureTracker::CalculateTransformationCorner(int iter_count) {
    int point_sel_num = laser_cloud_ori_->points.size();

    cv::Mat matA(point_sel_num, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, point_sel_num, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(point_sel_num, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transform_current_[0]);
    float crx = cos(transform_current_[0]);
    float sry = sin(transform_current_[1]);
    float cry = cos(transform_current_[1]);
    float srz = sin(transform_current_[2]);
    float crz = cos(transform_current_[2]);
    float tx = transform_current_[3];
    float ty = transform_current_[4];
    float tz = transform_current_[5];

    float b1 = -crz * sry - cry * srx * srz;
    float b2 = cry * crz * srx - sry * srz;
    float b3 = crx * cry;
    float b4 = tx * -b1 + ty * -b2 + tz * b3;
    float b5 = cry * crz - srx * sry * srz;
    float b6 = cry * srz + crz * srx * sry;
    float b7 = crx * sry;
    float b8 = tz * b7 - ty * b6 - tx * b5;

    float c5 = crx * srz;

    for (int i = 0; i < point_sel_num; i++) {
        point_ori_ = laser_cloud_ori_->points[i];
        coeff_ = coeff_sel_->points[i];

        float ary = (b1 * point_ori_.x + b2 * point_ori_.y - b3 * point_ori_.z + b4) * coeff_.x +
                    (b5 * point_ori_.x + b6 * point_ori_.y - b7 * point_ori_.z + b8) * coeff_.z;

        float atx = -b5 * coeff_.x + c5 * coeff_.y + b1 * coeff_.z;

        float atz = b7 * coeff_.x - srx * coeff_.y - b3 * coeff_.z;

        float d2 = coeff_.range;
        matA.at<float>(i, 0) = ary;
        matA.at<float>(i, 1) = atx;
        matA.at<float>(i, 2) = atz;
        matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iter_count == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        is_degenerate_ = false;
        float eign_th[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eign_th[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                is_degenerate_ = true;
            } else {
                break;
            }
        }
        matP_ = matV.inv() * matV2;
    }

    if (is_degenerate_) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP_ * matX2;
    }

    transform_current_[1] += matX.at<float>(0, 0);
    transform_current_[3] += matX.at<float>(1, 0);
    transform_current_[5] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++) {
        if (std::isnan(transform_current_[i])) transform_current_[i] = 0;
    }

    float delta_R = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2));
    float delta_T = sqrt(pow(matX.at<float>(1, 0) * 100, 2) + pow(matX.at<float>(2, 0) * 100, 2));

    if (delta_R < 0.1 && delta_T < 0.1) {
        return false;
    }
    return true;
}

void FeatureTracker::CheckSystemInitialization() {
    *laser_cloud_corner_last_ = *corner_points_less_sharp_;
    *laser_cloud_surf_last_ = *surf_points_less_flat_;

    kdtree_corner_last_->setInputCloud(laser_cloud_corner_last_);
    kdtree_surf_last_->setInputCloud(laser_cloud_surf_last_);

    laser_cloud_corner_last_num_ = laser_cloud_corner_last_->points.size();
    laser_cloud_surf_last_num_ = laser_cloud_surf_last_->points.size();

    system_inited_LM_ = true;
}

void FeatureTracker::UpdateTransformation() {
    if (laser_cloud_corner_last_num_ < 10 || laser_cloud_surf_last_num_ < 100) return;

    for (int iter_count1 = 0; iter_count1 < 25; iter_count1++) {
        laser_cloud_ori_->clear();
        coeff_sel_->clear();

        FindCorrespondingSurfFeatures(iter_count1);

        if (laser_cloud_ori_->points.size() < 10) continue;
        if (CalculateTransformationSurf(iter_count1) == false) break;
    }

    for (int iter_count2 = 0; iter_count2 < 25; iter_count2++) {
        laser_cloud_ori_->clear();
        coeff_sel_->clear();

        FindCorrespondingCornerFeatures(iter_count2);

        if (laser_cloud_ori_->points.size() < 10) continue;
        if (CalculateTransformationCorner(iter_count2) == false) break;
    }
}

void FeatureTracker::IntegrateTransformation() {
    float rx, ry, rz, tx, ty, tz;
    AccumulateRotation(transform_sum_[0], transform_sum_[1], transform_sum_[2], -transform_current_[0],
                       -transform_current_[1], -transform_current_[2], rx, ry, rz);

    float x1 = cos(rz) * (transform_current_[3]) - sin(rz) * (transform_current_[4]);
    float y1 = sin(rz) * (transform_current_[3]) + cos(rz) * (transform_current_[4]);
    float z1 = transform_current_[5];

    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    tx = transform_sum_[3] - (cos(ry) * x2 + sin(ry) * z2);
    ty = transform_sum_[4] - y2;
    tz = transform_sum_[5] - (-sin(ry) * x2 + cos(ry) * z2);

    if (params_.set_height_zero) {
        transform_sum_[0] = 0;
        transform_sum_[2] = 0;
        transform_sum_[4] = 0;
    } else {
        transform_sum_[0] = rx;
        transform_sum_[2] = rz;
        transform_sum_[4] = ty;
    }
    transform_sum_[1] = ry;
    transform_sum_[3] = tx;
    transform_sum_[5] = tz;
}

void FeatureTracker::AdjustDistortionCloudsLast() {
    // int corner_points_less_sharp_num = corner_points_less_sharp_->points.size();
    // for (int i = 0; i < corner_points_less_sharp_num; i++) {
    //   TransformToEnd(&corner_points_less_sharp_->points[i],
    //                  &corner_points_less_sharp_->points[i]);
    // }

    // int surf_points_less_flat_num = surf_points_less_flat_->points.size();
    // for (int i = 0; i < surf_points_less_flat_num; i++) {
    //   TransformToEnd(&surf_points_less_flat_->points[i],
    //                  &surf_points_less_flat_->points[i]);
    // }

    *laser_cloud_corner_last_ = *corner_points_less_sharp_;
    *laser_cloud_surf_last_ = *surf_points_less_flat_;

    laser_cloud_corner_last_num_ = laser_cloud_corner_last_->points.size();
    laser_cloud_surf_last_num_ = laser_cloud_surf_last_->points.size();

    if (laser_cloud_corner_last_num_ > 10 && laser_cloud_surf_last_num_ > 100) {
        kdtree_corner_last_->setInputCloud(laser_cloud_corner_last_);
        kdtree_surf_last_->setInputCloud(laser_cloud_surf_last_);
    }
}

void FeatureTracker::AdjustDistortionOutlierCloud() {
    common::PointType point;
    int cloud_size = outlier_cloud_->points.size();
    for (int i = 0; i < cloud_size; ++i) {
        point.x = outlier_cloud_->points[i].y;
        point.y = outlier_cloud_->points[i].z;
        point.z = outlier_cloud_->points[i].x;
        point.range = outlier_cloud_->points[i].range;
        point.intensity = outlier_cloud_->points[i].intensity;
        outlier_cloud_->points[i] = point;
    }
}

void FeatureTracker::SetCloudTimeStamp() {
    laser_cloud_corner_last_->header.stamp = corner_points_sharp_->header.stamp;
    laser_cloud_surf_last_->header.stamp = corner_points_sharp_->header.stamp;
    outlier_cloud_->header.stamp = corner_points_sharp_->header.stamp;
}

void FeatureTracker::Tracking() {
    UpdateTransformation();

    IntegrateTransformation();

    AdjustDistortionCloudsLast();

    SetCloudTimeStamp();
}

}  // namespace mapping::core
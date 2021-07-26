//
// Created by herenjie on 2021/4/22.
//

#include <glog/logging.h>
#include "local_map.h"

namespace mapping::core {
LocalMap::LocalMap() {
    cur_corner_ds_.reset(new pcl::PointCloud<PointXYZI>());  ///降采样后的当前帧的角点
    cur_surf_ds_.reset(new pcl::PointCloud<PointXYZI>());    ///降采样后的当前帧的平面点

    localmap_corner_ds_.reset(new pcl::PointCloud<PointXYZI>());  ///降采样后的局部地图的角点
    localmap_surf_ds_.reset(new pcl::PointCloud<PointXYZI>());    ///降采样后的局部地图的平面点

    corner_voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
    surf_voxel_filter_.setLeafSize(0.4, 0.4, 0.4);

    kdtree_corner_from_map_.reset(new pcl::KdTreeFLANN<PointXYZI>());
    kdtree_surf_from_map_.reset(new pcl::KdTreeFLANN<PointXYZI>());
}

SE3 LocalMap::track(const CloudPtr& corner_local_map, const CloudPtr& surf_local_map, const CloudPtr& corner_cur_scan,
                    const CloudPtr& surf_cur_scan, SE3& opt_pose_kf_w) {
    ClearCloud();
    corner_voxel_filter_.setInputCloud(corner_cur_scan);
    corner_voxel_filter_.filter(*cur_corner_ds_);
    corner_voxel_filter_.setInputCloud(corner_local_map);
    corner_voxel_filter_.filter(*localmap_corner_ds_);
    surf_voxel_filter_.setInputCloud(surf_local_map);
    surf_voxel_filter_.filter(*localmap_surf_ds_);
    surf_voxel_filter_.setInputCloud(surf_cur_scan);
    surf_voxel_filter_.filter(*cur_surf_ds_);

    if (localmap_corner_ds_->size() > 10 && localmap_surf_ds_->size() > 100) {
        t_opt_ = Eigen::Vector3d(opt_pose_kf_w.translation()[0], opt_pose_kf_w.translation()[1],
                                 opt_pose_kf_w.translation()[2]);
        //    q_opt_ = opt_pose_kf_w.unit_quaternion();
        q_opt_.w() = opt_pose_kf_w.unit_quaternion().w();
        q_opt_.x() = opt_pose_kf_w.unit_quaternion().x();
        q_opt_.y() = opt_pose_kf_w.unit_quaternion().y();
        q_opt_.z() = opt_pose_kf_w.unit_quaternion().z();

        POSE_SE3_ = opt_pose_kf_w.cast<float>();

        Scan2MapLM();
        opt_pose_kf_w = POSE_SE3_.cast<double>();

        ClearCloud();
    }

    return opt_pose_kf_w;
}

void LocalMap::Scan2MapLM() {
    kdtree_corner_from_map_->setInputCloud(localmap_corner_ds_);
    kdtree_surf_from_map_->setInputCloud(localmap_surf_ds_);
    int iter_num = 10;
    float last_sum_loss = 0.;
    //    float miu = 10e-6;
    float miu = 0.2;
    float L0_Lhlm_last = 0.;
    for (int i = 0; i < iter_num; i++) {
        PreCalCorner();
        PreCalSurf();
        int jac_rows = ppJacs_.size() + plJacs_.size();
        Eigen::Matrix<float, Eigen::Dynamic, 1> fr(jac_rows, 1);
        AccuHession(fr);
        float sum_loss = fr.norm();
        float delta_loss;
        if(i > 0){
            delta_loss = last_sum_loss - sum_loss;

            if (std::fabs(delta_loss) < 10e-6) {
//                LOG(INFO) << "ITER " << i << " : "
//                          << "delta_loss: " << delta_loss << ", sum_loss: " << sum_loss;
                return;
            }

            float ratio = delta_loss / L0_Lhlm_last;
            if (ratio < 0.25) {
                miu = 2 * miu;
            } else if (ratio > 0.75) {
                miu = miu / 3;
            }
//            LOG(INFO) << "ratio: " << ratio << ", delta_loss: " << delta_loss << ", L0_Lhlm_last: " << L0_Lhlm_last
//                      << ", miu: " << miu;
        }

        last_sum_loss = sum_loss;
        Eigen::Matrix<float, 6, 1> delta_x = (H_ + miu * Mat66f::Identity()).colPivHouseholderQr().solve(_JTf);
        //        Eigen::Matrix<float, 6,1> delta_x = H_.colPivHouseholderQr().solve(_JTf);

        L0_Lhlm_last = 0.5 * delta_x.transpose() * (miu * delta_x + _JTf);

        bool not_solved = false;
        for (int j = 0; j < 6; j++) {
            if (isnan(delta_x(j, 0))) {
                not_solved = true;
                break;
            }
        }
        if (not_solved) {
            LOG(INFO) << "not_solved ++++++++++++++++++++++++++++++";
            Eigen::Matrix<float, 6, 1> delta_x = (H_ + Mat66f::Identity()).colPivHouseholderQr().solve(_JTf);
        }
        for (int j = 0; j < 6; j++) {
            if (isnan(delta_x(j, 0))) {
                LOG(INFO) << "not_solved ---------------------------";
                return;
            }
        }

        if (delta_x.norm() < 10e-6) {
//            LOG(INFO) << "delta_x norm: " << delta_x.norm();
            return;
        }

        Sophus::Vector6f update_se3;  //更新量
        update_se3.setZero();         //设为0
        update_se3(0, 0) = delta_x(0, 0);
        update_se3(1, 0) = delta_x(1, 0);
        update_se3(2, 0) = delta_x(2, 0);
        update_se3(3, 0) = delta_x(3, 0);
        update_se3(4, 0) = delta_x(4, 0);
        update_se3(5, 0) = delta_x(5, 0);
        Sophus::SE3f se_update = Sophus::SE3f::exp(update_se3) * POSE_SE3_;
        POSE_SE3_ = se_update;
    }
}

void LocalMap::AccuHession(Eigen::Matrix<float, Eigen::Dynamic, 1>& fr) {
    const int jac_rows = ppJacs_.size() + plJacs_.size();
    Eigen::Matrix<float, Eigen::Dynamic, 6> Jacobian(jac_rows, 6);

    ///填入点到线雅克比
    for (size_t row = 0; row < plJacs_.size(); row++) {
        float scale = plJacs_[row].scale;
        Jacobian.row(row) = scale * plJacs_[row].deltaRes_deltaEspsilong.transpose();
        fr(row, 0) = -scale * plJacs_[row].residual;
    }
    ///填入点到面雅克比
    int pl_size = plJacs_.size();
    for (size_t row = 0; row < ppJacs_.size(); row++) {
        float scale = ppJacs_[row].scale;
        Jacobian.row(row + pl_size) = scale * ppJacs_[row].deltaRes_deltaEspsilong.transpose();
        fr(row + pl_size, 0) = -scale * ppJacs_[row].residual;
    }
    H_ = Mat66f::Identity();
    H_ = Jacobian.transpose() * Jacobian;
    _JTf = Jacobian.transpose() * fr;
}

void LocalMap::PreCalCorner() {
    ///将当前点云转到世界坐标系下
    CloudPtr cur_corner_ds_w(new pcl::PointCloud<PointXYZI>());
    SE3 T_kf2w(q_opt_, t_opt_);
    //    pcl::transformPointCloud(*cur_corner_ds_, *cur_corner_ds_w, T_kf2w.matrix());
    pcl::transformPointCloud(*cur_corner_ds_, *cur_corner_ds_w, POSE_SE3_.matrix());
    int num_cost = 0;
    plJacs_.clear();
    for (size_t i = 0; i < cur_corner_ds_w->size(); i++) {
        ///在局部地图中，找出距离当前帧这个角点最近的5个点
        auto& a_point_cur = cur_corner_ds_w->points[i];  ///当前帧角点中的一个点，世界坐标系下
        std::vector<int> point_search_index;
        std::vector<float> point_search_dis;
        kdtree_corner_from_map_->nearestKSearch(a_point_cur, 5, point_search_index, point_search_dis);
        if (point_search_dis[4] < 1.0) {
            /// 1.求出局部地图中5个点的中点（均值）
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += localmap_corner_ds_->points[point_search_index[j]].x;
                cy += localmap_corner_ds_->points[point_search_index[j]].y;
                cz += localmap_corner_ds_->points[point_search_index[j]].z;
            }
            cx /= 5;
            cy /= 5;
            cz /= 5;
            /// 2.求出局部地图中5个点的均方差
            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++) {
                float ax = localmap_corner_ds_->points[point_search_index[j]].x - cx;
                float ay = localmap_corner_ds_->points[point_search_index[j]].y - cy;
                float az = localmap_corner_ds_->points[point_search_index[j]].z - cz;

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

            cv::Mat matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

            matA1.at<float>(0, 0) = a11;
            matA1.at<float>(0, 1) = a12;
            matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12;
            matA1.at<float>(1, 1) = a22;
            matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13;
            matA1.at<float>(2, 1) = a23;
            matA1.at<float>(2, 2) = a33;

            /// 3.局部地图中5个点的协方差特征分解
            cv::Mat matD1 = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
            cv::eigen(matA1, matD1, matV1);
            ///第一个特征值远大于第二个特征值就认为5点共线
            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                float x0 = a_point_cur.x;
                float y0 = a_point_cur.y;
                float z0 = a_point_cur.z;
                ///局部地图5点所在的直线参数： 两点确定一直线：（x1,y1,z1）&（x2,y2,z2）
                float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                ///叉乘求平行四边形面积
                float A = (x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1);
                float B = (x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1);
                float C = (y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1);

                float a012 = sqrt(A * A + B * B + C * C);

                float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));  ///直线长度

                //                float la = ((y1 - y2) * A + (z1 - z2) * B) / a012 / l12;
                //                float lb = ((x2 - x1) * A + (z1 - z2) * C) / a012 / l12;
                //                float lc = ((x2 - x1) * B + (y2 - y1) * C) / a012 / l12;

                float ld2 = a012 / l12;  ///点到直线距离

                float s = 1. - 0.9 * fabs(ld2);

                if (s > 0.1) {
                    float Jac_x0_1 = ((y1 - y2) * A + (z1 - z2) * B) / a012 / l12;
                    float Jac_y0_2 = ((x2 - x1) * A + (z1 - z2) * C) / a012 / l12;
                    float Jac_z0_3 = ((x2 - x1) * B + (y2 - y1) * C) / a012 / l12;
                    Vec3f deltaRes_deltaP{Jac_x0_1, Jac_y0_2, Jac_z0_3};
                    Mat36f deltaP_deltaEspsilong;
                    deltaP_deltaEspsilong << 1, 0, 0, 0, z0, -y0, 0, 1, 0, -z0, 0, x0, 0, 0, 1, y0, -x0, 0;

                    Vec6f deltaRes_deltaEspsilong = deltaRes_deltaP.transpose() * deltaP_deltaEspsilong;
                    Point_Line_Jac plj(deltaRes_deltaEspsilong, ld2, s);
                    plJacs_.emplace_back(plj);

                    num_cost++;
                }
            }
        }
    }
}

void LocalMap::PreCalSurf() {
    CloudPtr cur_surf_ds_w(new pcl::PointCloud<PointXYZI>());
    SE3 T_kf2w(q_opt_, t_opt_);
    //    pcl::transformPointCloud(*cur_surf_ds_, *cur_surf_ds_w, T_kf2w.matrix());
    pcl::transformPointCloud(*cur_surf_ds_, *cur_surf_ds_w, POSE_SE3_.matrix());

    int num_cost = 0;
    ppJacs_.clear();
    for (size_t i = 0; i < cur_surf_ds_w->size(); i++) {
        ///在平面点中找5个最近的点
        auto& a_point_cur = cur_surf_ds_w->points[i];
        float x0 = a_point_cur.x;
        float y0 = a_point_cur.y;
        float z0 = a_point_cur.z;
        std::vector<int> point_search_index;
        std::vector<float> point_search_dis;
        kdtree_surf_from_map_->nearestKSearch(a_point_cur, 5, point_search_index, point_search_dis);
        cv::Mat matA0 = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matB0 = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
        cv::Mat matX0 = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));
        if (point_search_dis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0.at<float>(j, 0) = localmap_surf_ds_->points[point_search_index[j]].x;
                matA0.at<float>(j, 1) = localmap_surf_ds_->points[point_search_index[j]].y;
                matA0.at<float>(j, 2) = localmap_surf_ds_->points[point_search_index[j]].z;
            }
            cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

            /// 5个点所在平面组成的参数
            float pa = matX0.at<float>(0, 0);
            float pb = matX0.at<float>(1, 0);
            float pc = matX0.at<float>(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool plane_valid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * localmap_surf_ds_->points[point_search_index[j]].x +
                         pb * localmap_surf_ds_->points[point_search_index[j]].y +
                         pc * localmap_surf_ds_->points[point_search_index[j]].z + pd) > 0.2) {
                    plane_valid = false;
                    break;
                }
            }

            if (plane_valid) {
                float pd2 = pa * a_point_cur.x + pb * a_point_cur.y + pc * a_point_cur.z + pd;

                float s = 1 - 0.9 * fabs(pd2) /
                                  sqrt(sqrt(a_point_cur.x * a_point_cur.x + a_point_cur.y * a_point_cur.y +
                                            a_point_cur.z * a_point_cur.z));

                if (s > 0.1) {
                    float inv_sqrt_sumsquare = 1. / std::sqrt(pa * pa + pb * pb + pc * pc);
                    float Jac_x0_1 = pa * inv_sqrt_sumsquare;
                    float Jac_y0_2 = pb * inv_sqrt_sumsquare;
                    float Jac_z0_3 = pc * inv_sqrt_sumsquare;
                    Vec3f deltaRes_deltaP{Jac_x0_1, Jac_y0_2, Jac_z0_3};
                    Mat36f deltaP_deltaEspsilong;
                    deltaP_deltaEspsilong << 1, 0, 0, 0, z0, -y0, 0, 1, 0, -z0, 0, x0, 0, 0, 1, y0, -x0, 0;
                    Vec6f deltaRes_deltaEspsilong = deltaRes_deltaP.transpose() * deltaP_deltaEspsilong;
                    Point_Plane_Jac ppj(deltaRes_deltaEspsilong, pd2 * inv_sqrt_sumsquare, s);
                    ppJacs_.emplace_back(ppj);

                    num_cost++;
                }
            }
        }
    }
}

void LocalMap::ClearCloud() {
    cur_corner_ds_->clear();  ///降采样后的当前帧的角点
    cur_surf_ds_->clear();    ///降采样后的当前帧的平面点

    localmap_corner_ds_->clear();  ///降采样后的局部地图的角点
    localmap_surf_ds_->clear();    ///降采样后的局部地图的平面点
}

}
#include <glog/logging.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar16/lidar_matching/feature_matching/local_mapper.h"
#include "lidar16/lidar_matching/feature_matching/local_mapper_impl.h"

namespace mapping {
namespace core {

LocalMapper::LocalMapper(FeatureMatchingParams& params) : impl_(new LocalMapperImpl) {
    impl_->params_ = params;
    Initialization();
}

LocalMapper::~LocalMapper() { std::cout << "析构LocalMapper================：" << this << std::endl; }

void LocalMapper::Initialization() {
    impl_->laser_cloud_corner_last_.reset(new PointCloudType());
    impl_->laser_cloud_surf_last_.reset(new PointCloudType());
    impl_->laser_outlier_cloud_.reset(new PointCloudType());
    impl_->laser_cloud_surf_total_last_.reset(new PointCloudType());

    impl_->laser_cloud_corner_last_DS_.reset(new PointCloudType());
    impl_->laser_cloud_surf_last_DS_.reset(new PointCloudType());
    impl_->laser_cloud_surf_total_last_DS_.reset(new PointCloudType());
    impl_->ground_cloud_cur_scan_DS_.reset(new PointCloudType());

    impl_->laser_cloud_corner_from_map_.reset(new PointCloudType());
    impl_->laser_cloud_surf_from_map_.reset(new PointCloudType());
    impl_->laser_cloud_corner_from_map_DS_.reset(new PointCloudType());
    impl_->laser_cloud_surf_from_map_DS_.reset(new PointCloudType());
    impl_->ground_cloud_local_map_DS_.reset(new PointCloudType());

    impl_->laser_cloud_corner_from_map_DS_num_ = 0;
    impl_->laser_cloud_surf_from_map_DS_num_ = 0;
    impl_->ground_cloud_local_map_DS_num_ = 0;
    impl_->laser_cloud_corner_last_DS_num_ = 0;
    impl_->laser_cloud_surf_total_last_DS_num_ = 0;
    impl_->ground_cloud_cur_scan_DS_num_ = 0;

    impl_->kdtree_corner_from_map_.reset(new pcl::KdTreeFLANN<PointType>());
    impl_->kdtree_surf_from_map_.reset(new pcl::KdTreeFLANN<PointType>());
    impl_->kdtree_ground_from_map_.reset(new pcl::KdTreeFLANN<PointType>());

    impl_->downsize_filter_corner_.setLeafSize(0.2, 0.2, 0.2);
    impl_->downsize_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
    impl_->downsize_filter_ground_.setLeafSize(0.2, 0.2, 0.2);

    impl_->laser_cloud_ori_.reset(new PointCloudType());
    impl_->coeff_sel_.reset(new PointCloudType());
    impl_->correspond_dis_.reset(new PointCloudType());

    impl_->matA0_ = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
    impl_->matB0_ = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
    impl_->matX0_ = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

    impl_->matA1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    impl_->matD1_ = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    impl_->matV1_ = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
}

void LocalMapper::ClearCloud() {
    impl_->laser_cloud_corner_from_map_->clear();
    impl_->laser_cloud_surf_from_map_->clear();
    impl_->laser_cloud_corner_from_map_DS_->clear();
    impl_->laser_cloud_surf_from_map_DS_->clear();
    impl_->ground_cloud_local_map_DS_->clear();
}

///拆分lego-loam
bool LocalMapper::FrameLocalMapper(const CloudPtr& corner_cloud_local_map, const CloudPtr& surf_cloud_local_map, const CloudPtr& ground_cloud_local_map,
                                   const CloudPtr& corner_cloud_cur_scan, const CloudPtr& surf_cloud_cur_scan, const CloudPtr& ground_cloud_cur_scan,
                                   SE3& opt_pose_kf_w) {
    ClearCloud();
    auto downsample_cloud = []( pcl::VoxelGrid<PointType>& voxel_filter,
                                const CloudPtr& input_cloud,
                                CloudPtr& output_cloud_ds,
                                int& cloud_num_ds){
      voxel_filter.setInputCloud(input_cloud);
      voxel_filter.filter(*output_cloud_ds);
      cloud_num_ds = output_cloud_ds->size();  ///局部地图角点数量
    };
    ///局部地图角点
    downsample_cloud(impl_->downsize_filter_corner_, corner_cloud_local_map,
        impl_->laser_cloud_corner_from_map_DS_, impl_->laser_cloud_corner_from_map_DS_num_);
    ////局部地图平面点
    downsample_cloud(impl_->downsize_filter_surf_, surf_cloud_local_map,
        impl_->laser_cloud_surf_from_map_DS_, impl_->laser_cloud_surf_from_map_DS_num_);
    ////局部地图地面点
    downsample_cloud(impl_->downsize_filter_ground_, ground_cloud_local_map,
        impl_->ground_cloud_local_map_DS_, impl_->ground_cloud_local_map_DS_num_);

    ///当前帧角点
    downsample_cloud(impl_->downsize_filter_corner_, corner_cloud_cur_scan,
        impl_->laser_cloud_corner_last_DS_, impl_->laser_cloud_corner_last_DS_num_);
    ///当前帧平面点
    downsample_cloud(impl_->downsize_filter_surf_, surf_cloud_cur_scan,
        impl_->laser_cloud_surf_total_last_DS_, impl_->laser_cloud_surf_total_last_DS_num_);
    ///当前帧地面点
    downsample_cloud(impl_->downsize_filter_ground_, ground_cloud_cur_scan,
        impl_->ground_cloud_cur_scan_DS_, impl_->ground_cloud_cur_scan_DS_num_);

    impl_->t_prior_ =
        Eigen::Vector3d(opt_pose_kf_w.translation()[0], opt_pose_kf_w.translation()[1], opt_pose_kf_w.translation()[2]);
    impl_->q_prior_.w() = opt_pose_kf_w.unit_quaternion().w();
    impl_->q_prior_.x() = opt_pose_kf_w.unit_quaternion().x();
    impl_->q_prior_.y() = opt_pose_kf_w.unit_quaternion().y();
    impl_->q_prior_.z() = opt_pose_kf_w.unit_quaternion().z();
    //
    //    impl_->t_opt_last_ = impl_->t_prior_;
    //    impl_->q_opt_last_ = impl_->q_prior_;
    //
    impl_->t_opt_ =
        Eigen::Vector3d(opt_pose_kf_w.translation()[0], opt_pose_kf_w.translation()[1], opt_pose_kf_w.translation()[2]);
    impl_->q_opt_.w() = opt_pose_kf_w.unit_quaternion().w();
    impl_->q_opt_.x() = opt_pose_kf_w.unit_quaternion().x();
    impl_->q_opt_.y() = opt_pose_kf_w.unit_quaternion().y();
    impl_->q_opt_.z() = opt_pose_kf_w.unit_quaternion().z();

    Scan2MapOptimization_ceres();

    opt_pose_kf_w = SE3(impl_->q_opt_, impl_->t_opt_);
    //    opt_pose_kf_w = pose_from;
    ClearCloud();
    return true;
}

void LocalMapper::Scan2MapOptimization_ceres() {
    if (impl_->laser_cloud_corner_from_map_DS_num_ > 10 && impl_->laser_cloud_surf_from_map_DS_num_ > 100) {
        impl_->kdtree_corner_from_map_->setInputCloud(impl_->laser_cloud_corner_from_map_DS_);
        impl_->kdtree_surf_from_map_->setInputCloud(impl_->laser_cloud_surf_from_map_DS_);
        impl_->kdtree_ground_from_map_->setInputCloud(impl_->ground_cloud_local_map_DS_);

        double last_final_cost = 0.;
        bool if_first = true;
        //        for (int iter_count = 0; iter_count < 30; iter_count++) {
        for (int iter_count = 0; iter_count < 5; iter_count++) {
            impl_->laser_cloud_ori_->clear();
            impl_->coeff_sel_->clear();
            impl_->correspond_dis_->clear();

            ceres::Problem ceres_problem;

            ceres::LocalParameterization* quaternion_local_parameterization =
                new ceres::EigenQuaternionParameterization;
            ceres_problem.AddParameterBlock(impl_->q_opt_.coeffs().data(), 4, quaternion_local_parameterization);
            ceres_problem.AddParameterBlock(impl_->t_opt_.data(), 3);

            AddCornerCost(ceres_problem);
            AddSurfCost(ceres_problem);
            AddGroundCost(ceres_problem);

            Eigen::Matrix<double, 6, 6> sqrt_information_prior = Eigen::Matrix<double, 6, 6>::Identity();
            sqrt_information_prior(0, 0) = 30.;
            sqrt_information_prior(1, 1) = 30.;
            sqrt_information_prior(2, 2) = 30.;
            sqrt_information_prior(3, 3) = 20.;
            sqrt_information_prior(4, 4) = 20.;
            sqrt_information_prior(5, 5) = 20.;
            //            sqrt_information_prior(0,0) = 10.;
            //            sqrt_information_prior(1,1) = 10.;
            //            sqrt_information_prior(2,2) = 10.;
            //            sqrt_information_prior(3,3) = 10.;
            //            sqrt_information_prior(4,4) = 10.;
            //            sqrt_information_prior(5,5) = 10.;

            //            ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.02);//0.0009
            ceres::LossFunction* loss_function = new ceres::CauchyLoss(10.02);  // 0.0009
            ceres_problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<PosePrior, 6, 4, 3>(new PosePrior(impl_->t_prior_,  //测量
                                                                                  impl_->q_prior_,  //测量
                                                                                  sqrt_information_prior)),
                loss_function, impl_->q_opt_.coeffs().data(), impl_->t_opt_.data());

            ceres::Solver::Options options;
            //            options.max_num_iterations = 1;
            options.max_num_iterations = 3;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = false;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &ceres_problem, &summary);
            //            LOG(WARNING) <<"ceres solver optimizer summary.final_cost: "<< summary.final_cost << "\n";

            if (if_first) {
                if_first = false;
                last_final_cost = summary.final_cost;
                continue;
            } else if (fabs(summary.final_cost - last_final_cost) > 0.0001) {
                last_final_cost = summary.final_cost;
                continue;
            }
            break;
        }
    } else {
        impl_->matching_degenerate_.push_back(false);
    }
}

void LocalMapper::AddCornerCost(ceres::Problem& ceres_problem) {
    CloudPtr laser_cloud_corner_last_DS_w(new PointCloudType());
    SE3 T_kf2w(impl_->q_opt_, impl_->t_opt_);
    pcl::transformPointCloud(*impl_->laser_cloud_corner_last_DS_, *laser_cloud_corner_last_DS_w, T_kf2w.matrix());
    int num_cost = 0;
    for (int i = 0; i < impl_->laser_cloud_corner_last_DS_num_; i++) {
        ///在局部地图中，找出距离当前帧这个角点最近的5个点
        impl_->point_sel_ = laser_cloud_corner_last_DS_w->points[i];  ///当前帧角点中的一个点，世界坐标系下
        impl_->kdtree_corner_from_map_->nearestKSearch(impl_->point_sel_, 5, impl_->point_search_index_,
                                                       impl_->point_search_dis_);
        if (impl_->point_search_dis_[4] < 1.0) {
            /// 1.求出局部地图中5个点的中点（均值）
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].x;
                cy += impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].y;
                cz += impl_->laser_cloud_corner_from_map_DS_->points[impl_->point_search_index_[j]].z;
            }
            cx /= 5;
            cy /= 5;
            cz /= 5;
            /// 2.求出局部地图中5个点的均方差
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
            /// 3.局部地图中5个点的协方差特征分解
            cv::eigen(impl_->matA1_, impl_->matD1_, impl_->matV1_);
            ///第一个特征值远大于第二个特征值就认为5点共线
            if (impl_->matD1_.at<float>(0, 0) > 3 * impl_->matD1_.at<float>(0, 1)) {
                float x0 = impl_->point_sel_.x;
                float y0 = impl_->point_sel_.y;
                float z0 = impl_->point_sel_.z;
                ///局部地图5点所在的直线参数： 两点确定一直线：（x1,y1,z1）&（x2,y2,z2）
                float x1 = cx + 0.1 * impl_->matV1_.at<float>(0, 0);
                float y1 = cy + 0.1 * impl_->matV1_.at<float>(0, 1);
                float z1 = cz + 0.1 * impl_->matV1_.at<float>(0, 2);
                float x2 = cx - 0.1 * impl_->matV1_.at<float>(0, 0);
                float y2 = cy - 0.1 * impl_->matV1_.at<float>(0, 1);
                float z2 = cz - 0.1 * impl_->matV1_.at<float>(0, 2);

                ///叉乘求平行四边形面积
                float a012 = sqrt(
                    ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                    ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                    ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));  ///直线长度

                float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                            (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                           a012 / l12;

                float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                             (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                           a012 / l12;

                float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                             (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                           a012 / l12;

                float ld2 = a012 / l12;  ///点到直线距离

                float s = 1 - 0.9 * fabs(ld2);

                impl_->coeff_.x = s * la;
                impl_->coeff_.y = s * lb;
                impl_->coeff_.z = s * lc;
                impl_->coeff_.intensity = s * ld2;
                if (s > 0.1) {
                    impl_->laser_cloud_ori_->push_back(impl_->point_ori_);
                    impl_->coeff_sel_->push_back(impl_->coeff_);

                    double x = impl_->laser_cloud_corner_last_DS_->points[i].x;
                    double y = impl_->laser_cloud_corner_last_DS_->points[i].y;
                    double z = impl_->laser_cloud_corner_last_DS_->points[i].z;
                    Eigen::Vector3d point_tmp(x, y, z);    ///点坐标
                    Eigen::Matrix<double, 6, 1> line_tmp;  ///直线参数
                    line_tmp << x1, y1, z1, x2, y2, z2;
                    double scale = 1 - 0.9 * fabs(ld2);

                    //                    ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.01);//0.0009
                    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.01);  // 0.0009
                    ceres_problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Point2LineResidual, 1, 4, 3>(
                                                       new Point2LineResidual(point_tmp, line_tmp)),
                                                   loss_function, impl_->q_opt_.coeffs().data(), impl_->t_opt_.data());
                    num_cost++;
                }

                PointType point_dist;
                point_dist.intensity = ld2;
                impl_->correspond_dis_->push_back(point_dist);
            }
        }
    }
    //    LOG(INFO)<<"AddCornerCost: "<<num_cost;
}

void LocalMapper::AddSurfCost(ceres::Problem& ceres_problem) {
    CloudPtr laser_cloud_surf_total_last_DS_w(new PointCloudType());
    SE3 T_kf2w(impl_->q_opt_, impl_->t_opt_);
    pcl::transformPointCloud(*impl_->laser_cloud_surf_total_last_DS_, *laser_cloud_surf_total_last_DS_w,
                             T_kf2w.matrix());

    int num_cost = 0;
    for (int i = 0; i < impl_->laser_cloud_surf_total_last_DS_num_; i++) {
        ///在平面点中找5个最近的点
        impl_->point_sel_ = laser_cloud_surf_total_last_DS_w->points[i];
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

            /// 5个点所在平面组成的参数
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
                impl_->coeff_.intensity = s * pd2;

                if (s > 0.1) {
                    impl_->laser_cloud_ori_->push_back(impl_->point_ori_);
                    impl_->coeff_sel_->push_back(impl_->coeff_);

                    double x = impl_->laser_cloud_surf_total_last_DS_->points[i].x;
                    double y = impl_->laser_cloud_surf_total_last_DS_->points[i].y;
                    double z = impl_->laser_cloud_surf_total_last_DS_->points[i].z;
                    Eigen::Vector3d point_tmp(x, y, z);     ///点坐标
                    Eigen::Matrix<double, 4, 1> plane_tmp;  ///直线参数
                    plane_tmp << pa, pb, pc, pd;
                    double scale = s;
                    //                    ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.01);//0.0009
                    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.01);  // 0.0009
                    ceres_problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Point2planeResidual, 1, 4, 3>(
                                                       new Point2planeResidual(point_tmp, plane_tmp)),
                                                   loss_function, impl_->q_opt_.coeffs().data(), impl_->t_opt_.data());
                    num_cost++;
                }

                PointType point_dist;
                point_dist.intensity = pd2;
                impl_->correspond_dis_->push_back(point_dist);
            }
        }
    }

    //    LOG(INFO)<<"AddSurfCost: "<<num_cost;
}

void LocalMapper::AddGroundCost(ceres::Problem& ceres_problem){

    CloudPtr cur_ground_points_w(new pcl::PointCloud<pcl::PointXYZI>);
    SE3 T_kf2w(impl_->q_opt_,impl_->t_opt_);
    pcl::transformPointCloud(*impl_->ground_cloud_cur_scan_DS_, *cur_ground_points_w, T_kf2w.matrix());  ///将当前帧地面点转到世界坐标系

    for (int i = 0; i < cur_ground_points_w->size(); i++) {
        ///在平面点中找5个最近的点
        impl_->point_sel_ = cur_ground_points_w->points[i];
        impl_->kdtree_ground_from_map_->nearestKSearch(impl_->point_sel_, 5, impl_->point_search_index_,
            impl_->point_search_dis_);

//        if (impl_->point_search_dis_[4] < 1.0) {
        if (impl_->point_search_dis_[4] < 5.0) {
            for (int j = 0; j < 5; j++) {
                impl_->matA0_.at<float>(j, 0) =
                    impl_->ground_cloud_local_map_DS_->points[impl_->point_search_index_[j]].x;
                impl_->matA0_.at<float>(j, 1) =
                    impl_->ground_cloud_local_map_DS_->points[impl_->point_search_index_[j]].y;
                impl_->matA0_.at<float>(j, 2) =
                    impl_->ground_cloud_local_map_DS_->points[impl_->point_search_index_[j]].z;
            }
            cv::solve(impl_->matA0_, impl_->matB0_, impl_->matX0_, cv::DECOMP_QR);

            ///5个点所在平面组成的参数
            float pa = impl_->matX0_.at<float>(0, 0);
            float pb = impl_->matX0_.at<float>(1, 0);
            float pc = impl_->matX0_.at<float>(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool plane_valid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * impl_->ground_cloud_local_map_DS_->points[impl_->point_search_index_[j]].x +
                    pb * impl_->ground_cloud_local_map_DS_->points[impl_->point_search_index_[j]].y +
                    pc * impl_->ground_cloud_local_map_DS_->points[impl_->point_search_index_[j]].z + pd) > 0.2) {
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
                impl_->coeff_.intensity = s * pd2;

                if (s > 0.1) {
                    double x = impl_->ground_cloud_cur_scan_DS_->points[i].x;
                    double y = impl_->ground_cloud_cur_scan_DS_->points[i].y;
                    double z = impl_->ground_cloud_cur_scan_DS_->points[i].z;
                    Eigen::Vector3d           point_tmp(x,y,z);///点坐标
                    Eigen::Matrix<double,4,1> plane_tmp;///直线参数
                    plane_tmp << pa, pb, pc, pd;
                    double scale = s;
//                    ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.01);//0.0009
                    ceres::LossFunction* loss_function = new ceres::CauchyLoss(1.01);//0.0009
                    ceres_problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<Point2planeResidual,1,4,3>(
                            new Point2planeResidual(point_tmp,
                                plane_tmp)),
                        loss_function,
                        impl_->q_opt_.coeffs().data(),
                        impl_->t_opt_.data());
                }
            }
        }
    }

//    LOG(INFO)<<"AddSurfCost: "<<num_cost;
}

}  // namespace core
}  // namespace mapping

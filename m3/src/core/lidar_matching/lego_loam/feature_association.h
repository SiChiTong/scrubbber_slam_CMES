//
// Created by herenjie on 2021/4/28.
//

#ifndef CARLA_ROS_FEATURE_ASSOCIATION_H
#define CARLA_ROS_FEATURE_ASSOCIATION_H

#include "../../../common/mapping_point_types.h"
#include "utility.h"
#include "image_projection.h"
using namespace mapping::common;
namespace mapping::core {
class FeatureAssociation {
   public:
//    FeatureAssociation(const ros::NodeHandle& nh);
    FeatureAssociation();
    ~FeatureAssociation(){};
    void InsertLidar(const CloudPtr& lidar);
    void GetSegCloud(CloudPtr& sharp_cloud, CloudPtr& less_sharp_cloud, CloudPtr& flat_cloud,
                     CloudPtr& less_flat_cloud) {
        auto load_cloud = [&](CloudPtr out_cloud, const CloudPtr& in_cloud) {
            if (!out_cloud) {
                out_cloud = boost::make_shared<pcl::PointCloud<PointXYZI>>();
            }
            *out_cloud = *in_cloud;
        };
        load_cloud(sharp_cloud, cornerPointsSharp);
        load_cloud(less_sharp_cloud, cornerPointsLessSharp);
        load_cloud(flat_cloud, surfPointsFlat);
        load_cloud(less_flat_cloud, surfPointsLessFlat);
    };

   private:
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();
    void initializationValue();

//    void PubCloud();

   private:
//    ros::NodeHandle nh_;
//    ros::Publisher sharp_pub_;
//    ros::Publisher less_sharp_pub_;
//    ros::Publisher flat_pub_;
//    ros::Publisher less_flat_pub_;
    std::shared_ptr<ImageProjection> ipj_ptr_;

    CloudPtr segmentedCloud;
    CloudPtr outlierCloud;

    CloudPtr cornerPointsSharp;
    CloudPtr cornerPointsLessSharp;
    CloudPtr surfPointsFlat;
    CloudPtr surfPointsLessFlat;

    CloudPtr surfPointsLessFlatScan;
    CloudPtr surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointXYZI> downSizeFilter;

    segMsg segInfo;

    std::vector<smoothness_t> cloudSmoothness;
    float* cloudCurvature;
    int* cloudNeighborPicked;
    int* cloudLabel;

    int skipFrameNum;

    float transformCur[6];
    float transformSum[6];

    CloudPtr laserCloudCornerLast;
    CloudPtr laserCloudSurfLast;
    CloudPtr laserCloudOri;
    CloudPtr coeffSel;

    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtreeSurfLast;
};
}
#endif  // CARLA_ROS_FEATURE_ASSOCIATION_H

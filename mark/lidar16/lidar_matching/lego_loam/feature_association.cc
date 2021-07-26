//
// Created by herenjie on 2021/4/28.
//

#include "feature_association.h"

FeatureAssociation::FeatureAssociation(const ros::NodeHandle& nh):nh_(nh){
    ipj_ptr_ = std::make_shared<ImageProjection>(nh_);

    sharp_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sharp_cloud",1);
    less_sharp_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/less_sharp_cloud",1);
    flat_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/flat_cloud",1);
    less_flat_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/less_flat_cloud",1);
    initializationValue();
}

void FeatureAssociation::InsertLidar(const CloudPtr& lidar){
    ipj_ptr_->InsertLidar(lidar);
    segmentedCloud->clear();
    ipj_ptr_->GetSegCloud(segmentedCloud, segInfo);
    groundCloud->clear();
    *groundCloud = *ipj_ptr_->GetGroundCloud();

    calculateSmoothness();
    markOccludedPoints();
    extractFeatures();
    PubCloud();
}

void FeatureAssociation::calculateSmoothness()
{
    int cloudSize = segmentedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {

        float diffRange =
                  segInfo.segmentedCloudRange[i-5] + segInfo.segmentedCloudRange[i-4]
                + segInfo.segmentedCloudRange[i-3] + segInfo.segmentedCloudRange[i-2]
                + segInfo.segmentedCloudRange[i-1] + segInfo.segmentedCloudRange[i+1]
                + segInfo.segmentedCloudRange[i+2] + segInfo.segmentedCloudRange[i+3]
                + segInfo.segmentedCloudRange[i+4] + segInfo.segmentedCloudRange[i+5]
                - segInfo.segmentedCloudRange[i] * 10;

        cloudCurvature[i] = diffRange*diffRange;

        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;

        cloudSmoothness[i].value = cloudCurvature[i];
        cloudSmoothness[i].ind = i;
    }
}

void FeatureAssociation::markOccludedPoints()
{
    int cloudSize = segmentedCloud->points.size();

    for (int i = 5; i < cloudSize - 6; ++i){

        float depth1 = segInfo.segmentedCloudRange[i];
        float depth2 = segInfo.segmentedCloudRange[i+1];
        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[i+1] - segInfo.segmentedCloudColInd[i]));

        ///紧临的两个点可能不是实际挨着的列
        if (columnDiff < 10){

            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        }

        float diff1 = std::abs(float(segInfo.segmentedCloudRange[i-1] - segInfo.segmentedCloudRange[i]));
        float diff2 = std::abs(float(segInfo.segmentedCloudRange[i+1] - segInfo.segmentedCloudRange[i]));

        if (diff1 > 0.02 * segInfo.segmentedCloudRange[i] && diff2 > 0.02 * segInfo.segmentedCloudRange[i])
            cloudNeighborPicked[i] = 1;
    }
}

void FeatureAssociation::extractFeatures()
{
    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    for (int i = 0; i < N_SCAN; i++) {

        surfPointsLessFlatScan->clear();

        for (int j = 0; j < 6; j++) {
            int sp = (segInfo.startRingIndex[i] * (6 - j)    + segInfo.endRingIndex[i] * j) / 6;
            int ep = (segInfo.startRingIndex[i] * (5 - j)    + segInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

            int largestPickedNum = 0;

            for (int k = ep; k >= sp; k--) {
                int ind = cloudSmoothness[k].ind;

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > edgeThreshold &&
                    !segInfo.segmentedCloudGroundFlag[ind]) {

                    largestPickedNum++;
                    if (largestPickedNum <= 2) {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp->push_back(segmentedCloud->points[ind]);
                        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    } else if (largestPickedNum <= 20) {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;

            for (int k = sp; k <= ep; k++) {
                int ind = cloudSmoothness[k].ind;
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < surfThreshold &&
                    segInfo.segmentedCloudGroundFlag[ind]) {

                    cloudLabel[ind] = -1;
                    surfPointsFlat->push_back(segmentedCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {

                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {

                        int columnDiff = std::abs(int(segInfo.segmentedCloudColInd[ind + l] - segInfo.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
                }
            }

        }

        surfPointsLessFlatScanDS->clear();
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.filter(*surfPointsLessFlatScanDS);

        *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    }
}


void FeatureAssociation::initializationValue()
{
    cloudCurvature = new float[N_SCAN*Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
    cloudLabel = new int[N_SCAN*Horizon_SCAN];

    cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
    cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
    surfPointsFlat.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());
    groundCloud.reset(new pcl::PointCloud<PointType>());

    surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());


    skipFrameNum = 1;

    for (int i = 0; i < 6; ++i){
        transformCur[i] = 0;
        transformSum[i] = 0;
    }

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());
}

void FeatureAssociation::PubCloud(){
    auto pub_lambda = [&](const ros::Publisher& publisher, const CloudPtr& cloud){
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(*cloud, cloud_ros);
        cloud_ros.header.frame_id = "map";
        cloud_ros.header.stamp = ros::Time::now();
        publisher.publish(cloud_ros);
    };
    pub_lambda(sharp_pub_, cornerPointsSharp);
    pub_lambda(less_sharp_pub_, cornerPointsLessSharp);
    pub_lambda(flat_pub_, surfPointsFlat);
    pub_lambda(less_flat_pub_, surfPointsLessFlat);
}
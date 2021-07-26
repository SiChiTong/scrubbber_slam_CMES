//
// Created by herenjie on 2021/4/27.
//

#ifndef CARLA_ROS_IMAGE_PROJECTION_H
#define CARLA_ROS_IMAGE_PROJECTION_H

#include <glog/logging.h>
#include "common/utility.h"
#include "common/point_type.h"

class ImageProjection {
public:
    ImageProjection(const ros::NodeHandle & nh);
    ~ImageProjection(){};
    void InsertLidar(const CloudPtr& lidar);
    CloudPtr GetGroundCloud(){
        return groundCloud;
    };
    void GetSegCloud(CloudPtr seg_cloud, segMsg& segmsg){
        *seg_cloud = *segmentedCloud;
        segmsg =  seg_msg;
    };
private:
    void allocateMemory();
    void resetParameters();
    void findStartEndAngle();
    void projectPointCloud();
    void groundRemoval();
    void cloudSegmentation();
    void labelComponents(int row, int col);
private:
    ros::NodeHandle nh_;
    ros::Publisher ground_pub_;
    ros::Publisher seg_pub_;
    pcl::PointCloud<PointType>::Ptr laserCloudIn;        /// 接收到原始点云， 去除了nan点的
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;        ///地面点云
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    segMsg seg_msg;
//    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;

    std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

};

#endif //CARLA_ROS_IMAGE_PROJECTION_H

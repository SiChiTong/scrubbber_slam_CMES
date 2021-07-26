#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>


#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

const string pointCloudTopic = "/lidar";
const string imuTopic = "/imu/data";

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
const bool useCloudRing = false; // if true, ang_res_y and ang_bottom are not used

//// VLP-16
// const int N_SCAN = 16;
// const int Horizon_SCAN = 1800;
// const float ang_res_x = 0.2;
// const float ang_res_y = 2.0;
// const float ang_bottom = 15.0+0.1;
// const int groundScanInd = 7;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
//extern const int N_SCAN = 64;
//extern const int Horizon_SCAN = 1024;
//extern const float ang_res_x = 360.0/float(Horizon_SCAN);
//extern const float ang_res_y = 33.2/float(N_SCAN-1);
//extern const float ang_bottom = 16.6+0.1;
//extern const int groundScanInd = 15;
//
// Ouster suteng-80
const int N_SCAN = 80;
const int Horizon_SCAN = 1800;
const float ang_res_x = 0.2;
const float ang_res_y = 36.36 / float(80-1);
const float ang_bottom = 21; ///角度范围为[-20.5489 , 15.8124]
const int groundScanInd = 40;

const bool loopClosureEnableFlag = true;
//double mappingProcessInterval = 0.3;

const float scanPeriod = 0.1;
const int systemDelay = 0;
const int imuQueLength = 200;

const float sensorMinimumRange = 1.0;
const float sensorMountAngle = 0.0; //0.0
const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
const int segmentValidPointNum = 5;
const int segmentValidLineNum = 3;
const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


const int edgeFeatureNum = 2;
const int surfFeatureNum = 4;
const int sectionsTotal = 6;
const float edgeThreshold = 0.1;
const float surfThreshold = 0.1;
const float nearestFeatureSearchSqDist = 25;

// Mapping Paramss
const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)

// history key frames (history submap for loop closure)
const float historyKeyframeSearchRadius = 20.0; // NOT used in Scan Context-based loop detector / default 7.0; key frame that is within n meters from current pose will be considerd for loop closure
const int   historyKeyframeSearchNum = 25; // 2n+1 number of history key frames will be fused into a submap for loop closure
const float historyKeyframeFitnessScore = 1.5; // default 0.3; the smaller the better alignment

const float globalMapVisualizationSearchRadius = 1500.0; // key frames with in n meters will be visualized

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

struct segMsg{
    int startRingIndex[N_SCAN] = {0}; /// 每一线起始点
    int endRingIndex[N_SCAN] = {0};   /// 每一线结束点

    float startOrientation;  ///起始角度
    float endOrientation;    ///结束角度
    float orientationDiff;   ///结束角度 - 起始角度

    bool segmentedCloudGroundFlag[N_SCAN*Horizon_SCAN] = {false}; //# true - ground point, false - other points
    unsigned int  segmentedCloudColInd[N_SCAN*Horizon_SCAN] = {0}; //# point column index in range image
    float segmentedCloudRange[N_SCAN*Horizon_SCAN] = {0};//# point range
} ;

#endif

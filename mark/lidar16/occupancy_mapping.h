//
// Created by gaoxiang on 2020/10/28.
//

#ifndef SCRUBBER_SLAM_OCCUPANCY_MAPPING_H
#define SCRUBBER_SLAM_OCCUPANCY_MAPPING_H

#include "common/model_2d_point.h"
#include "common/num_types.h"
#include "common/point_type.h"

#include <map>
#include <opencv2/core/mat.hpp>

namespace scrubber_slam { namespace lidar16 {

struct MLSubmap;
struct MLFrame;

/// 从16线转Occupancy grid
class OccupancyMapping {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    explicit OccupancyMapping(float resolution, float width, float height);
//    ~OccupancyMapping() = default;
    ~OccupancyMapping(){
        LOG(INFO)<<"release OccupancyMapping++++++++++++++++++++++++++++++++++++++++++";
        occupancy_.release();
    };

    /// 往submap中增加一个点云，更新submap本身的grid
    void PushNewKF(std::shared_ptr<MLFrame> frame);

    bool Quit();


//   private:
    /// 3D 点云转2D scan并插入submap
    void Convert3DTo2DScan();

    bool DetectPlaneCoeffs();

    void Add2DPoints(const std::vector<Vec2> &pt2d, const Vec3 &t, bool black = true);

    /// 可视化显示2D点云
    void PlotAngleDistance(const std::vector<Vec2> &pt2d, const std::vector<Vec3> &pt3d);

    void BuildModel();

    /**
     * 增加一个点
     * @param pt 世界系下某个点
     * @param occupy true时为障碍，false时为free
     * @param relative_height 相对地面高度
     * @param enforce 是否强制设置（此版本不使用）
     * @return
     */
    bool AddPoint(const Vec2f &pt, bool occupy, double relative_height, bool enforce = false);
    void SetPoint(const Vec2f &pt, bool occupy, float height);
    Vec2i Submap2Image(const Vec2f pt);///submap的物理坐标转到图像的像素坐标
    void SetCenter(const Vec2f& c);///物理中心

    cv::Mat GetOccupancyImg();

    /// 2D位置转网格
    Vec2i PosToGrid(const Vec2f &pos);
    Vec2i PosToGrid(const Vec2 &pos);

    std::shared_ptr<MLFrame> current_frame_ = nullptr;

    struct comp_vec2i{  //Function Object
        bool operator()(const Vec2i &v1, const Vec2i &v2) const{
            return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
        }
    };

    std::map<Vec2i, Idtype, comp_vec2i> grid_to_index_;  // 需要定义Vec2i排序方式,栅格和ID
    Vec2i last_grid_;//关键帧经过的上一个栅格坐标
    std::map<Idtype, std::shared_ptr<MLSubmap>> map_data_;//所有的SUBMAP 及其ID
    Vec2 origin_point_ = Vec2::Zero();  // 地图原点,全局地图物理原点，第一帧点云的坐标位置

    std::shared_ptr<MLSubmap> current_submap_ = nullptr;

    /// 地面相关
    double floor_detection_th = 0.1;

  double min_th_floor_ = 0.15;///距离地面这个高的障碍物讲被扫入栅格地图中
  double max_th_floor_ = 1.2;///距离地面这个高的障碍物讲被扫入栅格地图中
  double lidar_ext_z_ = 1.2;///距离地面这个高的障碍物讲被扫入栅格地图中



  /// 是否独立线程模式
    bool threaded_ = false;  //
    bool quit_;///析构，停止线程
    bool is_saving_map_;///析构，停止线程
    void ComputeGlobalImg();

    std::vector<Vec3> current_cloud_;   // 当前有效点云
    std::vector<Vec2f> current_black_;  // 当前配置的黑色
    std::vector<Vec2f> current_white_;  // 当前配置的白色

    Idtype submap_id_ = 0;
    const float submap_size_ = 100;           // 每张submap大小

    cv::Mat occupancy_;         // 占据栅格. uchar类型,一个submap 一个栅格地图
    cv::Mat height_map_;        // 每个格子的占据高度
    bool has_outside_points_ = false;
    int submap_logit_increase_ = 1;  // logit 更新量
    float resolution_ = 0.05;                 // 分辨率
    // 图像尺寸
    int image_width_ = 0;
    int image_height_ = 0;
    // 物理尺寸
    float metric_width_ = 0;
    float metric_height_ = 0;
    std::vector<Model2DPoint<float>> model_;  // 模板
    float model_size_ = 20.0;                 // 模板物理大小

    Vec4 floor_coeffs_ = Vec4::Zero();  // 地面方程参数

    std::mutex frame_mutex_;

    Vec2f global_center_;
    Vec2f center_ = Vec2f::Zero();        /// 图像物理中心,初始化本submap的第一帧点云的位置
    Vec2f center_image_;
    int width_;
    int height_;
    Vec2f top_left_;
    Vec2f bottom_right_;
};

} }  // namespace scrubber_slam::lidar16

#endif  // SCRUBBER_SLAM_OCCUPANCY_MAPPING_H

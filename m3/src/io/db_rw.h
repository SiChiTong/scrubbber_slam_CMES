//
// Created by pengguoqi on 19-7-17.
//

#ifndef DB_READ_WRITE_INCLUD_H__
#define DB_READ_WRITE_INCLUD_H__

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <map>
#include <vector>

#include "common/num_type.h"
#include "sqlite3.h"

using UniqueType = int;

namespace mapping {
namespace io {

class DBRW {
   public:
    DBRW();

    ~DBRW();

    /// 打开db
    bool OpenDB(const char *szfileName);

    bool IsOpen();

    /// 关闭db
    void CloseDB();

    /// 存储db文件的加速
    bool BeginTransaction();

    void RollbackTransaction();

    /// 结束db文件加速，与BeginTransaction成对出现
    bool CommitTransaction();

    /// 写入地图原点信息
    bool WriteMoveOriginPoint(double x, double y, double z, int zone, bool bEast = false);

    /// 读取地图原点信息
    bool ReadMoveOriginPoint(double &x, double &y, double &z, int &zone, bool &bEast);

    /// 写入功能点信息
    bool WriteFunctionPoint(UniqueType uniqueId, int type, int number, const V3d &pos, double heading);

    /// 读取功能点信息
    bool ReadFunctionPoint(UniqueType uniqueId, int &type, int &number, V3d &pos, double &heading);

    /// 写入scancontext初始化文件
    bool WriteScanContextBlock(int x, int y, const void *pData, int nSize);

    /// 读取scancontext初始化文件
    bool ReadScanContextBlock(int x, int y, char *&pData, int &length);

    /// 写入histigram初始化文件
    bool WriteHistogramBlock(int x, int y, const void *pData, int nSize);

    /// 读取histigram初始化文件
    bool ReadHistogramBlock(int x, int y, char *&pData, int &length);

    /// 写入激光点云数据
    bool WriteRawLaserData(UniqueType uniqueId, const SE3 &pose, const void *pData, int length, int version);

    // 仅修改位姿数据
    bool WritePose(UniqueType uniqueId, const SE3 &pose);

    /// 修改keyframe id
    bool UpdatePoseID(UniqueType old_id, UniqueType new_id);

    /// 读取激光点云数据
    bool ReadRawLaserData(UniqueType uniqueId, SE3 &pose, void *&pData, int &length, int &version);

    /// 删除激光点云数据
    bool DeleteRawLaserData(UniqueType uniqueId);

    /// 清空激光点云数据
    bool ClearRawLaserData();

    /// 写入优化后的点云位姿
    bool WriteOptimizePoseData(UniqueType uniqueId, const SE3 &pose);

    /// 读取优化后的点云位姿
    bool ReadOptimizePoseData(UniqueType uniqueId, SE3 &pose);

    /// 删除优化后的点云位姿
    bool DeleteOptimizePoseData(UniqueType uniqueId);

    /// 清空优化后的点云位姿
    bool ClearOptimizePoseData();

    /// 写入图像数据
    bool WriteImage(UniqueType uniqueId, const void *image, int length);

    /// 读取图像数据
    bool ReadImage(UniqueType uniqueId, char *&pData, int &length);

    /// 写入轨迹数据
    bool WriteTrack(UniqueType uniqueId, const void *track, int length);

    /// 读取轨迹数据
    bool ReadTrack(UniqueType uniqueId, char *&pData, int &length);

    /// 获取全部的点云数据
    bool QueryAllRawLaserPose(std::map<int, SE3> &mMapIdxPose);

    /// 获取全部的优化位姿
    bool QueryAllOptimizePose(std::map<int, SE3> &mMapIdxPose);

    /// 获取全部图像
    bool QueryAllTrackImage(std::vector<UniqueType> &mImageIdx);

    /// 获取全部轨迹
    bool QueryAllTrackLine(std::vector<UniqueType> &mLineIdx);

    /// 获取全部功能点
    bool QueryAllFunction(std::vector<UniqueType> &mIdxArray);

   private:
    bool Pack_PointCloud(const void *szData, int length, int version, char *&pPackBuf, int &nPackLenth);

    bool Unpack_PointCloud(char *szData, int length, int &version, void *&pPackBuf, int &nPackLenth);

    void FormatNowTime(char *szBuffer, int nbufSize);

   private:
    void *pDB;

    std::string db_path_;
};

}  // namespace io
}  // namespace mapping
#endif  // DB_READ_WRITE_INCLUD_H__
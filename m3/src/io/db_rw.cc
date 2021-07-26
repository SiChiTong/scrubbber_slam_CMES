//
// Created by pengguoqi on 19-7-17.
//

#include "io/db_rw.h"
#include <glog/logging.h>

namespace mapping::io {

struct BPC_HDR {
    unsigned int magic;
    unsigned short version;
    unsigned short reserve;
    unsigned int crc32;
};

DBRW::DBRW() : pDB(NULL) {}

DBRW::~DBRW() { CloseDB(); }

bool DBRW::Pack_PointCloud(const void *szData, int length, int version, char *&pPackBuf, int &nPackLenth) {
    if (version > 0) {
        BPC_HDR hdr;

        hdr.magic = 0x435042;
        hdr.version = 1;
        hdr.reserve = 0;
        hdr.crc32 = 0;

        int nNewLen = sizeof(BPC_HDR) + length;
        char *pMemBuf = new char[nNewLen];

        if (pMemBuf != NULL) {
            memcpy(pMemBuf, &hdr, sizeof(BPC_HDR));
            memcpy(pMemBuf + sizeof(BPC_HDR), szData, length);

            pPackBuf = pMemBuf;
            nPackLenth = nNewLen;

            return true;
        }
    }

    pPackBuf = (char *)szData;
    nPackLenth = length;

    return false;
}

bool DBRW::Unpack_PointCloud(char *szData, int length, int &version, void *&pPackBuf, int &nPackLenth) {
    if (length > sizeof(BPC_HDR)) {
        BPC_HDR hdr;
        memcpy(&hdr, szData, sizeof(BPC_HDR));

        if (hdr.magic == 0x435042 && hdr.version > 0) {
            int datLen = length - sizeof(BPC_HDR);
            char *dataPtr = szData + sizeof(BPC_HDR);

            memmove(szData, dataPtr, datLen);

            pPackBuf = szData;
            nPackLenth = datLen;
            version = hdr.version;
            return true;
        }
    }

    version = 0;
    pPackBuf = szData;
    nPackLenth = length;

    return true;
}

void DBRW::FormatNowTime(char *szBuffer, int nbufSize) {
    time_t timer = time(NULL);
    struct tm *nowTime = localtime(&timer);

    sprintf(szBuffer, "%04d-%02d-%02d %02d:%02d:%02d", nowTime->tm_year + 1900, nowTime->tm_mon + 1, nowTime->tm_mday,
            nowTime->tm_hour, nowTime->tm_min, nowTime->tm_sec);
}

bool DBRW::WriteHistogramBlock(int x, int y, const void *pData, int nSize) {
    const char *strSql = "insert or replace into loction_histogram(X, Y, data_block, date) values(?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// pose
    sqlite3_bind_double(stmt, 1, x);
    sqlite3_bind_double(stmt, 2, y);
    sqlite3_bind_blob(stmt, 3, pData, nSize, NULL);

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 4, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);

        return true;
    }
}

bool DBRW::ReadHistogramBlock(int x, int y, char *&pData, int &length) {
    const char *strSql = "select X,Y,data_block from loction_histogram where x = ? and y = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, x);
    sqlite3_bind_int(stmt, 2, y);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        /// x = sqlite3_column_int(stmt, 0);
        /// y = sqlite3_column_int(stmt, 1);

        const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 2);
        const int nBlobSize = sqlite3_column_bytes(stmt, 2);
        if (pBlobAddr != NULL && nBlobSize > 0) {
            pData = new char[nBlobSize];
            memcpy(pData, pBlobAddr, nBlobSize);

            length = nBlobSize;
        }

        sqlite3_finalize(stmt);

        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::WriteScanContextBlock(int x, int y, const void *pData, int nSize) {
    const char *strSql = "insert or replace into scan_context(X, Y, data_block, date) values(?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// pose
    sqlite3_bind_double(stmt, 1, x);
    sqlite3_bind_double(stmt, 2, y);
    sqlite3_bind_blob(stmt, 3, pData, nSize, NULL);

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 4, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);

        return true;
    }
}

bool DBRW::ReadScanContextBlock(int x, int y, char *&pData, int &length) {
    const char *strSql = "select X,Y,data_block from scan_context where x = ? and y = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    // id
    sqlite3_bind_int(stmt, 1, x);
    sqlite3_bind_int(stmt, 2, y);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 2);
        const int nBlobSize = sqlite3_column_bytes(stmt, 2);
        if (pBlobAddr != NULL && nBlobSize > 0) {
            pData = new char[nBlobSize];
            memcpy(pData, pBlobAddr, nBlobSize);

            length = nBlobSize;
        }

        sqlite3_finalize(stmt);

        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::QueryAllOptimizePose(std::map<int, SE3> &mMapIdxPose) {
    const char *strSql = "select uniqueId, X,Y,Z,quatX,quatY,quatZ,quatW from optimize_point_cloud;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// read record
    do {
        int nRes = sqlite3_step(stmt);
        if (nRes == SQLITE_ROW) {
            SE3 pose;

            UniqueType uniqueId = sqlite3_column_int(stmt, 0);

            pose.translation()[0] = sqlite3_column_double(stmt, 1);
            pose.translation()[1] = sqlite3_column_double(stmt, 2);
            pose.translation()[2] = sqlite3_column_double(stmt, 3);

            double x = sqlite3_column_double(stmt, 4);
            double y = sqlite3_column_double(stmt, 5);
            double z = sqlite3_column_double(stmt, 6);
            double w = sqlite3_column_double(stmt, 7);
            pose.setQuaternion(Quatd(w, x, y, z));

            mMapIdxPose.insert(std::make_pair(uniqueId, pose));
        } else if (nRes == SQLITE_DONE) {
            break;
        } else {
            LOG(INFO) << "read sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
            break;
        }
    } while (true);

    sqlite3_finalize(stmt);

    return true;
}

bool DBRW::QueryAllRawLaserPose(std::map<int, SE3> &mMapIdxPose) {
    const char *strSql = "select uniqueId, X,Y,Z,quatX,quatY,quatZ,quatW from raw_point_cloud;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// read record
    do {
        int nRes = sqlite3_step(stmt);
        if (nRes == SQLITE_ROW) {
            SE3 pose;

            UniqueType uniqueId = sqlite3_column_int(stmt, 0);

            pose.translation()[0] = sqlite3_column_double(stmt, 1);
            pose.translation()[1] = sqlite3_column_double(stmt, 2);
            pose.translation()[2] = sqlite3_column_double(stmt, 3);

            double x = sqlite3_column_double(stmt, 4);
            double y = sqlite3_column_double(stmt, 5);
            double z = sqlite3_column_double(stmt, 6);
            double w = sqlite3_column_double(stmt, 7);
            pose.setQuaternion(Quatd(w, x, y, z));

            mMapIdxPose.insert(std::make_pair(uniqueId, pose));
        } else if (nRes == SQLITE_DONE) {
            break;
        } else {
            LOG(INFO) << "read sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
            break;
        }
    } while (true);

    sqlite3_finalize(stmt);

    return true;
}

bool DBRW::WriteFunctionPoint(UniqueType uniqueId, int type, int number, const V3d &pos, double heading) {
    const char *strSql =
        "insert or replace into function_point(uniqueId, type, number, X, Y, Z, heading, date) values(?,?,?,?,?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);
    sqlite3_bind_int(stmt, 2, type);
    sqlite3_bind_int(stmt, 3, number);

    /// pose
    sqlite3_bind_double(stmt, 4, pos[0]);
    sqlite3_bind_double(stmt, 5, pos[1]);
    sqlite3_bind_double(stmt, 6, pos[2]);
    sqlite3_bind_double(stmt, 7, heading);

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 8, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);

        return true;
    }
}

bool DBRW::ReadFunctionPoint(UniqueType uniqueId, int &type, int &number, V3d &pos, double &heading) {
    const char *strSql = "select type, number, X, Y, Z, heading from function_point where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    sqlite3_bind_int(stmt, 1, uniqueId);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        type = sqlite3_column_int(stmt, 0);
        number = sqlite3_column_int(stmt, 1);

        pos[0] = sqlite3_column_double(stmt, 2);
        pos[1] = sqlite3_column_double(stmt, 3);
        pos[2] = sqlite3_column_double(stmt, 4);

        heading = sqlite3_column_double(stmt, 5);

        sqlite3_finalize(stmt);

        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::ReadMoveOriginPoint(double &x, double &y, double &z, int &zone, bool &bEast) {
    const char *strSql = "select X,Y,Z,zone,east from point_table where uniqueId = 1;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        x = sqlite3_column_double(stmt, 0);
        y = sqlite3_column_double(stmt, 1);
        z = sqlite3_column_double(stmt, 2);

        zone = sqlite3_column_int(stmt, 3);
        bEast = sqlite3_column_int(stmt, 4);

        sqlite3_finalize(stmt);
        return true;
    } else {
        sqlite3_finalize(stmt);
        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::WriteMoveOriginPoint(double x, double y, double z, int zone, bool bEast) {
    const char *strSql =
        "insert or replace into point_table(uniqueId, X, Y, Z, zone, east, date) values(?,?,?,?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, 1);

    /// pose
    sqlite3_bind_double(stmt, 2, x);
    sqlite3_bind_double(stmt, 3, y);
    sqlite3_bind_double(stmt, 4, z);

    sqlite3_bind_double(stmt, 5, zone);
    sqlite3_bind_double(stmt, 6, bEast);

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 7, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);
        return true;
    }
}

bool DBRW::ClearRawLaserData() {
    const char *strSql = "delete from raw_point_cloud";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_DONE) {
        sqlite3_finalize(stmt);
        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "clear raw laser data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::DeleteRawLaserData(UniqueType uniqueId) {
    const char *strSql = "delete from raw_point_cloud where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    sqlite3_bind_int(stmt, 1, uniqueId);

    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_DONE) {
        sqlite3_finalize(stmt);
        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "delete  raw laser data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::ReadRawLaserData(UniqueType uniqueId, SE3 &pose, void *&pPackBuf, int &nPackLenth, int &version) {
    const char *strSql = "select X,Y,Z,quatX,quatY,quatZ,quatW,pcd from raw_point_cloud where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        pose.translation()[0] = sqlite3_column_double(stmt, 0);
        pose.translation()[1] = sqlite3_column_double(stmt, 1);
        pose.translation()[2] = sqlite3_column_double(stmt, 2);

        double x = sqlite3_column_double(stmt, 3);
        double y = sqlite3_column_double(stmt, 4);
        double z = sqlite3_column_double(stmt, 5);
        double w = sqlite3_column_double(stmt, 6);
        pose.setQuaternion(Quat(w, x, y, z));

        const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 7);
        const int nBlobSize = sqlite3_column_bytes(stmt, 7);
        if (pBlobAddr != NULL && nBlobSize > 0) {
            char *pData = new char[nBlobSize];
            memcpy(pData, pBlobAddr, nBlobSize);

            if (!Unpack_PointCloud(pData, nBlobSize, version, pPackBuf, nPackLenth)) {
                pPackBuf = pData;
                nPackLenth = nBlobSize;
            }
        }

        sqlite3_finalize(stmt);
        return true;
    } else {
        sqlite3_finalize(stmt);
        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        LOG(INFO) << "kfid: " << uniqueId << ", db path: " << db_path_;
        return false;
    }
}

bool DBRW::WriteRawLaserData(UniqueType uniqueId, const SE3 &pose, const void *pData, int length, int version) {
    const char *strSql =
        "insert or replace into raw_point_cloud(uniqueId,X,Y,Z,quatX,quatY,quatZ,quatW,pcd,date) "
        "values(?,?,?,?,?,?,?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    char *pPackBuf = NULL;
    int nPackLenth = 0;
    bool bRet = Pack_PointCloud(pData, length, version, pPackBuf, nPackLenth);

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// pose
    sqlite3_bind_double(stmt, 2, pose.translation()[0]);
    sqlite3_bind_double(stmt, 3, pose.translation()[1]);
    sqlite3_bind_double(stmt, 4, pose.translation()[2]);
    sqlite3_bind_double(stmt, 5, pose.unit_quaternion().x());
    sqlite3_bind_double(stmt, 6, pose.unit_quaternion().y());
    sqlite3_bind_double(stmt, 7, pose.unit_quaternion().z());
    sqlite3_bind_double(stmt, 8, pose.unit_quaternion().w());

    /// pcd data
    if (pPackBuf != NULL && nPackLenth > 0) {
        sqlite3_bind_blob(stmt, 9, pPackBuf, nPackLenth, NULL);
    }

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 10, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        if (bRet) {
            delete[] pPackBuf;
        }

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);

        if (bRet) {
            delete[] pPackBuf;
        }

        return true;
    }
}

bool DBRW::WritePose(UniqueType uniqueId, const SE3 &pose) {
    const char *strSql =
        "update raw_point_cloud set X=?, Y=?, Z=?, quatX=?, quatY=?, quatZ=?, quatW=? where uniqueId =?";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// pose
    sqlite3_bind_double(stmt, 1, pose.translation()[0]);
    sqlite3_bind_double(stmt, 2, pose.translation()[1]);
    sqlite3_bind_double(stmt, 3, pose.translation()[2]);
    sqlite3_bind_double(stmt, 4, pose.unit_quaternion().x());
    sqlite3_bind_double(stmt, 5, pose.unit_quaternion().y());
    sqlite3_bind_double(stmt, 6, pose.unit_quaternion().z());
    sqlite3_bind_double(stmt, 7, pose.unit_quaternion().w());
    /// id
    sqlite3_bind_int(stmt, 8, uniqueId);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);
        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);
        return true;
    }
}

bool DBRW::UpdatePoseID(UniqueType old_id, UniqueType new_id) {
    const char *strSql = "update raw_point_cloud set uniqueId =? where uniqueId =?";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, new_id);
    sqlite3_bind_int(stmt, 2, old_id);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);
        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);
        return true;
    }
}

bool DBRW::DeleteOptimizePoseData(UniqueType uniqueId) {
    const char *strSql = "delete from optimize_point_cloud where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    sqlite3_bind_int(stmt, 1, uniqueId);

    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_DONE) {
        sqlite3_finalize(stmt);
        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "delete optimize pose data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::ClearOptimizePoseData() {
    const char *strSql = "delete from optimize_point_cloud";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_DONE) {
        sqlite3_finalize(stmt);
        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "clear optimize pose data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::ReadOptimizePoseData(UniqueType uniqueId, SE3 &pose) {
    const char *strSql = "select X,Y,Z,quatX,quatY,quatZ,quatW from optimize_point_cloud where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        pose.translation()[0] = sqlite3_column_double(stmt, 0);
        pose.translation()[1] = sqlite3_column_double(stmt, 1);
        pose.translation()[2] = sqlite3_column_double(stmt, 2);

        double x = sqlite3_column_double(stmt, 3);
        double y = sqlite3_column_double(stmt, 4);
        double z = sqlite3_column_double(stmt, 5);
        double w = sqlite3_column_double(stmt, 6);
        pose.setQuaternion(Quat(w, x, y, z));

        sqlite3_finalize(stmt);

        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::WriteOptimizePoseData(UniqueType uniqueId, const SE3 &pose) {
    const char *strSql =
        "insert or replace into optimize_point_cloud(uniqueId,X,Y,Z,quatX,quatY,quatZ,quatW,date) "
        "values(?,?,?,?,?,?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// pose
    sqlite3_bind_double(stmt, 2, pose.translation()[0]);
    sqlite3_bind_double(stmt, 3, pose.translation()[1]);
    sqlite3_bind_double(stmt, 4, pose.translation()[2]);
    sqlite3_bind_double(stmt, 5, pose.unit_quaternion().x());
    sqlite3_bind_double(stmt, 6, pose.unit_quaternion().y());
    sqlite3_bind_double(stmt, 7, pose.unit_quaternion().z());
    sqlite3_bind_double(stmt, 8, pose.unit_quaternion().w());

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 9, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);

        return true;
    }
}

bool DBRW::WriteImage(UniqueType uniqueId, const void *image, int length) {
    assert(image != nullptr);
    const char *strSql = "insert or replace into track_image(uniqueId, info, image, date) values(?,?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// info
    sqlite3_bind_text(stmt, 2, "jpeg", -1, NULL);

    /// image
    sqlite3_bind_blob(stmt, 3, image, length, NULL);

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 4, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);
        LOG(WARNING) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);
        return true;
    }
}

bool DBRW::ReadImage(UniqueType uniqueId, char *&pData, int &length) {
    const char *strSql = "select image from track_image where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 0);
        const int nBlobSize = sqlite3_column_bytes(stmt, 0);
        if (pBlobAddr != NULL && nBlobSize > 0) {
            pData = new char[nBlobSize];
            memcpy(pData, pBlobAddr, nBlobSize);

            length = nBlobSize;
        }

        sqlite3_finalize(stmt);

        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::QueryAllTrackImage(std::vector<UniqueType> &mImageIdx) {
    const char *strSql = "select uniqueId from track_image;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// read record
    do {
        int nRes = sqlite3_step(stmt);
        if (nRes == SQLITE_ROW) {
            UniqueType uniqueId = sqlite3_column_int(stmt, 0);
            mImageIdx.push_back(uniqueId);

        } else if (nRes == SQLITE_DONE) {
            break;
        } else {
            LOG(INFO) << "read sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
            break;
        }
    } while (true);

    sqlite3_finalize(stmt);

    return true;
}

bool DBRW::QueryAllFunction(std::vector<UniqueType> &mIdxArray) {
    const char *strSql = "select uniqueId from function_point;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// read record
    do {
        int nRes = sqlite3_step(stmt);
        if (nRes == SQLITE_ROW) {
            UniqueType uniqueId = sqlite3_column_int(stmt, 0);
            mIdxArray.push_back(uniqueId);

        } else if (nRes == SQLITE_DONE) {
            break;
        } else {
            LOG(INFO) << "read sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
            break;
        }
    } while (true);

    sqlite3_finalize(stmt);

    return true;
}

bool DBRW::QueryAllTrackLine(std::vector<UniqueType> &mLineIdx) {
    const char *strSql = "select uniqueId from track_line;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// read record
    do {
        int nRes = sqlite3_step(stmt);
        if (nRes == SQLITE_ROW) {
            UniqueType uniqueId = sqlite3_column_int(stmt, 0);
            mLineIdx.push_back(uniqueId);

        } else if (nRes == SQLITE_DONE) {
            break;
        } else {
            LOG(INFO) << "read sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
            break;
        }
    } while (true);

    sqlite3_finalize(stmt);

    return true;
}

bool DBRW::ReadTrack(UniqueType uniqueId, char *&pData, int &length) {
    const char *strSql = "select track from track_line where uniqueId = ?;";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// read record
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
        const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 0);
        const int nBlobSize = sqlite3_column_bytes(stmt, 0);
        if (pBlobAddr != NULL && nBlobSize > 0) {
            pData = new char[nBlobSize];
            memcpy(pData, pBlobAddr, nBlobSize);

            length = nBlobSize;
        }

        sqlite3_finalize(stmt);

        return true;
    } else {
        sqlite3_finalize(stmt);

        LOG(INFO) << "read data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }
}

bool DBRW::WriteTrack(UniqueType uniqueId, const void *track, int length) {
    const char *strSql = "insert or replace into track_line(uniqueId, track, date) values(?,?,?)";

    sqlite3_stmt *stmt;
    int nStatus = sqlite3_prepare_v2((sqlite3 *)pDB, strSql, -1, &stmt, NULL);
    if (nStatus != SQLITE_OK) {
        LOG(INFO) << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    }

    /// id
    sqlite3_bind_int(stmt, 1, uniqueId);

    /// track
    sqlite3_bind_blob(stmt, 2, track, length, NULL);

    /// now time
    char szBuffer[128];
    FormatNowTime(szBuffer, 128);
    sqlite3_bind_text(stmt, 3, szBuffer, -1, NULL);

    /// save record
    int nRes = sqlite3_step(stmt);
    if (nRes != SQLITE_DONE) {
        sqlite3_finalize(stmt);

        LOG(INFO) << "insert data fail: " << sqlite3_errmsg((sqlite3 *)pDB) << std::endl;
        return false;
    } else {
        sqlite3_finalize(stmt);

        return true;
    }
}

bool DBRW::BeginTransaction() {
    char *cErrMsg;
    int ret = sqlite3_exec((sqlite3 *)pDB, "BEGIN", NULL, NULL, &cErrMsg);
    if (ret != SQLITE_OK) {
        LOG(INFO) << "begin transaction  fail: " << cErrMsg << std::endl;
        sqlite3_free(cErrMsg);
        return false;
    }
    return true;
}

void DBRW::RollbackTransaction() {
    char *cErrMsg;
    int ret = sqlite3_exec((sqlite3 *)pDB, "ROLLBACK", NULL, NULL, &cErrMsg);
    if (ret != SQLITE_OK) {
        LOG(INFO) << "rollback transaction  fail: " << cErrMsg << std::endl;
        sqlite3_free(cErrMsg);
    }
}

bool DBRW::CommitTransaction() {
    char *cErrMsg;
    int ret = sqlite3_exec((sqlite3 *)pDB, "COMMIT", NULL, NULL, &cErrMsg);
    if (ret != SQLITE_OK) {
        LOG(INFO) << "commit transaction  fail: " << cErrMsg << std::endl;
        sqlite3_free(cErrMsg);
        return false;
    }
    return true;
}

bool DBRW::IsOpen() { return (pDB != NULL); }

void DBRW::CloseDB() {
    if (pDB != NULL) {
        while (SQLITE_OK != sqlite3_close((sqlite3 *)pDB)) {
            /// 必须确实将DB关掉
            usleep(10000);
        }
        pDB = NULL;
    }
}

bool DBRW::OpenDB(const char *szfileName) {
    if (pDB) {
        std::cerr << "database of " << szfileName << " is already open";
        return true;
    }

    const char *sqltext =
        "create table if not exists raw_point_cloud( \
							    uniqueId integer primary KEY, X numeric, Y numeric, Z numeric, \
								quatX numeric, quatY numeric, quatZ numeric, quatW numeric, \
								pcd blob,                                         \
								date text not null)";

    const char *sqloptimize =
        "create table if not exists optimize_point_cloud( \
							  		uniqueId integer primary KEY, X numeric, Y numeric, Z numeric, \
									quatX numeric, quatY numeric, quatZ numeric, quatW numeric, \
									date text not null)";

    const char *sqlpoint =
        "create table if not exists point_table(uniqueId integer primary KEY, X numeric, Y numeric, Z numeric, \
							      zone integer, east integer, date text not null)";

    const char *sqlhistogram =
        "create table if not exists loction_histogram(X integer not null, Y integer  not null, data_block blob, date "
        "text not null,primary key (X,Y))";

    const char *sqlscancontext =
        "create table if not exists scan_context(X integer not null, Y integer  not null, data_block blob, date text "
        "not null,primary key (X,Y))";

    const char *imagelist =
        "create table if not exists track_image(uniqueId integer primary key, info text,  image blob not null, date "
        "text not null)";

    const char *tracklist =
        "create table if not exists track_line(uniqueId integer primary key,  track blob not null, date text not null)";

    const char *sqlfunction =
        "create table if not exists function_point(uniqueId integer primary key, type integer, number integer, X "
        "numeric, Y numeric, Z numeric, heading numeric, date text)";

    sqlite3 *sqlitedb = NULL;
    int nRes = sqlite3_open(szfileName, &sqlitedb);
    if (nRes != SQLITE_OK) {
        LOG(INFO) << "Open database fail: " << sqlite3_errmsg(sqlitedb) << " " << szfileName;
        return false;
    } else {
        pDB = sqlitedb;
    }

    char *cErrMsg;

    int nRet1 = sqlite3_exec((sqlite3 *)pDB, sqltext, 0, 0, &cErrMsg);
    int nRet2 = sqlite3_exec((sqlite3 *)pDB, sqlpoint, 0, 0, &cErrMsg);
    int nRet3 = sqlite3_exec((sqlite3 *)pDB, sqloptimize, 0, 0, &cErrMsg);
    int nRet4 = sqlite3_exec((sqlite3 *)pDB, sqlhistogram, 0, 0, &cErrMsg);
    int nRet5 = sqlite3_exec((sqlite3 *)pDB, imagelist, 0, 0, &cErrMsg);
    int nRet6 = sqlite3_exec((sqlite3 *)pDB, tracklist, 0, 0, &cErrMsg);
    int nRet7 = sqlite3_exec((sqlite3 *)pDB, sqlscancontext, 0, 0, &cErrMsg);
    int nRet8 = sqlite3_exec((sqlite3 *)pDB, sqlfunction, 0, 0, &cErrMsg);

    if (nRet1 != SQLITE_OK || nRet2 != SQLITE_OK || nRet3 != SQLITE_OK || nRet4 != SQLITE_OK || nRet5 != SQLITE_OK ||
        nRet6 != SQLITE_OK || nRet7 != SQLITE_OK || nRet8 != SQLITE_OK) {
        sqlite3_close((sqlite3 *)pDB);

        LOG(INFO) << "create table fail: " << cErrMsg << std::endl;
        return false;
    }

    db_path_ = szfileName;

    return true;
}
}  // namespace mapping::io

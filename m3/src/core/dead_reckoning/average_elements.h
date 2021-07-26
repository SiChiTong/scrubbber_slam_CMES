//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_AVERAGE_ELEMENTS_H
#define MAPPING_AVERAGE_ELEMENTS_H

#include "common/num_type.h"
#include "core/dead_reckoning/dr_constants.h"

namespace mapping::core {

struct AverageElements {
    int cnt = 0;
    int Status = 0;
    double Time = 0;
    double maxTime = kStaticTime;
    V3d src_1 = V3d::Zero();
    V3d src_2 = V3d::Zero();
    V3d avg_tmp_1 = V3d::Zero();
    V3d avg_tmp_2 = V3d::Zero();
    V3d avg1 = V3d::Zero();
    V3d avg2 = V3d::Zero();

    void InitAvg(double mTime) {
        cnt = 0;
        Status = 0;
        Time = 0.0;
        maxTime = mTime;
        src_1.setZero();
        src_2.setZero();
        avg_tmp_1.setZero();
        avg_tmp_2.setZero();
        avg1.setZero();
        avg2.setZero();
    }

    int GetAvg(const V3d& src1, const V3d& src2, double tau) {
        cnt++;
        Time += tau;

        if (1 == cnt) {
            src_1 = src1;
            src_2 = src2;
        }

        avg_tmp_1 += src1;
        avg_tmp_2 += src2;

        if (Time >= maxTime) {
            src_1.setZero();
            src_2.setZero();
            avg1 = avg_tmp_1 / cnt;
            avg2 = avg_tmp_2 / cnt;
            cnt = 0;
            Time = 0;
            avg_tmp_1.setZero();
            avg_tmp_2.setZero();
            if (Status != 2) {
                Status = 1;
            }
        }
        return Status;
    }
};

}  // namespace mapping::core

#endif  // MAPPING_AVERAGE_ELEMENTS_H

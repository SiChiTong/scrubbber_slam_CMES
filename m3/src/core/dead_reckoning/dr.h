//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_DR_H
#define MAPPING_DR_H

#include "common/message_def.h"
#include "common/num_type.h"
#include "core/dead_reckoning/dr_elements.h"
#include "core/dead_reckoning/dr_params.h"
#include "core/dead_reckoning/end_dr_elements.h"

namespace mapping::core {

struct DRImpl;

class DeadReckoning {
   public:
    explicit DeadReckoning();
    explicit DeadReckoning(const DrParams &dr_params);

    bool SetImuElements(const ImuMsg &msg);

    bool SetOdomElements(const OdomMsg &msg);

    void SetPosition(V3d position);

    void SetAzimuth(double azimuth);

    void SetInitalFlag();

    void SetEndDrElements(const EndDrElements &end_dr_elements) { UpdateEndDrElements(end_dr_elements); }

    void ApplyDeadReckoning();

    DrElements GetDRResult();

    inline EndDrElements GetEndDrElements() { return OutputEndDrElements(); }

    void Initialization();

   private:
    void InitialSystem();

    void SetDrElements();

    void CacheImuData(const ImuElements &msg);

    void QuaternionUpdate(const V3d &omega_i_b, const V3d &vel, double tau);

    void VelocityUpdate(V3d &vel, const V3d& fibb, double tau);

    void PositionUpdate(V3d &pos, const V3d& vel, double tau);

    void Predict(const V3d &fibb, double tau);

    void Correct(V3d innov, V3d &vel);

    void QuatCorrect(const V3d &attError);

    size_t GetSuitableIndex(double timetic);

    void CacheDrData(const DrElements &msg);

    void UpdateEndDrElements(const EndDrElements &end_dr_elements);

    EndDrElements OutputEndDrElements();

    void Update(V3d ang);

    bool StaticCheck(double x, double y);

   private:
    std::shared_ptr<DRImpl> impl_;
};

}  // namespace mapping::core

#endif  // MAPPING_DR_H

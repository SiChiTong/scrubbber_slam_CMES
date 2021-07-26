//
// Created by gaoxiang on 2019/12/16.
//

#ifndef HAMO_LINUX_OBJECT_H
#define HAMO_LINUX_OBJECT_H

namespace HAMO {
class Object {
   public:
    enum class DataVariance { DYNAMIC, STATIC, UNSPECIFIED };

   public:
    Object();

    virtual ~Object();

    DataVariance GetDataVariance() const { return data_variance_; }

   private:
    DataVariance data_variance_;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_OBJECT_H

//
// Created by gaoxiang on 2019/12/12.
//

#ifndef HAMO_LINUX_CONSTANT_H
#define HAMO_LINUX_CONSTANT_H

namespace HAMO {

template <typename Ty>
class Constant {
   public:
    static const Ty MAX_REAL;
    static const Ty ZERO_TOLERANCE;
};

//------------------------------------------------------------------------------
typedef Constant<float> Constantf;
typedef Constant<double> Constantd;

//------------------------------------------------------------------------------

}  // namespace HAMO

#endif  // HAMO_LINUX_CONSTANT_H

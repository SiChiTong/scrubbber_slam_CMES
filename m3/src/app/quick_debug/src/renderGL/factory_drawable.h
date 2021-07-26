//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_FACTORY_DRAWABLE_H
#define HAMO_LINUX_FACTORY_DRAWABLE_H

#include "drawable.h"
#include "point3d.h"
#include "renderGL/point_color.h"

namespace HAMO {

class FactoryDrawable {
   public:
    static Drawable* CreateNodeDrawable(const std::vector<PointColor>& items, float size, const V4f& clr);
};

}  // namespace HAMO
#endif  // HAMO_LINUX_FACTORY_DRAWABLE_H

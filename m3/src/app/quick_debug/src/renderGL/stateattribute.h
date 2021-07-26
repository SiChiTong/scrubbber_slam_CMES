//
// Created by hqw on.
//

#ifndef HAMO_LINUX_STATEATTRIBUTE_H
#define HAMO_LINUX_STATEATTRIBUTE_H

#include "renderGL/object.h"

namespace HAMO {

class StateAttribute : public Object {
   public:
    /** GLMode is the value used in glEnable/glDisable(mode) */
    typedef unsigned int GLMode;
    /** GLModeValue is used to specify whether a mode is enabled (ON) or
     * disabled (OFF). GLMoveValue is also used to specify the override behavior
     * of modes from parent to children. See enum Value description for more
     * details.*/
    typedef unsigned int GLModeValue;
    enum Values {
        /** means that associated GLMode and Override is disabled.*/
        OFF = 0x0,
        /** means that associated GLMode is enabled and Override is disabled.*/
        ON = 0x1,
        /** Overriding of GLMode's or StateAttributes is enabled, so that state
           below it is overridden.*/
        OVERRIDE = 0x2,
        /** Protecting of GLMode's or StateAttributes is enabled, so that state
           from above cannot override this and below state.*/
        PROTECTED = 0x4,
        /** means that GLMode or StateAttribute should be inherited from
           above.*/
        INHERIT = 0x8
    };

    enum Type { POINT, LINEWIDTH, LINESTIPPLE };

    StateAttribute() {}
};
}  // namespace HAMO

#endif  // HAMO_LINUX_STATEATTRIBUTE_H

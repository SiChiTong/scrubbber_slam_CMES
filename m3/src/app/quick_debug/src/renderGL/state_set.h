//
// Created by hqw on 2020/07/02.
//

#ifndef HAMO_LINUX_STATE_SET_H
#define HAMO_LINUX_STATE_SET_H

#include <map>
#include "renderGL/stateattribute.h"

namespace HAMO {

class StateSet : public Object {
    typedef std::map<StateAttribute::GLMode, StateAttribute::GLModeValue>
        ModeList;

   public:
    void setMode(StateAttribute::GLMode mode,
                 StateAttribute::GLModeValue value);

    // void removeMode(StateAttribute::GLMode mode);

    StateAttribute::GLModeValue getMode(StateAttribute::GLMode mode) const;

    //  inline void setModeList(ModeList& ml) { m_modeList=ml; }

    // inline ModeList& getModeList() { return m_modeList; }
   private:
    std::map<StateAttribute::GLMode, StateAttribute::GLModeValue> m_modeList;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_STATE_SET_H

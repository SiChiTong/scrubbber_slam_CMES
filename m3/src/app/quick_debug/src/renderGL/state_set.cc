//
// Created by hqw on 2020/07/02.
//

#include "renderGL/state_set.h"

namespace HAMO {

void StateSet::setMode(HAMO::StateAttribute::GLMode mode,
                       HAMO::StateAttribute::GLModeValue value) {
    m_modeList[mode] = value;
}

StateAttribute::GLModeValue StateSet::getMode(
    StateAttribute::GLMode mode) const {
    ModeList::const_iterator itr = m_modeList.find(mode);
    if (itr != m_modeList.end()) {
        return itr->second;
    } else {
        return StateAttribute::INHERIT;
    }
}

}  // namespace HAMO

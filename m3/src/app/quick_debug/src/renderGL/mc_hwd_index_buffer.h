//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_HWD_INDEX_BUFFER_H
#define HAMO_LINUX_MC_HWD_INDEX_BUFFER_H

#include "renderGL/mc_buffer.h"

namespace HAMO {

class HwdIndexBuffer {
public:
    HwdIndexBuffer();

    ~HwdIndexBuffer();

    void Create(const IndexBuffer *indexbuf);

    void Destroy();

    void Enable();

    void Disable();

    void *Lock(Buffer::LockMode mode);

    void Unlock();

    bool IsValidate() const;

    unsigned int GetHandle() const;

private:
    unsigned int handle_;
};

}

#endif //HAMO_LINUX_MC_HWD_INDEX_BUFFER_H

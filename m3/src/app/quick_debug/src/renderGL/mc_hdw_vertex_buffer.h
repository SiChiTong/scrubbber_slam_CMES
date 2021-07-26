//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_HDW_VERTEX_BUFFER_H
#define HAMO_LINUX_MC_HDW_VERTEX_BUFFER_H

#include "renderGL/mc_buffer.h"

namespace HAMO {

class HdwVertexBuffer {
   public:
    HdwVertexBuffer();

    virtual ~HdwVertexBuffer();

   public:
    bool Create(const VertexBuffer *vertex);

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

}  // namespace HAMO

#endif  // HAMO_LINUX_MC_HDW_VERTEX_BUFFER_H

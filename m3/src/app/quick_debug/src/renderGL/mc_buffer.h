//
// Created by gaoxiang on 2019/12/16.
//

#ifndef HAMO_LINUX_MC_BUFFER_H
#define HAMO_LINUX_MC_BUFFER_H

#include "renderGL/gl_api.h"

namespace HAMO {

class Buffer {
   public:
    enum Usage {
        BU_UNKNOWN,
        BU_STATIC = GL_STATIC_DRAW,
        BU_DYNAMIC = GL_DYNAMIC_DRAW,
    };

    enum LockMode {
        BL_READ_ONLY = GL_READ_ONLY,
        BL_WRITE_ONLY = GL_WRITE_ONLY,
        BL_READ_WRITE = GL_READ_WRITE,
    };

   public:
    Buffer();

    Buffer(int numVertices, int vertexSize, Usage usage);

    virtual ~Buffer();

   public:
    int GetNumElements() const;

    int GetElementSize() const;

    Usage GetUsage() const;

    int GetNumBytes() const;

    char *GetData() const;

   protected:
    int num_elements_;
    int element_size_;
    Usage usage_;
    int num_bytes_;
    char *pdata_;
};

//===========================================================
class VertexBuffer : public Buffer {
   public:
    VertexBuffer(int numVertices, int vertexSize,
                 Usage usage = Usage::BU_STATIC);

    virtual ~VertexBuffer();
};

//=======================================================================
class IndexBuffer : public Buffer {
   public:
    IndexBuffer(int numIndices, int indexSize, Usage usage = Usage::BU_STATIC);

    virtual ~IndexBuffer();

   public:
    void SetOffset(int offset);

    int GetOffset() const;

   protected:
    int offset_;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_MC_BUFFER_H

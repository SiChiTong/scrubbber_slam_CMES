//
// Created by gaoxiang on 2019/12/17.
//

#include <memory.h>
#include "renderGL/mc_hdw_vertex_buffer.h"
#include "renderGL/gl_api.h"

namespace HAMO {

HdwVertexBuffer::HdwVertexBuffer()
        : handle_(0) {

}

HdwVertexBuffer::~HdwVertexBuffer() {

}

bool HdwVertexBuffer::Create(const VertexBuffer *vertex) {
    glGenBuffers(1, &handle_);
    glBindBuffer(GL_ARRAY_BUFFER, handle_);

    glBufferData(GL_ARRAY_BUFFER, vertex->GetNumBytes(), 0, static_cast<unsigned int>(vertex->GetUsage()));

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    void *data = Lock(Buffer::LockMode::BL_WRITE_ONLY);

    memcpy(data, vertex->GetData(), vertex->GetNumBytes());

    Unlock();

    return true;
}

void HdwVertexBuffer::Destroy() {
    if (handle_ != 0) {
        glDeleteBuffers(1, &handle_);
        handle_ = 0;
    }
}

void HdwVertexBuffer::Enable() {
    glBindBuffer(GL_ARRAY_BUFFER, handle_);
}

void HdwVertexBuffer::Disable() {
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void *HdwVertexBuffer::Lock(Buffer::LockMode mode) {
    glBindBuffer(GL_ARRAY_BUFFER, handle_);

    void *videoMemory = glMapBuffer(GL_ARRAY_BUFFER, static_cast<unsigned int>(mode));

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return videoMemory;
}

void HdwVertexBuffer::Unlock() {
    glBindBuffer(GL_ARRAY_BUFFER, handle_);
    glUnmapBuffer(GL_ARRAY_BUFFER);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

bool HdwVertexBuffer::IsValidate() const {
    return (handle_ != 0);
}

unsigned int HdwVertexBuffer::GetHandle() const {
    return handle_;
}

}

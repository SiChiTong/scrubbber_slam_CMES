//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_hwd_index_buffer.h"
#include "renderGL/gl_api.h"

#include <memory.h>

namespace HAMO {

HwdIndexBuffer::HwdIndexBuffer() : handle_(0) {

}

HwdIndexBuffer::~HwdIndexBuffer() {

}

void HwdIndexBuffer::Create(const IndexBuffer *ibuffer) {
    glGenBuffers(1, &handle_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ibuffer->GetNumBytes(), 0, static_cast<unsigned int>(ibuffer->GetUsage()));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    void *data = Lock(Buffer::LockMode::BL_WRITE_ONLY);
    memcpy(data, ibuffer->GetData(), ibuffer->GetNumBytes());
    Unlock();
}

void HwdIndexBuffer::Destroy() {
    if (handle_ != 0) {
        glDeleteBuffers(1, &handle_);
        handle_ = 0;
    }
}

void HwdIndexBuffer::Enable() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle_);
}

void HwdIndexBuffer::Disable() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void *HwdIndexBuffer::Lock(Buffer::LockMode mode) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle_);
    void *videoMemory = glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, static_cast<unsigned int>(mode));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    return videoMemory;
}

void HwdIndexBuffer::Unlock() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, handle_);

    glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool HwdIndexBuffer::IsValidate() const {
    return (handle_ != 0);
}

unsigned int HwdIndexBuffer::GetHandle() const {
    return handle_;
}

}

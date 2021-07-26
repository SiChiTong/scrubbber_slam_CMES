//
// Created by gaoxiang on 2019/12/17.
//

#include <assert.h>
#include "renderGL/mc_buffer.h"

namespace HAMO {

Buffer::Buffer()
        : num_elements_(0), element_size_(0),
          usage_(Usage::BU_UNKNOWN), num_bytes_(0),
          pdata_(NULL) {

}

Buffer::Buffer(int numElements, int elementSize, Usage usage)
        : num_elements_(numElements), element_size_(elementSize),
          usage_(usage), num_bytes_(numElements * elementSize) {
    assert(num_elements_ > 0);
    assert(element_size_ > 0);

    pdata_ = new char[num_bytes_];
}

Buffer::~Buffer() {
    if (pdata_ != NULL) {
        delete[]  pdata_;
        pdata_ = NULL;
    }
}

int Buffer::GetNumElements() const {
    return num_elements_;
}

int Buffer::GetElementSize() const {
    return element_size_;
}

Buffer::Usage Buffer::GetUsage() const {
    return usage_;
}

int Buffer::GetNumBytes() const {
    return num_bytes_;
}

char *Buffer::GetData() const {
    return pdata_;
}


//=============================================================

VertexBuffer::VertexBuffer(int numVertices, int vertexSize, Usage usage)
        : Buffer(numVertices, vertexSize, usage) {

}

VertexBuffer::~VertexBuffer() {

}

//=======================================================

IndexBuffer::IndexBuffer(int numIndices, int indexSize, Usage usage)
        : Buffer(numIndices, indexSize, usage), offset_(0) {

}

IndexBuffer::~IndexBuffer() {

}

void IndexBuffer::SetOffset(int offset) {
    offset_ = offset;
}

int IndexBuffer::GetOffset() const {
    return offset_;
}

}
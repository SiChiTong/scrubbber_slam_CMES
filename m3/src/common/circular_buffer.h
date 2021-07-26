// Mapping 3.0
// Copyright (C) idriverplus(BeiJing ZhiXingZhe, Inc.)
// All rights reserved

/***********************************************
 *
 * History
 * Wang Qi     2019.02.12     original version
 *
 * ********************************************/

#ifndef MAPPING_CIRCULARBUFFER_H
#define MAPPING_CIRCULARBUFFER_H

#include <cstdlib>
#include <vector>

namespace mapping::common {

/// 用于激光点云原始数据的表达
template <class T>
class CircularBuffer {
   public:
    CircularBuffer(const size_t &capacity = 200) : _capacity(capacity), _size(0), _startIdx(0) {
        _buffer = new T[capacity];
    }

    ~CircularBuffer() {
        delete[] _buffer;
        _buffer = NULL;
    }

    const size_t &size() { return _size; }

    const size_t &capacity() { return _capacity; }

    bool empty() { return _size == 0; }

    const T &operator[](const size_t &i) { return _buffer[(_startIdx + i) % _capacity]; }

    const T &first() { return _buffer[_startIdx]; }

    const T &last() {
        size_t idx = _size == 0 ? 0 : (_startIdx + _size - 1) % _capacity;
        return _buffer[idx];
    }

    void push(const T &element) {
        if (_size < _capacity) {
            _buffer[_size] = element;
            _size++;
        } else {
            _buffer[_startIdx] = element;
            _startIdx = (_startIdx + 1) % _capacity;
        }
    }

   private:
    size_t _capacity;
    size_t _size;
    size_t _startIdx;
    T *_buffer;
};
}  // namespace mapping::common

#endif  // MAPPING_CIRCULARBUFFER_H

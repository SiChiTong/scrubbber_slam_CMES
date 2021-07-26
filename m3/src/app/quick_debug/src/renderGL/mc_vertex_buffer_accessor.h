//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_VERTEX_BUFFER_ACCESSOR_H
#define HAMO_LINUX_MC_VERTEX_BUFFER_ACCESSOR_H

#include "renderGL/mc_vertex_format.h"

namespace HAMO {

class VertexBuffer;

class VertexBufferAccessor {
   public:
    VertexBufferAccessor();

    VertexBufferAccessor(VertexFormat *vformat, VertexBuffer *vbuffer);

    ~VertexBufferAccessor();

   public:
    bool HasPosition() const;

    bool HasNormal() const;

    bool HasColor(int unit) const;

    bool HasTCoord(int unit) const;

    void Apply();

    template <typename T>
    T &Position(int i);

    template <typename T>
    T &Color(int unit, int i);

    template <typename T>
    T &Normal(int i);

    template <typename T>
    T &TCoord(int unit, int i);

   private:
    VertexFormat *format_;
    VertexBuffer *buffer_;

    int stride_;
    char *data_;

    char *position_;
    char *normal_;
    char *color_[VertexFormat::MC_MAX_COLOR_UNITS];
    char *tcoord_[VertexFormat::MC_MAX_TCOORD_UNITS];

    int position_channels_;
    int normal_channels_;
    int coord_channels_[VertexFormat::MC_MAX_TCOORD_UNITS];
    int color_channels_[VertexFormat::MC_MAX_COLOR_UNITS];
};

template <typename T>
inline T &VertexBufferAccessor::Position(int i) {
    return *(T *)(position_ + i * stride_);
}

template <typename T>
inline T &VertexBufferAccessor::Color(int unit, int i) {
    return *(T *)(color_[unit] + i * stride_);
}

template <typename T>
inline T &VertexBufferAccessor::Normal(int i) {
    return *(T *)(normal_ + i * stride_);
}

template <typename T>
inline T &VertexBufferAccessor::TCoord(int unit, int i) {
    return *(T *)(tcoord_[unit] + i * stride_);
}

}  // namespace HAMO
#endif  // HAMO_LINUX_MC_VERTEX_BUFFER_ACCESSOR_H

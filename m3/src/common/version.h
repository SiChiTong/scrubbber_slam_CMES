#pragma once
#ifndef MAPPING_VERSION_H
#define MAPPING_VERSION_H

/// mapping 的发布版本
namespace mapping::common {
constexpr int mapping_major_version = 4;
constexpr int mapping_minor_version = 1;
constexpr int mapping_revision = 1;

/// 是云端还是现场
constexpr bool mapping_in_cloud = true;
}  // namespace mapping::common

#endif

//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_MAP_DEFINE_H
#define HAMO_LINUX_MAP_DEFINE_H

#include <vector>

namespace HAMO {

#define ZXZ_ERROR(a, ...)
#define ZXZ_WARN(a, ...)

struct LatLon {
    double lon = 0;
    double lat = 0;

    LatLon() : lon(0.0), lat(0.0) {}

    LatLon(double lat, double lon) : lon(lon), lat(lat) {}

    LatLon(const LatLon &latlon) : lon(latlon.lon), lat(latlon.lat) {}
};

struct GPSPoint {
    LatLon latlon;
    float altitude = 0;

    GPSPoint() : altitude(0) {}

    GPSPoint(double lat, double lon, float alt)
        : latlon(lat, lon), altitude(0.0f) {}

    GPSPoint(const LatLon &latlon, float alt) : latlon(latlon), altitude(alt) {}
};

struct UTMPoint {
    double x = 0;
    double y = 0;
    int zone = 0;

    UTMPoint() : x(0.0), y(0.0), zone(0) {}

    UTMPoint(double dx, double dy) : x(dx), y(dy) {}
};

struct TileGrid {
    int x = 0;
    int y = 0;
    int zoom = 0;

    TileGrid() = default;

    TileGrid(int zoom) : x(0), y(0), zoom(zoom) {}

    TileGrid(int gridX, int gridY, int zm) : x(gridX), y(gridY), zoom(zm) {}
};

inline bool operator==(const TileGrid &lhs, const TileGrid &rhs) {
    if (lhs.x == rhs.x && lhs.y == rhs.y && lhs.zoom == rhs.zoom) {
        return true;
    }

    return false;
}

inline bool operator<(const TileGrid &lhs, const TileGrid &rhs) {
    if (lhs.zoom < rhs.zoom) {
        return true;
    } else if (lhs.zoom > rhs.zoom) {
        return false;
    } else if (lhs.x < rhs.x) {
        return true;
    } else if (lhs.x > rhs.x) {
        return false;
    } else if (lhs.y < rhs.y) {
        return true;
    } else if (lhs.y < rhs.y) {
        return false;
    }

    return false;
}

}  // namespace HAMO

#endif  // HAMO_LINUX_MAP_DEFINE_H

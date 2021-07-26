//
// Created by pengguoqi on 19-8-24.
//

#ifndef GMM_MAP_STRUCT_
#define GMM_MAP_STRUCT_

#include "common/mapping_point_types.h"
#include "common/num_type.h"

namespace mapping::tools {

#define PRE 1e-5
#define GMM_INIT_SCORE 2.0
#define GMM_SCORE 2.0
#define GMM_MATCHING_DIS 8
#define GMM_HEIGHT_TYPE 1
#define TILES_GMM_SIZE 500
#define TILES_GMM_INIT_SIZE 150
#define DEBUG false

template <typename T>
struct MapCellR {
    unsigned char u1;
    unsigned char s1;
    unsigned char u2;
    unsigned char s2;
    T w1;
    T w2;
};

template <typename T>
struct MapCellA {
    T w1;
    T u1;
    T s1;
    T w2;
    T u2;
    T s2;
};

template <typename T>
struct XYZI {
    T x;
    T y;
    T z;
    T r;
};

template <typename T>
struct TilesPoint {
    T x;
    T y;
    TilesPoint() : x(0), y(0){};
    TilesPoint(T _x, T _y) : x(_x), y(_y){};
};

template <typename T>
struct GmmMapManagement {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MapT;
};

struct GmmMapParam {
    int cell_point_size;
    float cell_size;
    float init_cell_size;
    float origin_x;
    float origin_y;
    float origin_z;
    long long rows;
    long long cols;
    long long init_rows;
    long long init_cols;
    std::string db_file;
    std::string gmm_map_file;

    long long ToCell(float x, float y, float z) {
        long long delta_y = (y - origin_y) / cell_size;
        long long delta_x = (x - origin_x) / cell_size;
        return cols * (long long)(delta_y) + (long long)(delta_x);
    }
};

struct MapRange {
    float r = 80;
    float minx = std::numeric_limits<float>::max();
    float miny = std::numeric_limits<float>::max();
    float maxx = std::numeric_limits<float>::min();
    float maxy = std::numeric_limits<float>::min();
    float minz = std::numeric_limits<float>::max();
    float maxz = std::numeric_limits<float>::min();
};

struct AR {
    float a;
    float r;
    AR(float aa, float rr) : a(aa), r(rr) {}
};

template <typename T>
struct MapCell {
  unsigned int row = 0;
  unsigned int col = 0;
  MapCellA<T> cella;
  MapCellR<T> cellr;
};

struct Cell {
    unsigned short int size = 0;
    short int ground = 0;
    MapCell<short int> cell;
};

struct Descriptor {
    float intensity = 0.0;
    float intensity_var = 0.0;
    float altitude = 0.0;
    float altitude_var = 0.0;
    unsigned int count = 0;

    void AddSample(const float new_altitude, const float new_intensity) {
        ++count;
        float fcount = static_cast<float>(count);
        float v1 = new_intensity - intensity;
        float value = v1 / fcount;
        intensity += value;
        float v2 = new_intensity - intensity;
        intensity_var = ((fcount - 1.0f) * intensity_var + v1 * v2) / fcount;

        v1 = new_altitude - altitude;
        value = v1 / fcount;
        altitude += value;
        v2 = new_altitude - altitude;
        altitude_var = ((fcount - 1.0f) * altitude_var + v1 * v2) / fcount;
    }
};

struct Descriptor2 {
    float mean = 0.0;
    float var = 0.0;
    unsigned int count = 0;

    void AddSample(const float sample) {
        ++count;
        float fcount = static_cast<float>(count);
        float v1 = sample - mean;
        float value = v1 / fcount;
        mean += value;
        float v2 = sample - mean;
        var = ((fcount - 1.0f) * var + v1 * v2) / fcount;
    }
};

inline short int To16(int n) {
    if (abs(n) <= 30000) return n;
    else if (n < -30000)
        return -30000;
    else
        return 30000;
}

inline unsigned char To8(int n) {
    if (n >= 0 || n <= 255) return n;
    else if (n > 255)
        return 255;
    else
        return 0;
}

typedef MapCellR<float> MapCellRf;
typedef MapCellA<float> MapCellAf;

typedef MapCellR<short int> MapCellRi16;
typedef MapCellA<short int> MapCellAi16;

typedef Eigen::Matrix<MapCellAi16, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MapTa;
typedef Eigen::Matrix<MapCellRi16, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MapTr;
typedef Eigen::Matrix<MapCell<short int>, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MapT;

typedef TilesPoint<double> TilesPointD;
typedef TilesPoint<float> TilesPointF;
typedef TilesPoint<int> TilesPointI;

typedef GmmMapManagement<float> GmmMapMf;
typedef GmmMapManagement<size_t> GmmMapMt;
typedef GmmMapManagement<std::vector<float>> GmmMapMv;
typedef GmmMapManagement<MapCellAf> GmmMapMc;

typedef XYZI<float> MapPointf;
typedef std::vector<MapPointf> MapPointsf;

}  // namespace mapping::tools

#endif

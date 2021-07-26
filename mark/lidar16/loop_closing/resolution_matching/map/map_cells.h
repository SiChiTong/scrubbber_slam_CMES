// lbh  creat  2019.7.31
#ifndef MAPPING_CORE_RESOLUTION_MATCHING_MAP_CELLS_
#define MAPPING_CORE_RESOLUTION_MATCHING_MAP_CELLS_

namespace mapping { namespace core {

// 每个地图栅格在z轴上的分层数量
constexpr int num_cell_layer = 2;

//用于匹配的统计量
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

// XY平面离散后形成一个个cell，每个cell内会包括2个Descriptor
typedef Descriptor Cell[num_cell_layer];

// XY平面离散用MapCells表示
class MapCells {
   public:
    MapCells() : rows_(0), cols_(0), map_cells_(nullptr) {}
    ~MapCells() {
        if (map_cells_) delete[] map_cells_;
        rows_ = cols_ = 0;
    }

    int Init(int rows, int cols) {
        if (rows <= 0 || cols <= 0) return -1;
        if (map_cells_) {
            delete[] map_cells_;
            map_cells_ = nullptr;
        }
        map_cells_ = new Cell[rows * cols];
        rows_ = rows;
        cols_ = cols;
        return 0;
    }

    inline Cell* GetMapCellPtrSafe(int row, int col) {
        if (row < 0 || row >= rows_ || col < 0 || col >= cols_) return nullptr;
        return map_cells_ + row * cols_ + col;
    }
    inline Cell& operator()(int row, int col) { return map_cells_[row * cols_ + col]; }

   protected:
    int rows_;
    int cols_;
    Cell* map_cells_;
};

} }  // namespace mapping::core
#endif

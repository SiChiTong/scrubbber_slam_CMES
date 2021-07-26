// lbh  creat 2019.7.31

#include "core/resolution_matching/map/resolution_map.h"

namespace mapping::core {

// ResolutionMap

ResolutionMap::ResolutionMap(int rows, int cols, float resolution)
    : resolution_(resolution), rows_(rows), cols_(cols), map_cells_(new MapCells()) {
    if (map_cells_->Init(rows, cols) < 0) {
        map_cells_ = nullptr;
    }
}

ResolutionMap::~ResolutionMap() {}

int ResolutionMap::AddPoint(const Eigen::Vector4f &point, float z_ground) {
    auto des = GetDescriptor(point, z_ground);
    if (des == nullptr) return -1;
    des->AddSample(point(2), point(3));
    return 0;
}

// log(N) = -1/2*log(2*pi) - 1/2*log(s**2) - 1/2*(x-u)**2/s**2  ~
// -log(s**2)-(x-u)**2/s**2 = log(NN)
// exp(log(NN)) = 1/s**2*exp(-(x-u)**2/s**2)
float ResolutionMap::Logp(const Eigen::Vector4f &point, float z_ground) {
    auto des = GetDescriptor(point, z_ground);
    const float min_logp = log(gauss_min_p);
    if (des == nullptr || des->count < 2) return min_logp;
    float logpa, logpi, p;
    logpa = -log(des->altitude_var + 0.1) - pow(des->altitude - point(2), 2) / (des->altitude_var + 0.1);
    logpi = -log(des->intensity_var + 0.5) - pow(des->intensity - point(3), 2) / (des->intensity_var + 0.5);
    p = logpa + logpi;
    return p < min_logp ? min_logp : p;
}

Eigen::MatrixXf ResolutionMap::Logps(const Eigen::Vector4f &point, float z_ground, int linear_search) {
    const float min_logp = log(gauss_min_p);
    int size = 2 * linear_search + 1;
    Eigen::MatrixXf ps(size, size);
    int layer = point(2) < z_ground ? 0 : 1;
    int r = point(1) / resolution_;
    int c = point(0) / resolution_;
    float logpa, logpi, p;
    int row, col;
    for (int i = 0; i < size; ++i)
        for (int j = 0; j < size; ++j) {
            row = r + i - linear_search;
            col = c + j - linear_search;
            auto des = (*map_cells_)(row, col) + layer;
            if (!CheckRange(row, col) || des->count < 2) {
                ps(i, j) = min_logp;
                continue;
            }
            logpa = -log(des->altitude_var + 0.1) - pow(des->altitude - point(2), 2) / (des->altitude_var + 0.1);
            logpi = -log(des->intensity_var + 0.5) - pow(des->intensity - point(3), 2) / (des->intensity_var + 0.5);
            p = logpa + logpi;
            ps(i, j) = p < min_logp ? min_logp : p;
        }
    return ps;
}

Descriptor *ResolutionMap::GetDescriptor(const Eigen::Vector4f &point, float z_ground) {
    int layer = point(2) < z_ground ? 0 : 1;
    int row = point(1) / resolution_;
    int col = point(0) / resolution_;
    if (!CheckRange(row, col)) return nullptr;
    return (*map_cells_)(row, col) + layer;
}
}  // namespace mapping::core

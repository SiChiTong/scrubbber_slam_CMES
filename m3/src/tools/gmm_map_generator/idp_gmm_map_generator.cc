//
// Created by pengguoqi on 19-8-24.
//

#include "tools/gmm_map_generator/idp_gmm_map_generator.h"
#include "tools/gmm_map_generator/gmm_map_io.h"

#include <glog/logging.h>
#include <pcl/common/common.h>

namespace mapping::tools {

using namespace cv;
using namespace cv::ml;
using namespace mapping::common;

IDPGmmMapGenerator::~IDPGmmMapGenerator() {
    line_a_.join();
    line_r_.join();
}

int IDPGmmMapGenerator::Init() {
    end_ = false;
    if (LoadRawData() < 0) {
        LOG(ERROR) << "failed to LoadRawData : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to Init.\n";
        return -1;
    }

    if (CalculateMapBoundary() < 0) {
        LOG(ERROR) << "failed to CalculateMapBoundary ";
        report_ += "GmmMapGenerator error : failed to Init.\n";
        return -2;
    }

    CalculateMapDilutionRatio();
    complete_gmm_map_param_ = gmm_map_param_;
    LOG(INFO) << "cell_point_size: " << gmm_map_param_.cell_point_size;
    LOG(INFO) << "origin_x: " << gmm_map_param_.origin_x;
    LOG(INFO) << "origin_y: " << gmm_map_param_.origin_y;
    LOG(INFO) << "origin_z: " << gmm_map_param_.origin_z;
    LOG(INFO) << "rows: " << gmm_map_param_.rows;
    LOG(INFO) << "cols: " << gmm_map_param_.cols;
    LOG(INFO) << "init_rows: " << gmm_map_param_.init_rows;
    LOG(INFO) << "init_cols: " << gmm_map_param_.init_cols;
    return 0;
}

int IDPGmmMapGenerator::Start() {
    if (WriteOffset() < 0) {
        LOG(ERROR) << "failed to WriteOffset ";
        report_ += "GmmMapGenerator error : failed to Start.\n";
        return -1;
    }
    line_a_ = std::thread([=] { return this->WriteMapa(); });
    line_r_ = std::thread([=] { return this->WriteMapr(); });
    return 0;
}

void IDPGmmMapGenerator::ZipGmm() {
    LOG(INFO) << "Zip....... ";
    std::string command = "zip -jr " + gmm_map_param_.gmm_map_file + "../gmm.zip " + gmm_map_param_.gmm_map_file +
                          " map_a.bin" + " init_a.bin" + " map_r.bin" + " init_r.bin" + " offset.txt";
    system((char *)command.c_str());
    command = "rm -rf " + gmm_map_param_.gmm_map_file;
    system((char *)command.c_str());
}

int IDPGmmMapGenerator::InitMap(const int cell_type) {
    int length = gmm_map_param_.rows * gmm_map_param_.cols;
    if (cell_type == 0) {
        map_a_.MapT.resize(1, length);
        gmm_map_a_.MapT.resize(1, length);
        for (int i = 0; i < length; ++i) {
            map_a_.MapT(0, i).clear();
            memset(&gmm_map_a_.MapT(0, i), 0.0, sizeof(MapCellAf));
        }
    } else if (cell_type == 1) {
        map_r_.MapT.resize(1, length);
        gmm_map_r_.MapT.resize(1, length);
        for (int i = 0; i < length; ++i) {
            map_r_.MapT(0, i).clear();
            memset(&gmm_map_r_.MapT(0, i), 0.0, sizeof(MapCellAf));
        }
    } else {
        LOG(WARNING) << "unrecognized cell type";
    }
    return 0;
}

int IDPGmmMapGenerator::LoadRawData() {
    point_cloud_ptr_ = boost::make_shared<PointCloudXYZI>();
    std::shared_ptr<GmmBaseIO> gb_io = std::make_shared<GmmBaseIO>();
    int r = gb_io->LoadRawMap(gmm_map_param_.db_file, point_cloud_ptr_);
    int s = point_cloud_ptr_->points.size();
    if (r == -1 || s == 0) {
        LOG(ERROR) << "failed to load db : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to load db : " + gmm_map_param_.db_file + ".\n";
        gb_io = nullptr;
        return -1;
    } else {
        gb_io = nullptr;
        return 0;
    }
}

void IDPGmmMapGenerator::SwitchMapParam(int param_type) {
    if (param_type == 0) {
        gmm_map_param_.cell_point_size = 20;
        gmm_map_param_.cell_size = gmm_map_param_.init_cell_size;
        gmm_map_param_.rows = gmm_map_param_.init_rows;
        gmm_map_param_.cols = gmm_map_param_.init_cols;
    } else if (param_type == 1) {
        gmm_map_param_ = complete_gmm_map_param_;
    } else {
        LOG(ERROR) << "param is error!!!";
        report_ += "GmmMapGenerator error : param is error.\n";
    }
}

int IDPGmmMapGenerator::CalculateMapBoundary() {
    PointXYZI min, max;
    pcl::getMinMax3D(*point_cloud_ptr_, min, max);
    gmm_map_param_.origin_x = min.x;
    gmm_map_param_.origin_y = min.y;
    gmm_map_param_.origin_z = min.z;
    if (fabs(gmm_map_param_.cell_size) <= PRE) {
        LOG(ERROR) << "error cell size: " << gmm_map_param_.cell_size;
        report_ += "GmmMapGenerator error : wrong cell size: " + std::to_string(gmm_map_param_.cell_size) + ".\n";
        return -1;
    } else {
        gmm_map_param_.rows = (max.y - min.y + 0.001) / gmm_map_param_.cell_size + 1;
        gmm_map_param_.cols = (max.x - min.x + 0.001) / gmm_map_param_.cell_size + 1;
        gmm_map_param_.init_rows = (max.y - min.y + 0.001) / gmm_map_param_.init_cell_size + 1;
        gmm_map_param_.init_cols = (max.x - min.x + 0.001) / gmm_map_param_.init_cell_size + 1;
        LOG(INFO) << "minx: " << min.x << ", maxx: " << max.x << ", miny: " << min.y << ", maxy: " << max.y
                  << ",minz: " << min.z << ", maxz: " << max.z;
        LOG(INFO) << "rows: " << gmm_map_param_.rows << ", cols: " << gmm_map_param_.cols;
    }
    return 0;
}

int IDPGmmMapGenerator::CreateMap(const int cell_type) {
    if (cell_type == 0 && InitMap(0) == 0) {
        for (auto &point : point_cloud_ptr_->points) {
            long long index_a = gmm_map_param_.ToCell(point.x, point.y, point.z);
            map_a_.MapT(0, index_a).push_back(point.z);
        }
    } else if (cell_type == 1 && InitMap(1) == 0) {
        for (auto &point : point_cloud_ptr_->points) {
            long long index_r = gmm_map_param_.ToCell(point.x, point.y, point.z);
            map_r_.MapT(0, index_r).push_back(point.intensity);
        }
    } else {
        LOG(ERROR) << "map container initialize failed";
        report_ += "GmmMapGenerator error : map container initialize failed.\n";
        return -1;
    }
    return 0;
}

int IDPGmmMapGenerator::CalculateMapDilutionRatio() {
    GmmMapMt grid_map;
    grid_map.MapT.resize(gmm_map_param_.rows, gmm_map_param_.cols);
    for (int i = 0; i < gmm_map_param_.rows; ++i) {
        for (int j = 0; j < gmm_map_param_.cols; ++j) {
            grid_map.MapT(i, j) = 0;
        }
    }
    size_t p_size = point_cloud_ptr_->points.size();
    for (size_t i = 0; i < p_size; ++i) {
        size_t row = (point_cloud_ptr_->points.at(i).y - gmm_map_param_.origin_y) / gmm_map_param_.cell_size;
        size_t col = (point_cloud_ptr_->points.at(i).x - gmm_map_param_.origin_x) / gmm_map_param_.cell_size;
        grid_map.MapT(row, col)++;
    }

    size_t max_size = 0;
    for (int i = 0; i < gmm_map_param_.rows; ++i) {
        for (int j = 0; j < gmm_map_param_.cols; ++j) {
            if (max_size < grid_map.MapT(i, j)) max_size = grid_map.MapT(i, j);
        }
    }

    std::vector<size_t> cell_point_histogram(max_size + 1, 0);
    size_t c_size = 0;
    for (int i = 0; i < gmm_map_param_.rows; ++i) {
        for (int j = 0; j < gmm_map_param_.cols; ++j) {
            cell_point_histogram[grid_map.MapT(i, j)]++;
            if (grid_map.MapT(i, j) > 0) c_size++;
        }
    }

    double cell_with_point = 100 - cell_point_histogram[0] * 100.0 / gmm_map_param_.rows / gmm_map_param_.cols;
    for (size_t i = 1; i < max_size; ++i) {
        double cell_point_percentage = cell_point_histogram[i] * 100.0 / c_size;
        if (cell_point_percentage >= 0.1) {
            LOG(INFO) << "cell with : " << i << " points (%) : " << cell_point_percentage;
            if (cell_point_percentage < 2.1) {
                gmm_map_param_.cell_point_size = i - 1;
                LOG(INFO) << "cell point size : " << gmm_map_param_.cell_point_size
                          << " cell point percentage: " << cell_point_percentage;
                return 0;
            }
        }
    }
    return -1;
}

void IDPGmmMapGenerator::GmmTrainZ(int row, int col) {
    long long block = gmm_map_param_.cols * row + col;
    int size = (int)map_a_.MapT(0, block).size();
    if (size < gmm_map_param_.cell_point_size) return;
    Mat mat_z = Mat(size, 1, CV_32FC1);
    for (int i = 0; i < size; ++i) mat_z.at<float>(i, 0) = map_a_.MapT(0, block)[i];
    em_a_->setClustersNumber(2);
    em_a_->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
    em_a_->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
    if (em_a_->trainEM(mat_z, noArray(), noArray(), noArray())) {
        std::vector<cv::Mat> z_covs_vec;
        em_a_->getCovs(z_covs_vec);
        gmm_map_a_.MapT(0, block).w1 = em_a_->getWeights().at<double>(0);
        gmm_map_a_.MapT(0, block).u1 = em_a_->getMeans().at<double>(0);
        gmm_map_a_.MapT(0, block).s1 = z_covs_vec[0].at<double>(0, 0);
        gmm_map_a_.MapT(0, block).w2 = em_a_->getWeights().at<double>(1);
        gmm_map_a_.MapT(0, block).u2 = em_a_->getMeans().at<double>(1);
        gmm_map_a_.MapT(0, block).s2 = z_covs_vec[1].at<double>(0, 0);
    }
}

void IDPGmmMapGenerator::GmmTrainI(int row, int col) {
    long long block = gmm_map_param_.cols * row + col;
    int size = (int)map_r_.MapT(0, block).size();
    if (size < gmm_map_param_.cell_point_size) return;
    Mat mat_i = Mat(size, 1, CV_32FC1);
    for (int i = 0; i < size; ++i) mat_i.at<float>(i, 0) = map_r_.MapT(0, block)[i];
    em_r_->setClustersNumber(2);
    em_r_->setCovarianceMatrixType(EM::COV_MAT_SPHERICAL);
    em_r_->setTermCriteria(TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 300, 0.1));
    if (em_r_->trainEM(mat_i, noArray(), noArray(), noArray())) {
        std::vector<cv::Mat> r_covs_vec;
        em_r_->getCovs(r_covs_vec);
        gmm_map_r_.MapT(0, block).w1 = em_r_->getWeights().at<double>(0);
        gmm_map_r_.MapT(0, block).u1 = em_r_->getMeans().at<double>(0);
        gmm_map_r_.MapT(0, block).s1 = r_covs_vec[0].at<double>(0, 0);
        gmm_map_r_.MapT(0, block).w2 = em_r_->getWeights().at<double>(1);
        gmm_map_r_.MapT(0, block).u2 = em_r_->getMeans().at<double>(1);
        gmm_map_r_.MapT(0, block).s2 = r_covs_vec[1].at<double>(0, 0);
    }
}

int IDPGmmMapGenerator::TrainMap(const int cell_type) {
    if (cell_type == 0) {
        for (long long i = 0; i < gmm_map_param_.rows; i++)
            for (long long j = 0; j < gmm_map_param_.cols; j++) GmmTrainZ(i, j);
        LOG(INFO) << "altitude data trained finish ";
    } else if (cell_type == 1) {
        for (long long i = 0; i < gmm_map_param_.rows; i++)
            for (long long j = 0; j < gmm_map_param_.cols; j++) GmmTrainI(i, j);
        LOG(INFO) << "intensity data trained finish ";
    } else {
        LOG(ERROR) << "undefined map type ";
        report_ += "GmmMapGenerator error : undefined map type.\n";
        return -1;
    }
    return 0;
}

int IDPGmmMapGenerator::WriteOffset() {
    std::ofstream write_offset;
    write_offset.open(gmm_map_param_.gmm_map_file + "offset.txt");
    if (!write_offset.is_open()) {
        LOG(ERROR) << "offset.txt open failed ";
        report_ += "GmmMapGenerator error : offset.txt open failed.\n";
        return -1;
    }
    write_offset << gmm_map_param_.rows << " " << gmm_map_param_.cols << " " << gmm_map_param_.origin_x << " "
                 << gmm_map_param_.origin_y << " " << gmm_map_param_.init_rows << " " << gmm_map_param_.init_cols << " "
                 << "7.0"
                 << " "
                 << "7.0"
                 << " "
                 << "8"
                 << " " << std::endl;
    if (write_offset.bad()) {
        LOG(ERROR) << "offset.txt write bad ";
        report_ += "GmmMapGenerator error : offset.txt write bad.\n";
        write_offset.close();
        return -1;
    }
    write_offset.close();
    return 0;
}

int IDPGmmMapGenerator::WriteMapa() {
    /// loc
    SwitchMapParam(1);
    CreateMap(0);
    TrainMap(0);
    std::ofstream outfile_a;
    std::string path_map_a = gmm_map_param_.gmm_map_file + "map_a.bin";
    outfile_a.open(path_map_a.c_str(), std::ios::binary);
    if (!outfile_a.is_open()) {
        LOG(ERROR) << "map_a.bin open failed ";
        report_ += "GmmMapGenerator error : map_a.bin open failed.\n";
        return -1;
    }
    size_t size = gmm_map_a_.MapT.rows() * gmm_map_a_.MapT.cols() * sizeof(MapCellAf);
    outfile_a.write((char *)gmm_map_a_.MapT.data(), size);
    if (outfile_a.bad()) {
        LOG(ERROR) << "map_a.bin write bad ";
        report_ += "GmmMapGenerator error : map_a.bin write bad.\n";
        outfile_a.close();
        return -1;
    }
    LOG(INFO) << path_map_a << " write done: " << std::endl;
    outfile_a.close();
    /// init
    SwitchMapParam(0);
    CreateMap(0);
    TrainMap(0);
    std::ofstream outfile_init_a;
    std::string path_init_a = gmm_map_param_.gmm_map_file + "init_a.bin";
    outfile_init_a.open(path_init_a.c_str(), std::ios::binary);
    if (!outfile_init_a.is_open()) {
        LOG(ERROR) << "init_a.bin open failed ";
        report_ += "GmmMapGenerator error : init_a.bin open failed.\n";
        return -2;
    }
    size = gmm_map_a_.MapT.rows() * gmm_map_a_.MapT.cols() * sizeof(MapCellAf);
    outfile_init_a.write((char *)gmm_map_a_.MapT.data(), size);
    if (outfile_init_a.bad()) {
        LOG(ERROR) << "init_a.bin write bad ";
        report_ += "GmmMapGenerator error : init_a.bin write bad.\n";
        outfile_init_a.close();
        return -2;
    }
    outfile_init_a.close();
    LOG(INFO) << path_init_a << " write done: ";
    if (end_) ZipGmm();
    end_ = true;
    return 0;
}

int IDPGmmMapGenerator::WriteMapr() {
    /// loc
    SwitchMapParam(1);
    CreateMap(1);
    TrainMap(1);
    std::ofstream outfile_r;
    std::string path_map_r = gmm_map_param_.gmm_map_file + "map_r.bin";
    outfile_r.open(path_map_r.c_str(), std::ios::binary);
    if (!outfile_r.is_open()) {
        LOG(ERROR) << "map_r.bin open failed ";
        report_ += "GmmMapGenerator error : map_r.bin open failed.\n";
        return -1;
    }
    size_t size = gmm_map_r_.MapT.rows() * gmm_map_r_.MapT.cols() * sizeof(MapCellAf);
    outfile_r.write((char *)gmm_map_r_.MapT.data(), size);
    if (outfile_r.bad()) {
        LOG(ERROR) << "map_r.bin write bad ";
        report_ += "GmmMapGenerator error : map_r.bin write bad.\n";
        outfile_r.close();
        return -1;
    }
    LOG(INFO) << path_map_r << " write done: ";
    outfile_r.close();
    /// init
    SwitchMapParam(0);
    CreateMap(1);
    TrainMap(1);
    std::ofstream outfile_init_r;
    std::string path_init_r = gmm_map_param_.gmm_map_file + "init_r.bin";
    outfile_init_r.open(path_init_r.c_str(), std::ios::binary);
    if (!outfile_init_r.is_open()) {
        LOG(ERROR) << "init_r.bin open failed ";
        report_ += "GmmMapGenerator error : init_r.bin open failed.\n";
        return -2;
    }
    size = gmm_map_r_.MapT.rows() * gmm_map_r_.MapT.cols() * sizeof(MapCellAf);
    outfile_init_r.write((char *)gmm_map_r_.MapT.data(), size);
    if (outfile_init_r.bad()) {
        LOG(ERROR) << "init_r.bin write bad ";
        report_ += "GmmMapGenerator error : init_r.bin write bad.\n";
        outfile_init_r.close();
        return -2;
    }
    LOG(INFO) << path_init_r << " write done: ";
    outfile_init_r.close();
    if (end_) ZipGmm();
    end_ = true;
    return 0;
}

bool IDPGmmMapGenerator::GenerateReport(std::string &report, bool verbose) {
    report = report_;
    return true;
}

}  // namespace mapping::tools

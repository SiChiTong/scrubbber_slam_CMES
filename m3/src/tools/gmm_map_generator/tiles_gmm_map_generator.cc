//
// Created by pengguoqi on 20-03-23.
//

#include "tools/gmm_map_generator/tiles_gmm_map_generator.h"
#include <glog/logging.h>

namespace mapping::tools {
using namespace cv;
using namespace cv::ml;
using namespace mapping::common;

TilesGmmMapGenerator::~TilesGmmMapGenerator() {
    gb_io_ = nullptr;
    std::unordered_map<unsigned long long, Cell>().swap(map_cells_);
}

int TilesGmmMapGenerator::Init() {
    if (LoadRawData() < 0) {
        LOG(ERROR) << "failed to LoadRawData : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to Init.\n";
        return -1;
    }
    return 0;
}

int TilesGmmMapGenerator::Start() {
    std::string binpath = gmm_map_param_.gmm_map_file + "map/";
    resolution_ = gmm_map_param_.cell_size;
    select_gmm_size_ = true;
    LOG(INFO) << "build map.";
    if (BuildMap(binpath) < 0) {
        LOG(ERROR) << "failed to map build ";
        return -1;
    }
    size_ = 0;
    debug_size_ = 0;
    resolution_ = gmm_map_param_.init_cell_size;
    binpath = gmm_map_param_.gmm_map_file + "init_map/";
    select_gmm_size_ = false;
    LOG(INFO) << "build init map.";
    if (BuildMap(binpath) < 0) {
        LOG(ERROR) << "failed to init map build ";
        return -2;
    }
    size_ = 0;
    debug_size_ = 0;
    if (WriteOffset() < 0) {
        LOG(ERROR) << "failed to Write offset ";
        return -3;
    }
    if (WriteGroud() < 0) {
        LOG(ERROR) << "failed to Write groud ";
        return -4;
    }
    if (DEBUG) ViewResult();
    ZipGmm();
    return 0;
}

void TilesGmmMapGenerator::ZipGmm() {
    LOG(INFO) << "Zip....... ";
    std::string command = "python ./scripts/gmmzipfile.py " + gmm_map_param_.gmm_map_file + "../";
    system((char *)command.c_str());
    command = "rm -rf " + gmm_map_param_.gmm_map_file;
    system((char *)command.c_str());
}

int TilesGmmMapGenerator::LoadRawData() {
    gb_io_ = std::make_shared<GmmBaseIO>();
    int r = gb_io_->LoadRawMap(gmm_map_param_.db_file);
    map_range_ = gb_io_->GetRange();
    if (r == -1) {
        LOG(ERROR) << "failed to load db : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to load db : " + gmm_map_param_.db_file + ".\n";
        gb_io_ = nullptr;
        return -1;
    } else {
        return 0;
    }
}

int TilesGmmMapGenerator::BuildMap(const std::string &filepath) {
    int cols = 0, rows = 0;
    std::string dir = filepath;
    float size = resolution_ * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE);
    cols = (int)((map_range_.maxx - map_range_.minx) / size) + 1;
    rows = (int)((map_range_.maxy - map_range_.miny) / size) + 1;
    LOG(INFO) << "map size : size = " << size << ", cols = " << cols << ", rows = " << rows;
    if (0 != access(filepath.c_str(), 0)) {
        mkdir(filepath.c_str(), 0777);
    }
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            TilesPointF center;
            center.x = map_range_.minx + (col + 0.5) * size;
            center.y = map_range_.miny + (row + 0.5) * size;
            auto r = Build(center, size);
            if (r < 0) {
                LOG(ERROR) << "failed to Build map ";
                report_ += "GmmMapGenerator error : failed to Build map.\n";
                return -1;
            } else if (1 == r) {
                continue;
            } else {
                int re = EmTrain();
                if (re < 0) {
                    LOG(ERROR) << "failed to train map ";
                    report_ += "GmmMapGenerator error : failed to train map.\n";
                    return -2;
                } else if (1 == re) {
                    LOG(WARNING) << "all cells are invalid. ";
                    continue;
                } else {
                    std::string path = filepath + std::to_string(row) + "_" + std::to_string(col) + "_" + "ar.bin";
                    int re = WriteMap(path, col, row);
                    if (re < 0) return -1;
                    path = filepath + std::to_string(row) + "_" + std::to_string(col) + "_" + "effective.txt";
                    WriteEffectiveCell(path);
                }
            }
        }
    }
    return 0;
}

int TilesGmmMapGenerator::Build(const TilesPointF &center, const float &size) {
    LOG(INFO) << "hashmap...";
    gb_io_->SetResolution(resolution_);
    gb_io_->ClearHashRef();
    map_cells_.clear();
    int r = gb_io_->HandlingPCD(gmm_map_param_.db_file, center, size);
    if (r < 0) {
        LOG(ERROR) << "failed to handling PCD : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to handling PCD : " + gmm_map_param_.db_file + ".\n";
        gb_io_ = nullptr;
        return -1;
    } else if (r == 1) {
        return 1;
    } else {
        return 0;
    }
}

int TilesGmmMapGenerator::EmTrain() {
    LOG(INFO) << "emtrain...";
    auto &hcs = gb_io_->GetHashRef();
    size_t size = hcs.size();
    if (size < 0) return -1;
    int num = 0;
    LOG(INFO) << "tiles train : size : " << size;
    debug_size_ += size;
    for (auto& hash : hcs) {
      Cell cell;
      unsigned long long rc = hash.first;
      if (Train3(hash.second, cell)) {
        cell.cell.row = *((unsigned int*)&rc);
        cell.cell.col = *((unsigned int*)&rc + 1);
        map_cells_.emplace(hash.first, cell);
      }
      hash.second.reserve(0);
      num++;
    }
    if (num < 1) return -2;
    if (map_cells_.size() < 1) return 1;
    size_ = map_cells_.size();
    LOG(INFO) << "train end";
    return 0;
}

bool TilesGmmMapGenerator::Train3(std::vector<AR> &ars, Cell &cell) {
    int size = ars.size();
    if (size == 0) return false;
    cell.size = size;
    if (size <= 6) return false;
    cv::Mat mat_a = cv::Mat(size, 1, CV_32FC1);
    cv::Mat mat_r = cv::Mat(size, 1, CV_32FC1);
    for (int i = 0; i < size; ++i) {
        mat_a.at<float>(i, 0) = ars[i].a;
        mat_r.at<float>(i, 0) = ars[i].r;
    }
    cv::Mat labelsa, labelsr;
    cv::kmeans(mat_a, 2, labelsa, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.1), 2,
               cv::KMEANS_PP_CENTERS);
    cv::kmeans(mat_r, 2, labelsr, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 1), 2,
               cv::KMEANS_PP_CENTERS);

    Descriptor2 da1, da2, dr1, dr2;
    for (int i = 0; i < size; ++i) {
        if (labelsa.at<float>(i, 0) == 0) da1.AddSample(ars[i].a);
        else
            da2.AddSample(ars[i].a);
        if (labelsr.at<float>(i, 0) == 0) dr1.AddSample(ars[i].r);
        else
            dr2.AddSample(ars[i].r);
    }
    auto &ca = cell.cell.cella;
    auto &cr = cell.cell.cellr;
    if (da1.var < 0.01) da1.var = 0.01;
    if (da2.var < 0.01) da2.var = 0.01;
    if (dr1.var < 1) dr1.var = 1;
    if (dr2.var < 1) dr2.var = 1;
    ca.w1 = To16(da1.count * 100 / size);
    ca.u1 = To16(da1.mean * 100);
    ca.s1 = To16(da1.var * 100);
    ca.w2 = To16(da2.count * 100 / size);
    ca.u2 = To16(da2.mean * 100);
    ca.s2 = To16(da2.var * 100);
    cr.w1 = To16(dr1.count * 100 / size);
    cr.u1 = To16(dr1.mean);
    cr.s1 = To16(dr1.var);
    cr.w2 = To16(dr2.count * 100 / size);
    cr.u2 = To16(dr2.mean);
    cr.s2 = To16(dr2.var);
    return true;
}

int TilesGmmMapGenerator::WriteMap(const std::string &file, const int &col, const int &row) {
    LOG(INFO) << "debug_size:" << debug_size_;
    std::vector<MapCell<short int>> map;
    invalid_num_ = 0;
    map.reserve((select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE) * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE));
    for (auto& pair : map_cells_) {
      auto& m = pair.second;
      if (m.cell.row < row * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE) || m.cell.row >= (row + 1) * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE)
         || m.cell.col < col * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE) || m.cell.col >= (col + 1) * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE)) {
        LOG(WARNING) << " out of bounds: " << m.cell.row << " , " << m.cell.col
		                     << " | " << row * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE) << " , "<< col * (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE);
        invalid_num_ ++;
        continue; 
      }
      auto& ca = m.cell.cella;
      auto& cr = m.cell.cellr;
      ca.s1 = sqrt(ca.s1 / 100.) * 100;
      ca.s2 = sqrt(ca.s2 / 100.) * 100;
      cr.s1 = sqrt((float)cr.s1);
      cr.s2 = sqrt((float)cr.s2);
      map.push_back(m.cell);
    }
    LOG(INFO) << "effective size:" << map.size();
    size_ = size_ - invalid_num_;
    int size = map.size() * sizeof(MapCell<short int>);
    if (WriteBin(file, map.data(), size) < 0) return -1;
    else
        return 0;
}

int TilesGmmMapGenerator::WriteBin(const std::string &filename, const void *data, const int &size) {
    std::ofstream os;
    os.open(filename.c_str(), std::ios::binary);
    if (!os.is_open()) {
        LOG(ERROR) << "failed to open : " << filename;
        report_ += "GmmMapGenerator error : " + filename + " open failed.\n";
        return -1;
    }
    os.write((char *)data, size);
    if (os.bad()) {
        os.close();
        LOG(ERROR) << filename << " is bad.";
        report_ += "GmmMapGenerator error : " + filename + " is bad.\n";
        return -1;
    }
    os.close();
    {
       std::ifstream infile;
       infile.open(filename.c_str(), std::ios::binary);
       if (!infile.is_open()) return 0;
       infile.seekg(0, infile.end);
       LOG(INFO) << " size_tellg : " << infile.tellg() << ", " << filename;
       if (infile.tellg() != size) {
          LOG(ERROR) << filename << " is bad (write != read).";
          infile.close();
         return -1;
       }
       infile.close();
    }
    return 0;
}

int TilesGmmMapGenerator::WriteOffset() {
    std::ofstream write_offset;
    int rows1, cols1, rows2, cols2;
    float minx, miny;
    float score1, score2;
    int distance;
    write_offset.open(gmm_map_param_.gmm_map_file + "offset.txt");
    if (!write_offset.is_open()) {
        LOG(ERROR) << "offset open failed ";
        report_ += "GmmMapGenerator error : offset open failed.\n";
        return -1;
    }
    rows1 = (map_range_.maxy - map_range_.miny) / gmm_map_param_.cell_size + 1;
    cols1 = (map_range_.maxx - map_range_.minx) / gmm_map_param_.cell_size + 1;
    rows2 = (map_range_.maxy - map_range_.miny) / gmm_map_param_.init_cell_size + 1;
    cols2 = (map_range_.maxx - map_range_.minx) / gmm_map_param_.init_cell_size + 1;
    minx = map_range_.minx;
    miny = map_range_.miny;
    score1 = GMM_SCORE;
    score2 = GMM_INIT_SCORE;
    distance = GMM_MATCHING_DIS;

    write_offset << rows1 << " " << cols1 << " " << minx << " " << miny << " " << rows2 << " " << cols2 << " " << score1
                 << " " << score2 << " " << distance << std::endl;

    if (write_offset.bad()) {
        LOG(ERROR) << "offset write bad ";
        report_ += "GmmMapGenerator error : offset write bad.\n";
        write_offset.close();
        return -1;
    }
    write_offset.close();
    return 0;
}

int TilesGmmMapGenerator::WriteEffectiveCell(const std::string &path) {
    std::ofstream write_effective;
    int rows1, cols1;
    write_effective.open(path);
    if (!write_effective.is_open()) {
        LOG(ERROR) << "effective open failed ";
        report_ += "GmmMapGenerator error : effective open failed.\n";
        return -1;
    }
    rows1 = (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE);
    cols1 = (select_gmm_size_? TILES_GMM_SIZE : TILES_GMM_INIT_SIZE);
    write_effective << rows1 << " " << cols1 << " " << size_ << std::endl;

    if (write_effective.bad()) {
        LOG(ERROR) << "effective write bad ";
        report_ += "GmmMapGenerator error : effective write bad.\n";
        write_effective.close();
        return -1;
    }
    LOG(INFO) << "effective: " << size_ << ", " << path;
    write_effective.close();
    return 0;
}

int TilesGmmMapGenerator::WriteGroud() {
    std::string binfile = gmm_map_param_.gmm_map_file + "groud.bin";
    std::vector<short int> gs;
    int rows, cols;
    if (gb_io_->GetZ(gmm_map_param_.cell_size, rows, cols, gs, GMM_HEIGHT_TYPE, binfile) < 0) {
        LOG(ERROR) << " failed to get groud ";
        report_ += "GmmMapGenerator error : failed to get groud.\n";
        return -1;
    }
    if (1 == GMM_HEIGHT_TYPE) return 0;
    else
        return WriteBin(binfile, gs.data(), rows * cols * sizeof(short int));
}

int TilesGmmMapGenerator::ViewResult() {
    std::vector<std::string> files;
    std::string path = gmm_map_param_.gmm_map_file + "map/";
    std::string vs_path = gmm_map_param_.gmm_map_file + "../gmm_view_result/";
    std::string vs_init_path = vs_path + "init_map/";
    std::string vs_map_path = vs_path + "map/";
    auto r = gb_io_->FindFileInfolder(path.c_str(), "txt", files);
    if (0 != access(vs_path.c_str(), 0)) {
        mkdir(vs_path.c_str(), 0777);
    }
    if (0 != access(vs_init_path.c_str(), 0)) {
        mkdir(vs_init_path.c_str(), 0777);
    }
    if (0 != access(vs_map_path.c_str(), 0)) {
        mkdir(vs_map_path.c_str(), 0777);
    }
    if (r < 1) {
        LOG(ERROR) << " files is empty!";
        return -1;
    }
    for (size_t t = 0; t < files.size(); t++) {
        std::string gmm_offset = path + files.at(t);
        std::string binfile;
        std::ifstream infile;
        int rows, cols, map_ar_size;
        MapTa bina;
        MapTr binr;
        infile.open(gmm_offset);
        if (!infile.is_open()) {
            LOG(ERROR) << "failed to open :" << gmm_offset;
            return -3;
        }
        infile.seekg(0, infile.end);
        if (0 == infile.tellg()) {
            LOG(ERROR) << gmm_offset << " file size " << infile.tellg() << "= 0";
            infile.close();
            return -4;
        }
        infile.seekg(0, infile.beg);
        infile >> rows >> cols >> map_ar_size;
        LOG(INFO) << "rows: " << rows << ", cols: " << cols << ", size: " << map_ar_size;
        infile.close();
        bina.resize(rows, cols);
        binr.resize(rows, cols);
        std::string file = (std::string)files.at(t);
        for (size_t i = 0; i < 13; i++) file.pop_back();
        LOG(INFO) << file;
        if (gb_io_->ReadTilesBin(path + file + "ar.bin", map_ar_size, bina, binr) < 0) return -1;
        gb_io_->Visualize(vs_map_path + file, gmm_map_param_.cell_size, bina, binr, true, false);
    }
    path = gmm_map_param_.gmm_map_file + "init_map/";
    files.clear();
    r = gb_io_->FindFileInfolder(path.c_str(), "txt", files);
    if (r < 1) {
        LOG(ERROR) << " init files is empty!";
        return -1;
    }
    for (size_t t = 0; t < files.size(); t++) {
        std::string gmm_offset = path + files.at(t);
        std::string binfile;
        std::ifstream infile;
        int rows, cols, map_ar_size;
        MapTa bina;
        MapTr binr;
        infile.open(gmm_offset);
        if (!infile.is_open()) {
            LOG(ERROR) << "failed to open :" << gmm_offset;
            return -3;
        }
        infile.seekg(0, infile.end);
        if (0 == infile.tellg()) {
            LOG(ERROR) << gmm_offset << " file size " << infile.tellg() << "= 0";
            infile.close();
            return -4;
        }
        infile.seekg(0, infile.beg);
        infile >> rows >> cols >> map_ar_size;
        LOG(INFO) << "rows: " << rows << ", cols: " << cols << ", size: " << map_ar_size;
        infile.close();
        bina.resize(rows, cols);
        binr.resize(rows, cols);
        std::string file = (std::string)files.at(t);
        for (size_t i = 0; i < 13; i++) file.pop_back();
        LOG(INFO) << file;
        if (gb_io_->ReadTilesBin(path + file + "ar.bin", map_ar_size, bina, binr) < 0) return -1;
        gb_io_->Visualize(vs_init_path + file, gmm_map_param_.init_cell_size, bina, binr, true, true);
    }
    return 0;
}

bool TilesGmmMapGenerator::GenerateReport(std::string &report, bool verbose) {
    report = report_;
    return true;
}

}  // namespace mapping::tools

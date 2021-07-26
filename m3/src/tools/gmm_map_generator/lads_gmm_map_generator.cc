//
// Created by pengguoqi on 20-02-25.
//

#include "tools/gmm_map_generator/lads_gmm_map_generator.h"
#include <glog/logging.h>

namespace mapping::tools {

using namespace cv;
using namespace cv::ml;
using namespace mapping::common;

LADSGmmMapGenerator::~LADSGmmMapGenerator() {
    gb_io_ = nullptr;
    std::unordered_map<unsigned long long, Cell>().swap(map_cells_);
}

int LADSGmmMapGenerator::Init() {
    if (LoadRawData() < 0) {
        LOG(ERROR) << "failed to LoadRawData : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to Init.\n";
        return -1;
    }
    return 0;
}

int LADSGmmMapGenerator::Start() {
    std::string binpath = gmm_map_param_.gmm_map_file + "map_ar.bin";
    resolution_ = gmm_map_param_.cell_size;
    if (Build() < 0) {
        LOG(ERROR) << "failed to gmm map build ";
        return -1;
    }
    if (WriteMap(binpath) < 0) {
        LOG(ERROR) << "failed to write map_ar ";
        return -2;
    }
    size1_ = map_cells_.size();
    invalid_num_1_ = invalid_num_;
    resolution_ = gmm_map_param_.init_cell_size;
    binpath = gmm_map_param_.gmm_map_file + "init_ar.bin";
    if (Build() < 0) {
        LOG(ERROR) << "failed to init map build ";
        return -3;
    }
    if (WriteMap(binpath) < 0) {
        LOG(ERROR) << "failed to write init_ar ";
        return -4;
    }
    size2_ = map_cells_.size();
    invalid_num_2_ = invalid_num_;
    if (WriteOffset() < 0) {
        LOG(ERROR) << "failed to Write offset ";
        return -5;
    }
    if (WriteGroud() < 0) {
        LOG(ERROR) << "failed to Write groud ";
        return -6;
    }
    if (DEBUG) ViewResult();
    ZipGmm();
    return 0;
}

void LADSGmmMapGenerator::ZipGmm() {
    LOG(INFO) << "Zip....... ";
    std::string command = "zip -jr " + gmm_map_param_.gmm_map_file + "../gmm.zip " + gmm_map_param_.gmm_map_file +
                          " map_ar.bin" + " init_ar.bin" + " groud.bin" + " offset.txt";
    system((char*)command.c_str());
    command = "rm -rf " + gmm_map_param_.gmm_map_file;
    system((char*)command.c_str());
}

int LADSGmmMapGenerator::LoadRawData() {
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

int LADSGmmMapGenerator::Build() {
    LOG(INFO) << "hashmap...";
    gb_io_->SetResolution(resolution_);
    gb_io_->ClearHashRef();
    map_cells_.clear();
    int r = gb_io_->HandlingPCD(gmm_map_param_.db_file);
    if (r < 0) {
        LOG(ERROR) << "failed to handling PCD : " << gmm_map_param_.db_file;
        report_ += "GmmMapGenerator error : failed to handling PCD : " + gmm_map_param_.db_file + ".\n";
        gb_io_ = nullptr;
        return -1;
    }
    LOG(INFO) << "emtrain...";
    EmTrain();
    return 0;
}

void LADSGmmMapGenerator::EmTrain() {
    auto& hcs = gb_io_->GetHashRef();
    size_t size = hcs.size();
    if (size < 0) return;
    int num = 0;
    LOG(INFO) << "train start : size " << size;
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
    LOG(INFO) << "train end";
}

bool LADSGmmMapGenerator::Train1(std::vector<AR>& ars, Cell& cell) {
    int size = ars.size();
    if (size == 0) return false;
    cell.size = size;
    if (size < 6) return false;
    cv::Mat mat_a = cv::Mat(size, 1, CV_32FC1);
    cv::Mat mat_r = cv::Mat(size, 1, CV_32FC1);
    for (int i = 0; i < size; ++i) {
        mat_a.at<float>(i, 0) = ars[i].a;
        mat_r.at<float>(i, 0) = ars[i].r;
    }
    cv::Ptr<cv::ml::EM> em = cv::ml::EM::create();
    em->setClustersNumber(2);
    em->setCovarianceMatrixType(cv::ml::EM::COV_MAT_GENERIC);
    em->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-2));
    if (!em->trainEM(mat_a)) return false;
    {
        std::vector<cv::Mat> covs_vec;
        em->getCovs(covs_vec);
        int w1 = em->getWeights().at<double>(0) * 100;
        int u1 = em->getMeans().at<double>(0) * 100;
        int s1 = sqrt(covs_vec[0].at<double>(0, 0)) * 100;
        int w2 = em->getWeights().at<double>(1) * 100;
        int u2 = em->getMeans().at<double>(1) * 100;
        int s2 = sqrt(covs_vec[1].at<double>(0, 0)) * 100;
        auto& ca = cell.cell.cella;
        ca.w1 = To16(w1);
        ca.u1 = To16(u1);
        ca.s1 = To16(s1);
        ca.w2 = To16(w2);
        ca.u2 = To16(u2);
        ca.s2 = To16(s2);
    }
    if (!em->trainEM(mat_r)) return false;
    {
        std::vector<cv::Mat> covs_vec;
        em->getCovs(covs_vec);
        int w1 = em->getWeights().at<double>(0) * 100;
        int u1 = em->getMeans().at<double>(0);
        int s1 = sqrt(covs_vec[0].at<double>(0, 0));
        int w2 = em->getWeights().at<double>(1) * 100;
        int u2 = em->getMeans().at<double>(1);
        int s2 = sqrt(covs_vec[1].at<double>(0, 0));
        auto& cr = cell.cell.cellr;
        cr.w1 = To16(w1);
        cr.u1 = To8(u1);
        cr.s1 = To8(s1);
        cr.w2 = To16(w2);
        cr.u2 = To8(u2);
        cr.s2 = To8(s2);
    }
    return true;
}

bool LADSGmmMapGenerator::Train2(std::vector<AR>& ars, Cell& cell) {
    int size = ars.size();
    if (size == 0) return false;
    cell.size = size;
    if (size < 6) return false;
    Descriptor d1, d2;
    float cut = cell.ground + 2;
    for (int i = 0; i < size; ++i) {
        if (ars[i].a < cut) d1.AddSample(ars[i].a, ars[i].r);
        else
            d2.AddSample(ars[i].a, ars[i].r);
    }
    auto& ca = cell.cell.cella;
    auto& cr = cell.cell.cellr;
    if (d1.count > 1) {
        ca.w1 = To16(50);
        ca.u1 = To16(d1.altitude * 100);
        ca.s1 = To16(d1.altitude_var * 100);
        cr.w1 = To16(50);
        cr.u1 = To16(d1.intensity);
        cr.s1 = To16(d1.intensity_var);
    }
    if (d2.count > 1) {
        ca.w2 = To16(50);
        ca.u2 = To16(d2.altitude * 100);
        ca.s2 = To16(d2.altitude_var * 100);
        cr.w2 = To16(50);
        cr.u2 = To16(d2.intensity);
        cr.s2 = To16(d2.intensity_var);
    }
    if (d1.count <= 1) {
        ca.w1 = cr.w1 = To16(0);
        ca.w2 = cr.w2 = To16(100);
    }
    if (d2.count <= 1) {
        ca.w1 = cr.w1 = To16(100);
        ca.w2 = cr.w2 = To16(0);
    }
    return true;
}

bool LADSGmmMapGenerator::Train3(std::vector<AR>& ars, Cell& cell) {
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
    auto& ca = cell.cell.cella;
    auto& cr = cell.cell.cellr;
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

int LADSGmmMapGenerator::WriteMap(const std::string& file) {
    int rows = (map_range_.maxy - map_range_.miny) / resolution_ + 1;
    int cols = (map_range_.maxx - map_range_.minx) / resolution_ + 1;
    invalid_num_ = 0;
    LOG(INFO) << "rows: " << rows << " ,cols:" << cols;
    LOG(INFO) << "all cells: " << rows * cols << " , effective cells:" << map_cells_.size()
              << " ,ratio:" << map_cells_.size() / float(rows * cols);
    std::vector<MapCell<short int>> map;
    map.reserve(rows * cols);
    for (auto& pair : map_cells_) {
      auto& m = pair.second;
      if (m.cell.row < 0 || m.cell.row >= rows || m.cell.col < 0 ||
          m.cell.col >= cols) {
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
    int size = map.size() * sizeof(MapCell<short int>);
    LOG(INFO) << " map size(bin) : " << size;
    if (WriteBin(file, map.data(), size) < 0) return -1;
    else
        return 0;
}

int LADSGmmMapGenerator::WriteBin(const std::string& filename, const void* data, const int& size) {
    std::ofstream os;
    os.open(filename.c_str(), std::ios::binary);
    if (!os.is_open()) {
        LOG(ERROR) << "failed to open : " << filename;
        report_ += "GmmMapGenerator error : " + filename + " open failed.\n";
        return -1;
    }
    os.write((char*)data, size);
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

int LADSGmmMapGenerator::WriteOffset() {
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

    write_offset << rows1 << " " << cols1 << " " << minx << " " << miny << " "
                 << rows2 << " " << cols2 << " "
                 << score1 << " " << score2 << " " << distance
                 << " " << size1_ - invalid_num_1_ << " " << size2_ - invalid_num_2_ << std::endl;

    if (write_offset.bad()) {
        LOG(ERROR) << "offset write bad ";
        report_ += "GmmMapGenerator error : offset write bad.\n";
        write_offset.close();
        return -1;
    }
    write_offset.close();
    return 0;
}

int LADSGmmMapGenerator::ViewResult() {
    std::string gmm_offset = gmm_map_param_.gmm_map_file + "offset.txt";
    std::string vs_path = gmm_map_param_.gmm_map_file + "../gmm_view_result/";
    std::ifstream infile;
    int rows, cols, rows1, cols1, map_ar_size, map_ar1_size, gmm_offset_threshold;
    float map_min_x, map_min_y, gmm_score_threshold, gmm_score_threshold33;

    if (0 != access(vs_path.c_str(), 0)) {
        mkdir(vs_path.c_str(), 0777);
    }
    infile.open(gmm_offset);
    if (!infile.is_open()) {
        LOG(ERROR) << "failed to open :" << gmm_offset;
        return -3;
    }
    infile.seekg(0, infile.end);
    if (0 == infile.tellg()) {
        LOG(ERROR) << gmm_offset << " file size " << infile.tellg() << "=0";
        infile.close();
        return -4;
    }
    infile.seekg(0, infile.beg);
    infile >> rows >> cols >> map_min_x >> map_min_y >> rows1 >> cols1 >> gmm_score_threshold >>
        gmm_score_threshold33 >> gmm_offset_threshold >> map_ar_size >> map_ar1_size;
    infile.close();
    bina_.resize(rows, cols);
    binr_.resize(rows, cols);
    if (gb_io_->ReadBin(gmm_map_param_.gmm_map_file + "map_ar.bin", map_ar_size, bina_, binr_) < 0) return -1;
    gb_io_->Visualize(vs_path + "map_", gmm_map_param_.cell_size, bina_, binr_);
    bina2_.resize(rows1, cols1);
    binr2_.resize(rows1, cols1);
    if (gb_io_->ReadBin(gmm_map_param_.gmm_map_file + "init_ar.bin", map_ar1_size, bina2_, binr2_) < 0) return -2;
    gb_io_->Visualize(vs_path + "init_", gmm_map_param_.init_cell_size, bina2_, binr2_);
}

int LADSGmmMapGenerator::WriteGroud() {
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

bool LADSGmmMapGenerator::GenerateReport(std::string& report, bool verbose) {
    report = report_;
    return true;
}

}  // namespace mapping::tools

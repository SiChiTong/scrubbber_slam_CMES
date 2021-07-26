//
// Created by gaoxiang on 2020/9/28.
//

#include "pipeline/check_out/check_out.h"
#include "common/mapping_math.h"
#include "common/track_pose.h"
#include "core/coordinate_transform/gps_trans.h"
#include "io/db_io.h"
#include "io/file_io.h"
#include "pipeline/check_in/file_filter.h"

#include <glog/logging.h>
#include <rosbag/view.h>

namespace mapping::pipeline {

template <typename T>
bool ParseFunctionPointFromAppMsg(const rosbag::MessageInstance &m, common::FunctionPoint &fp) {
    auto msg = m.instantiate<T>();
    if (msg == nullptr) {
        return false;
    }

    fp = common::CreateFromAppMsg<boost::shared_ptr<T>>(msg);
    if (!fp.valid) {
        return false;
    }

    fp.timestamp = m.getTime().toSec();
    return true;
}

CheckOut::CheckOut(const io::YAML_IO &yaml_file, RunMode run_mode) : yaml_(yaml_file), PipelineContext(run_mode) {
    context_name_ = "CheckOut";
    local_data_path_ = yaml_.GetValue<std::string>("data_fetching", "local_data_path");
}

CheckOut::~CheckOut() { LOG(INFO) << "check out deconstructed."; }

bool CheckOut::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    local_data_path_ = yaml_.GetValue<std::string>("data_fetching", "local_data_path");
    local_db_path_ = yaml_.GetValue<std::string>("local_db_path");

    enable_map_splitter_ = yaml_.GetValue<bool>("map_splitter", "enable_map_splitter");

    origin_info_.map_origin_x = yaml_.GetValue<double>("map_origin_param", "map_origin_x");
    origin_info_.map_origin_y = yaml_.GetValue<double>("map_origin_param", "map_origin_y");
    origin_info_.map_origin_z = yaml_.GetValue<double>("map_origin_param", "map_origin_z");
    origin_info_.map_origin_zone = yaml_.GetValue<int>("map_origin_param", "map_origin_zone");
    origin_info_.is_southern = yaml_.GetValue<bool>("map_origin_param", "is_southern");

    double ant_x = yaml_.GetValue<double>("gps_params", "ant_x");
    double ant_y = yaml_.GetValue<double>("gps_params", "ant_y");
    double ant_angle = yaml_.GetValue<double>("gps_params", "ant_angle");
    gmm_map_ = yaml_.GetValue<int>("simulation_param", "gmm_map");  // for gmm_map

    gps_trans_ = std::make_shared<core::GpsTransform>(
        V3d(origin_info_.map_origin_x, origin_info_.map_origin_y, origin_info_.map_origin_z), ant_x, ant_y, ant_angle);

    if (!io::LoadTrajectoryBagName(local_data_path_ + "trajectory_info.txt", trajectory_map_, false)) {
        LOG(ERROR) << "failed to load trajectory_info.txt";
        return false;
    }

    if (!io::LoadKeyframes(local_data_path_ + "keyframes.txt", keyframes_)) {
        LOG(ERROR) << "cannot load keyframes";
        return false;
    }

    for (auto &kfp : keyframes_) {
        for (auto &kf : kfp.second) {
            keyframes_map_.insert({kf->timestamp_, kf});
        }
    }

    LOG(INFO) << "trajectories: " << trajectory_map_.size() << ", keyframes: " << keyframes_map_.size();

    if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
        db_io_ = std::make_shared<io::DB_IO>(local_data_path_ + "map.db");
        scan_builder_ = std::make_shared<ScanBuilder>(local_data_path_, report_);
    } else {
        db_io_ = std::make_shared<io::DB_IO>(local_db_path_ + "map.db");
        scan_builder_ = std::make_shared<ScanBuilder>(local_db_path_, report_);
    }

    // load car type
    std::string car_type = yaml_.GetValue<std::string>("car_type");
    if (car_type == "wxx") {
        car_type_ = common::CarType::WXX;
    } else if (car_type == "lads") {
        car_type_ = common::CarType::LADS;
    } else {
        car_type_ = common::CarType::OTHERS;
        LOG(ERROR) << "Unknown car type: " << car_type;
        return false;
    }

    if (run_mode_ == RunMode::PIPELINE && car_type != "lads") {
        io::LoadLoopCandidatesScore(local_data_path_ + "loops_score.txt", ndt_data_);
    }

    return true;
}

bool CheckOut::Start() {
    SetStatus(ContextStatus::WORKING);

    if (car_type_ == common::CarType::WXX) {
        CheckOutForWXX();
    } else if (car_type_ == common::CarType::LADS) {
        CheckOutForLADS();
    }

    LOG(INFO) << "Saving origin info";
    SaveOriginInfo();

    LOG(INFO) << "Splitting submap";
    SpliteSubMap();

    LOG(INFO) << "Saving GMM map";
    SaveGmmMap();

    LOG(INFO) << "Making figures";
    MakeFigures();
    task_info_.checkout_passed = true;

    SetStatus(ContextStatus::SUCCEED);

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        yaml_.SetValue("task_info", "checkout_passed", task_info_.checkout_passed);
        yaml_.Save();

        SaveGemReport(local_data_path_ + gem_report_name_);
    }

    return true;
}

bool CheckOut::CheckOutForWXX() {
    for (auto &tp : trajectory_map_) {
        ParseTrajectory(tp.second);
    }

    LOG(INFO) << "Saving scan context";
    if (!SaveScanContext()) {
        LOG(ERROR) << "write scan context failed!!";
    }

    LOG(INFO) << "Saving Function points";
    SaveFunctionPoints();

    if (run_mode_ == RunMode::PIPELINE) {
        LOG(INFO) << "Saving tracks";
        SaveTracks();
    }

    return true;
}

bool CheckOut::CheckOutForLADS() { return true; }

void CheckOut::ParseTrajectory(std::shared_ptr<common::Trajectory> trajectory) {
    for (auto &bag_pair : trajectory->bag_files) {
        LOG(INFO) << "parsing " << bag_pair.second;
        rosbag::Bag bag;
        bag.open(bag_pair.second, rosbag::bagmode::Read);
        LOG(INFO) << "parsing function points";
        for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
            common::FunctionPoint fp;
            if (ParseFunctionPointFromAppMsg<AppWxbMsg>(m, fp) || ParseFunctionPointFromAppMsg<AppWbdMsg>(m, fp)) {
                AddFunctionPoint(std::move(fp));
            }

            if (!trajectory->is_mapping_bag_ && ParseImageData(m)) {
                num_images_loaded_++;
            }
        }

        LOG(INFO) << "saving image data";
        std::vector<io::DB_IO::ImageData> image_data;
        std::set<int> save_image_index;

        /// 由于图片频率更高，所以在保存时尽量不要使用同一张图像
        for (auto &img_msg : image_msgs_) {
            double timestamp = img_msg->header.stamp.toSec();
            SE3 pose;
            common::KFPtr best_match = nullptr;
            if (common::PoseInterp<common::KFPtr>(
                    timestamp, keyframes_map_,
                    [](const common::KFPtr &kf) -> SE3 { return kf->optimized_pose_stage_2_; }, pose, best_match)) {
                if (save_image_index.find(best_match->id_) == save_image_index.end()) {
                    io::DB_IO::ImageData data{};
                    data.id = best_match->id_;
                    data.data = (void *)&img_msg->data[0];
                    data.length = sizeof(uint8_t) * img_msg->data.size();
                    image_data.push_back(data);
                    save_image_index.insert(data.id);
                }
            }
        }

        LOG(INFO) << "write image to db";
        db_io_->WriteImageToDB(image_data);
        LOG(INFO) << "saved image data: " << image_data.size();
        image_msgs_.clear();
    }
}

void CheckOut::MakeFigures() {
    LOG(INFO) << "making figures";
    std::string cmd;
    if (run_mode_ == RunMode::PIPELINE) {
        std::string keyframes_path_out = "./scripts/keyframes_path_out.py";
        if (access(keyframes_path_out.c_str(), F_OK) == 0) {
            cmd = "python3 scripts/keyframes_path_out.py " + local_data_path_;
        } else {
            cmd = "python scripts/keyframes_path_out.pyc " + local_data_path_;
        }
    } else if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        cmd = "python3 ext/mapping/scripts/keyframes_path_out.py " + local_data_path_;
    }
    system(cmd.c_str());

    if (run_mode_ == RunMode::PIPELINE) {
        cmd = "python3 scripts/plot_scattered_path_out.py " + local_data_path_ + "gps_path.txt " + local_data_path_ +
              "gps-path";
        std::string plot_scattered_path_out = "./scripts/plot_scattered_path_out.py";
        if (access(plot_scattered_path_out.c_str(), F_OK) == 0) {
            cmd = "python3 scripts/plot_scattered_path_out.py " + local_data_path_ + "gps_path.txt " +
                  local_data_path_ + "gps-path";
        } else {
            cmd = "python3 scripts/plot_scattered_path_out.pyc " + local_data_path_ + "gps_path.txt " +
                  local_data_path_ + "gps-path";
        }
    } else if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        cmd = "python3 ext/mapping/scripts/plot_scattered_path_out.py " + local_data_path_ + "gps_path.txt " +
              local_data_path_ + "gps-path";
    }
    system(cmd.c_str());
}

void CheckOut::SaveTracks() {
    LOG(INFO) << " RunForNdtFilter start ";
    std::shared_ptr<FilterNdtScore> filter_ndt_(new FilterNdtScore());
    filter_ndt_->SetNdtScoreData(ndt_data_);
    filter_ndt_->RunFilter();
    std::vector<std::tuple<float, float, float>> ndt_filter;  // xg, yg, theshold
    ndt_filter = filter_ndt_->GetNdtScoreFilter();
    float ndt_threshold = 0;
    float last_ndt_threshold = 0;
    if (ndt_filter.size() < 1) {
        LOG(ERROR) << "no data for ndt filter!";
        return;
    }
    std::vector<std::tuple<float, float, float>> ndt_filter_seg;
    for (size_t i = 0; i < ndt_filter.size(); ++i) {
        ndt_threshold = std::get<2>(ndt_filter[i]);
        if (i == 0) {
            last_ndt_threshold = std::get<2>(ndt_filter[0]);
        } else if (ndt_threshold != last_ndt_threshold) {
            ndt_filter_seg.push_back(
                std::make_tuple(std::get<0>(ndt_filter[i - 1]), std::get<1>(ndt_filter[i - 1]), last_ndt_threshold));
            last_ndt_threshold = ndt_threshold;
        }
        if (i == (ndt_filter.size() - 1)) {
            ndt_filter_seg.push_back(
                std::make_tuple(std::get<0>(ndt_filter[i]), std::get<1>(ndt_filter[i]), last_ndt_threshold));
        }
    }

    if (ndt_filter_seg.size() < 1) {
        LOG(ERROR) << "ndt_filter_seg is null!";
        return;
    }
    // convert keyframes to tracks
    std::vector<common::TrackPoseD> tracks;
    if (ndt_filter_seg.size() == 1) {
        ndt_threshold = std::get<2>(ndt_filter_seg[0]);
        for (auto &kfp : keyframes_map_) {
            if (kfp.second->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
                // 不导入建图用的
                continue;
            }
            auto re = core::GpsTransform::UtmXYToLatLon(
                kfp.second->optimized_pose_stage_2_.translation()[0] + origin_info_.map_origin_x,
                kfp.second->optimized_pose_stage_2_.translation()[1] + origin_info_.map_origin_y,
                origin_info_.map_origin_zone, origin_info_.is_southern);

            double lat = re[0];
            double lon = re[1];

            tracks.emplace_back(kfp.second->id_, V3d(lon, lat, kfp.second->optimized_pose_stage_2_.translation()[2]),
                                ndt_threshold);
        }
    } else {
        size_t theshold_index = 0;
        float theshold_x = std::get<0>(ndt_filter_seg[0]);
        float theshold_y = std::get<1>(ndt_filter_seg[0]);
        ndt_threshold = std::get<2>(ndt_filter_seg[0]);

        for (auto &kfp : keyframes_map_) {
            if (kfp.second->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
                // 不导入建图用的
                continue;
            }
            float current_x = kfp.second->optimized_pose_stage_2_.translation()[0];
            float current_y = kfp.second->optimized_pose_stage_2_.translation()[1];
            float delta_d = (current_x - theshold_x) * (current_x - theshold_x) +
                            (current_y - theshold_y) * (current_y - theshold_y);
            if (delta_d < 0.001) {
                theshold_index++;
                if (theshold_index == ndt_filter_seg.size()) theshold_index = ndt_filter_seg.size() - 1;
                theshold_x = std::get<0>(ndt_filter_seg[theshold_index]);
                theshold_y = std::get<1>(ndt_filter_seg[theshold_index]);
                ndt_threshold = std::get<2>(ndt_filter_seg[theshold_index]);
            }
            auto re = core::GpsTransform::UtmXYToLatLon(
                kfp.second->optimized_pose_stage_2_.translation()[0] + origin_info_.map_origin_x,
                kfp.second->optimized_pose_stage_2_.translation()[1] + origin_info_.map_origin_y,
                origin_info_.map_origin_zone, origin_info_.is_southern);

            double lat = re[0];
            double lon = re[1];

            tracks.emplace_back(kfp.second->id_, V3d(lon, lat, kfp.second->optimized_pose_stage_2_.translation()[2]),
                                ndt_threshold);
        }
    }
    db_io_->WriteTrackToDB(tracks);
}

void CheckOut::AddFunctionPoint(common::FunctionPoint fp) {
    assert(fp.valid);
    // find pose of this function point
    SE3 pose;
    common::KFPtr best_match;
    if (!common::PoseInterp<common::KFPtr>(
            fp.timestamp, keyframes_map_, [](const common::KFPtr &kf) -> SE3 { return kf->optimized_pose_stage_2_; },
            pose, best_match)) {
        LOG(ERROR) << "cannot find pose for function point " << fp.id;
        return;
    }

    V3d t = pose.translation();
    V2d latlon = core::GpsTransform::UtmXYToLatLon(t[0] + origin_info_.map_origin_x, t[1] + origin_info_.map_origin_y,
                                                   origin_info_.map_origin_zone, origin_info_.is_southern);
    fp.pose[0] = latlon[0];
    fp.pose[1] = latlon[1];
    fp.pose[2] = t[2];
    fp.heading = common::SE3ToRollPitchYaw(pose).yaw * 180 / M_PI;  // yaw

    // find id
    if (func_id_to_number.find(fp.id) == func_id_to_number.end()) {
        func_id_to_number.insert({fp.id, 1});
        fp.number = 1;
    } else {
        fp.number = func_id_to_number[fp.id] + 1;
        func_id_to_number[fp.id]++;
    }

    function_points_.emplace_back(std::move(fp));
}

bool CheckOut::SpliteSubMap() {
    if (!enable_map_splitter_) {
        report_ += "不需要进行子地图切分.\n";
        return true;
    }
    std::shared_ptr<tools::MapSplitter> ms 
            = std::make_shared<tools::MapSplitter>(yaml_);
    if (ms->Init() < 0) {
        report_ += R"(\textcolor[rgb]{0.667,0.0,0.0}{错误 : 子地图切割初始化失败.})";
        report_ += "\n ";
        LOG(ERROR) << " Init() is error.";
        return false;
    }
    if (ms->Start() < 0) {
        report_ += R"(\textcolor[rgb]{0.667,0.0,0.0}{错误 : 子地图切割失败.})";
        report_ += "\n ";
        LOG(ERROR) << " Start() is error.";
        return false;
    }
    return true;
}

bool CheckOut::SaveGmmMap() {
    if (0 == gmm_map_) {
        LOG(WARNING) << "no need to save gmm map";
        report_ += "Warning : no need to save gmm map.\n";
        return true;
    }

    tools::GmmMapParam mp;
    if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
        mp.db_file = local_data_path_ + "map.db";
    } else {
        mp.db_file = local_db_path_ + "map.db";
    }

    mp.gmm_map_file = local_data_path_ + "gmm/";
    std::shared_ptr<tools::GmmMapGenerator> gmm_map_generator;

    std::string path = mp.gmm_map_file;
    if (0 != access(path.c_str(), 0)) {
        mkdir(path.c_str(), 0777);
    }

    if (1 == gmm_map_) {
        mp.cell_size = 0.128;
        mp.init_cell_size = 2.048;
        gmm_map_generator = std::make_shared<tools::IDPGmmMapGenerator>(mp);
    } else if (2 == gmm_map_) {
        mp.cell_size = 0.15;
        mp.init_cell_size = 2.1;
        gmm_map_generator = std::make_shared<tools::LADSGmmMapGenerator>(mp);
    } else if (3 == gmm_map_) {
        mp.cell_size = 0.15;
        mp.init_cell_size = 2.1;
        gmm_map_generator = std::make_shared<tools::TilesGmmMapGenerator>(mp);
    } else {
        LOG(ERROR) << "gmm param is error.";
        report_ += R"(\textcolor[rgb]{0.667,0.0,0.0}{Error : gmm param is error.})";
        report_ += "\n ";
        return false;
    }

    if (gmm_map_generator->Init() < 0) {
        std::string temp;
        gmm_map_generator->GenerateReport(temp);
        report_ += temp;
        return false;
    }
    if (gmm_map_generator->Start() < 0) {
        std::string temp;
        gmm_map_generator->GenerateReport(temp);
        report_ += temp;
        return false;
    }
    gmm_map_generator = nullptr;
    return true;
}

bool CheckOut::GenerateReport(std::string &report, bool verbose) {
    report_ += report;
    return true;
}

void CheckOut::SaveFunctionPoints() {
    LOG(INFO) << "saving function points: " << function_points_.size();
    db_io_->WriteFunctionPointsToDB(function_points_);
}

bool CheckOut::FillTaskInfo(TaskInfo &info) {
    info.checkout_passed = task_info_.checkout_passed;
    return true;
}

bool CheckOut::Save() { return true; }

bool CheckOut::Load() { return true; }

bool CheckOut::ParseImageData(const rosbag::MessageInstance &m) {
    // 解析图像
    auto img_msg = m.instantiate<ImageMsg>();
    if (img_msg == nullptr) {
        return false;
    }
    image_msgs_.push_back(img_msg);

    return true;
}

void CheckOut::SaveOriginInfo() { db_io_->WriteOriginPointInformationToDB(origin_info_); }

bool CheckOut::SaveScanContext() {
    if (!ScanContextInitial()) {
        LOG(ERROR) << " failed to ScanContextInitial!!! ";
        report_ += "Warning : failed to scancontext initial.\n";
    }

    scan_contexts_.clear();
    for (size_t i = 0; i < scan_context_data_.size(); ++i) {
        int x = scan_context_data_.at(i).loc_pos.x;
        int y = scan_context_data_.at(i).loc_pos.y;
        int size = sizeof(core::ScanContextWithPose);
        scan_contexts_.push_back(io::DB_IO::ScanContextData(x, y, &(scan_context_data_.at(i)), size));
    }
    if (scan_contexts_.size() < 1) {
        LOG(ERROR) << "scan context size less than 1.";
        report_ += R"(\textcolor[rgb]{0.667,0.0,0.0}{Error : scan context size less than 1 })";
        report_ += "\n";
        return false;
    }
    bool re = db_io_->WriteScanContextToDB(scan_contexts_);
    if (!re) {
        LOG(ERROR) << "write scan context failed!!";
        report_ += R"(\textcolor[rgb]{0.667,0.0,0.0}An error occurred while saving the initialization data. })";
        report_ += "\n";
        return false;
    }
    return true;
}

bool CheckOut::ScanContextInitial() {
    scan_builder_->Start();

    auto scan_context = scan_builder_->GetScanContextfiles();

    SetInitialFiles(scan_context);

    return true;
}

}  // namespace mapping::pipeline

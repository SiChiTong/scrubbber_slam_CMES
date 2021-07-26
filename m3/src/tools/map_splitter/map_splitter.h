//
// Created by pengguoqi on 21-2-24.
//

#ifndef MAP_SPLITTER_H
#define MAP_SPLITTER_H

#include "map_splitter_base.h"

namespace mapping::tools {

/**
 * 子地图切割功能实现
 */

class MapSplitter : public MapSplitterBase {
   public:
    explicit MapSplitter(const io::YAML_IO &yaml_file);

    virtual ~MapSplitter() override;

    virtual bool Init() override;

    virtual bool Start() override;

   private:
   	void SplitKeyFrames();

   	void SplitMapDB();

   	int GenerateTopology(std::map<IdType, TOPOS>& topos, const bool& model);

   	double ComputeDistance(const PointAttributes& first, 
                           const PointAttributes& second,
                           bool model = true);

   	IdTypeKFs ComputeApproachPoints(const PointAttributes& point);

    bool IsSameLevel(const PointAttributes& point, const IdTypeKFs_map& ids, double resolution = 0.2);

   	void OptimizationSubmap1(std::map<IdType, IdTypeKFs_map>& result_map_id);

    void OptimizationSubmap2(std::map<IdType, IdTypeKFs_map>& result_map_id);

    void DealWithSubmap(bool &same_level, const IdType& begin, const IdType& end, 
                        const IdType& push_map_no, const IdType& pop_map_no,
                        std::map<IdType, IdTypeKFs_map>& result_map_id,
                        double resolution = 0.2);

    void UpdateSubMap(const IdType& begin, const IdType& end, 
                      const IdType& push_map_no, const IdType& pop_map_no,
                      std::map<IdType, IdTypeKFs_map>& result_map_id);

    void CheckSubMap(std::map<IdType, IdTypeKFs_map>& result_map_id);

    void UpdateSubMapResultID(std::map<IdType, IdTypeKFs_map>& result_map_id);

    void SaveTopology(const std::map<IdType, TOPOS>& topos);

    void ShowMCSINFO(const MCS& mcs);


   private:
    std::deque<PointAttributes> open_kfs_;
    std::vector<PointAttributes> ref_kfs_;

    std::map<IdType, PointAttributes> mapid_and_pointid_list_;
    IdTypeKFs close_kfs_;

    std::string test_info_path_;

    bool split_kf_ = false;
    bool size_limit_ = true;
};

}  // namespace mapping::tools
#endif  // MAP_SPLITTER_H

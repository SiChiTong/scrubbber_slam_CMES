// lbh  creat 2019.8.03
#ifndef MAPPING_RESOLUTION_MATCHING_SCORE_BOARD_
#define MAPPING_RESOLUTION_MATCHING_SCORE_BOARD_

#include <Eigen/Core>
#include <memory>

namespace mapping::core {

//栅格遍历过程中的概率矩阵统计工具
class ScoreBoard {
   public:
    struct BestOffset {
        int offset_row = 0, offset_col = 0;
        float offset_rad = 0;
        float score = 0;
    };

    typedef std::shared_ptr<BestOffset> BestOffsetPtr;

   public:
    ScoreBoard() : score_init_(false) {}

    int AddLogp(const Eigen::MatrixXf& logp, float rad);

    std::shared_ptr<BestOffset> GetScoreOffset();

    Eigen::MatrixXf& best_logp() { return best_logp_; }

    Eigen::MatrixXf& best_rad() { return best_rad_; }

   protected:
    void InitBest(int rows, int cols);

   protected:
    Eigen::MatrixXf best_logp_, best_rad_;
    bool score_init_;
};

}  // namespace mapping::core

#endif
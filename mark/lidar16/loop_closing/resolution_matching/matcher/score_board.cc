// lbh  creat 2019.8.03

#include "score_board.h"

namespace mapping { namespace core {

// ScoreBoard
int ScoreBoard::AddLogp(const Eigen::MatrixXf& logp, float rad) {
    if (!score_init_) {
        InitBest(logp.rows(), logp.cols());
        score_init_ = true;
    } else {
        if (logp.rows() != best_logp_.rows() || logp.cols() != best_logp_.cols()) return -1;
    }
    for (int i = 0; i < logp.rows(); ++i)
        for (int j = 0; j < logp.cols(); ++j) {
            if (best_logp_(i, j) < logp(i, j)) {
                best_logp_(i, j) = logp(i, j);
                best_rad_(i, j) = rad;
            }
        }
    return 0;
}

ScoreBoard::BestOffsetPtr ScoreBoard::GetScoreOffset() {
    auto offset = std::make_shared<ScoreBoard::BestOffset>();
    offset->score = best_logp_.maxCoeff(&offset->offset_row, &offset->offset_col);
    offset->offset_rad = best_rad_(offset->offset_row, offset->offset_col);
    offset->offset_row -= best_logp_.rows() / 2;
    offset->offset_col -= best_logp_.cols() / 2;
    return offset;
}

void ScoreBoard::InitBest(int rows, int cols) {
    best_logp_.resize(rows, cols);
    best_rad_.resize(rows, cols);
    best_logp_.setConstant(-1e6);
    best_rad_.setConstant(0);
}

} }  // namespace mapping::core

//
// Created by gaoxiang on 2019/12/18.
//

#include "algorithmutils.h"

namespace HAMO {

std::vector<int> AlgorithmUtils::vectors_intersection(std::vector<int> v1,
                                                      std::vector<int> v2) {
    std::vector<int> v;
    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());
    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                     back_inserter(v));  //求交集
    return v;
}

}  // namespace HAMO
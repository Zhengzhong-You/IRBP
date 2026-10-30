/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#pragma once
#include "irbp/local_search_options.hpp"

namespace RouteOpt::Application::IRBP {
    struct PoolLPInfo {
        bool optimal{};
        double objective{}, seconds{};
        std::vector<std::vector<double>> selection, volume, jrc;
    };
    struct PoolReduction {
        PathPool logical;
        std::vector<char> fixed;
        Json audit;
    };
    PoolReduction reducePool(const Problem &, const Plan &, const PathPool &,
                             const PoolLPInfo &, const LocalSearchOptions &);
    PathPool selectGuidancePaths(const PathPool &, const Plan &, int,
                                const std::function<double(int, const std::vector<int> &)> &);
    std::vector<char> selectRepairDemands(
        const Problem &, const Plan &, const PathPool &, int,
        const std::function<double(int, const std::vector<int> &)> &, Json &);


}

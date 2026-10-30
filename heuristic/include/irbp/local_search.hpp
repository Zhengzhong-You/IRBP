/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#pragma once
#include "irbp/local_search_options.hpp"

namespace RouteOpt::Application::IRBP {
    struct LocalSearchResult {
        Plan plan;
        double objective{}, seconds{};
        Json history = Json::array();
        Json profile;
    };
    LocalSearchResult optimizeCandidatePaths(const Problem &, const Plan &, const PathPool &, const PathPool &,
                                             Budget &, const LocalSearchOptions &, bool path_generation_only = false);
    LocalSearchResult solveLocalSearch(const Problem &, Budget &);
    LocalSearchResult solveLocalSearch(const Problem &, Budget &, const LocalSearchOptions &,
                                      const Plan *incumbent = nullptr, bool construct = true);
} // namespace RouteOpt::Application::IRBP

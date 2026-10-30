/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "irbp/problem.hpp"
namespace RouteOpt::Application::IRBP {
    struct PricedPath {
        double cost = 1e100;
        std::vector<int> edges;
    };
    class Labeling {
        struct Label {
            int node{}, parent{-1}, edge{-1}, next{-1};
            double cost{}, length{};
            bool alive{true};
        };
        std::vector<Label> labels;
        std::vector<std::uint64_t> visited;
        std::vector<int> heads;
        std::size_t limit;

      public:
        explicit Labeling(std::size_t bytes) : limit(bytes) {}
        PricedPath run(const Graph &, int origin, int destination, const std::vector<double> &cost,
                       const std::vector<double> &length, double max_length,
                       const std::vector<double> &completion, const std::vector<char> &allowed, Budget &,
                       Stats &, bool heuristic = false, double threshold = 0,
                       const std::function<void(PricedPath)> &collect = {});
    };
    std::vector<std::vector<std::vector<int>>>
    supportPaths(const Problem &, const Plan &, const std::vector<double> &minimum_unit_cost,
                 const std::vector<std::vector<std::vector<int>>> &candidates, Budget &, Stats &,
                 std::size_t label_bytes, int paths_per_demand);
    std::vector<PricedPath> physicalAlternatives(const Problem &, int block,
                                                 const std::vector<std::vector<int>> &existing, Budget &,
                                                 Stats &, int keep,
                                                 const std::vector<double> &length_weights = {});
    std::vector<double> shortestLengths(const Problem &, int destination, Budget &);
    std::vector<PricedPath> shortestSimplePaths(const Graph &, int origin, int destination,
                                                const std::vector<double> &cost,
                                                const std::vector<char> &allowed, Budget &, int keep,
                                                bool *completed = nullptr);
} // namespace RouteOpt::Application::IRBP

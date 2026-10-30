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
        explicit Labeling(std::size_t bytes) : limit(bytes ? bytes : std::numeric_limits<std::size_t>::max()) {}
        PricedPath run(const Graph &, int origin, int destination, const std::vector<double> &cost,
                       const std::vector<double> &length, double max_length,
                       const std::vector<double> &completion, const std::vector<char> &allowed, Budget &,
                       Stats &, bool heuristic = false, double threshold = 0,
                       const std::function<void(PricedPath)> &collect = {});
    };
    std::vector<double> shortestLengths(const Problem &, int destination, Budget &);
} // namespace RouteOpt::Application::IRBP

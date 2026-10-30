/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace RouteOpt::Application::IRBP {
    using PathPool = std::vector<std::vector<std::vector<int>>>;
    using Json = nlohmann::json;
    constexpr double FeasTol = 1e-7;
    constexpr double RelativeRcTol = 1e-6;
    inline double pricingTolerance(double lp) {
        return RelativeRcTol * std::max(1., std::abs(lp));
    }
    // Objective summation only: feasibility tolerances and improvement rules are unchanged.
    struct CostSum {
        double value{}, correction{};
        void operator+=(double term) {
            double adjusted = term - correction, next = value + adjusted;
            correction = (next - value) - adjusted;
            value = next;
        }
    };
    inline double objectiveTolerance(double a, double b) {
        return std::max(1e-5, 64 * std::numeric_limits<double>::epsilon() *
                                  std::max({1., std::abs(a), std::abs(b)}));
    }
    inline bool objectiveEqual(double a, double b) {
        return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= objectiveTolerance(a, b);
    }
    struct LimitReached : std::runtime_error {
        using std::runtime_error::runtime_error;
    };
    struct Budget {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        double seconds = 60;
        double elapsed() const {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        }
        double remaining() const {
            return std::max(0., seconds - elapsed());
        }
        void check() const {
            if (remaining() <= 0)
                throw LimitReached("time_limit");
        }
    };
    struct Node {
        bool yard{};
        double tracks{}, handling_capacity{}, handling_cost{};
    };
    struct Arc {
        int from{}, to{}, link{};
        double length{}, cost{};
    };
    struct Block {
        int from{}, to{}, slots{};
        double lower{}, upper{}, length_limit{}, opening{}, interchange{}, transport_factor{1};
        std::map<int, double> arc_cost;
    };
    struct Demand {
        int from{}, to{};
        double quantity{}, penalty{};
        std::vector<int> blocks;
        int block_group{-1};
    };
    struct Graph {
        std::vector<int> offsets, edge_ids, from, to;
        void build(int nodes, const std::vector<int> &tails, const std::vector<int> &heads);
        int size() const {
            return static_cast<int>(offsets.size()) - 1;
        }
    };
    struct Problem {
        std::string name, source_json_sha256;
        std::vector<Node> nodes;
        std::vector<Arc> arcs;
        std::vector<double> capacities;
        std::vector<Block> blocks;
        std::vector<Demand> demands;
        std::vector<std::vector<int>> block_demands;
        std::vector<std::vector<int>> compatibility_groups, group_demands;
        Graph block_groups;
        Graph physical, reverse_physical, logical;
        double rejection{};
        double transport(int b, int a) const;
        const std::vector<int> &compatibleBlocks(int k) const {
            const auto &d = demands[k];
            return d.block_group < 0 ? d.blocks : compatibility_groups.at(d.block_group);
        }
        bool compatible(int k, int b) const {
            const auto &bs = compatibleBlocks(k);
            return std::binary_search(bs.begin(), bs.end(), b);
        }
        std::vector<int> compatibleDemands(int b) const;
        void indexCompatibility();
        static Problem read(const std::string &path, double demand_multiplier = 1., Budget *budget = nullptr);
    };
    struct Plan {
        std::vector<std::vector<int>> logical, physical;
        std::vector<double> served;
        static Plan empty(const Problem &p);
        double validate(const Problem &p) const;
        Json json() const;
    };
    struct Stats {
        std::uint64_t labels{}, dominated{}, searches{}, heuristic_searches{}, nodes{};
        std::size_t peak_labels{}, peak_label_bytes{};
        int columns{-1}, rows{-1};
        double solver_seconds{}, build_seconds{}, first_best{}, search_seconds{};
        Json json() const;
    };
    struct Result {
        std::string method, status{"time_limit"};
        double objective{}, bound{}, seconds{};
        Plan plan;
        Stats stats;
        Json json(const Problem &p) const;
    };
    Result solveArc(const Problem &, Budget &, int threads, const Plan *incumbent = nullptr);
} // namespace RouteOpt::Application::IRBP

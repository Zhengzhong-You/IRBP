/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include <algorithm>
#include <cmath>
#include <chrono>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace RouteOpt::Application::IRBP {
    using Json = nlohmann::json;
    constexpr double FeasTol = 1e-7;
    constexpr double RelativeRcTol = 1e-12;
    inline double pricingTolerance(double lp) {
        return RelativeRcTol * std::max(1., std::abs(lp));
    }
    constexpr double IntTol = 1e-6;
    struct LimitReached : std::runtime_error {
        using std::runtime_error::runtime_error;
    };
    struct Progress {
        std::string stage;
        std::uint64_t iteration{}, nodes{}, paths{};
        int columns{-1}, rows{-1};
        double lp{std::numeric_limits<double>::quiet_NaN()}, lower{}, upper{}, seconds{};
        double lp_seconds{}, pricing_seconds{};
    };
    struct BranchingProgress {
        std::uint64_t node{};
        int depth{}, bkf_k{}, lp_requested{}, winner{};
        std::array<int, 3> screened{}, lp{};
        double lp_seconds{}, score{}, r_best{};
    };
    struct PricingProgress {
        std::uint64_t iteration{}, node{}, paths{}, removed{}, regenerated{};
        bool heuristic{};
        int added{}, extra_rows{};
        double lp{}, logical_min{}, physical_min{}, tolerance{}, correction{}, seconds{};
    };
    struct Path;
    struct Budget {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        double seconds = 60;
        std::function<void(const Progress &)> report;
        std::function<void(const BranchingProgress &)> report_testing;
        std::function<void(const PricingProgress &)> report_pricing;
        std::function<void(const Path &)> report_regenerated;
        void emit(Progress p) const {
            if (report) {
                p.seconds = elapsed();
                report(p);
            }
        }
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
    };
    struct Graph {
        std::vector<int> offsets, edge_ids, from, to;
        void build(int nodes, const std::vector<int> &tails, const std::vector<int> &heads);
        int size() const {
            return static_cast<int>(offsets.size()) - 1;
        }
    };
    struct Problem {
        std::string name;
        std::vector<Node> nodes;
        std::vector<Arc> arcs;
        std::vector<double> capacities;
        std::vector<Block> blocks;
        std::vector<Demand> demands;
        std::vector<std::vector<int>> block_demands;
        Graph physical, reverse_physical, logical;
        double rejection{};
        double transport(int b, int a) const;
        static Problem read(const std::string &path);
    };
    struct Path {
        bool physical{};
        int owner{};
        std::vector<int> edges;
        int x{-1}, v{-1}, upper_row{-1}, lower_row{-1};
    };
    struct Plan {
        std::vector<std::vector<int>> logical, physical;
        std::vector<double> served;
        static Plan empty(const Problem &p);
        double validate(const Problem &p) const;
        Json json() const;
    };
    struct Stats {
        std::uint64_t labels{}, dominated{}, searches{}, reuse{}, cg_iterations{}, nodes{}, activations{},
            covers{}, flow_covers{}, flow_link_covers{}, flow_yard_covers{}, heuristic_searches{};
        std::uint64_t branches[3]{};
        std::size_t peak_labels{}, peak_label_bytes{}, paths{};
        std::uint64_t pool_cleanups{}, paths_removed{}, paths_generated{}, paths_regenerated{};
        std::uint64_t volume_bound_rows{};
        std::uint64_t pricing_candidates{};
        std::uint64_t candidate_retained{}, candidate_duplicate_hits{};
        std::size_t max_paths_per_round{}, peak_candidate_bytes{};
        int columns{-1}, rows{-1};
        bool root_complete{}, bounded_candidate_collection{};
        std::uint64_t heuristic_calls{}, heuristic_improvements{}, heuristic_seed_paths{};
        double heuristic_initial_objective{};
        std::uint64_t branch_lp_reused{}, branch_lp_candidates{}, phase_one_nodes{}, phase_one_infeasible{};
        double branching_seconds{}, branch_lp_seconds{}, branch_copy_seconds{};
        std::uint64_t master_primal_solves{}, master_dual_solves{}, cut_dual_solves{}, lp_barrier_solves{};
        std::uint64_t bkf_nodes{}, bkf_lp_requested{};
        std::array<std::uint64_t, 3> bkf_lp_tested{};
        double pricing_seconds{}, label_work_seconds{}, master_seconds{}, build_seconds{}, heuristic_seconds{},
            separation_seconds{}, root_bound{}, first_best{}, root_seconds{};
        int last_master_status{};
        Json json() const;
    };
    struct Result {
        std::string method, status{"time_limit"};
        double objective{}, bound{}, seconds{};
        Plan plan;
        Stats stats;
        Json lp_certificate;
        Json json(const Problem &p) const;
    };
    Result solveArc(const Problem &, Budget &, int threads,
                    const Plan *incumbent = nullptr, bool relaxation = false, bool root_only = false);
    struct BpcOptions {
        bool root_only{};
        bool dw_root{};
        int pricing_workers{1};
        int branch_workers{8};
        bool dual_after_columns{};
    };
    Result solveBpc(const Problem &, Budget &, int threads, std::size_t label_budget,
                    const Plan *incumbent = nullptr, const BpcOptions &options = {});
    Result solveSingleCg(const Problem &, Budget &, int threads, std::size_t label_budget,
                         const Plan *incumbent = nullptr, const BpcOptions &options = {});
} // namespace RouteOpt::Application::IRBP

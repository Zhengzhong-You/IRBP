/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "irbp/model.hpp"
#include "irbp/pricing.hpp"
#include "irbp/pricing_workers.hpp"
#include <set>
#include <tuple>
#include <unordered_map>

namespace RouteOpt::Application::IRBP::SingleCG {
    class Master {
      public:
        const Problem &p;
        const bool capacity_bounds;
        Model m;
        std::vector<int> y, f, z, v, h, select, volume, link, binary;
        std::vector<std::vector<int>> xd, g, ld;
        std::vector<Path> paths;
        std::unordered_multimap<std::uint64_t, int> known;
        std::vector<std::vector<std::pair<int, int>>> incidence_rows; // (arc, row), including inactive rows.
        std::vector<char> closed_blocks;
        bool phase_one{};
        Master(const Problem &, Budget &, int threads = 1, bool capacity_bounds = true);
        double pathUpper(int block, const std::vector<int> &edges) const;
        bool add(int block, std::vector<int> edges);
        Plan recover() const;
        Plan recover(const std::vector<double> &values) const;
    };

    struct Candidate {
        int block;
        std::vector<int> edges;
        double cost;
    };
    struct PricingResult {
        std::vector<Candidate> columns;
        std::vector<double> by_block;
        double minimum{1e100}, correction{};
    };
    class Pricing {
        struct Request {
            int block;
            double load;
        };
        struct Layer {
            double capacity;
            std::vector<Request> requests;
        };
        struct Group {
            std::vector<int> blocks;
            std::vector<Layer> layers;
        };
        struct Completion {
            std::vector<double> distance;
            std::vector<int> next;
        };
        struct CandidateLess {
            bool operator()(const Candidate &a, const Candidate &b) const {
                return std::tie(a.cost, a.block, a.edges) < std::tie(b.cost, b.block, b.edges);
            }
        };
        struct Worker {
            Labeling labeling;
            Stats stats;
            std::vector<char> allowed;
            std::map<std::pair<int, double>, Completion> cache;
            std::size_t cache_bytes{}, cache_limit{};
            std::set<Candidate, CandidateLess> columns;
            Worker(std::size_t bytes, std::size_t cache_limit) : labeling(bytes), cache_limit(cache_limit) {}
        };
        const Problem &p;
        Master &master;
        Budget &budget;
        PricingWorkers team;
        std::vector<Worker> workers;
        std::vector<Group> groups;
        std::vector<double> length;
        const Completion &completion(Worker &, int destination, double capacity);
        void retain(Worker &, Candidate);

      public:
        Stats stats;
        Pricing(Master &, Budget &, int workers = 8, std::size_t label_bytes = 256 * 1024 * 1024);
        PricingResult search(const std::vector<double> &dual, double tolerance, bool heuristic = false);
        std::tuple<int, double, double> run(bool heuristic = false);
        void seed();
    };
} // namespace RouteOpt::Application::IRBP::SingleCG

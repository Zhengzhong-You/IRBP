/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#pragma once
#include "master.hpp"
#include <numeric>

namespace RouteOpt::Application::IRBP::SingleCG {
    // Direct bundles and demand-neighborhood repair adapt the construction/portfolio
    // idea in informs-ras-2026, extra0101 commit 4ee5e4d. No legacy code is copied.
    class IncumbentSearch {
        SearchMaster &master;
        const Problem &p;
        std::size_t round{};

      public:
        explicit IncumbentSearch(SearchMaster &m) : master(m), p(m.p) {}
        int seed(Budget &budget, double seconds) {
            Budget local;
            local.seconds = std::min(seconds, budget.remaining());
            Labeling labeling(32 * 1024 * 1024);
            Stats stats;
            std::map<int, std::vector<double>> completion;
            std::vector<double> cost(p.arcs.size()), length;
            std::vector<char> allowed(p.arcs.size());
            for (const auto &arc : p.arcs)
                length.push_back(arc.length);
            int added = 0;
            try {
                for (int b = 0; b < (int)p.blocks.size(); ++b) {
                    local.check();
                    const auto &block = p.blocks[b];
                    if (p.block_demands[b].empty())
                        continue;
                    auto [it, inserted] = completion.emplace(block.to, std::vector<double>{});
                    if (inserted)
                        it->second = shortestLengths(p, block.to, local);
                    std::set<double> levels{block.lower};
                    for (double capacity : p.capacities)
                        if (capacity > block.lower && capacity <= block.upper)
                            levels.insert(capacity);
                    for (int a = 0; a < (int)p.arcs.size(); ++a)
                        cost[a] = p.transport(b, a);
                    for (double load : levels) {
                        for (int a = 0; a < (int)p.arcs.size(); ++a)
                            allowed[a] = p.capacities[p.arcs[a].link] >= load;
                        auto path = labeling.run(p.physical, block.from, block.to, cost, length,
                                                 block.length_limit, it->second, allowed, local, stats, true);
                        if (path.edges.empty())
                            continue;
                        bool exists =
                            std::any_of(master.paths.begin(), master.paths.end(),
                                        [&](const Path &r) { return r.owner == b && r.edges == path.edges; });
                        if (!exists)
                            added += master.add(b, std::move(path.edges));
                    }
                }
            } catch (const LimitReached &) {
                // Optional seeding is bounded; exact pricing will find omitted paths.
            }
            return added;
        }
        Plan construct(const Plan &incumbent, Budget &budget) const {
            Plan best = incumbent;
            best.validate(p);
            std::vector<double> links(p.capacities.size()), tracks(p.nodes.size()), loads(p.blocks.size());
            for (int k = 0; k < (int)p.demands.size(); ++k)
                for (int b : best.logical[k])
                    loads[b] += best.served[k];
            for (int b = 0; b < (int)p.blocks.size(); ++b)
                if (!best.physical[b].empty()) {
                    tracks[p.blocks[b].from] += p.blocks[b].slots;
                    for (int a : best.physical[b])
                        links[p.arcs[a].link] += loads[b];
                }
            for (;;) {
                budget.check();
                double best_gain = 1e-7, best_load = 0;
                const Path *chosen = nullptr;
                std::vector<std::pair<int, double>> chosen_volumes;
                for (const auto &path : master.paths) {
                    int b = path.owner;
                    const auto &block = p.blocks[b];
                    if (!best.physical[b].empty() ||
                        tracks[block.from] + block.slots > p.nodes[block.from].tracks)
                        continue;
                    double capacity = block.upper, unit = block.interchange;
                    for (int a : path.edges) {
                        capacity = std::min(capacity, p.capacities[p.arcs[a].link] - links[p.arcs[a].link]);
                        unit += p.transport(b, a);
                    }
                    if (capacity < block.lower)
                        continue;
                    std::vector<int> demands;
                    for (int k : p.block_demands[b]) {
                        const auto &d = p.demands[k];
                        if (best.logical[k].empty() && d.from == block.from && d.to == block.to &&
                            d.penalty > unit)
                            demands.push_back(k);
                    }
                    std::stable_sort(demands.begin(), demands.end(), [&](int a, int b) {
                        return p.demands[a].penalty > p.demands[b].penalty;
                    });
                    double load = 0, gain = -block.opening;
                    std::vector<std::pair<int, double>> volumes;
                    for (int k : demands) {
                        double volume = std::min(p.demands[k].quantity, capacity - load);
                        if (volume <= 1e-9)
                            break;
                        volumes.emplace_back(k, volume);
                        load += volume;
                        gain += volume * (p.demands[k].penalty - unit);
                    }
                    if (load >= block.lower && gain > best_gain) {
                        best_gain = gain;
                        best_load = load;
                        chosen = &path;
                        chosen_volumes = std::move(volumes);
                    }
                }
                if (!chosen)
                    break;
                int b = chosen->owner;
                best.physical[b] = chosen->edges;
                tracks[p.blocks[b].from] += p.blocks[b].slots;
                for (int a : chosen->edges)
                    links[p.arcs[a].link] += best_load;
                for (auto [k, volume] : chosen_volumes) {
                    best.logical[k] = {b};
                    best.served[k] = volume;
                }
            }
            best.validate(p);
            return best;
        }
        Plan improve(const Plan &incumbent, Budget &budget, double seconds) {
            const int K = p.demands.size();
            if (!K)
                return incumbent;
            std::vector<int> anchors(K);
            std::iota(anchors.begin(), anchors.end(), 0);
            std::stable_sort(anchors.begin(), anchors.end(), [&](int a, int b) {
                return (p.demands[a].quantity - incumbent.served[a]) * p.demands[a].penalty >
                       (p.demands[b].quantity - incumbent.served[b]) * p.demands[b].penalty;
            });
            const int anchor = anchors[round++ % K];
            const auto &d = p.demands[anchor];
            std::vector<char> used(p.blocks.size());
            for (int b : incumbent.logical[anchor])
                used[b] = true;
            auto affinity = [&](int k) {
                int common = 0;
                for (int b : incumbent.logical[k])
                    common += used[b];
                return 8 * (k == anchor) + 4 * common + 2 * (p.demands[k].from == d.from) +
                       2 * (p.demands[k].to == d.to);
            };
            std::stable_sort(anchors.begin(), anchors.end(),
                             [&](int a, int b) { return affinity(a) > affinity(b); });
            std::vector<char> free(K);
            for (int j = 0; j < std::min(K, 8); ++j)
                free[anchors[j]] = true;
            return master.heuristic(budget, seconds, incumbent, free);
        }
    };
} // namespace RouteOpt::Application::IRBP::SingleCG

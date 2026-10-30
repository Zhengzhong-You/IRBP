/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/path_search.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <set>
using namespace RouteOpt::Application::IRBP;
int main() {
    std::mt19937 rng(815);
    Budget budget;
    budget.seconds = 30;
    Stats stats;
    Labeling labeling(64 * 1024 * 1024);
    for (int test = 0; test < 150; ++test) {
        int n = 5 + test % 3;
        std::vector<int> tail, head;
        std::vector<double> cost, length;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (i != j && rng() % 3) {
                    tail.push_back(i);
                    head.push_back(j);
                    cost.push_back(int(rng() % 21) - 10);
                    length.push_back(1 + rng() % 5);
                }
        Graph g;
        g.build(n, tail, head);
        double limit = 3 + test % 17, best = 1e100;
        std::vector<bool> seen(n);
        seen[0] = true;
        std::function<void(int, double, double)> dfs = [&](int i, double c, double l) {
            if (i == n - 1) {
                best = std::min(best, c);
                return;
            }
            for (int z = g.offsets[i]; z < g.offsets[i + 1]; ++z) {
                int a = g.edge_ids[z], j = g.to[a];
                if (!seen[j] && l + length[a] <= limit) {
                    seen[j] = true;
                    dfs(j, c + cost[a], l + length[a]);
                    seen[j] = false;
                }
            }
        };
        dfs(0, 0, 0);
        std::vector<PricedPath> collected;
        auto got = labeling.run(g, 0, n - 1, cost, length, limit, {}, {}, budget, stats, false, 2.5,
                                [&](PricedPath path) { collected.push_back(std::move(path)); });
        if (got.cost != best) {
            std::cerr << "Pricing mismatch case " << test << '\n';
            return 1;
        }
        double collected_best = 1e100;
        for (const auto &path : collected) {
            int at = 0;
            double c = 0, l = 0;
            std::vector<bool> visited(n);
            visited[0] = true;
            for (int a : path.edges) {
                if (g.from[a] != at || visited[g.to[a]])
                    return 4;
                at = g.to[a];
                visited[at] = true;
                c += cost[a];
                l += length[a];
            }
            if (at != n - 1 || c != path.cost || l > limit || c >= 2.5)
                return 5;
            collected_best = std::min(collected_best, c);
        }
        if (best < 2.5 && collected_best != best)
            return 6;
    }
    Graph large;
    std::vector<int> tails, heads;
    std::vector<double> costs, lengths;
    for (int i = 0; i < 69; ++i) {
        tails.push_back(i);
        heads.push_back(i + 1);
        costs.push_back(1);
        lengths.push_back(1);
    }
    tails.push_back(65);
    heads.push_back(2);
    costs.push_back(-1000);
    lengths.push_back(1);
    large.build(70, tails, heads);
    auto large_result = labeling.run(large, 0, 69, costs, lengths, 100, {}, {}, budget, stats);
    if (large_result.cost != 69 || large_result.edges.size() != 69)
        return 2;
    bool limited = false;
    try {
        Labeling tiny(8);
        tiny.run(large, 0, 69, costs, lengths, 100, {}, {}, budget, stats);
    } catch (const LimitReached &) {
        limited = true;
    }
    if (!limited)
        return 3;
    Problem joint;
    joint.nodes = std::vector<Node>(5, {true, 10, 100, 0});
    std::vector<std::vector<std::vector<int>>> candidates(5);
    for (int b = 0; b < 4; ++b) {
        joint.blocks.push_back({b, b + 1, 1, 5, 20, 1, 0, 0, 1, {}});
        joint.arcs.push_back({b, b + 1, b, 1, 2});
        joint.capacities.push_back(20);
        joint.demands.push_back({b, b + 1, 4, 10, {b}});
        candidates[b] = {{b}};
    }
    joint.blocks.push_back({0, 4, 1, 1, 20, 1, 0, 0, 1, {}});
    joint.arcs.push_back({0, 4, 4, 1, .5});
    joint.capacities.push_back(20);
    joint.demands.push_back({0, 4, 1, 1, {0, 1, 2, 3, 4}});
    candidates[4] = {{4}};
    joint.logical.build(5, {0, 1, 2, 3, 0}, {1, 2, 3, 4, 4});
    joint.physical.build(5, {0, 1, 2, 3, 0}, {1, 2, 3, 4, 4});
    auto incumbent = Plan::empty(joint);
    incumbent.logical[4] = {4};
    incumbent.served[4] = 1;
    incumbent.physical[4] = {4};
    if (std::abs(incumbent.validate(joint) - 160.5) > 1e-8)
        throw std::runtime_error("Support-search fixture is not a valid incumbent");
    Budget support_budget;
    support_budget.seconds = 1;
    auto joint_paths = supportPaths(joint, incumbent, {2, 2, 2, 2, .5}, candidates, support_budget, stats,
                                    16 * 1024 * 1024, 2);
    if (std::find(joint_paths[4].begin(), joint_paths[4].end(), std::vector<int>{0, 1, 2, 3}) ==
        joint_paths[4].end())
        throw std::runtime_error("Joint minimum-load support chain was omitted");
    joint.demands[4].blocks = {0, 1, 2, 4};
    Budget compatibility_budget;
    compatibility_budget.seconds = 1;
    auto forbidden = supportPaths(joint, incumbent, {2, 2, 2, 2, .5}, candidates, compatibility_budget, stats,
                                  16 * 1024 * 1024, 2);
    for (const auto &path : forbidden[4])
        if (std::find(path.begin(), path.end(), 3) != path.end())
            throw std::runtime_error("Support search ignored block compatibility");
    Problem alternatives;
    alternatives.nodes = std::vector<Node>(2, {true, 10, 100, 0});
    alternatives.blocks = {
        {0, 1, 1, 5, 10, 1, 0, 0, 1, {}}, {0, 1, 1, 5, 10, 1, 0, 0, 1, {}}, {0, 1, 1, 1, 10, 1, 0, 0, 1, {}}};
    alternatives.arcs = {{0, 1, 0, 1, 5}, {0, 1, 1, 1, 1}, {0, 1, 2, 1, 1}};
    alternatives.capacities = {10, 10, 10};
    alternatives.demands = {{0, 1, 4, 10, {0, 1}}, {0, 1, 1, 10, {0, 2}}};
    alternatives.logical.build(2, {0, 0, 0}, {1, 1, 1});
    alternatives.physical.build(2, {0, 0, 0}, {1, 1, 1});
    auto alternative_plan = Plan::empty(alternatives);
    alternative_plan.logical[1] = {2};
    alternative_plan.served[1] = 1;
    alternative_plan.physical[2] = {2};
    if (std::abs(alternative_plan.validate(alternatives) - 41.) > 1e-8)
        throw std::runtime_error("Alternative-anchor fixture is not feasible");
    Budget alternative_budget;
    alternative_budget.seconds = 1;
    auto alternative_paths = supportPaths(alternatives, alternative_plan, {5, 1, 1}, {{{0}, {1}}, {{2}}},
                                          alternative_budget, stats, 16 * 1024 * 1024, 2);
    if (std::find(alternative_paths[1].begin(), alternative_paths[1].end(), std::vector<int>{0}) ==
        alternative_paths[1].end())
        throw std::runtime_error("The best anchor hid a compatible profitable alternative");
    Problem detour;
    detour.nodes = std::vector<Node>(4, {true, 10, 100, 0});
    detour.blocks = {{0, 3, 1, 4, 10, 3, 0, 0, 1, {}}};
    detour.arcs = {{0, 1, 0, 1, 1},   {1, 3, 1, 1, 1}, {0, 2, 2, 1, 1.5},
                   {2, 3, 3, 1, 1.5}, {1, 2, 4, 1, 3}, {1, 0, 5, .1, 0}};
    detour.capacities = {10, 10, 10, 10, 10, 10};
    detour.physical.build(4, {0, 1, 0, 2, 1, 1}, {1, 3, 2, 3, 2, 0});
    detour.reverse_physical.build(4, {1, 3, 2, 3, 2, 0}, {0, 1, 0, 2, 1, 1});
    Budget detour_budget;
    detour_budget.seconds = 1;
    const auto labels_before_deviation = stats.labels;
    auto detours =
        physicalAlternatives(detour, 0, {{0, 1}, {2, 3}}, detour_budget, stats, 2);
    if (detours.size() != 1 || detours[0].edges != std::vector<int>{0, 4, 3} ||
        std::abs(detours[0].cost - 5.5) > 1e-8 || stats.labels != labels_before_deviation)
        throw std::runtime_error("Physical prefix deviation failed or revisited a prefix node");
    detour.blocks[0].length_limit = 2.9;
    Budget short_budget;
    if (!physicalAlternatives(detour, 0, {{0, 1}, {2, 3}}, short_budget, stats, 2).empty())
        throw std::runtime_error("Physical deviation exceeded the full path length budget");
    detour.blocks[0].length_limit = 3;
    detour.capacities[4] = 3;
    Budget capacity_budget;
    if (!physicalAlternatives(detour, 0, {{0, 1}, {2, 3}}, capacity_budget, stats, 2)
             .empty())
        throw std::runtime_error("Physical deviation ignored the minimum block load");
    Problem repeated_link;
    repeated_link.nodes.resize(3);
    repeated_link.arcs = {{0, 2, 1, 1, 1}, {0, 1, 0, 1, .1}, {1, 2, 0, 1, .1}};
    repeated_link.capacities = {3, 10};
    repeated_link.blocks = {{0, 2, 0, 2, 4, 4, 0, 0, 1, {}}};
    repeated_link.physical.build(3, {0, 0, 1}, {2, 1, 2});
    Budget repeated_link_budget;
    if (!physicalAlternatives(repeated_link, 0, {{0}}, repeated_link_budget, stats, 2).empty())
        throw std::runtime_error("SPP deviation ignored repeated use of one capacity resource");
    {
        const int n = 5000;
        std::vector<int> from, to;
        std::vector<double> cost;
        for (int i = 0; i < n - 1; ++i) {
            from.push_back(i); to.push_back(i + 1); cost.push_back(1);
        }
        from.push_back(0); to.push_back(n - 1); cost.push_back(4000);
        Graph large;
        large.build(n, from, to);
        Budget large_budget;
        large_budget.seconds = 2;
        bool finished = false;
        auto paths = shortestSimplePaths(large, 0, n - 1, cost, std::vector<char>(cost.size(), true),
                                         large_budget, 2, &finished);
        if (!finished || paths.size() != 2 || paths[0].cost != 4000 || paths[1].cost != n - 1)
            throw std::runtime_error("Large sparse-graph SPP check failed");
    }
    for (int test = 0; test < 80; ++test) {
        const int n = 4 + test % 3, keep = 1 + test % 5;
        std::vector<int> from, to;
        std::vector<double> costs;
        std::vector<char> allowed;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (i != j && rng() % 2)
                    for (int copy = 0; copy < 1 + int(rng() % 2); ++copy) {
                        from.push_back(i);
                        to.push_back(j);
                        costs.push_back(rng() % 10);
                        allowed.push_back(rng() % 4 != 0);
                    }
        Graph graph;
        graph.build(n, from, to);
        std::vector<double> exact;
        std::vector<char> visited(n);
        visited[0] = true;
        std::function<void(int, double)> enumerate = [&](int i, double value) {
            if (i == n - 1) {
                exact.push_back(value);
                return;
            }
            for (int z = graph.offsets[i]; z < graph.offsets[i + 1]; ++z) {
                int a = graph.edge_ids[z], j = to[a];
                if (allowed[a] && !visited[j]) {
                    visited[j] = true;
                    enumerate(j, value + costs[a]);
                    visited[j] = false;
                }
            }
        };
        enumerate(0, 0);
        std::sort(exact.begin(), exact.end());
        Budget search;
        auto paths = shortestSimplePaths(graph, 0, n - 1, costs, allowed, search, keep);
        if (paths.size() != std::min(exact.size(), std::size_t(keep)))
            throw std::runtime_error("Logical alternative count differs from exhaustive enumeration");
        std::set<std::vector<int>> unique;
        for (std::size_t i = 0; i < paths.size(); ++i) {
            if (paths[i].cost != exact[i] || !unique.insert(paths[i].edges).second)
                throw std::runtime_error("Logical alternative ranking or identity mismatch");
            std::fill(visited.begin(), visited.end(), false);
            int at = 0;
            visited[at] = true;
            for (int a : paths[i].edges) {
                if (!allowed[a] || from[a] != at || visited[to[a]])
                    throw std::runtime_error("Logical alternative violates compatibility or elementarity");
                at = to[a];
                visited[at] = true;
            }
            if (at != n - 1) throw std::runtime_error("Logical alternative endpoint mismatch");
        }
    }
    std::cout << "80 exhaustive loopless logical-alternative comparisons passed\n";
    Graph limited_graph;
    limited_graph.build(2, {0}, {1});
    Budget expired;
    expired.seconds = 0;
    bool completed = true;
    if (!shortestSimplePaths(limited_graph, 0, 1, {1}, {true}, expired, 2, &completed).empty() || completed)
        throw std::runtime_error("Interrupted logical generation was reported as exhausted");
    std::cout << "Minimum-load support chain and compatibility checks passed\n";
    std::cout << "150 exhaustive pricing comparisons, multiword visitation and memory-limit checks passed\n";
}

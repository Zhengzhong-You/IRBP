/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/pricing.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
using namespace RouteOpt::Application::IRBP;
int main() {
    std::mt19937 rng(815);
    Budget budget;
    budget.seconds = 30;
    Stats stats;
    Labeling labeling(0);
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
            int at = 0; double c = 0, l = 0; std::vector<bool> visited(n); visited[0] = true;
            for (int a : path.edges) {
                if (g.from[a] != at || visited[g.to[a]]) return 4;
                at = g.to[a]; visited[at] = true; c += cost[a]; l += length[a];
            }
            if (at != n - 1 || c != path.cost || l > limit || c >= 2.5) return 5;
            collected_best = std::min(collected_best, c);
        }
        if (best < 2.5 && collected_best != best) return 6;
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
    std::cout << "150 exhaustive pricing comparisons, multiword visitation and memory-limit checks passed\n";
}

/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
/* Exact physical-pricing certificate for nonnegative arc costs. */
#pragma once
#include "irbp/problem.hpp"
#include <queue>
#include <tuple>

namespace RouteOpt::Application::IRBP::BPC {
    struct PhysicalCostTree {
        std::vector<double> weights, cost, length;
        std::vector<int> next;
    };
    inline PhysicalCostTree physicalCostTree(const Problem &p, int destination,
                                             const std::vector<double> &weights, Budget &budget) {
        PhysicalCostTree tree{weights, std::vector<double>(p.nodes.size(), 1e100),
                              std::vector<double>(p.nodes.size(), 1e100),
                              std::vector<int>(p.nodes.size(), -1)};
        using Entry = std::tuple<double, double, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
        tree.cost[destination] = tree.length[destination] = 0;
        queue.emplace(0, 0, destination);
        while (!queue.empty()) {
            budget.check();
            auto [cost, length, node] = queue.top();queue.pop();
            if (cost != tree.cost[node] || length != tree.length[node]) continue;
            for (int z=p.reverse_physical.offsets[node];z<p.reverse_physical.offsets[node+1];++z) {
                int a=p.reverse_physical.edge_ids[z], from=p.arcs[a].from;
                const double next_cost=cost+weights[a], next_length=length+p.arcs[a].length;
                if (std::tie(next_cost,next_length)<std::tie(tree.cost[from],tree.length[from])) {
                    tree.cost[from]=next_cost;tree.length[from]=next_length;tree.next[from]=a;
                    queue.emplace(next_cost,next_length,from);
                }
            }
        }
        return tree;
    }
}

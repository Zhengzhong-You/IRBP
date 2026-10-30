/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/cuts.hpp"
#include <iostream>
#include <random>
using namespace RouteOpt::Application::IRBP;
int main() {
    Budget budget;
    budget.seconds = 30;
    auto example = separateFlowCover({{0, 6, .8, 4.8}, {1, 6, .8, 4.8}}, 10, budget);
    if (!example || std::abs(example->violation - 1.2) > 1e-8)
        return 1;
    auto singleton = separateFlowCover({{0, 10, .6, 6}}, 6, budget);
    if (!singleton || singleton->members.size() != 1)
        return 2;
    std::mt19937 random(47);
    int separated = 0;
    for (int trial = 0; trial < 500; ++trial) {
        int n = 2 + random() % 7;
        double capacity = 1 + random() % 20, total = 0;
        std::vector<FlowItem> items;
        for (int j = 0; j < n; ++j) {
            double u = 1 + random() % 20, x = (1 + random() % 99) / 100.;
            double v = u * x * ((1 + random() % 100) / 100.);
            items.push_back({j, u, x, v});
            total += v;
        }
        for (auto &item : items)
            item.flow *= std::min(1., capacity / total);
        auto cut = separateFlowCover(items, capacity, budget);
        if (!cut)
            continue;
        ++separated;
        for (int mask = 0; mask < (1 << n); ++mask) {
            double available = 0, penalty = 0;
            for (int j : cut->members) {
                if (mask & (1 << j))
                    available += items[j].upper;
                else
                    penalty += std::max(0., items[j].upper - cut->excess);
            }
            // This maximizes the cut's flow term over every continuous feasible allocation.
            if (std::min(capacity, available) + penalty > capacity + 1e-8)
                return 3;
        }
    }
    if (separated < 100)
        return 4;
    std::cout << "Flow-cover separation and binary/continuous validity checks passed\n";
}

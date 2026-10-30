/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "irbp/problem.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>

namespace RouteOpt::Application::IRBP {
    struct FlowItem {
        int id{};
        double upper{}, selection{}, flow{};
    };
    struct FlowCover {
        std::vector<int> members;
        double excess{}, violation{};
    };
    // Standard flow cover: sum_S v_i + sum_S (U_i-lambda)^+ (1-x_i) <= C, U(S)=C+lambda.
    // For the closed members I, flow(S) <= C-max(0,U(I)-lambda), while the added penalty
    // is at most max(0,U(I)-lambda). This proves validity for binary x and continuous v.
    // Reference: https://www.cirrelt.ca/documentstravail/cirrelt-2009-20.pdf, Section 2.4.
    inline std::optional<FlowCover> separateFlowCover(const std::vector<FlowItem> &items,
                                                     double capacity, Budget &budget) {
        if (items.empty() || capacity <= 0)
            return std::nullopt;
        const double tolerance = FeasTol * std::max(1., capacity);
        std::optional<FlowCover> best;
        auto inspect = [&](const std::vector<int> &set, double upper_sum, double flow_sum) {
            const double excess = upper_sum - capacity;
            if (excess <= tolerance)
                return;
            double violation = flow_sum - capacity;
            for (int j : set)
                violation += std::max(0., items[j].upper - excess) * (1 - items[j].selection);
            if (violation <= tolerance || (best && violation <= best->violation))
                return;
            FlowCover cut{{}, excess, violation};
            for (int j : set)
                cut.members.push_back(items[j].id);
            std::sort(cut.members.begin(), cut.members.end());
            best = std::move(cut);
        };
        double largest_upper = 0;
        for (int j = 0; j < (int)items.size(); ++j) {
            inspect({j}, items[j].upper, items[j].flow);
            largest_upper = std::max(largest_upper, items[j].upper);
        }
        std::vector<int> order(items.size());
        std::iota(order.begin(), order.end(), 0);
        for (int mode = 0; mode < 3; ++mode) {
            auto score = [&](int j) {
                const auto &v = items[j];
                if (mode == 0)
                    return -v.selection;
                if (mode == 1)
                    return -v.flow;
                return (v.upper * v.selection - v.flow) / v.upper;
            };
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                double x = score(a), y = score(b);
                return x == y ? items[a].id < items[b].id : x < y;
            });
            std::vector<int> prefix;
            double upper_sum = 0, flow_sum = 0;
            for (int j : order) {
                budget.check();
                prefix.push_back(j);
                upper_sum += items[j].upper;
                flow_sum += items[j].flow;
                inspect(prefix, upper_sum, flow_sum);
                // Beyond this point all positive-part coefficients vanish for every later prefix.
                if (upper_sum - capacity >= largest_upper)
                    break;
            }
        }
        return best;
    }
} // namespace RouteOpt::Application::IRBP

/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <thread>

namespace RouteOpt::Application::IRBP {
    constexpr int MaxLpCandidatesPerWorker = 4;
    constexpr int RootLpCandidates = 64;
    constexpr std::array<int, 3> BranchWeights{9, 4, 1};
    inline int branchLpThreads(int threads, int workers) {
        return std::max(1, std::min({8, threads, int(std::thread::hardware_concurrency() / workers)}));
    }
    inline int lpTestingBudget(int bkf_k, int workers, bool root) {
        return root ? RootLpCandidates : bkf_k * workers;
    }
    inline std::array<int, 3> branchingQuota(const std::array<int, 3> &available, int count) {
        const auto &weights = BranchWeights;
        std::array<int, 3> result{};
        count = std::min(count, std::accumulate(available.begin(), available.end(), 0));
        while (count > 0) {
            int total_weight = 0;
            for (int f = 0; f < 3; ++f)
                if (result[f] < available[f]) total_weight += weights[f];
            std::array<double, 3> shares{};
            bool saturated = false;
            const int remaining = count;
            for (int f = 0; f < 3; ++f) {
                const int capacity = available[f] - result[f];
                if (!capacity) continue;
                shares[f] = double(remaining) * weights[f] / total_weight;
                if (capacity <= shares[f]) {
                    result[f] += capacity;
                    count -= capacity;
                    saturated = true;
                }
            }
            if (saturated) continue;
            for (int f = 0; f < 3; ++f) {
                const int whole = static_cast<int>(std::floor(shares[f]));
                result[f] += whole;
                count -= whole;
                shares[f] -= whole;
            }
            std::array<int, 3> order{0, 1, 2};
            std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return shares[a] > shares[b]; });
            for (int f : order)
                if (count > 0 && result[f] < available[f]) {
                    ++result[f];
                    --count;
                }
        }
        return result;
    }
}

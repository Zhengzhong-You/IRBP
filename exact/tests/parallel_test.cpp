/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/pricing_workers.hpp"
#include <iostream>
#include <stdexcept>
using namespace RouteOpt::Application::IRBP;
int main() {
    for (int count : {1, 4}) {
        PricingWorkers workers(count);
        for (int round = 0; round < 10; ++round) {
            std::vector<std::atomic<int>> seen(10000);
            for (auto &visits : seen)
                visits.store(0);
            workers.run(seen.size(), [&](int worker, int job) {
                if (worker < 0 || worker >= count)
                    throw std::runtime_error("Invalid worker");
                ++seen[job];
            });
            for (auto &visits : seen)
                if (visits.load() != 1)
                    return 1;
            bool caught = false;
            try {
                workers.run(100, [](int, int job) {
                    if (job == 3)
                        throw std::runtime_error("Expected task failure");
                });
            } catch (const std::runtime_error &) {
                caught = true;
            }
            if (!caught)
                return 2;
        }
        workers.run(0, [](int, int) { throw std::runtime_error("Empty batch executed"); });
    }
    std::cout << "Persistent pricing workers: exactly-once jobs, cancellation, reuse and empty batches passed\n";
}

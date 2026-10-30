/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/problem.hpp"
#include "irbp/model.hpp"
#include <iostream>
using namespace RouteOpt::Application::IRBP;
int main(int argc, char **argv) try {
    if (argc != 2)
        return 2;
    {
        Model repair(1, "repair-parameter-isolation"), untouched(1, "untouched-model");
        int before = -1, chosen = -1, after = -1, fresh = -1;
        check(untouched.solver.getEnvIntegralityFocus(&before));
        check(repair.solver.setEnvIntegralityFocus(1));
        check(repair.solver.getEnvIntegralityFocus(&chosen));
        check(untouched.solver.getEnvIntegralityFocus(&after));
        Model later(1, "later-model");
        check(later.solver.getEnvIntegralityFocus(&fresh));
        if (chosen != 1 || after != before || fresh != before)
            throw std::runtime_error("Repair IntegralityFocus leaked to another or future LP/ARC model");
    }
    for (const auto &[name, expected] : std::vector<std::pair<std::string, double>>{
             {"partial_service", 28}, {"shared_link_cover", 82}, {"parallel_blocks", 16}}) {
        auto p = Problem::read(std::string(argv[1]) + "/" + name + ".json");
        Budget budget;
        budget.seconds = 15;
        auto result = solveArc(p, budget, 1);
        if (result.status != "optimal" || std::abs(result.objective - expected) > 1e-6 ||
            std::abs(result.bound - expected) > 1e-5 || std::abs(result.plan.validate(p) - expected) > 1e-6)
            throw std::runtime_error("Compact regression failed: " + name);
        Budget restart;
        restart.seconds = 15;
        auto known = solveArc(p, restart, 1, &result.plan);
        if (known.status != "optimal" || std::abs(known.objective - expected) > 1e-6)
            throw std::runtime_error("Validated incumbent/cutoff regression failed");
        if (name == "partial_service") {
            if (std::abs(result.plan.served[0] - 6) > 1e-6)
                throw std::runtime_error("Partial service lost");
            auto bad = result.plan;
            bad.served[0] = 10;
            bool rejected = false;
            try {
                bad.validate(p);
            } catch (const std::runtime_error &) {
                rejected = true;
            }
            if (!rejected)
                throw std::runtime_error("Capacity violation accepted");
        }
    }
    std::cout << "Compact objectives, partial service, parallel blocks and shared capacity passed\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

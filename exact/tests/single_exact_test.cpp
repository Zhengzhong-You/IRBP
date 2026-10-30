/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#include "../src/single_cg/master.hpp"
#include <iostream>
using namespace RouteOpt;
using namespace RouteOpt::Application::IRBP;
static void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
int main(int argc, char **argv) {
    try {
        if (argc != 3)
            return 2;
        auto p = Problem::read(argv[2]);
        Budget budget;
        budget.seconds = 30;
        SingleCG::SearchMaster master(p, budget, 1);
        SingleCG::Pricing price(master, budget, 2, 32 * 1024 * 1024);
        price.seed();
        master.activate({{1, 0, 1, 1}});
        require(master.m.optimize(budget) == SOLVER_MIP_INFEASIBLE,
                "Missing-column branch unexpectedly feasible");
        master.phaseOne(true);
        bool feasible = false;
        for (int i = 0; i < 20; ++i) {
            require(master.m.optimize(budget) == SOLVER_OPTIMAL, "Phase I solve failed");
            if (master.m.objective() < 1e-8) {
                feasible = true;
                break;
            }
            auto [added, minimum, correction] = price.run();
            require(added > 0, "Phase I failed to generate a required path");
        }
        require(feasible, "Feasibility pricing did not close");
        master.phaseOne(false);
        require(master.m.optimize(budget) == SOLVER_OPTIMAL, "Restored objective failed");
        require(std::abs(master.m.objective() - 82) < 1e-6, "Restored path cost is wrong");
        master.activate({{1, 0, 1, 1}, {1, 0, 3, 1}});
        master.phaseOne(true);
        bool proved = false;
        for (int i = 0; i < 20; ++i) {
            require(master.m.optimize(budget) == SOLVER_OPTIMAL, "Infeasible-node Phase I failed");
            double value = master.m.objective();
            auto [added, minimum, correction] = price.run();
            if (!added) {
                require(value + correction > FeasTol, "Missing infeasibility certificate");
                proved = true;
                break;
            }
        }
        require(proved, "Infeasible branch not certified");
        master.phaseOne(false);
        master.activate({});
        require(master.m.optimize(budget) == SOLVER_OPTIMAL, "Sibling restore failed");
        require(std::abs(master.m.objective() - 82) < 1e-6, "Sibling objective changed");
        master.activate({{0, 0, 0, 0}});
        require(master.m.optimize(budget) == SOLVER_OPTIMAL, "Closed-block solve failed");
        auto [added, minimum, correction] = price.run();
        require(!added && correction == 0, "Closed block was priced");

        std::uint64_t nodes = 0, tests = 0;
        for (const char *name :
             {"partial_service", "parallel_blocks", "syn_y4_n6_k8_s42", "syn_y5_n8_k10_s43"}) {
            auto instance = Problem::read(std::string(argv[1]) + "/" + name + ".json");
            Budget arc_time;
            arc_time.seconds = 30;
            auto arc = solveArc(instance, arc_time, 1);
            require(arc.status == "optimal", "ARC reference unfinished");
            for (int workers : {1, 4}) {
                Budget time;
                time.seconds = 30;
                BpcOptions options;
                options.pricing_workers = workers;
                options.branch_workers = workers;
                time.report = [&](const Progress &row) {
                    require(row.lower <= arc.objective + 1e-5, "Node bound exceeds optimum");
                    require(row.upper >= arc.objective - 1e-5, "Incumbent below optimum");
                };
                auto result = solveSingleCg(instance, time, 1, 64 * 1024 * 1024, nullptr, options);
                std::cout << name << " workers=" << workers << " status=" << result.status
                          << " obj=" << result.objective << " bound=" << result.bound
                          << " nodes=" << result.stats.nodes << '\n';
                require(result.status == "optimal", "Single-CG tree unfinished");
                require(std::abs(result.objective - arc.objective) <= pricingTolerance(arc.objective),
                        "Integer optimum mismatch");
                require(result.objective - result.bound <= pricingTolerance(result.objective),
                        "Optimality not certified");
                result.plan.validate(instance);
                require(result.stats.master_seconds + result.stats.pricing_seconds +
                            result.stats.heuristic_seconds + result.stats.branching_seconds <= result.seconds + .001,
                        "Stage timers overlap");
                nodes += result.stats.nodes;
                tests += result.stats.branch_lp_candidates;
            }
        }
        require(nodes > 8 && tests > 0, "Regression did not exercise branching");
        std::cout << "Single-CG exact tree, Phase I pricing, branch restore and ARC comparisons passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

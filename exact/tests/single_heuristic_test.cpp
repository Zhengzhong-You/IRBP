/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#include "../src/single_cg/heuristic.hpp"
#include <iostream>
using namespace RouteOpt;
using namespace RouteOpt::Application::IRBP;
static void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
int main(int argc, char **argv) try {
    if (argc != 2)
        return 2;
    auto p = Problem::read(argv[1]);
    Budget budget;
    budget.seconds = 30;
    SingleCG::SearchMaster master(p, budget, 1);
    SingleCG::Pricing pricing(master, budget, 1);
    SingleCG::IncumbentSearch search(master);
    pricing.seed();
    const int added = search.seed(budget, 2);
    require(added > 0, "Cost-directed path seeding did not add an alternative");
    auto plan = search.construct(Plan::empty(p), budget);
    require(std::abs(plan.validate(p) - 82) < 1e-6, "Partial-volume bundle is wrong");
    master.activate({{0, 0, 0, 0}});
    auto repaired = search.improve(plan, budget, 2);
    require(std::abs(repaired.validate(p) - 82) < 1e-6, "Tree restrictions contaminated global local search");
    require(master.m.optimize(budget) == SOLVER_OPTIMAL, "Local search damaged master");
    require(std::abs(master.m.objective() - p.rejection) < 1e-6, "Local search changed parent branch");
    // Two six-unit demands must be moved together to meet a ten-unit minimum.
    p.blocks[0].lower = 10;
    p.blocks[0].length_limit = 6;
    p.demands[0].quantity = 6;
    p.demands.push_back(p.demands[0]);
    p.block_demands[0] = {0, 1};
    p.rejection = 120;
    SingleCG::SearchMaster bundles(p, budget, 1);
    bundles.add(0, {1, 3});
    SingleCG::IncumbentSearch bundle_search(bundles);
    auto bundle = bundle_search.construct(Plan::empty(p), budget);
    require(std::abs(bundle.served[0] + bundle.served[1] - 10) < 1e-6,
            "Joint load or residual capacity is wrong");
    require(std::abs(bundle.validate(p) - 22) < 1e-6, "Joint bundle cost mismatch");
    // A frozen demand occupies six units, leaving exactly four for its neighbor.
    std::vector<char> free{0, 1};
    auto local = bundles.heuristic(budget, 2, bundle, free);
    require(local.served[0] == bundle.served[0], "Frozen demand changed");
    require(local.validate(p) <= bundle.validate(p) + 1e-6, "Local search worsened incumbent");
    // No direct block exists: both demands must move together through one hub.
    p.nodes[1] = {true, 1, 10, 1};
    p.capacities = {12, 12};
    p.arcs = {{0, 1, 0, 1, 0}, {1, 2, 1, 1, 0}};
    p.physical.build(3, {0, 1}, {1, 2});
    p.reverse_physical.build(3, {1, 2}, {0, 1});
    p.blocks = {{0, 1, 1, 10, 12, 1, 2, 0, 1, {}}, {1, 2, 1, 10, 12, 1, 2, 0, 1, {}}};
    p.logical.build(3, {0, 1}, {1, 2});
    p.block_demands = {{0, 1}, {0, 1}};
    for (auto &d : p.demands)
        d.blocks = {0, 1};
    SingleCG::SearchMaster hubs(p, budget, 1);
    hubs.add(0, {0});
    hubs.add(1, {1});
    SingleCG::IncumbentSearch hub_search(hubs);
    auto hub = hub_search.improve(Plan::empty(p), budget, 2);
    require(std::abs(hub.validate(p) - 34) < 1e-6, "Joint hub move or handling capacity is wrong");
    std::cout
        << "Joint minimum loads, partial service, frozen flow, branch isolation and path seeding passed\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

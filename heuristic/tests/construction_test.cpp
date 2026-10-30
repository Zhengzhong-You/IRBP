/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search.hpp"
#include <iostream>
using namespace RouteOpt::Application::IRBP;

void finish(Problem &p) {
    std::vector<int> from, to;
    for (const auto &a : p.arcs) {
        from.push_back(a.from);
        to.push_back(a.to);
    }
    p.physical.build(p.nodes.size(), from, to);
    p.reverse_physical.build(p.nodes.size(), to, from);
    from.clear();
    to.clear();
    for (const auto &b : p.blocks) {
        from.push_back(b.from);
        to.push_back(b.to);
    }
    p.logical.build(p.nodes.size(), from, to);
    p.block_demands.resize(p.blocks.size());
    for (int k = 0; k < int(p.demands.size()); ++k) {
        p.rejection += p.demands[k].quantity * p.demands[k].penalty;
        for (int b : p.demands[k].blocks) p.block_demands[b].push_back(k);
    }
}

int main() try {
    int failures = 0;
    auto check = [&](Problem p, double expected, bool warm = false, double tolerance = 1e-6) {
        finish(p);
        Plan initial = Plan::empty(p);
        if (warm) {
            initial.logical[0] = {0};
            initial.physical[0] = {0};
            initial.served[0] = p.demands[0].quantity;
        }
        LocalSearchOptions options;
        // Exclude two-hub pairs and all subsequent LP/MIP stages from these checks.
        options.shared_hubs_per_sweep = 1;
        options.min_joint_seconds = options.min_pool_seconds = 1000;
        options.construction_relative_tolerance = tolerance;
        Budget budget;
        budget.seconds = 5;
        const auto result = solveLocalSearch(p, budget, options, warm ? &initial : nullptr);
        if (result.profile.at("guidance_lp_calls") != 0 || result.profile.at("joint_mip_calls") != 0 ||
            result.profile.at("pool_mip_calls") != 0)
            throw std::runtime_error("Construction test invoked a later optimization stage");
        if (!objectiveEqual(result.plan.validate(p), expected) || !objectiveEqual(result.objective, expected)) {
            std::cerr << p.name << ": expected " << expected << ", got " << result.objective << '\n';
            ++failures;
        }
        return result;
    };

    Problem hub;
    hub.name = "one-hub demands jointly support their outgoing block";
    hub.nodes = std::vector<Node>(4, {true, 10, 100, 0});
    hub.arcs = {{0, 2, 0, 1, 1}, {1, 2, 1, 1, 1}, {2, 3, 2, 1, 1}};
    hub.capacities = {100, 100, 100};
    hub.blocks = {{0, 2, 1, 1, 100, 5, 1, 0, 1, {}},
                  {1, 2, 1, 1, 100, 5, 1, 0, 1, {}},
                  {2, 3, 1, 8, 100, 5, 1, 0, 1, {}}};
    hub.demands = {{0, 3, 4, 10, {0, 2}}, {1, 3, 4, 10, {1, 2}}};
    check(hub, 19);

    Problem partner;
    partner.name = "ordinary OD block supported by a three-block partner";
    partner.nodes = std::vector<Node>(4, {true, 10, 100, 0});
    partner.arcs = {{1, 2, 0, 1, 1}, {0, 1, 1, 1, 1}, {2, 3, 2, 1, 1}};
    partner.capacities = {100, 100, 100};
    partner.blocks = {{1, 2, 1, 8, 100, 5, 1, 0, 1, {}},
                      {0, 1, 1, 1, 100, 5, 1, 0, 1, {}},
                      {2, 3, 1, 1, 100, 5, 1, 0, 1, {}}};
    partner.demands = {{1, 2, 4, 10, {0, 1, 2}}, {0, 3, 4, 10, {0, 1, 2}}};
    check(partner, 19);

    Problem savings;
    savings.name = "already served traffic does not reclaim its rejection saving";
    savings.nodes = std::vector<Node>(8, {true, 10, 100, 0});
    savings.arcs = {{0, 5, 0, 1, 1}, {0, 3, 1, 1, 1}, {1, 3, 2, 1, 1}, {2, 3, 3, 1, 1},
                    {3, 4, 4, 1, 1}, {4, 5, 5, 1, 1}, {4, 6, 6, 1, 1}, {4, 7, 7, 1, 1}};
    savings.capacities = std::vector<double>(8, 100);
    savings.capacities[4] = 8;
    for (const auto &a : savings.arcs)
        savings.blocks.push_back({a.from, a.to, 1, 1, 100, 5, 0, 0, 1, {}});
    savings.blocks[4].lower = savings.blocks[4].upper = 8;
    savings.demands = {{0, 5, 4, 100, {0, 1, 4, 5}}, {1, 6, 4, 20, {2, 4, 6}},
                       {2, 7, 4, 10, {3, 4, 7}}, {3, 4, 1, .01, {4}}};
    check(savings, 28.01, true);

    Problem independent;
    independent.name = "relative stopping waits for the complete sweep";
    independent.nodes = std::vector<Node>(3, {true, 10, 100, 0});
    independent.arcs = {{0, 1, 0, 1, 1}, {0, 2, 1, 1, 1}};
    independent.capacities = {100, 100};
    independent.blocks = {{0, 1, 1, 1, 100, 5, 0, 0, 1, {}},
                          {0, 2, 1, 1, 100, 5, 0, 0, 1, {}}};
    independent.demands = {{0, 1, 1, 10, {0}}, {0, 2, 1, 10, {1}}};
    // The first move saves 45%, but the whole sweep saves 90% and must serve both demands.
    const auto stopped = check(independent, 2, false, .9);
    const auto complete = check(independent, 2, false, 0);
    if (stopped.profile.at("incremental_stop_reason") != "relative_sweep_tailoff" ||
        stopped.profile.at("incremental_sweeps") != 1 ||
        complete.profile.at("incremental_stop_reason") != "no_improving_sweep" ||
        complete.profile.at("incremental_sweeps") != 2)
        throw std::runtime_error("Whole-sweep threshold equality or zero override failed");
    for (auto &d : independent.demands) d.penalty = 0;
    const auto zero = check(independent, 0);
    if (zero.profile.at("incremental_sweeps") != 1 ||
        zero.profile.at("incremental_stop_reason") != "no_improving_sweep")
        throw std::runtime_error("Zero-cost construction did not stop after a complete sweep");
    if (failures) return 1;
    std::cout << "CONSTRUCTION_TESTS_OK\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

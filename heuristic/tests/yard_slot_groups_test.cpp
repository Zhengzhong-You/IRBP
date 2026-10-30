/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/pool_reduction.hpp"
#include "irbp/local_search.hpp"
#include <iostream>
#include <set>

using namespace RouteOpt::Application::IRBP;

void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

struct Fixture {
    Problem p;
    Plan center;
    PathPool paths;
    Fixture() {
        p.name = "yard-slot-candidate-groups";
        p.nodes = {{true, 1, 1000, 0}, {true, 1, 1000, 0}, {true, 1, 1000, 0}};
        p.arcs = {{0, 1, 0, 1, 1}, {0, 2, 1, 1, 1}, {1, 2, 2, 1, 1}};
        p.capacities = {1000, 1000, 1000};
        p.blocks = {{0, 1, 1, 1, 100, 2, 0, 0, 1, {}},
                    {0, 2, 1, 1, 100, 2, 0, 0, 1, {}},
                    {1, 2, 0, 1, 100, 2, 0, 0, 1, {}}};
        p.demands = {{0, 2, 10, 100, {1}}, {0, 1, 5, 1, {0}}, {0, 1, 5, 1, {0}},
                     {1, 2, 1, 250, {2}}, {1, 2, 1, 250, {2}}, {1, 2, 1, 250, {2}}};
        p.rejection = 1760;
        p.physical.build(3, {0, 0, 1}, {1, 2, 2});
        p.reverse_physical.build(3, {1, 2, 2}, {0, 0, 1});
        p.logical.build(3, {0, 0, 1}, {1, 2, 2});
        center = Plan::empty(p);
        center.logical[1] = center.logical[2] = {0};
        center.served[1] = center.served[2] = 5;
        center.physical[0] = {0};
        paths = {{{1}}, {{0}}, {{0}}, {{2}}, {{2}}, {{2}}};
    }
};

std::set<int> selected(const Fixture &f, int limit) {
    Json report;
    const auto fixed = selectRepairDemands(f.p, f.center, f.paths, limit,
        [](int, const std::vector<int> &) { return 1.; }, report);
    std::set<int> result;
    for (int k = 0; k < int(fixed.size()); ++k) if (!fixed[k]) result.insert(k);
    require(int(result.size()) <= limit, "Group cap was exceeded");
    return result;
}

int main() try {
    Fixture f;
    require(selected(f, 3) == std::set<int>({0, 1, 2}), "Whole donor users plus target were not selected");
    f.p.nodes[0].tracks = 2;
    require(selected(f, 3) == std::set<int>({0, 3, 4}), "Spare tracks incorrectly created slot-swap group");
    f.p.nodes[0].tracks = 1;
    f.p.blocks[1].slots = 0;
    require(selected(f, 3) == std::set<int>({0, 3, 4}), "Zero-slot target incorrectly required a donor");
    f.p.blocks[1].slots = 1;
    f.p.blocks[0].slots = 0; f.p.nodes[0].tracks = 0;
    require(selected(f, 3) == std::set<int>({0, 3, 4}), "Zero-slot donor invented a free track");
    f = Fixture();
    f.p.blocks[0].slots = f.p.blocks[1].slots = 2; f.p.nodes[0].tracks = 2;
    require(selected(f, 3) == std::set<int>({0, 1, 2}), "Multiple-slot donor release was miscounted");
    f.p.blocks[1].slots = 3;
    require(selected(f, 3) == std::set<int>({0, 3, 4}), "Insufficient donor slots admitted a target");
    f = Fixture();
    require(selected(f, 2) == std::set<int>({0, 3}), "Oversized donor group was partially admitted");
    f.p.demands[3].penalty = f.p.demands[4].penalty = 400;
    require(selected(f, 3) == std::set<int>({0, 3, 4}), "New group bypassed the original mean-score ordering");
    f = Fixture();
    f.p.blocks.push_back({1, 2, 0, 8, 100, 2, 0, 0, 1, {}});
    f.center.physical.resize(4); f.center.physical[3] = {2};
    f.center.logical[1] = {0, 3};
    f.p.demands[1].to = 2; f.p.demands[1].blocks = {0, 3}; f.paths[1] = {{0, 3}};
    f.p.demands[5] = {1, 2, 5, 1, {3}};
    f.center.logical[5] = {3}; f.center.served[5] = 5; f.paths[5] = {{3}};
    require(selected(f, 4) == std::set<int>({0, 1, 2, 5}), "Minimum-load closure partner was omitted");
    require(selected(f, 3) == std::set<int>({0, 3, 4}), "Oversized closure was partly released");

    // Different direct targets need different slot releases; tie index chooses demand0.
    f = Fixture();
    f.p.nodes.push_back({true, 1, 1000, 0}); f.p.nodes[0].tracks = 2;
    f.p.arcs.push_back({0, 3, 3, 1, 1}); f.p.capacities.push_back(1000);
    f.p.blocks.push_back({0, 3, 1, 1, 100, 2, 0, 0, 1, {}});
    f.p.blocks.push_back({0, 2, 2, 1, 100, 2, 0, 0, 1, {}});
    f.center.physical.resize(5); f.center.physical[3] = {3};
    f.center.logical[2] = {3}; f.p.demands[2].to = 3; f.p.demands[2].blocks = {3}; f.paths[2] = {{3}};
    f.p.demands[5] = {0, 2, 10, 100, {4}}; f.paths[5] = {{4}};
    require(selected(f, 3) == std::set<int>({0, 1, 5}), "Anchor score tie did not use demand index");
    f.p.demands[5].penalty = 101;
    require(selected(f, 3) == std::set<int>({0, 3, 5}), "Anchor did not use the existing highest demand score");

    f = Fixture();
    Json report;
    const auto all = selectRepairDemands(f.p, f.center, f.paths, 6,
        [](int, const std::vector<int> &) -> double { throw std::runtime_error("Small early return changed"); }, report);
    require(std::none_of(all.begin(), all.end(), [](char value) { return value; }), "Small instance was not fully free");
    // Candidate grouping does not force a closed block open or waive its minimum.
    f.p.blocks[1].lower = 20; f.p.indexCompatibility();
    LocalSearchOptions options; options.joint_repair_max_demands = 3;
    Budget budget; budget.seconds = 5;
    const auto result = optimizeCandidatePaths(f.p, f.center, f.paths, {{{0}}, {{1}}, {{2}}}, budget, options);
    require(result.plan.served[0] == 0, "An unsupportable target was forced into service");
    result.plan.validate(f.p);
    std::cout << "YARD_SLOT_GROUP_BOUNDARIES_PASS whole-donor/closure/score/slots/early-return/model-minimum\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

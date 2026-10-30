/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search.hpp"
#include "irbp/pool_reduction.hpp"
#include <iostream>
using namespace RouteOpt::Application::IRBP;
namespace {
    void require(bool value, const char *message) {
        if (!value)
            throw std::runtime_error(message);
    }
    void absoluteDemandCap() {
        Problem p;
        p.nodes.resize(2);
        p.blocks = {{0, 1, 1, 10, 20, 2, 1000, 0, 1, {}},
                    {0, 1, 1, 1, 20, 2, 1, 0, 1, {}}};
        p.demands = {{0, 1, 6, 10, {0}}, {0, 1, 6, 10, {0}}, {0, 1, 5, 10, {1}}};
        Plan center = Plan::empty(p);
        center.logical = {{0}, {0}, {}};
        center.served = {6, 6, 0};
        const PathPool paths = {{{0}}, {{0}}, {{1}}};
        Json audit;
        auto zero_cost = [](int, const auto &) { return 0.; };
        require(selectRepairDemands(p, center, paths, 1, zero_cost, audit) == std::vector<char>({1, 1, 0}),
                "An oversized minimum-load group was truncated instead of skipped");
        require(selectRepairDemands(p, center, paths, 2, zero_cost, audit) == std::vector<char>({0, 0, 1}),
                "The complete high-opening-cost group was not released together");
        int calls = 0;
        require(selectRepairDemands(p, center, paths, 3, [&](int, const auto &) { ++calls; return 0.; }, audit)
                    == std::vector<char>({0, 0, 0}) && calls == 0,
                "A small instance was restricted or needlessly ranked");
        require(selectRepairDemands(p, center, paths, 0, zero_cost, audit) == std::vector<char>({0, 0, 0}),
                "Zero cap did not preserve the unrestricted model");
        p.demands.assign(301, {0, 1, 1, 1, {1}});
        for (int k = 0; k < 301; ++k) p.demands[k].penalty = k + 1;
        p.demands[0].penalty = 1e12;
        center = Plan::empty(p);
        PathPool large(301, {{1}});
        large[0].clear();
        const auto fixed = selectRepairDemands(p, center, large, 300, zero_cost, audit);
        require(fixed[0] && std::count(fixed.begin(), fixed.end(), false) == 300,
                "The absolute cap exceeded300 or spent a place on a no-path demand");
        const auto default_fixed = selectRepairDemands(
            p, center, large, LocalSearchOptions{}.joint_repair_max_demands, zero_cost, audit);
        require(default_fixed[0] && default_fixed[50] && !default_fixed[51] && !default_fixed[300] &&
                    std::count(default_fixed.begin(), default_fixed.end(), false) == 250,
                "The default 250-demand cap crossed its boundary or released a no-path demand");
        const auto two = selectRepairDemands(p, center, large, 2, zero_cost, audit);
        require(!two[300] && !two[299] && std::count(two.begin(), two.end(), false) == 2,
                "Demand losses were not ranked in the improving direction");
    }
    void bindingLinkGroups() {
        Problem p;
        p.nodes.assign(11, {true, 10, 100, 0});
        p.arcs = {{0, 1, 0, 1, 1}, {1, 2, 1, 1, 1}, {3, 0, 2, 1, 1},
                  {2, 4, 3, 1, 1}, {2, 6, 4, 1, 1}, {7, 8, 5, 1, 1}};
        p.capacities = {10, 100, 100, 100, 100, 100};
        p.blocks = {{0, 2, 1, 1, 100, 10, 100, 0, 1, {}},
                    {3, 4, 1, 1, 100, 10, 80, 0, 1, {}},
                    {2, 6, 1, 1, 100, 10, 90, 0, 1, {}},
                    {7, 8, 1, 1, 100, 10, 1, 0, 1, {}}};
        p.demands = {{0, 2, 5, 1, {0}}, {3, 4, 5, 1, {1}},
                     {2, 6, 5, 1, {2}}, {7, 8, 5, 1, {3}}};
        Plan incumbent = Plan::empty(p);
        incumbent.logical = {{0}, {1}, {2}, {3}};
        incumbent.physical = {{0, 1}, {2, 0, 1, 3}, {4}, {5}};
        incumbent.served = {5, 5, 5, 5};
        p.physical.build(p.nodes.size(), {0, 1, 3, 2, 2, 7}, {1, 2, 0, 4, 6, 8});
        p.logical.build(p.nodes.size(), {0, 3, 2, 7}, {2, 4, 6, 8});
        require(std::isfinite(incumbent.validate(p)), "Invalid binding-link group fixture");
        PathPool paths = {{{0}}, {{1}}, {{2}}, {{3}}};
        Json audit;
        auto unit = [](int, const std::vector<int> &) { return 0.; };
        const auto linked = selectRepairDemands(p, incumbent, paths, 2, unit, audit);
        require(linked == std::vector<char>({0, 0, 1, 1}) && audit["free_count"] == 2,
                "Binding physical-link users with different blocks were not released together");
        p.capacities[0] = 11;
        const auto slack = selectRepairDemands(p, incumbent, paths, 2, unit, audit);
        require(slack == std::vector<char>({0, 1, 0, 1}),
                "A slack physical link changed the ordinary group selection");
        p.capacities[0] = p.capacities[1] = 10;
        const auto duplicate = selectRepairDemands(p, incumbent, paths, 2, unit, audit);
        require(duplicate == linked,
                "Duplicate binding-resource user groups changed selected demands");
        require(selectRepairDemands(p, incumbent, paths, 4, unit, audit) ==
                    std::vector<char>({0, 0, 0, 0}),
                "The small-instance early return changed");
        // Releasing k0 underfills another block and closure must also include k2.
        p.blocks[2].lower = 8;
        p.demands[0].to = 6; p.demands[0].blocks = {0, 2};
        incumbent.logical[0] = {0, 2}; paths[0] = {{0, 2}};
        require(std::isfinite(incumbent.validate(p)), "Invalid closure fixture");
        require(selectRepairDemands(p, incumbent, paths, 3, unit, audit) ==
                    std::vector<char>({0, 0, 0, 1}),
                "Binding-link group bypassed the minimum-load closure");
        require(selectRepairDemands(p, incumbent, paths, 2, unit, audit) ==
                    std::vector<char>({0, 1, 0, 1}),
                "An oversized binding-link closure was truncated instead of skipped");
    }
    void fixedAssignmentAccounting() {
        Problem p;
        p.nodes = {{true, 2, 100, 0}, {true, 2, 100, 0}};
        p.arcs = {{0, 1, 0, 1, 1}, {0, 1, 1, 1, 1}};
        p.capacities = {10, 1};
        p.blocks = {{0, 1, 1, 4, 10, 2, 1, 0, 1, {}},
                    {0, 1, 1, 1, 10, 2, 2, 0, 3, {}}};
        p.demands = {{0, 1, 6, 2, {0}}, {0, 1, 4, 10, {0, 1}}, {0, 1, 1, 1, {1}}};
        p.rejection = 53;
        p.indexCompatibility();
        p.physical.build(2, {0, 0}, {1, 1});
        p.reverse_physical.build(2, {1, 1}, {0, 0});
        p.logical.build(2, {0, 0}, {1, 1});
        Plan center = Plan::empty(p);
        center.logical = {{0}, {}, {1}};
        center.served = {6, 0, 1};
        center.physical = {{0}, {1}};
        const PathPool logical = {{{0}}, {{0}}, {{1}}}, physical = {{{0}}, {{1}}};
        LocalSearchOptions opt;
        opt.joint_repair_max_demands = 1;
        opt.joint_arc_max_model_size = 0;
        opt.record_candidate_pool = true;
        Budget budget; budget.seconds = 4;
        auto result = optimizeCandidatePaths(p, center, logical, physical, budget, opt);
        require(std::abs(result.objective - 16) < 1e-7 && result.plan.served == std::vector<double>({6, 4, 1}),
                "Fixed outside volumes or fixed block costs were lost");
        const auto &audit = result.profile["fixing"];
        require(audit["outside_load"] == Json::array({6., 1.}) && audit["active_blocks"] == Json::array({1, 0}) &&
                    audit["fixed_links"] == Json::array({0., 1.}) && audit["objective_constant"] == 45.,
                "Shared loads, fixed capacities or objective constants were substituted incorrectly");
        require(result.profile["columns"] == 4 && result.plan.physical[1] == center.physical[1],
                "Fixed variables were constructed or an unaffected physical path changed");
    }
    void supportRestoration() {
        Problem p;
        p.nodes = {{true, 5, 100, 0}, {true, 5, 100, 0}};
        p.arcs = {{0, 1, 0, 1, 1}};
        p.capacities = {100};
        p.blocks = {{0, 1, 1, 1, 100, 2, 1, 0, 1, {}},
                    {0, 1, 1, 15, 100, 2, 2, 0, 1, {}}};
        p.demands = {{0, 1, 8, 10, {0, 1}}, {0, 1, 7, 2, {0, 1}}};
        Plan center = Plan::empty(p);
        center.logical[1] = {0};
        center.physical[0] = {0};
        center.served[1] = 7;
        PathPool paths = {{{1}}, {{0}, {1}}};
        PoolLPInfo lp{true, 100, 0, {{0}, {1, 0}}, {{0}, {7, 0}}, {{0}, {0, 20}}};
        LocalSearchOptions opt;
        opt.pool_fix_fraction = .5;
        opt.pool_extra_paths = 1;
        auto reduced = reducePool(p, center, paths, lp, opt);
        require(reduced.audit["eligible_demands"] == Json::array({1}), "Wrong fixing eligibility");
        require(reduced.audit["unfixed_for_support"] == Json::array({1}),
                "Minimum-load partner was not released");
        require(reduced.logical == paths && !reduced.fixed[0] && !reduced.fixed[1],
                "Minimum-load partner path or unserved demand removed");
        lp.volume[1][0] = 0;
        auto no_service = reducePool(p, center, paths, lp, opt);
        require(no_service.audit["eligible_demands"].empty(), "LP-unserved demand was fixed");
    }
    void protectedPaths() {
        Problem p;
        p.nodes = {{true, 10, 100, 0}, {true, 10, 100, 0}};
        p.arcs = {{0, 1, 0, 1, 1}};
        p.capacities = {100};
        for (int b = 0; b < 7; ++b)
            p.blocks.push_back({0, 1, 1, 1, 100, 2, 1, 0, 1, {}});
        p.demands = {{0, 1, 5, 10, {0, 1, 2, 3, 4, 5, 6}}};
        Plan center = Plan::empty(p);
        center.logical[0] = {0}; center.served[0] = 5; center.physical[0] = {0};
        PathPool paths = {{{0}, {1}, {2}, {3}, {4}, {5}, {6}}};
        PoolLPInfo lp{true, 10, 0, {{.6, .4, 0, 0, 0, 0, 0}}, {{3, 2, 0, 0, 0, 0, 0}},
                      {{0, 0, 1, 2, 3, 4, 5}}};
        LocalSearchOptions opt;
        opt.pool_fix_fraction = .5; opt.pool_extra_paths = 2;
        auto reduced = reducePool(p, center, paths, lp, opt);
        require(reduced.logical[0] == PathPool::value_type{{0}, {1}, {2}, {3}},
                "Protected paths consumed the additional-path allowance");
        require(!reduced.fixed[0], "Fractional LP path choice was fixed");
    }
    void samePoolAndCutoff() {
        Problem p;
        p.name = "explicit pool and free outside volume";
        p.nodes = {{true, 2, 100, 0}, {true, 2, 100, 0}};
        p.arcs = {{0, 1, 0, 1, 1}};
        p.capacities = {6};
        p.blocks = {{0, 1, 1, 4, 10, 2, 1, 0, 1, {}}};
        p.demands = {{0, 1, 4, 10, {0}}, {0, 1, 6, 2, {0}}};
        p.block_demands = {{0, 1}}; p.rejection = 52;
        p.physical.build(2, {0}, {1}); p.reverse_physical.build(2, {1}, {0});
        p.logical.build(2, {0}, {1});
        Plan center = Plan::empty(p);
        center.logical[1] = {0}; center.served[1] = 6; center.physical[0] = {0};
        require(std::abs(center.validate(p) - 47) < 1e-8, "Invalid pool fixture");
        PathPool paths = {{{0}}, {{0}}}, physical = {{{0}}};
        for (double fixed : {0., .5})
            for (int extra : {0, 4}) {
                LocalSearchOptions opt;
                opt.pool_fix_fraction = fixed; opt.pool_extra_paths = extra;
                opt.joint_arc_max_model_size = 0;
                opt.record_candidate_pool = true;
                Budget budget; budget.seconds = 4;
                auto result = optimizeCandidatePaths(p, center, paths, physical, budget, opt);
                require(std::abs(result.objective - 15) < 1e-7, "Pool replay lost an explicit logical path");
                require(std::abs(result.plan.served[1] - 2) < 1e-7,
                        "Fixing a logical path incorrectly fixed its volume");
                require(result.profile["cutoff"] == 47 &&
                        result.profile["pool_model_history"][0]["cutoff"] == 47,
                        "Cutoff differs from the common incumbent");
                const auto &model = result.profile["pool_model_history"][0];
                require(model["physical_paths"] == 1, "Physical pool was reduced");
                require(center.served[1] == 6 && paths[0] == PathPool::value_type{{0}},
                        "Replay mutated the frozen input");
            }
        p.blocks[0].lower = 7;
        p.capacities[0] = 10;
        const auto empty = Plan::empty(p);
        const PathPool no_logical = {{}, {}};
        LocalSearchOptions opt;
        opt.joint_arc_max_model_size = 0;
        Budget budget; budget.seconds = 4;
        auto result = optimizeCandidatePaths(p, empty, no_logical, physical, budget, opt);
        require(std::abs(result.objective - 52.) < 1e-7,
                "Repair created logical paths outside the explicit candidate pool");
    }
}
int main() try {
    absoluteDemandCap();
    fixedAssignmentAccounting();
    bindingLinkGroups();
    supportRestoration();
    protectedPaths();
    samePoolAndCutoff();
    std::cout << "POOL_REDUCTION_OK support, protected paths, explicit pool, free volumes, common cutoff\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

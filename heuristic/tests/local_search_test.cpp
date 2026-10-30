/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search.hpp"
#include "irbp/model.hpp"
#include "solver.hpp"
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
    for (int k = 0; k < (int)p.demands.size(); ++k) {
        p.rejection += p.demands[k].quantity * p.demands[k].penalty;
        for (int b : p.demands[k].blocks)
            p.block_demands[b].push_back(k);
    }
}
void expect(Problem p, double optimum) {
    finish(p);
    Budget budget;
    budget.seconds = 3;
    LocalSearchOptions options;
    options.record_candidate_pool = true;
    auto r = solveLocalSearch(p, budget, options);
    if (std::abs(r.objective - optimum) > 1e-6 || std::abs(r.plan.validate(p) - optimum) > 1e-6)
        throw std::runtime_error(p.name + ": expected " + std::to_string(optimum) + ", got " +
                                 std::to_string(r.objective));
    if (r.profile.at("pool_mip_calls").get<int>() > 0) {
        const auto &trace = r.profile.at("candidate_pool");
        if (trace.at("iteration") != r.profile.at("pool_mip_calls") ||
            trace.at("logical_paths").size() != p.demands.size() ||
            trace.at("physical_paths").size() != p.blocks.size())
            throw std::runtime_error("Candidate diagnostic does not describe the last joint MIP");
        int outside_pool = 0;
        for (int k = 0; k < (int)p.demands.size(); ++k)
            if (!r.plan.logical[k].empty()) {
                const auto &paths = trace.at("logical_paths")[k];
                const auto &implicit = trace.at("implicit_logical_demands");
                if (std::find(paths.begin(), paths.end(), Json(r.plan.logical[k])) == paths.end() &&
                    std::find(implicit.begin(), implicit.end(), Json(k)) == implicit.end())
                    throw std::runtime_error("Returned logical path is missing from the diagnostic pool");
                outside_pool += std::find(paths.begin(), paths.end(), Json(r.plan.logical[k])) == paths.end();
            }
        if (outside_pool > trace.at("implicit_logical_limit").get<int>())
            throw std::runtime_error("Joint repair exceeded its implicit-path limit");
        for (int b = 0; b < (int)p.blocks.size(); ++b)
            if (!r.plan.physical[b].empty()) {
                const auto &paths = trace.at("physical_paths")[b];
                if (std::find(paths.begin(), paths.end(), Json(r.plan.physical[b])) == paths.end())
                    throw std::runtime_error("Returned physical path is missing from the diagnostic pool");
            }
    }
    std::cout << p.name << " " << r.objective << " " << r.seconds << '\n';
}
void checkJointArcGuard() {
    Problem p;
    p.name = "size-guarded joint ARC finds an unlisted logical chain";
    p.nodes = std::vector<Node>(3, {true, 10, 100, 0});
    p.arcs = {{0, 2, 0, 1, 10}, {0, 1, 1, 1, 1}, {1, 2, 2, 1, 1}};
    p.capacities = {100, 100, 100};
    p.blocks = {{0, 2, 1, 1, 10, 2, 0, 0, 1, {}}, {0, 1, 1, 1, 10, 2, 0, 0, 1, {}},
                {1, 2, 1, 1, 10, 2, 0, 0, 1, {}}};
    p.demands = {{0, 2, 10, 100, {0, 1, 2}}};
    finish(p);
    const PathPool physical = {{{0}}, {{1}}, {{2}}};
    auto solve = [&](const Problem &problem, int limit, int free_limit = 300) {
        Plan incumbent = Plan::empty(problem);
        PathPool logical(problem.demands.size(), {{0}});
        for (int k = 0; k < int(problem.demands.size()); ++k) {
            incumbent.logical[k] = {0}; incumbent.served[k] = 10;
        }
        incumbent.physical[0] = {0};
        Budget budget; budget.seconds = 3;
        LocalSearchOptions options;
        options.joint_arc_max_model_size = limit;
        options.joint_repair_max_demands = free_limit;
        options.record_candidate_pool = true;
        return optimizeCandidatePaths(problem, incumbent, logical, physical, budget, options);
    };
    const auto off = solve(p, 0), on = solve(p, 100000);
    const auto &guard = on.profile.at("pool_model_history")[0].at("arc_size_guard");
    const int threshold = guard.at("estimated_total_upper");
    const auto boundary = solve(p, threshold), rejected = solve(p, threshold - 1);
    if (!objectiveEqual(off.objective, 100) || !objectiveEqual(on.objective, 20) ||
        !objectiveEqual(boundary.objective, on.objective) || !objectiveEqual(rejected.objective, off.objective) ||
        rejected.profile.at("columns") != off.profile.at("columns") ||
        rejected.profile.at("rows") != off.profile.at("rows") ||
        rejected.profile.at("pool_model_history")[0].at("arc_size_guard").at("enabled").get<bool>() ||
        !guard.at("enabled").get<bool>() ||
        on.profile.at("columns").get<int>() != guard.at("estimated_variables").get<int>() ||
        on.profile.at("rows").get<int>() > guard.at("estimated_rows_upper").get<int>() ||
        on.plan.logical[0] != std::vector<int>({1, 2}) || !objectiveEqual(on.plan.validate(p), on.objective))
        throw std::runtime_error("Joint ARC threshold, fallback or unlisted path semantics failed");
    Problem two = p;
    two.name = "joint ARC retains the global one-demand limit";
    two.demands.push_back(p.demands[0]);
    for (auto &block : two.blocks) block.upper = 20;
    two.rejection = 0; two.block_demands.assign(two.blocks.size(), {}); finish(two);
    const auto one_arc = solve(two, 100000), fixed_outside = solve(two, 100000, 1);
    if (!objectiveEqual(one_arc.objective, 120) || !objectiveEqual(fixed_outside.objective, 120) ||
        !objectiveEqual(one_arc.plan.validate(two), 120) ||
        !objectiveEqual(fixed_outside.plan.validate(two), 120) ||
        one_arc.profile.at("pool_model_history")[0].at("implicit_selected").size() != 1)
        throw std::runtime_error("Joint ARC split/global-limit/fixed-outside semantics failed");
    std::cout << "JOINT_ARC_SIZE_GUARD_AND_GLOBAL_LIMIT_OK\n";
}
int main() try {
    checkJointArcGuard();
    {
        Model model(1, "incumbent callback");
        model.var(-1, 1, RouteOpt::SOLVER_BINARY);
        check(model.solver.updateModel());
        bool called = false;
        check(model.solver.setSolutionCallback(1, 0., [&](double cost, const std::vector<double> &x) {
            if (!objectiveEqual(cost, -1) || x.size() != 1 || x[0] != 1)
                throw std::runtime_error("Invalid MIP incumbent callback");
            called = true;
            throw std::runtime_error("simulated incumbent callback failure");
        }));
        bool propagated = false;
        try { check(model.solver.mipOptimize()); }
        catch (const std::runtime_error &e) {
            propagated = std::string(e.what()) == "simulated incumbent callback failure";
        }
        if (!called || !propagated) throw std::runtime_error("Callback failure was lost");
    }
    CostSum total;
    {
        Model model(1, "validated callback threshold");
        model.var(-1, 1, RouteOpt::SOLVER_BINARY);
        check(model.solver.updateModel());
        double validated = 0.;
        check(model.solver.setSolutionCallback(1, validated, [&](double, const std::vector<double> &) {
            validated = -.5;
        }, &validated));
        check(model.solver.mipOptimize());
        if (model.solver.solution_best != validated || validated != -.5)
            throw std::runtime_error("Callback threshold used raw rather than validated objective");
    }
    total += 1e10;
    for (int i = 0; i < 10000; ++i)
        total += .001;
    if (!objectiveEqual(total.value, 1e10 + 10) || objectiveEqual(1e10, 1e10 + 1) ||
        objectiveEqual(std::numeric_limits<double>::infinity(), 1.))
        throw std::runtime_error("Objective roundoff allowance is incorrect");
    {
        Problem p;
        p.nodes = std::vector<Node>(2, {true, 1, 10, 0});
        p.arcs = {{0, 1, 0, 1, 1}};
        p.capacities = {1};
        p.blocks = {{0, 1, 1, 1, 1, 1, 1e9, 0, 1, {}}};
        p.demands = {{0, 1, 2, 1e9, {0}}};
        finish(p);
        Plan decoded = Plan::empty(p);
        decoded.logical[0] = {0};
        decoded.physical[0] = {0};
        decoded.served[0] = 1;
        const double cost = decoded.validate(p);
        Model model(1, "decoded objective replay");
        model.constant(p.rejection);
        model.var(1e9, 1, RouteOpt::SOLVER_BINARY);
        model.var(-1e9, 2);
        model.var(1, 1);
        // Tiny integer/balance residuals can cause a large raw-cost discrepancy.
        if (objectiveEqual(model.objective({1 - 1e-9, 1, 1 + 1e-9}), cost) ||
            !objectiveEqual(model.objective({1, 1, 1}), cost))
            throw std::runtime_error("Decoded feasible objective was confused with raw MIP values");
        Model wrong(1, "incorrect coefficient remains detectable");
        wrong.constant(p.rejection);
        wrong.var(1e9, 1, RouteOpt::SOLVER_BINARY);
        wrong.var(-1e9, 2);
        wrong.var(2, 1);
        if (objectiveEqual(wrong.objective({1, 1, 1}), cost))
            throw std::runtime_error("Objective replay masked an incorrect model coefficient");
    }
    {
        // Reassigning already fully served traffic is a no-op even when rejection costs
        // dwarf routing costs. Large cancellation must not abort a feasible warm start.
        Problem p;
        p.name = "large-cost warm-start cancellation";
        p.nodes = std::vector<Node>(3, {true, 10, 2e6, 0});
        p.arcs = {{0, 1, 0, 1, .156789}};
        p.capacities = {2e6};
        p.blocks = {{0, 1, 1, 1, 2e6, 2, .35, .210987, 1, {}}};
        for (int k = 0; k < 1024; ++k)
            p.demands.push_back({0, 1, 1234.567, 76543.21, {0}});
        p.demands.push_back({0, 2, 1, 3302141634.82724, {}});
        finish(p);
        Plan initial = Plan::empty(p);
        for (int k = 0; k < 1024; ++k) {
            initial.logical[k] = {0};
            initial.served[k] = p.demands[k].quantity;
        }
        initial.physical[0] = {0};
        const double optimum = initial.validate(p);
        LocalSearchOptions options;
        options.min_joint_seconds = options.min_pool_seconds = 100;
        Budget budget;
        budget.seconds = 5;
        const auto result = solveLocalSearch(p, budget, options, &initial);
        if (!objectiveEqual(result.objective, optimum) || !result.history.empty())
            throw std::runtime_error("Cancellation created a fictitious improvement");
    }
    Problem direct;
    direct.name = "continuous joint minimum";
    direct.nodes = {{true, 2, 100, 0}, {true, 2, 100, 0}};
    direct.arcs = {{0, 1, 0, 1, 1}};
    direct.capacities = {6};
    direct.blocks = {{0, 1, 1, 6, 8, 2, 3, 0, 1, {}}};
    direct.demands = {{0, 1, 4, 10, {0}}, {0, 1, 4, 10, {0}}};
    expect(direct, 29);
    {
        auto mapped = direct;
        mapped.name = "active-domain final IDs";
        mapped.blocks.insert(mapped.blocks.begin(), Block{1, 0, 1, 1, 8, 2, 3, 0, 1, {}});
        for (auto &d : mapped.demands) d.blocks = {1};
        finish(mapped);
        LocalSearchOptions options;
        options.proposal_block_threshold = 1;
        Budget budget;
        budget.seconds = 3;
        auto result = solveLocalSearch(mapped, budget, options);
        if (!objectiveEqual(result.objective, 29) || !objectiveEqual(result.plan.validate(mapped), 29) ||
            result.plan.physical.size() != mapped.blocks.size() || !result.plan.physical[0].empty() ||
            result.plan.physical[1] != std::vector<int>({0}))
            throw std::runtime_error("Active-domain final plan lost original block IDs");
        for (int k = 0; k < int(mapped.demands.size()); ++k)
            if (result.plan.served[k] > 0 && result.plan.logical[k] != std::vector<int>({1}))
                throw std::runtime_error("Active-domain final logical path lost original block IDs");
    }
    Problem hub;
    hub.name = "shared hub and partial volume";
    hub.nodes = std::vector<Node>(4, {true, 3, 100, 0});
    hub.nodes[0].tracks = 1;
    hub.nodes[1].handling_cost = 2;
    hub.nodes[1].handling_capacity = 6;
    hub.arcs = {{0, 1, 0, 1, 1}, {1, 2, 1, 1, 1}, {1, 3, 2, 1, 1}};
    hub.capacities = {6, 10, 10};
    hub.blocks = {
        {0, 1, 1, 6, 8, 2, 1, 0, 1, {}}, {1, 2, 1, 2, 8, 2, 1, 0, 1, {}}, {1, 3, 1, 2, 8, 2, 1, 0, 1, {}}};
    hub.demands = {{0, 2, 4, 10, {0, 1}}, {0, 3, 4, 10, {0, 2}}};
    expect(hub, 47);
    hub.name = "minimum cannot fit handling";
    hub.nodes[1].handling_capacity = 5;
    expect(hub, 80);
    Problem parallel;
    parallel.name = "parallel blocks and physical paths";
    parallel.nodes = direct.nodes;
    parallel.arcs = {{0, 1, 0, 1, 1}, {0, 1, 1, 1, 2}};
    parallel.capacities = {4, 4};
    parallel.blocks = {{0, 1, 1, 1, 4, 2, 1, 0, 1, {}}, {0, 1, 1, 1, 4, 2, 1, 0, 1, {}}};
    parallel.demands = {{0, 1, 4, 20, {0}}, {0, 1, 4, 20, {1}}};
    expect(parallel, 14);
    Problem no_split = parallel;
    no_split.name = "one logical path cannot split one demand";
    no_split.demands = {{0, 1, 8, 20, {0, 1}}};
    expect(no_split, 85);
    Problem shared;
    shared.name = "opposite directions share one physical link";
    shared.nodes = direct.nodes;
    shared.arcs = {{0, 1, 0, 1, 1}, {1, 0, 0, 1, 1}};
    shared.capacities = {6};
    shared.blocks = {{0, 1, 1, 3, 4, 2, 1, 0, 1, {}}, {1, 0, 1, 3, 4, 2, 1, 0, 1, {}}};
    shared.demands = {{0, 1, 4, 10, {0}}, {1, 0, 4, 10, {1}}};
    expect(shared, 28);
    Problem length;
    length.name = "length cap and block costs";
    length.nodes = direct.nodes;
    length.arcs = {{0, 1, 0, 10, 1}, {0, 1, 1, 1, 5}};
    length.capacities = {10, 10};
    length.blocks = {{0, 1, 0, 1, 10, 2, 3, 2, 1, {{1, 7}}}};
    length.demands = {{0, 1, 4, 20, {0}}};
    expect(length, 39);
    Problem chain;
    chain.name = "three-block logical path";
    chain.nodes = std::vector<Node>(4, {true, 3, 10, 0});
    chain.arcs = hub.arcs;
    chain.arcs[2] = {2, 3, 2, 1, 1};
    chain.capacities = {10, 10, 10};
    chain.blocks = {
        {0, 1, 1, 1, 10, 2, 1, 0, 1, {}}, {1, 2, 1, 1, 10, 2, 1, 0, 1, {}}, {2, 3, 1, 1, 10, 2, 1, 0, 1, {}}};
    chain.demands = {{0, 3, 5, 20, {0, 1, 2}}};
    expect(chain, 18);
    chain.name = "discard unsupported cheap logical shortcut";
    chain.arcs.push_back({0, 3, 3, 1, .1});
    chain.capacities.push_back(10);
    chain.blocks.push_back({0, 3, 1, 6, 10, 2, 1, 0, 1, {}});
    chain.demands[0].blocks.push_back(3);
    expect(chain, 18);
    chain.name = "compatibility prevents forbidden transshipment";
    chain.demands[0].blocks = {0, 2};
    expect(chain, 100);
    Problem corridor;
    corridor.name = "joint three-block corridor and unsupported demand";
    corridor.nodes = std::vector<Node>(7, {true, 5, 100, 0});
    corridor.arcs = {{0, 2, 0, 1, 1}, {1, 2, 1, 1, 1},  {2, 3, 2, 1, 1},  {3, 4, 3, 1, 1},
                     {3, 5, 4, 1, 1}, {0, 4, 5, 1, .1}, {1, 5, 6, 1, .1}, {6, 2, 7, 1, 1}};
    corridor.capacities = std::vector<double>(8, 100);
    corridor.blocks = {{0, 2, 1, 4, 20, 2, 1, 0, 1, {}}, {1, 2, 1, 4, 20, 2, 1, 0, 1, {}},
                       {2, 3, 1, 8, 20, 2, 1, 0, 1, {}}, {3, 4, 1, 4, 20, 2, 1, 0, 1, {}},
                       {3, 5, 1, 4, 20, 2, 1, 0, 1, {}}, {0, 4, 1, 6, 20, 2, 0, 0, 1, {}},
                       {1, 5, 1, 6, 20, 2, 0, 0, 1, {}}, {6, 2, 1, 2, 20, 2, 1, 0, 1, {}}};
    corridor.demands = {{0, 4, 4, 20, {0, 2, 3, 5}}, {1, 5, 4, 20, {1, 2, 4, 6}}, {6, 5, 1, 100, {2, 4, 7}}};
    expect(corridor, 129);
    Problem outside;
    outside.name = "fixed outside flow uses the opposite direction capacity";
    outside.nodes = direct.nodes;
    outside.arcs = {{0, 1, 0, 1, 1}, {1, 0, 0, 1, 1}};
    outside.capacities = {6};
    outside.blocks = {{0, 1, 1, 1, 10, 2, 1, 0, 1, {}}, {1, 0, 1, 1, 10, 2, 1, 0, 1, {}}};
    for (int k = 0; k < 8; ++k)
        outside.demands.push_back({0, 1, 1, 10, {0}});
    for (int k = 0; k < 2; ++k)
        outside.demands.push_back({1, 0, 1, 100, {1}});
    expect(outside, 48);
    finish(outside);
    Budget continuation_budget;
    continuation_budget.seconds = 3;
    LocalSearchOptions continuation_options;
    continuation_options.pool_mip_max_seconds = 1e-12;
    const auto continued = solveLocalSearch(outside, continuation_budget, continuation_options);
    const auto &schedule = continued.profile.at("schedule_history");
    bool saw_joint = false, returned_to_local = false;
    std::string previous;
    for (const auto &event : schedule) {
        const std::string stage = event.at("stage");
        if (stage == "joint_start") {
            if (previous != "pricing_round_complete" ||
                event.at("completed_rounds").get<int>() < continuation_options.overall_stall_rounds ||
                event.at("stalls").get<int>() < continuation_options.overall_stall_rounds)
                throw std::runtime_error("Joint repair ran before tested pricing/local rounds tailed off");
            saw_joint = true;
        }
        if (saw_joint && stage == "local_start") returned_to_local = true;
        previous = stage;
    }
    if (!saw_joint || !returned_to_local)
        throw std::runtime_error("The paper schedule did not return from joint repair to local search");
    const int continuation_status = continued.profile.at("pool_mip_status").get<int>();
    if (std::abs(continued.objective - 48.) > 1e-6 ||
        std::abs(continued.plan.validate(outside) - 48.) > 1e-6 ||
        (continued.profile.at("pool_mip_continuations").get<int>() == 0 &&
         continuation_status != RouteOpt::SOLVER_OPTIMAL && continuation_status != RouteOpt::SOLVER_CUTOFF))
        throw std::runtime_error("A capped pool MIP stopped without using the remaining budget: " +
                                 std::to_string(continued.objective) + " " + continued.profile.dump());
    for (const auto &round : continued.profile.at("pool_model_history"))
        if (round.at("first_status") == RouteOpt::SOLVER_TIME_LIMIT &&
            objectiveEqual(round.at("first_validated_objective").get<double>(),
                           round.at("cutoff").get<double>()) && !round.at("continued").get<bool>())
            throw std::runtime_error("A time-limited unchanged feasible incumbent was not continued");
    Budget repeated_budget;
    repeated_budget.seconds = .4;
    LocalSearchOptions repeated_options;
    repeated_options.continuous_search = true;
    repeated_options.finite_stall_cycles = 2;
    repeated_options.continuous_idle_seconds = .005;
    auto repeated_search = solveLocalSearch(outside, repeated_budget, repeated_options, &continued.plan);
    if (repeated_search.profile.at("search_cycles").get<int>() < 2 ||
        repeated_search.profile.at("pool_repeat_skips").get<int>() == 0 ||
        !objectiveEqual(repeated_search.plan.validate(outside), 48) || repeated_search.seconds < .1 ||
        repeated_search.profile.at("finite_patience_stopped").get<bool>())
        throw std::runtime_error("Continuous search stopped or regressed after exhausting its candidate pool");
    Problem patience;
    patience.name = "finite patience completes post-joint local and pricing returns";
    patience.nodes = std::vector<Node>(2, {true, 2, 10, 0});
    for (int a = 0; a < 8; ++a) {
        patience.arcs.push_back({0, 1, a, 1, 0});
        patience.capacities.push_back(2);
    }
    patience.blocks = {{0, 1, 1, 1, 2, 2, 1, 0, 1, {}}};
    patience.demands = {{0, 1, 2, 10, {0}}};
    finish(patience);
    Budget patience_budget;
    patience_budget.seconds = 3;
    LocalSearchOptions patience_options;
    patience_options.open_logical_paths_per_demand = 2;
    patience_options.finite_stall_cycles = 2;
    auto patient = solveLocalSearch(patience, patience_budget, patience_options);
    const auto &profile = patient.profile;
    if (!objectiveEqual(patient.plan.validate(patience), 1) ||
        !profile.at("finite_patience_stopped").get<bool>() ||
        profile.at("joint_return_epoch").get<int>() < 2 ||
        profile.at("post_joint_local_epoch") != profile.at("joint_return_epoch") ||
        profile.at("post_joint_exposure_epoch") != profile.at("joint_return_epoch") ||
        profile.at("open_path_cache_hits").get<int>() == 0 ||
        profile.at("open_path_exhausted").get<int>() == 0)
        throw std::runtime_error("Finite patience stopped before its completion barrier or lost the incumbent");
    std::string last_stage;
    for (const auto &event : profile.at("schedule_history")) {
        if (event.at("stage") == "finite_patience_stop" && last_stage != "pricing_round_complete")
            throw std::runtime_error("Patience stopped without a completed guidance/local exposure");
        last_stage = event.at("stage");
    }
    Problem bridges;
    bridges.name = "long existing chain joined by two new blocks";
    bridges.nodes = std::vector<Node>(7, {true, 3, 100, 0});
    for (int i = 0; i < 6; ++i) {
        bridges.arcs.push_back({i, i + 1, i, 1, 1});
        bridges.capacities.push_back(10);
        bridges.blocks.push_back({i, i + 1, 1, 1, 10, 1, 1, 0, 1, {}});
    }
    bridges.arcs.push_back({0, 6, 6, 1, .1});
    bridges.capacities.push_back(1);
    bridges.blocks.push_back({0, 6, 1, 1, 1, 1, 1, 0, 1, {}});
    bridges.blocks.push_back({0, 6, 1, 1, 5, 1, 1, 0, 1, {}});
    for (int b : {1, 3, 4, 5})
        bridges.demands.push_back({b, b + 1, 1, 1000, {b}});
    bridges.demands.push_back({0, 6, 1, 1000, {6}});
    bridges.demands.push_back({0, 6, 5, 20, {0, 1, 2, 3, 4, 5, 7}});
    expect(bridges, 41.1);
    Problem unsplit;
    unsplit.name = "pool repair keeps one logical path per demand";
    unsplit.nodes = std::vector<Node>(4, {true, 3, 100, 0});
    unsplit.arcs = {{0, 1, 0, 1, 1}, {1, 3, 1, 1, 1}, {0, 2, 2, 1, 1}, {2, 3, 3, 1, 1}};
    unsplit.capacities = {4, 4, 4, 4};
    unsplit.blocks = {{0, 1, 1, 1, 4, 1, 1, 0, 1, {}},
                      {1, 3, 1, 1, 4, 1, 1, 0, 1, {}},
                      {0, 2, 1, 1, 4, 1, 1, 0, 1, {}},
                      {2, 3, 1, 1, 4, 1, 1, 0, 1, {}}};
    unsplit.demands = {{0, 3, 8, 10, {0, 1, 2, 3}}};
    expect(unsplit, 50);
    Problem resource;
    resource.name = "length feasible physical path missed by weighted distances";
    resource.nodes = std::vector<Node>(2, {true, 2, 100, 0});
    resource.arcs = {{0, 1, 0, 1, 21}, {0, 1, 1, 2, 22}, {0, 1, 2, 8, 20}, {0, 1, 3, 11, 1}};
    resource.capacities = {1, 1, 1, 1};
    resource.blocks = {{0, 1, 1, 1, 1, 9, 0, 0, 1, {}}};
    resource.demands = {{0, 1, 1, 100, {0}}};
    expect(resource, 20);
    // A shared group is stored once, including correct overlap and reverse queries.
    Problem grouped = hub;
    finish(grouped);
    grouped.compatibility_groups = {{0, 1}, {0, 2}};
    for (int k = 0; k < 2; ++k) {
        grouped.demands[k].blocks.clear();
        grouped.demands[k].block_group = k;
    }
    grouped.indexCompatibility();
    if (grouped.compatibleDemands(0) != std::vector<int>({0, 1}) || grouped.compatible(0, 2) ||
        !grouped.demands[0].blocks.empty())
        throw std::runtime_error("Shared compatibility changed the original relation");
    grouped.nodes[1].handling_capacity = 6;
    grouped.rejection = 80;
    Budget proposal_budget;
    proposal_budget.seconds = 3;
    LocalSearchOptions proposal_options;
    proposal_options.proposal_block_threshold = 1;
    proposal_options.record_candidate_pool = true;
    auto projected = solveLocalSearch(grouped, proposal_budget, proposal_options);
    if (std::abs(projected.objective - 47) > 1e-6 || std::abs(projected.plan.validate(grouped) - 47) > 1e-6 ||
        !projected.profile.contains("candidate_domain"))
        throw std::runtime_error("Demand-driven projection lost a joint hub or its original IDs");
    Problem repeated;
    repeated.name = "equal capacity rows retain the tightest original resource";
    repeated.nodes = {{true, 2, 100, 0}, {false, 0, 0, 0}, {true, 2, 100, 0}};
    repeated.arcs = {{0, 1, 0, 1, 1}, {1, 2, 1, 1, 1}};
    repeated.capacities = {6, 4, 100};
    repeated.blocks = {{0, 2, 1, 1, 4, 2, .5, 0, 1, {}}, {0, 2, 1, 1, 4, 2, .5, 0, 1, {}}};
    repeated.demands = {{0, 2, 3, 10, {0}}, {0, 2, 3, 20, {1}}};
    expect(repeated, 29);
    finish(repeated);
    Budget row_budget;
    row_budget.seconds = 3;
    auto row_result = solveLocalSearch(repeated, row_budget);
    bool compressed = false;
    for (const auto &entry : row_result.profile.at("capacity_row_history"))
        if (entry.at("stage") == "pool_lp") {
            if (entry.at("original") != 3 || entry.at("retained") != 1 || entry.at("empty") != 1 ||
                entry.at("duplicate") != 1)
                throw std::runtime_error("Capacity-row equivalence class was not preserved");
            compressed = true;
        }
    if (!compressed || !objectiveEqual(row_result.plan.validate(repeated), 29))
        throw std::runtime_error("Compressed capacity LP failed validation");
    {
        // A completed non-improving sweep hands off to LP/MIP with an unlimited budget.
        Problem p;
        p.name = "unlimited search hands off after a complete non-improving sweep";
        p.nodes = std::vector<Node>(2, {true, 2, 4000, 0});
        p.arcs = {{0, 1, 0, 1, 1}};
        p.capacities = {4000};
        p.blocks = {{0, 1, 1, 1, 4000, 1, 1, 0, 1, {}}};
        p.demands = std::vector<Demand>(512, {0, 1, 1, 10, {0}});
        finish(p);
        Plan incumbent = Plan::empty(p);
        incumbent.physical[0] = {0};
        for (int k = 0; k < int(p.demands.size()); ++k) {
            incumbent.logical[k] = {0};
            incumbent.served[k] = 1;
        }
        for (double tolerance : {0., 1e-6}) {
            Budget budget;
            budget.seconds = 1e100;
            LocalSearchOptions options;
            options.construction_relative_tolerance = tolerance;
            options.progress_log_seconds = 0;
            auto result = solveLocalSearch(p, budget, options, &incumbent);
            const auto &profile = result.profile;
            if (profile.at("incremental_stop_reason") != "no_improving_sweep" ||
                profile.at("incremental_sweeps") != 1 ||
                profile.at("guidance_lp_calls").get<int>() == 0 ||
                profile.at("pool_mip_calls").get<int>() == 0 ||
                !objectiveEqual(result.objective, 513) ||
                !objectiveEqual(result.plan.validate(p), 513))
                throw std::runtime_error("Unlimited phase handoff failed: " + profile.dump());
            if (profile.at("group_skipped_exhausted").get<int>() == 0)
                throw std::runtime_error("Rotating seeds re-solved an identical optimal subproblem");
        }
    }
    std::cout << "LOCAL_SEARCH_TESTS_OK\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

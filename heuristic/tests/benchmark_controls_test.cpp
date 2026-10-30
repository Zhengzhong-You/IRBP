/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
// Exercise the actual scheduler and private component gates, including C0 equality.
#include "../src/local_search.cpp"
#include <iostream>
#include <sstream>

using namespace RouteOpt::Application::IRBP;

void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

Problem fixture() {
    Problem p;
    p.name = "controlled-component-boundaries";
    p.nodes = {{true, 10, 1000, 0}, {true, 10, 1000, 0}, {true, 10, 1000, 0}};
    p.arcs = {{0, 2, 0, 1, 5}, {0, 1, 1, 1, 1}, {1, 2, 2, 1, 1}};
    p.capacities = {1000, 1000, 1000};
    p.blocks = {{0, 2, 1, 1, 100, 5, 1, 0, 1, {}},
                {0, 1, 1, 1, 100, 5, 1, 0, 1, {}},
                {1, 2, 1, 1, 100, 5, 1, 0, 1, {}}};
    p.demands = {{0, 2, 10, 100, {0, 1, 2}}};
    p.rejection = 1000;
    p.physical.build(3, {0, 0, 1}, {2, 1, 2});
    p.reverse_physical.build(3, {2, 1, 2}, {0, 0, 1});
    p.logical.build(3, {0, 0, 1}, {2, 1, 2});
    p.indexCompatibility();
    return p;
}

Plan incumbent(const Problem &p) {
    Plan s = Plan::empty(p);
    s.logical[0] = {0}; s.physical[0] = {0}; s.served[0] = 10;
    return s;
}

LocalSearchOptions settings() {
    LocalSearchOptions o;
    o.benchmark_record_initial_pool = true;
    o.joint_mip_max_seconds = .02; o.joint_repair_max_seconds = .05;
    o.joint_phase_max_seconds = .1; o.guidance_max_seconds = 0;
    o.support_search_seconds = 0; o.physical_alternative_seconds = 0;
    o.guidance_solver_log = false;
    o.min_joint_seconds = .001; o.min_pool_seconds = .001;
    o.final_return_reserve = .001; o.solver_start_reserve = .001;
    o.overall_stall_rounds = 2;
    return o;
}

LocalSearchResult run(const Problem &p, const LocalSearchOptions &o) {
    Budget b; b.seconds = 1;
    auto seed = incumbent(p);
    auto r = solveLocalSearch(p, b, o, &seed, false);
    require(objectiveEqual(r.objective, r.plan.validate(p)), "Returned plan failed original validation");
    return r;
}

int main() try {
    auto p = fixture();
    const auto base_options = settings();
    const auto full = run(p, base_options);
    require(!base_options.benchmark_targeted_destroy && full.profile.at("destroy_steps") == 0 &&
            full.profile.at("joint_mip_calls").get<int>() > 0,
            "Related-demand default disabled local repair or ran block-targeted destruction");
    require(full.profile.at("local_mip_integrality_focus") == 1 &&
            full.profile.at("joint_mip_integrality_focus") == 1,
            "Heuristic MIPs did not explicitly set and verify IntegralityFocus1");
    const auto c0 = full.profile.at("initial_candidate_pool");
    require(c0.at("incumbent_objective") == incumbent(p).validate(p), "C0 was recorded after improvement");
    require(!c0.at("demand_driven").get<bool>(), "Small fixture changed domain mode");
    for (int which = 0; which < 4; ++which) {
        auto o = base_options;
        if (which == 0) o.benchmark_path_generation = false;
        if (which == 1) o.benchmark_joint_repair = false;
        if (which == 2) o.benchmark_local_repair = false;
        if (which == 3) o.benchmark_targeted_destroy = true;
        const auto r = run(p, o);
        require(r.profile.at("initial_candidate_pool") == c0, "A component switch changed ordered C0");
        if (which == 0) {
            require(r.profile.at("guidance_lp_calls") == 0, "NoPG still solved a guidance LP");
            require(r.profile.at("guided_logical_paths") == 0 && r.profile.at("guided_physical_paths") == 0,
                    "NoPG still admitted dual-generated paths");
            require(r.profile.at("joint_mip_calls").get<int>() > 0 && r.profile.at("pool_mip_calls").get<int>() > 0,
                    "NoPG incorrectly disabled local or joint repair");
        }
        if (which == 1)
            require(r.profile.at("pool_mip_calls") == 0 && r.profile.at("joint_mip_calls").get<int>() > 0,
                    "NoJoint did not isolate the global coordination solve");
        if (which == 2) {
            require(r.profile.at("joint_mip_calls") == 0 && r.profile.at("destroy_steps") == 0,
                    "NoLocal still ran a local MIP or destroy");
            require(r.profile.at("guidance_lp_calls").get<int>() > 0 && r.profile.at("pool_mip_calls").get<int>() > 0,
                    "NoLocal blocked the LP-to-joint schedule");
            require(r.profile.at("completed_pricing_rounds").get<int>() >= 2,
                    "Skipped local stage was not counted as completed exposure");
        }
        if (which == 3)
            require(r.profile.at("destroy_steps").get<int>() > 0 && r.profile.at("joint_mip_calls").get<int>() > 0,
                    "Block-targeted comparator failed to retain destruction and local repair");
        const Json encoded = o;
        require(Json(readLocalSearchOptions(encoded)) == encoded, "Private flags failed option round trip");
    }
    // Without joint optimization, nondual enrichment must still be built and exposed.
    auto no_joint = base_options;
    no_joint.benchmark_joint_repair = false;
    Budget b; b.seconds = 1;
    LocalSearchResult out; out.plan = incumbent(p); out.objective = out.plan.validate(p);
    Search search(p, b, out, no_joint);
    search.ensureRoutes(0);
    require(search.improveAllDemands(), "NoJoint lost candidate enrichment");
    require(search.pool_mip_calls == 0 && !search.guided[0].empty(), "NoJoint solved a MIP or failed to add paths");
    no_joint.benchmark_path_generation = false;
    const auto enrich_then_local = run(p, no_joint);
    // In the experiments, NoJoint bypasses the whole enrichment and joint-repair call.
    require(enrich_then_local.profile.at("guidance_lp_calls") == 0 &&
            enrich_then_local.profile.at("pool_mip_calls") == 0,
            "LP or joint calls ran although both were disabled");
    // Deterministic no-PG, joint-enabled case: model reserve skips the MIP after enrichment.
    auto no_pg = base_options;
    no_pg.benchmark_path_generation = false;
    no_pg.final_return_reserve = 100.;
    const auto skipped_joint = run(p, no_pg);
    require(skipped_joint.profile.at("pool_mip_calls") == 0 &&
            skipped_joint.profile.at("guidance_lp_calls") == 0 &&
            skipped_joint.profile.at("guidance_seconds") == 0 &&
            skipped_joint.profile.at("guidance_pricing_seconds") == 0 &&
            skipped_joint.profile.at("pool_mip_seconds").get<double>() > 0 &&
            skipped_joint.profile.at("search_cycles").get<int>() >= 2,
            "NoPG discarded enriched paths when the enabled joint MIP could not start");
    // The switches are downstream of construction and preserve its initial candidate state.
    Json constructed;
    for (bool pg : {true, false}) {
        auto o = base_options; o.benchmark_path_generation = pg;
        Budget from_zero; from_zero.seconds = 1;
        const auto r = solveLocalSearch(p, from_zero, o);
        if (pg) constructed = r.profile.at("initial_candidate_pool");
        else require(r.profile.at("initial_candidate_pool") == constructed,
                     "A component switch changed construction or its ordered pools");
    }
    // A projected large-domain replay must retain demand-driven mode and original IDs.
    Budget large_budget; large_budget.seconds = 1;
    LocalSearchResult projected; projected.plan = incumbent(p); projected.objective = projected.plan.validate(p);
    Search large(p, large_budget, projected, base_options, true, {100, 200, 300});
    large.run(false);
    require(projected.profile.at("initial_candidate_pool").at("demand_driven").get<bool>() &&
            projected.profile.at("initial_candidate_pool").at("original_block_ids") == Json({100, 200, 300}) &&
            projected.profile.at("bridge_generation_seconds") == 0,
            "Demand-driven mode lost its original IDs or enabled all-yard bridges");
    // A globally expired call retains its validated feasible incumbent and never reads duals.
    Budget tiny; tiny.seconds = 0;
    LocalSearchResult retained; retained.plan = out.plan; retained.objective = out.objective;
    Search expired(p, tiny, retained, base_options);
    expired.solvePathModel();
    require(expired.guidance_lp_calls == 0 && objectiveEqual(retained.objective, retained.plan.validate(p)),
            "Expired global LP call lost the incumbent or tried pricing");
    // Diagnostics must preserve the original hard minimum-load failure.
    auto invalid = p;
    invalid.blocks[0].lower = 11.;
    Budget diag_budget; diag_budget.seconds = 1;
    LocalSearchResult diag_out; diag_out.plan = incumbent(invalid);
    Search diagnostic(invalid, diag_budget, diag_out, base_options);
    std::ostringstream captured;
    auto *original_log = std::clog.rdbuf(captured.rdbuf());
    bool hard_failure = false;
    try {
        diagnostic.validateRepairCandidate(diag_out.plan, "test", [](int, Json &record) {
            record["outside_load"] = 0.;
            record["physical_columns"] = {{{"x", 1.}, {"v", 11.}, {"min_row_residual", 0.}}};
            record["logical_columns"] = {{{"x", 0.}, {"v", 1.}, {"k", 0}}};
            record["balance_residual"] = 0.;
        });
    } catch (const std::runtime_error &error) {
        hard_failure = std::string(error.what()) == "Minimum block load";
    }
    std::clog.rdbuf(original_log);
    require(hard_failure && captured.str().find("MIP_MINIMUM_LOAD_DIAGNOSTIC") != std::string::npos &&
            captured.str().find("decoded_load") != std::string::npos &&
            captured.str().find("logical_columns") != std::string::npos,
            "Minimum-load diagnostic swallowed or changed the original validation failure");
    // Near-numerical candidates are rejected, never made feasible or installed as the incumbent.
    auto near_problem = p;
    near_problem.blocks[0].lower = 10.;
    auto valid_seed = incumbent(near_problem);
    Budget near_budget; near_budget.seconds = 1;
    LocalSearchResult near_out; near_out.plan = valid_seed; near_out.objective = valid_seed.validate(near_problem);
    const double valid_cost = near_out.objective;
    Search guarded(near_problem, near_budget, near_out, base_options);
    auto near = valid_seed; near.served[0] -= 5e-6;
    const auto rejected = guarded.validateRepairCandidate(near, "test-near", [](int, Json &) {});
    require(!rejected && guarded.rejected_candidate_checks == 1 &&
            near_out.plan.json() == valid_seed.json() && near_out.objective == valid_cost,
            "Rejected numerical candidate changed the validated incumbent");
    bool strict_min_still_fails = false;
    try { near.validate(near_problem); }
    catch (const std::runtime_error &error) { strict_min_still_fails = std::string(error.what()) == "Minimum block load"; }
    require(strict_min_still_fails, "Original Plan feasibility tolerance was relaxed");
    auto large_candidate = valid_seed; large_candidate.served[0] -= .1;
    bool large_throws = false;
    try { guarded.validateRepairCandidate(large_candidate, "test-large", [](int, Json &) {}); }
    catch (const std::runtime_error &error) { large_throws = std::string(error.what()) == "Minimum block load"; }
    require(large_throws && guarded.rejected_candidate_checks == 1, "A substantive violation was quietly rejected");
    auto other = valid_seed; other.physical[0].clear();
    bool other_throws = false;
    try { guarded.validateRepairCandidate(other, "test-other", [](int, Json &) {}); }
    catch (const std::runtime_error &error) { other_throws = std::string(error.what()) != "Minimum block load"; }
    require(other_throws && guarded.rejected_candidate_checks == 1, "A different validation error was swallowed");
    auto mixed_problem = near_problem; mixed_problem.blocks[1].lower = 10.; mixed_problem.blocks[2].lower = 10.1;
    LocalSearchResult mixed_out; mixed_out.plan = valid_seed; mixed_out.objective = valid_cost;
    Search mixed_search(mixed_problem, near_budget, mixed_out, base_options);
    auto mixed = Plan::empty(mixed_problem); mixed.logical[0] = {1, 2}; mixed.served[0] = 10. - 5e-6;
    mixed.physical[1] = {1}; mixed.physical[2] = {2};
    bool mixed_throws = false;
    try { mixed_search.validateRepairCandidate(mixed, "test-mixed", [](int, Json &) {}); }
    catch (const std::runtime_error &error) { mixed_throws = std::string(error.what()) == "Minimum block load"; }
    require(mixed_throws && mixed_search.rejected_candidate_checks == 0,
            "Near violation on one block hid a substantive violation on another");
    // Normalize only a previously validated candidate, then recheck all coupled loads.
    auto upper = valid_seed;
    upper.served[0] += 1e-6;
    const auto normalized_cost = guarded.validateRepairCandidate(upper, "test-upper", [](int, Json &) {});
    require(normalized_cost && upper.served[0] == near_problem.demands[0].quantity &&
            objectiveEqual(*normalized_cost, valid_cost),
            "A tolerated upper-bound residual retained excess volume or its understated cost");
    auto lower = Plan::empty(near_problem);
    lower.served[0] = -5e-7;
    const auto zero_cost = guarded.validateRepairCandidate(lower, "test-lower", [](int, Json &) {});
    require(zero_cost && lower.served[0] == 0. && objectiveEqual(*zero_cost, near_problem.rejection),
            "A tolerated negative volume was not normalized to zero");
    auto gross = valid_seed;
    gross.served[0] += .01;
    bool gross_throws = false;
    try { guarded.validateRepairCandidate(gross, "test-gross", [](int, Json &) {}); }
    catch (const std::runtime_error &error) { gross_throws = std::string(error.what()) == "Plan capacity violation"; }
    require(gross_throws && gross.served[0] > near_problem.demands[0].quantity,
            "Normalization hid a gross raw volume violation");
    auto normalization_problem = near_problem;
    normalization_problem.blocks[0].lower += 5e-6;
    LocalSearchResult normalization_out;
    normalization_out.plan = Plan::empty(normalization_problem);
    normalization_out.objective = normalization_out.plan.validate(normalization_problem);
    const auto normalization_incumbent = normalization_out.plan.json();
    Search normalizing(normalization_problem, near_budget, normalization_out, base_options);
    auto broken_minimum = valid_seed;
    broken_minimum.served[0] += 5e-6;
    broken_minimum.validate(normalization_problem);
    const auto normalized_rejected = normalizing.validateRepairCandidate(
        broken_minimum, "test-normalized-minimum", [](int, Json &) {});
    require(!normalized_rejected && normalizing.rejected_candidate_checks == 1 &&
            normalization_out.plan.json() == normalization_incumbent,
            "Normalization accepted a minimum-load violation or changed the valid incumbent");
    // Reproduce both strict-box residuals observed in the fresh ARC-guard campaign.
    for (const auto &[quantity, raw] : std::vector<std::pair<double,double>>{
             {700., 700.000001}, {1529., 1529.0000000000155}}) {
        Problem problem;
        problem.nodes = {{true, 10, 10000, 0}, {true, 10, 10000, 0}};
        problem.arcs = {{0, 1, 0, 1, 1}}; problem.capacities = {10000};
        problem.blocks = {{0, 1, 1, 1, 10000, 2, 0, 0, 1, {}}};
        problem.demands = {{0, 1, quantity, 100, {0}}}; problem.rejection = 100 * quantity;
        problem.indexCompatibility();
        problem.physical.build(2, {0}, {1}); problem.reverse_physical.build(2, {1}, {0});
        problem.logical.build(2, {0}, {1});
        LocalSearchResult checked; checked.plan = Plan::empty(problem); checked.objective = problem.rejection;
        Budget check_budget; check_budget.seconds = 1;
        Search regression(problem, check_budget, checked, base_options);
        auto candidate = Plan::empty(problem); candidate.logical[0] = {0}; candidate.physical[0] = {0};
        candidate.served[0] = raw;
        const double before = candidate.validate(problem);
        require(candidate.served[0] > quantity, "The strict-box regression no longer reproduces");
        const auto cost = regression.validateRepairCandidate(candidate, "test-campaign-box", [](int, Json &) {});
        require(cost && candidate.served[0] == quantity && *cost == quantity && *cost >= before &&
                candidate.validate(problem) == *cost,
                "Strict-box normalization did not rebuild the original loads and full cost");
        candidate.served[0] = quantity + .01;
        bool rejected = false;
        try { regression.validateRepairCandidate(candidate, "test-campaign-gross", [](int, Json &) {}); }
        catch (const std::runtime_error &error) { rejected = std::string(error.what()) == "Plan capacity violation"; }
        require(rejected && candidate.served[0] > quantity,
                "Normalization hid a substantive raw demand-capacity violation");
    }
    std::cout << "CONTROLLED_COMPONENTS_AND_C0_OK\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

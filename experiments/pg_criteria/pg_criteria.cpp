/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
// External fixed-pool mechanism experiment; no formal optimizer file is modified.
#define main embedded_post_c0_main
#include "post_c0.cpp"
#undef main

namespace {
PathPool physicalPaths(const Search &search) {
    PathPool paths(search.pool.size());
    for (int b = 0; b < int(paths.size()); ++b)
        for (const auto &route : search.pool[b]) paths[b].push_back(route.arcs);
    return paths;
}

void freezePhysical(Search &search, const Json &capture, const PathPool &logical) {
    restore(search, capture);
    search.guided = logical;
    std::fill(search.seeded.begin(), search.seeded.end(), true);
}

void unchangedPhysical(const Search &search, const Json &capture) {
    if (Json(physicalPaths(search)) != capture.at("profile").at("candidate_pool").at("physical_paths") ||
        search.physical_searches != 0)
        throw std::runtime_error("Mechanism changed the frozen physical pool");
    const auto &units = capture.at("state").at("route_unit_costs");
    for (int b = 0; b < int(search.pool.size()); ++b)
        for (int r = 0; r < int(search.pool[b].size()); ++r)
            if (search.pool[b][r].unit != units.at(b).at(r).get<double>())
                throw std::runtime_error("Mechanism changed a cached physical unit cost");
}

// Exact native mixed-return EUC weights, on the complete compact logical graph.
Json independentGraph(Search &search, const PathPool &base, const std::string &arm) {
    const auto &p = search.p;
    std::vector<int> node_ids;
    const auto graph = compactLogicalGraph(p, node_ids);
    std::vector<double> minimum(p.blocks.size(), std::numeric_limits<double>::infinity()), current = minimum;
    for (int b = 0; b < int(p.blocks.size()); ++b)
        for (const auto &route : search.pool[b]) {
            minimum[b] = std::min(minimum[b], route.unit);
            if (route.arcs == search.out.plan.physical[b]) current[b] = route.unit;
        }
    Json returned = Json::array();
    int completed_searches = 0;
    for (int k = 0; k < int(p.demands.size()); ++k) {
        if (search.budget.remaining() <= 0) break;
        const auto &d = p.demands[k];
        if (!(d.quantity > 0)) throw std::runtime_error("EUC needs positive demand quantity");
        std::vector<double> cost(p.blocks.size());
        std::vector<char> allowed(p.blocks.size());
        for (int b : p.compatibleBlocks(k)) {
            if (!std::isfinite(minimum[b])) continue;
            const bool open = !search.out.plan.physical[b].empty();
            const double reference = open ? current[b] : minimum[b];
            cost[b] = arm == "Travel" ? reference - p.blocks[b].interchange :
                reference + (open ? 0 : p.blocks[b].opening / d.quantity) +
                (p.blocks[b].from == d.from ? 0 : p.nodes[p.blocks[b].from].handling_cost);
            if (!std::isfinite(cost[b]) || cost[b] < 0)
                throw std::runtime_error("EUC requires finite nonnegative edge costs");
            allowed[b] = true;
        }
        std::set<std::vector<int>> existing(base[k].begin(), base[k].end());
        if (base[k].size() > std::size_t(std::numeric_limits<int>::max() - 12))
            throw std::runtime_error("Too many base paths");
        bool completed = false;
        const auto ranked = shortestSimplePaths(graph, node_ids[d.from], node_ids[d.to], cost, allowed,
                                                search.budget, int(base[k].size()) + 12, &completed);
        completed_searches += completed;
        int kept = 0;
        for (const auto &path : ranked) if (existing.insert(path.edges).second) {
            search.guided[k].push_back(path.edges);
            returned.push_back({{"demand", k}, {"rank", ++kept}, {"source", arm},
                                {"criterion_weight", path.cost}, {"path", path.edges}});
            if (kept == 12) break;
        }
    }
    return {{"returned_paths", returned}, {"completed_searches", completed_searches},
            {"demands", p.demands.size()}, {"LP_calls", 0}, {"complete_compatible_graph", true}};
}

Json fullPoolLP(const Problem &p, const Json &capture, const PathPool &paths,
                LocalSearchOptions options, const std::vector<int> &ids) {
    options.guidance_logical_paths_per_demand = 0;
    Budget budget;
    budget.seconds = 1e100;
    LocalSearchResult out;
    Search search(p, budget, out, options, capture.at("domain").at("demand_driven").get<bool>(), ids);
    freezePhysical(search, capture, paths);
    freshBudget(budget, 1e100);
    PoolLPInfo information;
    search.solvePathModel(false, &paths, &information); // LP-info returns before pricing.
    const double elapsed = budget.elapsed();
    unchangedPhysical(search, capture);
    if (search.pool_mip_calls || !search.path_generation_history.empty() || search.guided != paths)
        throw std::runtime_error("Quality LP unexpectedly generated paths or ran an IP");
    return {{"optimal", information.optimal}, {"objective", information.optimal ? Json(information.objective) : Json(nullptr)},
            {"status", search.guidance_status}, {"seconds", elapsed},
            {"LP_solver_seconds", search.guidance_lp_seconds}, {"LP_calls", search.guidance_lp_calls},
            {"full_pool", true}, {"seed_cap", 0}};
}

Json runMechanism(const Problem &original, const Problem &p, const Json &capture,
                  const std::vector<int> &ids, const std::string &arm, LocalSearchOptions options) {
    if (arm != "Current" && arm != "EUC" && arm != "Travel")
        throw std::runtime_error("Arm must be Current, Travel or EUC");
    // Existing numerical controls, fixed once for this mechanism experiment.
    options.joint_arc_max_model_size = 0;
    options.joint_repair_max_demands = 250;
    options.pool_fix_fraction = 0; options.pool_extra_paths = 0;
    options.final_mip_focus = 0; options.random_seed = 1; options.solver_threads = 1;
    options.record_candidate_pool = false;
    options.benchmark_path_generation = true;
    options.guidance_logical_paths_per_demand = 4;
    options.guidance_max_iterations = 10;
    options.guidance_iteration_seconds = 10;
    options.guidance_plateau_window = 3;
    options.guidance_plateau_relative_tolerance = 1e-5;
    options.guidance_max_seconds = 1e100;
    options.guided_paths_per_demand = 12;
    options.guided_jrc_paths_per_demand = 8;
    options.guided_diverse_paths_per_demand = 2;
    options.pool_mip_max_seconds = 5;
    options.validate();
    const auto &cp = capture.at("profile").at("candidate_pool");
    const auto base = cp.at("logical_paths").get<PathPool>();
    Budget generation_budget;
    LocalSearchResult generated;
    Search generator(p, generation_budget, generated, options,
                     capture.at("domain").at("demand_driven").get<bool>(), ids);
    freezePhysical(generator, capture, base);
    const auto initial_plan = generated.plan.json();
    freshBudget(generation_budget, 5);
    Json generation;
    if (arm == "EUC" || arm == "Travel") generation = independentGraph(generator, base, arm);
    else {
        generator.solvePathModel(false, &base);
        generation["path_generation_history"] = generator.path_generation_history;
        generation["LP_calls"] = generator.guidance_lp_calls;
        generation["LP_solver_seconds"] = generator.guidance_lp_seconds;
        generation["pricing_seconds"] = generator.guidance_pricing_seconds;
    }
    const double generation_seconds = generation_budget.elapsed();
    unchangedPhysical(generator, capture);
    if (generated.plan.json() != initial_plan || generator.pool_mip_calls)
        throw std::runtime_error("Generation changed the incumbent or ran an IP");
    const PathPool paths = generator.guided;
    std::vector<int> new_counts(paths.size());
    for (int k = 0; k < int(paths.size()); ++k) {
        std::set<std::vector<int>> known(base[k].begin(), base[k].end()), seen;
        for (const auto &path : paths[k]) {
            if (!seen.insert(path).second) throw std::runtime_error("Duplicate candidate path");
            new_counts[k] += !known.count(path);
        }
        if (new_counts[k] > 12)
            throw std::runtime_error("Supplement quota violated");
        for (const auto &path : base[k])
            if (!seen.count(path)) throw std::runtime_error("Supplement discarded a base path");
    }
    generation["seconds"] = generation_seconds;
    generation["budget_seconds"] = 5;
    generation["new_counts"] = new_counts;
    Budget ip_budget;
    LocalSearchResult final;
    Search integer(p, ip_budget, final, options,
                   capture.at("domain").at("demand_driven").get<bool>(), ids);
    freezePhysical(integer, capture, paths);
    freshBudget(ip_budget, 5);
    integer.solvePathModel(true, &paths);
    const double ip_seconds = ip_budget.elapsed();
    unchangedPhysical(integer, capture);
    for (const auto &model : integer.pool_model_history)
        if (model.at("free_demands").get<int>() != int(p.demands.size()) ||
            model.at("arc_size_guard").at("enabled").get<bool>() || !model.at("implicit_demands").empty())
            throw std::runtime_error("IP is not an all-free path-only fixed-pool model");
    for (int k = 0; k < int(paths.size()); ++k)
        if (final.plan.served[k] > Tol &&
            std::find(paths[k].begin(), paths[k].end(), final.plan.logical[k]) == paths[k].end())
            throw std::runtime_error("IP used a logical path outside the candidate pool");
    for (int b = 0; b < int(integer.pool.size()); ++b) if (!final.plan.physical[b].empty()) {
        bool found = false;
        for (const auto &route : integer.pool[b]) found = found || route.arcs == final.plan.physical[b];
        if (!found) throw std::runtime_error("IP used a physical path outside the frozen pool");
    }
    // Diagnostic LP is after the complete generation/repair experiment, with its own clock.
    const auto candidate_lp = fullPoolLP(p, capture, paths, options, ids);
    const Plan plan = lift(original, final.plan, ids);
    const double validated = plan.validate(original);
    if (!objectiveEqual(validated, final.objective)) throw std::runtime_error("Original-model IP cost mismatch");
    return {{"instance", original.name}, {"source_commit", SourceCommit}, {"arm", arm},
        {"objective", final.objective}, {"validated_objective", validated}, {"solution", plan.json()},
        {"initial_objective", cp.at("incumbent_objective")}, {"generation", generation},
        {"candidate_LP", candidate_lp}, {"algorithm_seconds", generation_seconds + ip_seconds},
        {"diagnostic_seconds", candidate_lp.at("seconds")},
        {"C0_state", capture.at("state")},
        {"IP", {{"seconds", ip_seconds}, {"budget_seconds", 5}, {"status", integer.pool_mip_status},
                {"model_calls", integer.pool_mip_calls}, {"continuations", integer.pool_mip_continuations},
                {"solver_seconds", integer.pool_mip_solver_seconds}, {"build_seconds", integer.pool_mip_build_seconds},
                {"conditional_bound", integer.pool_mip_bound}, {"model_history", integer.pool_model_history}}},
        {"search_options", options}, {"physical_pool_frozen", true}, {"all_demands_free", true},
        {"domain", capture.at("domain")},
        {"profile", {{"candidate_pool", {{"incumbent", cp.at("incumbent")},
                     {"incumbent_objective", cp.at("incumbent_objective")},
                     {"logical_paths", paths}, {"physical_paths", cp.at("physical_paths")}}}}}};
}
} // namespace

#ifndef POOL_MECHANISM_TEST
int main(int argc, char **argv) try {
    Budget whole;
    if (argc != 7) throw std::runtime_error("pg_criteria INPUT CONSTRUCTION_C0.json {Current|Travel|EUC} OUTPUT CONFIG MULTIPLIER");
    const double multiplier = std::stod(argv[6]);
    const auto original = Problem::read(argv[1], multiplier);
    const auto capture = readJson(argv[2]);
    if (capture.at("format") != "external-post-construction-v1" ||
        capture.at("source_commit") != SourceCommit || !capture.at("complete").get<bool>() ||
        capture.at("boundary") != "post_construction" || capture.at("instance") != original.name ||
        capture.at("source_json_sha256") != original.source_json_sha256 ||
        capture.at("demand_multiplier").get<double>() != multiplier)
        throw std::runtime_error("Complete post-construction snapshot identity mismatch");
    if (multiplier != 1 || original.demands.size() != 12 ||
        capture.at("domain").at("demand_driven").get<bool>())
        throw std::runtime_error("This study requires the fixed full-domain K12 cohort at multiplier1");
    auto options = readLocalSearchOptions(readJson(argv[5]));
    if (options.solver_threads != 1 || options.random_seed != 1 || options.benchmark_targeted_destroy ||
        options.guidance_logical_paths_per_demand != 4 || options.guidance_max_iterations != 10 ||
        options.guidance_plateau_window != 3 || options.guidance_plateau_relative_tolerance != 1e-5 ||
        !options.guidance_heuristic_pricing || options.guidance_neighbors != 32 ||
        options.guided_paths_per_demand != 12 || options.guided_jrc_paths_per_demand != 8 ||
        options.guided_diverse_paths_per_demand != 2 || options.joint_repair_max_demands != 250 ||
        options.joint_arc_max_model_size != 0)
        throw std::runtime_error("Current procedure configuration mismatch");
    const auto ids = capture.at("domain").at("original_block_ids").get<std::vector<int>>();
    Problem active;
    const Problem *domain = &original;
    if (capture.at("domain").at("demand_driven").get<bool>()) { active = capturedDomain(original, ids); domain = &active; }
    else if (!ids.empty()) throw std::runtime_error("Unexpected active ID map");
    auto result = runMechanism(original, *domain, capture, ids, argv[3], options);
    result["source_json_sha256"] = original.source_json_sha256;
    result["demand_multiplier"] = multiplier;
    result["input_snapshot_path"] = argv[2];
    result["input_boundary"] = capture.at("boundary");
    result["input_complete"] = capture.at("complete");
    result["process_before_write_seconds"] = whole.elapsed();
    result["pool_hash_contract"] = "Collection runner computes canonical SHA256 from this final candidate_pool outside optimizer clocks";
    std::ofstream output(argv[4]);
    if (!output || !(output << result.dump() << '\n')) throw std::runtime_error("Cannot write mechanism result");
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
#else
#include "pg_criteria_test.inc"
#endif

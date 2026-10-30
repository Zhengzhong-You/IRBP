/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
// External benchmark adapter. The included optimizer is the unchanged 83117e1 source.
#include "local_search.cpp"
#include <fstream>
#include <sstream>

using namespace RouteOpt::Application::IRBP;
namespace {
constexpr const char *SourceCommit = "83117e158ea9cd5ab191ab5843d3dce086a21490";

Json readJson(const std::string &path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot read " + path);
    Json value;
    input >> value;
    return value;
}

// Rebuild the captured domain in its original order; never rerun candidate selection.
Problem capturedDomain(const Problem &p, const std::vector<int> &ids) {
    Problem q;
    q.name = p.name;
    q.source_json_sha256 = p.source_json_sha256;
    q.nodes = p.nodes; q.arcs = p.arcs; q.capacities = p.capacities;
    q.physical = p.physical; q.reverse_physical = p.reverse_physical;
    q.rejection = p.rejection;
    std::set<int> seen;
    for (int b : ids) {
        if (b < 0 || b >= int(p.blocks.size()) || !seen.insert(b).second)
            throw std::runtime_error("Invalid captured original block IDs");
        q.blocks.push_back(p.blocks[b]);
    }
    q.demands = p.demands;
    std::map<int, int> groups;
    for (int k = 0; k < int(p.demands.size()); ++k) {
        auto &d = q.demands[k];
        int group = d.block_group < 0 ? -1 - k : d.block_group;
        auto [it, added] = groups.emplace(group, int(q.compatibility_groups.size()));
        if (added) {
            std::vector<int> members;
            for (int b = 0; b < int(ids.size()); ++b)
                if (p.compatible(k, ids[b])) members.push_back(b);
            q.compatibility_groups.push_back(std::move(members));
        }
        d.blocks.clear(); d.block_group = it->second;
    }
    std::vector<int> from, to;
    for (const auto &b : q.blocks) { from.push_back(b.from); to.push_back(b.to); }
    q.logical.build(q.nodes.size(), from, to);
    q.indexCompatibility();
    return q;
}

Plan lift(const Problem &original, const Plan &plan, const std::vector<int> &ids) {
    if (ids.empty()) return plan;
    Plan result = Plan::empty(original);
    result.served = plan.served;
    result.logical = plan.logical;
    for (auto &path : result.logical) for (int &b : path) b = ids.at(b);
    for (int b = 0; b < int(ids.size()); ++b) result.physical[ids[b]] = plan.physical[b];
    return result;
}

Json snapshot(const Search &search, const char *boundary, bool complete) {
    PathPool physical(search.pool.size()), logical = search.guided;
    std::vector<std::vector<double>> route_unit_costs(search.pool.size());
    for (int b = 0; b < int(search.pool.size()); ++b)
        for (const auto &route : search.pool[b]) {
            physical[b].push_back(route.arcs);
            route_unit_costs[b].push_back(route.unit);
        }
    for (int k = 0; k < int(logical.size()); ++k) {
        const auto &path = search.out.plan.logical[k];
        if (!path.empty() && std::find(logical[k].begin(), logical[k].end(), path) == logical[k].end())
            logical[k].push_back(path);
    }
    std::ostringstream random;
    random << search.random;
    return {{"format", "external-post-construction-v1"}, {"source_commit", SourceCommit},
            {"boundary", boundary}, {"complete", complete}, {"instance", search.p.name},
            {"domain", {{"demand_driven", search.demand_driven},
                         {"original_block_ids", search.original_blocks}}},
            {"state", {{"guided", search.guided}, {"seeded", search.seeded},
                        {"route_unit_costs", route_unit_costs},
                        {"incumbent_generation", search.incumbent_generation}, {"random", random.str()}}},
            {"profile", {{"candidate_pool", {{"incumbent", search.out.plan.json()},
                         {"incumbent_objective", search.out.objective},
                         {"logical_paths", logical}, {"physical_paths", physical}}}}}};
}

// Preserve cached units exactly; reconstruct integer links and min capacities in native order.
void restore(Search &search, const Json &capture) {
    const auto &cp = capture.at("profile").at("candidate_pool");
    const auto &s = cp.at("incumbent");
    search.out.plan.logical = s.at("logical_paths").get<std::vector<std::vector<int>>>();
    search.out.plan.physical = s.at("physical_paths").get<std::vector<std::vector<int>>>();
    search.out.plan.served = s.at("served").get<std::vector<double>>();
    search.out.objective = search.out.plan.validate(search.p);
    if (!objectiveEqual(search.out.objective, cp.at("incumbent_objective").get<double>()))
        throw std::runtime_error("Restored incumbent cost mismatch");
    search.guided = capture.at("state").at("guided").get<PathPool>();
    search.seeded = capture.at("state").at("seeded").get<std::vector<char>>();
    const auto units = capture.at("state").at("route_unit_costs").get<std::vector<std::vector<double>>>();
    const auto physical = cp.at("physical_paths").get<PathPool>();
    if (search.guided.size() != search.p.demands.size() || physical.size() != search.pool.size() ||
        search.seeded.size() != search.pool.size() || units.size() != physical.size())
        throw std::runtime_error("Snapshot dimension mismatch");
    for (int b = 0; b < int(physical.size()); ++b) {
        if (units[b].size() != physical[b].size()) throw std::runtime_error("Route unit dimension mismatch");
        for (int r = 0; r < int(physical[b].size()); ++r) {
            const auto &path = physical[b][r];
            Route route;
            route.arcs = path;
            route.unit = units[b][r];
            route.capacity = search.p.blocks[b].upper;
            std::map<int, int> links;
            for (int a : path) ++links[search.p.arcs.at(a).link];
            for (auto [e, n] : links) {
                route.links.emplace_back(e, n);
                route.capacity = std::min(route.capacity, search.p.capacities[e] / n);
            }
            search.pool[b].push_back(std::move(route));
        }
    }
    search.incumbent_generation = capture.at("state").at("incumbent_generation").get<std::uint64_t>();
    std::istringstream random(capture.at("state").at("random").get<std::string>());
    if (!(random >> search.random)) throw std::runtime_error("Invalid captured RNG state");
}

void freshBudget(Budget &budget, double seconds) {
    budget.start = std::chrono::steady_clock::now();
    budget.seconds = seconds;
}

bool constructOnly(Search &search) {
    State state(search.p);
    search.rebuild(state);
    search.incremental_begin = search.last_global_improvement = search.budget.elapsed();
    search.incremental_active = true;
    try { search.descend(state, "construction"); } catch (const LimitReached &) {}
    search.incremental_active = false;
    search.incremental_seconds = search.budget.elapsed() - search.incremental_begin;
    const bool complete = search.budget.remaining() > 0 &&
        (search.incremental_stop_reason == "no_improving_sweep" ||
         search.incremental_stop_reason == "relative_sweep_tailoff");
    if (!complete) search.incremental_stop_reason = "time_limit";
    return complete;
}

Json firstLocal(Search &search) {
    // Match run(false)'s incumbent insertion, without entering its subsequent PG loop.
    for (int b = 0; b < int(search.pool.size()); ++b)
        if (!search.out.plan.physical[b].empty()) search.ensureRoutes(b);
    for (int k = 0; k < int(search.p.demands.size()); ++k)
        if (!search.out.plan.logical[k].empty()) search.guided[k].push_back(search.out.plan.logical[k]);
    const double begin = search.budget.elapsed();
    const bool completed = search.localSearchStage(search.budget.seconds);
    const double seconds = search.budget.elapsed() - begin;
    Json result = snapshot(search, "first_transition_local_to_PG", true);
    result["local_boundary"] = {{"native_stage_returned_completed", completed},
        {"local_call_records", search.group_history.size()}, {"native_MIP_calls", search.mip_calls},
        {"stall_threshold_reached", search.stale_repairs >= search.options.joint_after_stalls},
        {"demand_round_limit_reached", search.group_history.size() >= search.p.demands.size()},
        {"phase_budget_reached", seconds + search.options.min_joint_seconds >= search.options.joint_phase_max_seconds},
        {"overall_budget_reached", search.budget.remaining() <= search.options.min_joint_seconds},
        {"stage_seconds", seconds}, {"stale_repairs", search.stale_repairs},
        {"group_history", search.group_history}};
    return result;
}
} // namespace

#ifndef POST_C0_TEST
int main(int argc, char **argv) try {
    Budget whole;
    if (argc < 7 || argc > 8)
        throw std::runtime_error("post_c0 INPUT {capture|continue|first-local} SECONDS OUTPUT CONFIG MULTIPLIER [SNAPSHOT]");
    const std::string mode = argv[2];
    const double seconds = std::stod(argv[3]), multiplier = std::stod(argv[6]);
    if (!(seconds > 0) || !std::isfinite(seconds) || !(multiplier > 0) || !std::isfinite(multiplier))
        throw std::runtime_error("Positive finite budget and multiplier required");
    const bool capture_mode = mode == "capture";
    if ((!capture_mode && mode != "continue" && mode != "first-local") || argc != (capture_mode ? 7 : 8))
        throw std::runtime_error("Invalid benchmark mode/arguments");
    auto options = readLocalSearchOptions(readJson(argv[5]));
    options.validate();
    Problem original = Problem::read(argv[1], multiplier);
    const double input_seconds = whole.elapsed();
    Budget budget;
    budget.seconds = seconds;
    Problem active;
    const Problem *domain = &original;
    std::vector<int> ids;
    Json saved;
    bool demand_driven = false;
    if (capture_mode && original.blocks.size() > std::size_t(options.proposal_block_threshold)) {
        active = proposalProblem(original, budget, options, ids, nullptr);
        domain = &active; demand_driven = true;
    } else if (!capture_mode) {
        saved = readJson(argv[7]);
        if (saved.at("source_commit") != SourceCommit || !saved.at("complete").get<bool>() ||
            saved.at("boundary") != "post_construction" || saved.at("instance") != original.name ||
            saved.at("source_json_sha256") != original.source_json_sha256 ||
            saved.at("demand_multiplier").get<double>() != multiplier)
            throw std::runtime_error("Snapshot source/input/boundary mismatch");
        demand_driven = saved.at("domain").at("demand_driven").get<bool>();
        ids = saved.at("domain").at("original_block_ids").get<std::vector<int>>();
        if (demand_driven) { active = capturedDomain(original, ids); domain = &active; }
        else if (!ids.empty()) throw std::runtime_error("Unexpected active block map");
    }
    LocalSearchResult out;
    out.plan = Plan::empty(*domain); out.objective = domain->rejection;
    Search search(*domain, budget, out, options, demand_driven, ids);
    Json result;
    bool complete = true;
    double restore_seconds = 0;
    if (capture_mode) {
        complete = constructOnly(search);
        result = snapshot(search, "post_construction", complete);
        result["construction_seconds"] = search.incremental_seconds;
        result["construction_stop"] = search.incremental_stop_reason;
        result["construction_sweeps"] = search.sweeps;
        result["construction_history"] = out.history;
    } else {
        restore(search, saved);
        if (!objectiveEqual(lift(original, out.plan, ids).validate(original), out.objective))
            throw std::runtime_error("Restored original-domain cost mismatch");
        restore_seconds = whole.elapsed() - input_seconds;
        freshBudget(budget, seconds); // Input/restore never consume the post-C0 optimization budget.
        if (mode == "first-local") result = firstLocal(search);
        else {
            search.run(false);
            result = {{"instance", original.name}, {"source_commit", SourceCommit},
                      {"objective", out.objective}, {"profile", out.profile}, {"history", out.history}};
        }
    }
    const Plan original_plan = lift(original, out.plan, ids);
    const double validated = original_plan.validate(original);
    if (!objectiveEqual(validated, out.objective)) throw std::runtime_error("Final original-domain cost mismatch");
    result["solution"] = original_plan.json();
    result["objective"] = out.objective;
    result["validated_objective"] = validated;
    result["source_json_sha256"] = original.source_json_sha256;
    result["demand_multiplier"] = multiplier;
    result["search_options"] = options;
    result["domain"] = {{"demand_driven", demand_driven}, {"original_block_ids", ids}};
    result["benchmark_mode"] = mode;
    result["requested_search_seconds"] = seconds;
    result["search_seconds"] = budget.elapsed();
    result["input_seconds"] = input_seconds;
    result["restore_seconds"] = restore_seconds;
    result["process_before_write_seconds"] = whole.elapsed();
    result["restore_policy"] = "Ordered pools/seed flags/RNG/generation restored; physical and solver caches cold for every arm";
    std::ofstream output(argv[4]);
    if (!output || !(output << result.dump() << '\n')) throw std::runtime_error("Cannot write benchmark result");
    return complete ? 0 : 2;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}
#else
#include "post_c0_test.inc"
#endif

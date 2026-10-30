/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
// Developer checks only; never part of the formal optimizer.
#include <cstring>
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

Problem fixture() {
    Problem p;
    p.name = "external post-C0 restore";
    p.source_json_sha256 = "fixture";
    p.nodes = std::vector<Node>(4, {true, 10, 100, 0});
    p.arcs = {{0,2,0,1,4}, {0,2,1,2,1}, {1,2,2,1,1}, {2,3,3,1,1}};
    p.capacities = {100,100,100,100};
    p.blocks = {{0,2,1,1,100,3,1,0,1,{}}, {1,2,1,1,100,3,1,0,1,{}},
                {2,3,1,8,100,3,1,0,1,{}}, {3,0,1,1,100,3,1,0,1,{}}};
    p.demands = {{0,3,4,10,{0,2}}, {1,3,4,10,{1,2}}};
    p.rejection = 80;
    p.physical.build(4,{0,0,1,2},{2,2,2,3});
    p.reverse_physical.build(4,{2,2,2,3},{0,0,1,2});
    p.logical.build(4,{0,1,2,3},{2,2,3,0});
    p.indexCompatibility();
    return p;
}

void requireSamePool(const Search &a, const Search &b) {
    require(a.guided == b.guided && a.seeded == b.seeded && a.pool.size() == b.pool.size(),
            "Ordered guided/seed flags changed during restore");
    for (int block = 0; block < int(a.pool.size()); ++block) {
        require(a.pool[block].size() == b.pool[block].size(), "Physical pool size changed");
        for (int r = 0; r < int(a.pool[block].size()); ++r) {
            const auto &x = a.pool[block][r], &y = b.pool[block][r];
            require(x.arcs == y.arcs && x.links == y.links && x.unit == y.unit && x.capacity == y.capacity,
                    "Physical route order or native arithmetic changed");
        }
    }
}

Json trajectory(const LocalSearchResult &out) {
    Json result = Json::array();
    for (const auto &h : out.history) result.push_back({h.at("stage"), h.at("objective")});
    return result;
}

int main() try {
    const auto p = fixture();
    LocalSearchOptions construction_options;
    construction_options.min_joint_seconds = construction_options.min_pool_seconds = 1000;
    construction_options.progress_log_seconds = 0;
    Budget direct_budget, capture_budget;
    direct_budget.seconds = capture_budget.seconds = 10;
    LocalSearchResult direct, captured;
    direct.plan = captured.plan = Plan::empty(p);
    direct.objective = captured.objective = p.rejection;
    Search direct_search(p, direct_budget, direct, construction_options);
    Search capture_search(p, capture_budget, captured, construction_options);
    direct_search.run(true);
    require(constructOnly(capture_search), "Tiny construction did not terminate naturally");
    require(direct.plan.json() == captured.plan.json() && direct.objective == captured.objective &&
            direct_search.sweeps == capture_search.sweeps && trajectory(direct) == trajectory(captured),
            "Construction capture differs from native run(true)");
    requireSamePool(direct_search, capture_search);

    // A partial seed with an intentionally noncanonical order must not be silently sorted on import.
    require(capture_search.pool[0].size() > 1, "Fixture lacks parallel physical alternatives");
    std::reverse(capture_search.pool[0].begin(), capture_search.pool[0].end());
    capture_search.pool[0][0].unit = std::nextafter(capture_search.pool[0][0].unit,
                                                  std::numeric_limits<double>::infinity());
    capture_search.seeded[0] = false;
    const Json state = Json::parse(snapshot(capture_search, "post_construction", true).dump());
    LocalSearchOptions options;
    options.finite_stall_cycles = 1;
    options.progress_log_seconds = 0;
    options.guidance_solver_log = false;
    Budget restored_budget;
    LocalSearchResult restored;
    restored.plan = Plan::empty(p);
    Search restored_search(p, restored_budget, restored, options);
    restore(restored_search, state);
    requireSamePool(capture_search, restored_search);
    require(std::memcmp(&capture_search.pool[0][0].unit, &restored_search.pool[0][0].unit, sizeof(double)) == 0,
            "Cached nextafter unit did not survive JSON restore bit exactly");
    require(restored.plan.json() == captured.plan.json(), "Incumbent changed on restore");
    require(capture_search.random == restored_search.random &&
            capture_search.incumbent_generation == restored_search.incumbent_generation,
            "RNG/generation changed on restore");

    capture_search.options = options;
    captured.history = Json::array();
    capture_budget.start -= std::chrono::seconds(100);
    restored_budget.start -= std::chrono::seconds(100);
    freshBudget(capture_budget, 10);
    freshBudget(restored_budget, 10);
    require(restored_budget.remaining() > 9.99, "Setup consumed continuation budget");
    capture_search.run(false);
    restored_search.run(false);
    require(capture_budget.remaining() > 1 && restored_budget.remaining() > 1,
            "Tiny native schedule did not terminate naturally");
    require(captured.plan.json() == restored.plan.json() && captured.objective == restored.objective &&
            trajectory(captured) == trajectory(restored), "Restored continuation differs from direct native continuation");
    requireSamePool(capture_search, restored_search);
    require(objectiveEqual(restored.plan.validate(p), restored.objective), "Original-domain validation failed");

    Budget projection_budget;
    projection_budget.seconds = 10;
    std::vector<int> ids;
    const auto proposed = proposalProblem(p, projection_budget, options, ids, nullptr);
    const auto rebuilt = capturedDomain(p, ids);
    require(proposed.logical.from == rebuilt.logical.from && proposed.logical.to == rebuilt.logical.to &&
            proposed.logical.edge_ids == rebuilt.logical.edge_ids &&
            proposed.compatibility_groups == rebuilt.compatibility_groups,
            "Captured active domain differs from native proposal domain");
    LocalSearchResult projected;
    projected.plan = Plan::empty(rebuilt); projected.objective = rebuilt.rejection;
    Search projected_search(rebuilt, projection_budget, projected, construction_options, true, ids);
    require(constructOnly(projected_search), "Projected construction did not terminate");
    require(objectiveEqual(lift(p, projected.plan, ids).validate(p), projected.objective),
            "Active-domain plan did not lift to the original model");

    LocalSearchResult local;
    local.plan = Plan::empty(p);
    Budget local_budget;
    Search local_search(p, local_budget, local, options);
    restore(local_search, state);
    freshBudget(local_budget, 10);
    const auto transition = firstLocal(local_search);
    require(local_search.guidance_lp_calls == 0 && local_search.pool_mip_calls == 0 &&
            transition.at("boundary") == "first_transition_local_to_PG" &&
            transition.contains("local_boundary"), "First transition accidentally entered PG/joint repair");
    std::cout << "POST_C0_TESTS_OK\n";
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}

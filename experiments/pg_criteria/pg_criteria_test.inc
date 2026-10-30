/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
void check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

int main() try {
    Problem p;
    p.name = "fixed-pool mechanism fixture";
    p.nodes = std::vector<Node>(3, {true, 10, 1000, 0});
    p.arcs = {{0,2,0,1,5}, {0,1,1,1,1}, {1,2,2,1,1}};
    p.capacities = {1000,1000,1000};
    p.blocks = {{0,2,1,1,100,5,0,0,1,{{1,100},{2,100}}},
                {0,1,1,1,100,5,0,0,1,{}}, {1,2,1,1,100,5,0,0,1,{}}};
    p.demands = {{0,2,10,100,{0,1,2}}};
    p.rejection = 1000;
    p.physical.build(3,{0,0,1},{2,1,2});
    p.reverse_physical.build(3,{2,1,2},{0,0,1});
    p.logical.build(3,{0,0,1},{2,1,2});
    p.indexCompatibility();
    Budget setup;
    setup.seconds = 10;
    LocalSearchResult initial;
    initial.plan = Plan::empty(p);
    initial.plan.logical[0] = {0}; initial.plan.physical[0] = {0}; initial.plan.served[0] = 10;
    initial.objective = initial.plan.validate(p);
    LocalSearchOptions options;
    options.progress_log_seconds = 0;
    options.guidance_solver_log = false;
    Search search(p, setup, initial, options);
    for (int b = 0; b < int(p.blocks.size()); ++b) search.ensureRoutes(b);
    search.guided = {{{0}}};
    const auto capture = Json::parse(snapshot(search, "post_construction", true).dump());
    std::map<std::string, Json> results;
    for (const std::string arm : {"EUC", "Travel", "Current"}) {
        auto result = runMechanism(p, p, capture, {}, arm, options);
        check(result.at("physical_pool_frozen").get<bool>(), "Physical freeze flag missing");
        check(result.at("profile").at("candidate_pool").at("physical_paths") ==
              capture.at("profile").at("candidate_pool").at("physical_paths"), "Physical pool changed");
        check(result.at("candidate_LP").at("optimal").get<bool>(), "Full-pool quality LP not optimal");
        check(result.at("candidate_LP").at("seed_cap") == 0,
              "Quality evaluation used the PG seed cap");
        check(result.at("IP").at("model_calls") == 1 && result.at("IP").at("budget_seconds") == 5,
              "Common IP model or global budget changed");
        check(result.at("search_options").at("joint_arc_max_model_size") == 0 &&
              result.at("IP").at("model_history").at(0).at("free_demands") == 1,
              "IP did not use all-free explicit paths");
        const double expected = 20;
        check(objectiveEqual(result.at("objective").get<double>(), expected), "Unexpected fixture final cost");
        check(result.at("generation").at("new_counts").at(0) == 1,
              "Wrong number of supplemented paths");
        results.emplace(arm, std::move(result));
    }
    check(results.at("Travel").at("generation").at("LP_calls") == 0 &&
          results.at("EUC").at("generation").at("LP_calls") == 0, "Graph arm ran generation LP");
    const auto &current = results.at("Current").at("generation").at("path_generation_history").at(0);
    check(current.at("converged").get<bool>() && current.at("return_mode") == "JRC_only",
          "Current converged return rule changed");
    p.blocks[1].opening = 100;
    p.nodes[1].handling_cost = 3;
    auto travel = runMechanism(p,p,capture,{},"Travel",options);
    auto euc = runMechanism(p,p,capture,{},"EUC",options);
    check(travel.at("generation").at("returned_paths").at(0).at("criterion_weight") == 2,
          "Travel added a nontransportation charge");
    check(euc.at("generation").at("returned_paths").at(0).at("criterion_weight") == 15,
          "EUC omitted opening allocation or handling");
    std::cout << "PG_CRITERIA_SCALAR_TESTS_OK\n";
    return 0;
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}

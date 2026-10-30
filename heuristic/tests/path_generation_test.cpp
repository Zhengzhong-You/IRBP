/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
// Exercise the real scratch-RMP loop, with deterministic LP failure at the Solver boundary.
#include "irbp/model.hpp"
namespace RouteOpt::Application::IRBP {
    struct FaultModel : Model {
        using Model::Model;
        inline static int calls{}, fail_from{};
        inline static std::vector<int> methods;
        inline static Budget *outer_clock{};
        inline static double advance_seconds{};
        inline static int advance_on_call{1};
        int optimize(Budget &budget, bool mip = false, int method = SOLVER_DUAL_SIMPLEX) {
            ++calls;
            if (outer_clock && calls == advance_on_call)
                outer_clock->start -= std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(advance_seconds));
            methods.push_back(method);
            if (fail_from && calls >= fail_from) return SOLVER_TIME_LIMIT;
            return Model::optimize(budget, mip, method);
        }
    };
}
#define Model FaultModel
#include "../src/local_search.cpp"
#undef Model
#include <iostream>

using namespace RouteOpt::Application::IRBP;
void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
Problem fixture() {
    Problem p;
    p.name = "bounded-path-generation";
    p.nodes = {{true, 10, 1000, 0}, {true, 10, 1000, 0}, {true, 10, 1000, 0}};
    p.arcs = {{0, 2, 0, 1, 5}, {0, 1, 1, 1, 1}, {1, 2, 2, 1, 1}};
    p.capacities = {1000, 1000, 1000};
    p.blocks = {{0, 2, 1, 1, 100, 1, 1, 0, 1, {}},
                {0, 1, 1, 1, 100, 1, 1, 0, 1, {}},
                {1, 2, 1, 1, 100, 1, 1, 0, 1, {}}};
    p.demands = {{0, 2, 10, 100, {0, 1, 2}}};
    p.rejection = 1000;
    p.physical.build(3, {0, 0, 1}, {2, 1, 2});
    p.reverse_physical.build(3, {2, 1, 2}, {0, 0, 1});
    p.logical.build(3, {0, 0, 1}, {2, 1, 2});
    p.indexCompatibility();
    return p;
}

struct Run {
    Json audit;
    PathPool guided;
    int lp_calls{};
};
Run run(const PathPool &c0, int seed_cap, int max_iterations, int fail_from = 0, double pricing_seconds = 1., bool heuristic = false,
        double iteration_target = 10., double clock_advance = 0., int advance_call = 1, bool record = true, int plateau_window = 3, double plateau_tolerance = 1e-5) {
    const auto p = fixture();
    Budget budget; budget.seconds = clock_advance ? 1000 : 10;
    LocalSearchResult out;
    out.plan = Plan::empty(p);
    out.plan.logical[0] = {0}; out.plan.physical[0] = {0}; out.plan.served[0] = 10;
    out.objective = out.plan.validate(p);
    const auto original = out.plan;
    const double original_cost = out.objective;
    LocalSearchOptions options;
    options.guidance_logical_paths_per_demand = seed_cap;
    options.guidance_heuristic_pricing = heuristic;
    options.guidance_max_iterations = max_iterations;
    options.guidance_max_seconds = pricing_seconds;
    options.guidance_iteration_seconds = iteration_target;
    options.guidance_plateau_window = plateau_window;
    options.guidance_plateau_relative_tolerance = plateau_tolerance;
    options.guidance_solver_log = false;
    options.record_candidate_pool = record;
    Search search(p, budget, out, options);
    for (int b = 0; b < 3; ++b) search.ensureRoutes(b);
    std::vector<std::vector<std::vector<int>>> physical_before(3);
    for (int b = 0; b < 3; ++b)
        for (const auto &route : search.pool[b]) physical_before[b].push_back(route.arcs);
    FaultModel::calls = 0; FaultModel::fail_from = fail_from; FaultModel::methods.clear();
    FaultModel::outer_clock = &budget; FaultModel::advance_seconds = clock_advance;
    FaultModel::advance_on_call = advance_call;
    search.solvePathModel(false, &c0);
    FaultModel::outer_clock = nullptr;
    require(out.objective == original_cost && out.plan.logical == original.logical &&
            out.plan.physical == original.physical && out.plan.served == original.served,
            "Path generation changed the feasible incumbent");
    require(search.mip_calls == 0 && search.pool_mip_calls == 0, "Scratch exploration ran an intermediate IP");
    for (int b = 0; b < 3; ++b) {
        std::vector<std::vector<int>> after;
        for (const auto &route : search.pool[b]) after.push_back(route.arcs);
        require(after == physical_before[b], "Path generation changed the physical pool");
    }
    require(FaultModel::methods.front() == RouteOpt::SOLVER_BARRIER &&
            std::all_of(FaultModel::methods.begin() + 1, FaultModel::methods.end(),
                        [](int method) { return method == RouteOpt::SOLVER_DUAL_SIMPLEX; }),
            "Scratch LP did not warm-start dual simplex after its initial barrier solve");
    require(search.path_generation_history.size() == 1, "Missing scratch-RMP audit");
    const auto audit = search.path_generation_history.back();
    require(audit.at("terminal_optimal") == true && audit.at("terminal_row_violation").get<double>() <= FeasTol,
            "Return did not use an optimal row-feasible solved snapshot");
    require(heuristic ? audit.at("heuristic_pricing_searches").get<int>() > 0 :
                        audit.at("heuristic_pricing_searches") == 0,
            "Logical pricing ignored the configured dominance mode");
    for (const auto &returned : audit.at("returned_paths")) {
        const auto path = returned.at("path").get<std::vector<int>>();
        require(std::find(c0[0].begin(), c0[0].end(), path) == c0[0].end(), "C0 was counted against new returns");
        require(path == std::vector<int>({1,2}), "Fixture returned a nonelementary or unrecognized path");
    }
    return {audit, search.guided, search.guidance_lp_calls};
}
void compactGraphEquivalence() {
    Problem p;
    p.nodes.resize(1000);
    p.nodes[410].yard = true;
    const std::vector<int> from = {13, 13, 410, 13, 720, 13, 998};
    const std::vector<int> to = {410, 410, 720, 720, 410, 720, 720};
    p.blocks.resize(from.size());
    for (int b = 0; b < int(from.size()); ++b) {
        p.blocks[b].from = from[b]; p.blocks[b].to = to[b];
    }
    p.demands = {{13, 720, 10, 100, {}}, {999, 998, 1, 100, {}}};
    p.logical.build(p.nodes.size(), from, to);
    std::vector<int> node_ids;
    const auto compact = compactLogicalGraph(p, node_ids);
    require(compact.size() == 5 && node_ids[0] == -1 && node_ids[999] >= 0 &&
            node_ids[13] >= 0 && node_ids[720] >= 0,
            "Compaction lost a non-yard or isolated demand endpoint");
    require(compact.edge_ids == p.logical.edge_ids && compact.from.size() == from.size(),
            "Logical compaction changed block IDs or adjacency extension order");
    for (int b = 0; b < int(from.size()); ++b)
        require(compact.from[b] == node_ids[from[b]] && compact.to[b] == node_ids[to[b]],
                "Logical compaction reordered parallel block edges");
    const std::vector<double> cost = {3, 2, -5, 1, -2, 2, 0};
    for (bool heuristic : {false, true}) {
        Labeling original(16 * 1024 * 1024), dense(16 * 1024 * 1024);
        Budget budget; budget.seconds = 10;
        Stats original_stats, dense_stats;
        std::vector<PricedPath> before, after;
        auto a = original.run(p.logical, 13, 720, cost, {}, 1e100, {}, {}, budget, original_stats,
                             heuristic, 1e99, [&](PricedPath path) { before.push_back(std::move(path)); });
        auto b = dense.run(compact, node_ids[13], node_ids[720], cost, {}, 1e100, {}, {}, budget, dense_stats,
                          heuristic, 1e99, [&](PricedPath path) { after.push_back(std::move(path)); });
        require(a.cost == b.cost && a.edges == b.edges && before.size() == after.size() && before.size() >= 3,
                "Compaction changed pricing or discarded a parallel-edge path");
        for (int i = 0; i < int(before.size()); ++i)
            require(before[i].cost == after[i].cost && before[i].edges == after[i].edges,
                    "Compaction changed terminal callback order");
        require(original_stats.labels == dense_stats.labels && original_stats.dominated == dense_stats.dominated &&
                dense_stats.peak_label_bytes < original_stats.peak_label_bytes,
                "Compaction changed dominance or failed to reduce visited-set memory");
    }
}
void neighborhoodConnections() {
    Problem p; p.nodes.resize(5);
    const std::vector<int> from{0,0,0,0,1,1,2,3,3,0,0};
    const std::vector<int> to{1,1,2,4,2,4,4,4,2,2,4};
    p.blocks.resize(from.size());
    for (int b=0;b<int(from.size());++b) { p.blocks[b].from=from[b]; p.blocks[b].to=to[b]; }
    std::vector<int> compatible(from.size()); std::iota(compatible.begin(),compatible.end(),0);
    p.demands={{0,4,1,100,compatible}}; p.logical.build(5,from,to); p.indexCompatibility();
    Plan incumbent=Plan::empty(p); incumbent.logical[0]={0,5};
    const std::vector<double> length{9,3,2,100,1,100,1,100,1,7,std::numeric_limits<double>::infinity()};
    require(logicalPricingNeighborhood(p,p.logical,0,incumbent,length,1)==std::vector<int>({0,2,3,4,5,6,8,9}),
            "Neighborhood lost OD/incumbent/parallel connections or protected every incoming destination edge");
    require(logicalPricingNeighborhood(p,p.logical,0,incumbent,length,32)==std::vector<int>({0,1,2,3,4,5,6,7,8,9}),
            "A nonbinding neighborhood removed a small-instance connection");
}
void shortageDoesNotRefill() {
    Problem p; p.name="quota shortages stay short"; p.nodes={{true,100,10000,0},{true,100,10000,0}};
    p.arcs={{0,1,0,1,1}}; p.capacities={10000};
    std::vector<int> compatible,from,to;
    for(int b=0;b<16;++b) { p.blocks.push_back({0,1,1,0,100,1,0,0,double(b+1),{}}); compatible.push_back(b); from.push_back(0);to.push_back(1); }
    p.demands={{0,1,10,100,compatible}};p.rejection=1000;p.physical.build(2,{0},{1});p.reverse_physical.build(2,{1},{0});p.logical.build(2,from,to);p.indexCompatibility();
    Budget budget;budget.seconds=10;LocalSearchResult out;out.plan=Plan::empty(p);out.plan.logical[0]={0};out.plan.physical[0]={0};out.plan.served[0]=10;out.objective=out.plan.validate(p);
    LocalSearchOptions options;options.guidance_max_seconds=0;options.guidance_solver_log=false;
    Search search(p,budget,out,options);for(int b=0;b<16;++b)search.ensureRoutes(b);search.guided={{{0}}};
    FaultModel::calls=0;FaultModel::fail_from=0;FaultModel::outer_clock=nullptr;
    const PathPool c0={{{0}}};search.solvePathModel(false,&c0);const auto &h=search.path_generation_history.back();
    require(h.at("returned")==2 && search.guided[0].size()==3 &&
            std::all_of(h.at("returned_paths").begin(),h.at("returned_paths").end(),[](const Json &row){return row.at("source")=="EUC";}),
            "Missing RMP candidates were silently replaced by EUC fill up to the total cap");
    require(search.mip_calls==0&&search.pool_mip_calls==0,"Unified generation ran an IP");
}
void physicalCacheAppendInvalidates() {
    auto p=fixture();p.blocks.push_back(p.blocks[0]);p.logical.build(3,{0,0,1,0},{2,1,2,2});p.indexCompatibility();
    Budget budget;budget.seconds=10; LocalSearchResult out;out.plan=Plan::empty(p);
    out.plan.logical[0]={0};out.plan.physical[0]={0};out.plan.served[0]=10;out.objective=out.plan.validate(p);
    LocalSearchOptions options;options.guidance_solver_log=false;Search search(p,budget,out,options);
    for(int b=0;b<4;++b)search.ensureRoutes(b);search.guided={{{0}}};
    search.seeded[3]=false; // A valid partial seed: routes exist, but its final sort/completion is pending.
    FaultModel::calls=0;FaultModel::fail_from=0;FaultModel::outer_clock=nullptr;
    search.solvePathModel();const int first=search.guidance_lp_calls;search.solvePathModel();
    require(search.guidance_lp_calls==first&&search.lp_repeat_skips==1,"Unchanged candidate model did not reuse pricing");
    const auto count=search.pool[3].size();search.ensureRoutes(3);
    require(search.pool[3].size()==count&&search.seeded[3],"Partial-seed fixture unexpectedly added a route");
    search.solvePathModel();require(search.guidance_lp_calls>first,"Same-size partial-to-complete seed failed to invalidate cache");
    const int completed=search.guidance_lp_calls;search.pool[0].push_back(search.pool[0].front());
    search.solvePathModel();require(search.guidance_lp_calls>completed,"Physical append failed to invalidate cached model");
}
void parallelPricingMatchesSerial() {
    auto p=fixture();p.demands.push_back(p.demands[0]);p.rejection=2000;p.indexCompatibility();
    auto solve=[&](int threads) {
        Budget budget;budget.seconds=10;LocalSearchResult out;out.plan=Plan::empty(p);
        for(int k=0;k<2;++k){out.plan.logical[k]={0};out.plan.served[k]=10;}out.plan.physical[0]={0};out.objective=out.plan.validate(p);
        LocalSearchOptions options;options.solver_threads=threads;options.guidance_solver_log=false;Search search(p,budget,out,options);
        for(int b=0;b<3;++b)search.ensureRoutes(b);const PathPool c0={{{0}},{{0}}};
        FaultModel::calls=0;FaultModel::fail_from=0;FaultModel::outer_clock=nullptr;search.solvePathModel(false,&c0);
        require(search.mip_calls==0&&search.pool_mip_calls==0,"Parallel pricing solved an intermediate IP");
        return std::make_pair(search.guided,search.path_generation_history.back());
    };
    const auto serial=solve(1),parallel=solve(2);
    require(serial.first==parallel.first&&parallel.second.at("pricing_workers")==2&&
            std::abs(serial.second.at("terminal_objective").get<double>()-parallel.second.at("terminal_objective").get<double>())<1e-5,
            "Shared exact-worker dispatch changed the collected path pool or LP value");
}
int main() try {
    compactGraphEquivalence();
    neighborhoodConnections();
    shortageDoesNotRefill();
    physicalCacheAppendInvalidates();
    parallelPricingMatchesSerial();
    const PathPool direct = {{{0}}}, full = {{{0}, {1, 2}}};
    const auto limited = run(direct, 0, 1);
    require(limited.audit.at("stop") == "iteration_limit" && limited.audit.at("batches") == 1 &&
            limited.lp_calls == 3, "Iteration cap lost its post-batch or terminal LP");
    require(limited.guided == PathPool{{{1, 2}}} && limited.audit.at("returned_paths").at(0).at("jrc") >= 0,
            "Terminal return required negative JRC or lost the newly discovered path");
    const auto closed = run(full, 0, 30);
    require(closed.audit.at("stop") == "no_new_columns" && closed.audit.at("batches") == 0 &&
            closed.audit.at("returned") == 0 && closed.lp_calls == 2,
            "No-new-column termination searched again after its terminal LP");
    const auto heuristic = run(full, 0, 30, 0, 1., true);
    require(heuristic.audit.at("stop") == "heuristic_no_new_columns",
            "Heuristic no-new-column termination was reported as exact closure");
    const auto seeded = run(full, 1, 1);
    require(seeded.audit.at("batches") == 1 && seeded.audit.at("initial_logical_paths") == 1 &&
            seeded.audit.at("scratch_logical_paths") == 2 && seeded.audit.at("returned") == 0,
            "Unseeded C0 path was wrongly excluded from scratch pricing or counted as new");
    const auto failed_batch = run(direct, 0, 1, 2);
    require(failed_batch.audit.at("stop") == "terminal_lp_failure" &&
            failed_batch.audit.at("fresh_terminal_solve") == false &&
            failed_batch.audit.at("solved_batches") == 0 &&
            std::all_of(failed_batch.audit.at("returned_paths").begin(), failed_batch.audit.at("returned_paths").end(),
                        [](const Json &row) { return row.at("source") == "EUC"; }),
            "An unsolved new column was scored with stale LP duals");
    const auto failed_terminal = run(direct, 0, 1, 3);
    require(failed_terminal.audit.at("fresh_terminal_solve") == false &&
            failed_terminal.audit.at("solved_batches") == 1 && failed_terminal.guided == limited.guided,
            "Terminal LP failure discarded the last complete solved snapshot");
    const auto pricing_limit = run(direct, 0, 30, 0, 0.);
    require(pricing_limit.audit.at("stop") == "pricing_limit" && pricing_limit.audit.at("returned") == 1 &&
            pricing_limit.audit.at("returned_paths")[0].at("source") == "EUC",
            "Interrupted pricing was falsely called closed");
    const auto unchanged = run(direct, 0, 10, 0, 1., false, 0.);
    const auto cheap = run(direct, 0, 10, 0, 1., false, 10.);
    require(cheap.guided == unchanged.guided && cheap.audit.at("lp_objectives") == unchanged.audit.at("lp_objectives") &&
            cheap.audit.at("returned_paths") == unchanged.audit.at("returned_paths") && cheap.lp_calls == unchanged.lp_calls,
            "Nonbinding per-round target changed the scratch trajectory or selected pool");
    const auto expensive = run(direct, 0, 10, 0, 1., false, 10., 20.);
    require(expensive.audit.at("stop") == "iteration_time_target" &&
            expensive.audit.at("iteration_seconds")[0].get<double>() >= 20. &&
            expensive.audit.at("batches") == 1 && expensive.audit.at("solved_batches") == 1 && expensive.lp_calls == 3,
            "Round target interrupted a batch or failed to stop before the next round");
    const auto terminal_slow = run(direct, 0, 10, 0, 1., false, 10., 20., 3);
    require(terminal_slow.audit.at("stop") == "no_new_columns" &&
            std::all_of(terminal_slow.audit.at("iteration_seconds").begin(), terminal_slow.audit.at("iteration_seconds").end(),
                        [](const Json &value) { return value.get<double>() < 10.; }),
            "Terminal LP or return work was incorrectly used to predict a future iteration cap");
    const auto thin = run(direct, 0, 10, 0, 1., false, 10., 20., 1, false);
    require(thin.guided == expensive.guided && thin.audit.at("returned_paths") == expensive.audit.at("returned_paths") &&
            !thin.audit.contains("mixed_candidate_paths") && !thin.audit.contains("initial_logical") &&
            !thin.audit.contains("scratch_logical"), "Unified selector built a full validation ledger");
    require(selectDiversePaths({0,1,2,3,4,5,6,7,8,9,10,11},
                {{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{1,2},{2,3},{4,5},{6,7},{4,5}},10,8) ==
                std::vector<int>({0,1,2,3,4,5,6,7,9,10}) &&
            selectDiversePaths({0,1},{},10,8) == std::vector<int>({0,1}),
            "The selector discarded one of the first eight JRC candidates or used unused resources");
    const auto tiny = run(direct, 0, 10, 0, 1., false, 1e-300);
    require(tiny.audit.at("stop") == "iteration_time_target" && tiny.audit.at("batches") == 1,
            "A tiny soft target interrupted the first complete batch");
    const auto plateau = run(direct,0,10,0,1.,false,0.,0.,1,false,1,.99);
    require(plateau.audit.at("stop")=="lp_plateau" && plateau.audit.at("converged") &&
            plateau.audit.at("return_mode")=="JRC_only" && plateau.audit.at("native_euc_completed_searches")==0 &&
            plateau.audit.at("native_euc_seconds")==0,
            "LP plateau did not disable EUC/diversity work");
    require(!limited.audit.at("converged").get<bool>() && limited.audit.at("return_mode")=="mixed" &&
            limited.audit.at("native_euc_completed_searches")==1,
            "Incomplete iteration limit incorrectly used the converged selector");
    const auto plateau_limit = run(direct,0,1,0,1.,false,0.,0.,1,false,1,.99);
    require(plateau_limit.audit.at("stop")=="iteration_limit" && plateau_limit.audit.at("converged") &&
            plateau_limit.audit.at("return_mode")=="JRC_only" && plateau_limit.audit.at("native_euc_completed_searches")==0,
            "Iteration-limit stop priority hid an actual latest-objective plateau");
    const auto plateau_time = run(direct,0,10,0,1.,false,10.,20.,1,false,1,.99);
    require(plateau_time.audit.at("stop")=="iteration_time_target" && plateau_time.audit.at("converged") &&
            plateau_time.audit.at("return_mode")=="JRC_only" && plateau_time.audit.at("native_euc_completed_searches")==0,
            "Time-limit stop priority hid an actual latest-objective plateau");
    require(closed.audit.at("converged") && closed.audit.at("return_mode")=="JRC_only",
            "Completed no-new-pricing pass was not convergence");
    LocalSearchOptions options;
    options.guidance_plateau_window = 3;
    options.guidance_plateau_relative_tolerance = .01;
    require(!options.guidancePlateau({100., 100., 100.}) &&
            options.guidancePlateau({100., 100., 100., 99.}) &&
            !options.guidancePlateau({100., 100., 100., 98.9}) &&
            options.guidancePlateau({-100., -100., -100., -101.}),
            "Plateau window stopped early or changed relative-improvement scale");
    require(LocalSearchOptions{}.guidance_heuristic_pricing, "Default logical pricer is not heuristic");
    require(LocalSearchOptions{}.final_mip_focus == 0, "Unsafe legacy MIPFocus default returned");
    std::cout << "BOUNDED_PATH_GENERATION_OK\n";
} catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
}

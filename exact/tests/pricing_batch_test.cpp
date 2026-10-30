/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "../src/bpc/pricing.hpp"
#include <iostream>
using namespace RouteOpt::Application::IRBP;
using namespace RouteOpt::Application::IRBP::BPC;
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    auto p = Problem::read(std::string(argv[1]) + "/partial_service.json");
    p.arcs.clear();
    for (int i = 0; i < 1200; ++i) p.arcs.push_back({0, 1, 0, 1, double(i + 1)});
    std::vector<int> from(1200, 0), to(1200, 1);
    p.physical.build(2, from, to); p.reverse_physical.build(2, to, from);
    p.demands[0].quantity = 1; p.demands[0].penalty = p.rejection = 10000;
    p.blocks[0].lower = p.blocks[0].upper = 1; p.blocks[0].opening = 0;
    Stats stats; Budget budget; Master master(p, 1, stats);
    if (master.model.optimize(budget) != RouteOpt::SOLVER_OPTIMAL) return 2;
    std::vector<double> dual(master.model.nrows); dual[master.balance[0]] = 2000;
    int pricing_reports = 0;
    budget.report_pricing = [&](const PricingProgress &log) {
        if (log.heuristic || log.added != 1000 || log.paths != 1000 || log.regenerated || log.removed ||
            std::abs(log.logical_min + 8000) > 1e-8 || std::abs(log.physical_min + 1999) > 1e-8 ||
            std::abs(log.correction + 9999) > 1e-8)
            throw std::runtime_error("Incorrect minimum JRC report");
        ++pricing_reports;
    };
    Pricing pricing(p, master, budget, stats, 64 * 1024 * 1024, 4);
    if (pricing.run(dual, false).first != 1000 || stats.pricing_candidates != 1201) return 3;
    if (pricing_reports != 1 || stats.volume_bound_rows != 1999) return 9;
    budget.report_pricing = {};
    if (master.paths.front().physical || stats.max_paths_per_round != 1000) return 4;
    for (int i = 1; i < 1000; ++i)
        if (!master.paths[i].physical || master.paths[i].edges != std::vector<int>{i - 1}) return 5;

    for (bool correction : {false, true}) {
        auto q = p;
        q.arcs = {{0, 1, 0, 1, .1}, {0, 1, 0, 1, .6}};
        q.physical.build(2, {0, 0}, {1, 1}); q.reverse_physical.build(2, {1, 1}, {0, 0});
        q.blocks[0].lower = 1; q.blocks[0].upper = 2;
        q.blocks[0].opening = correction ? 0 : 3;
        q.demands[0].quantity = 2; q.demands[0].penalty = 10; q.rejection = 20;
        Stats s; Master m(q, 1, s);
        int row = -1;
        if (correction) row = m.addExtra({3, 0, 0, 0, {}}, '=', 0);
        if (m.model.optimize(budget) != RouteOpt::SOLVER_OPTIMAL) return 6;
        std::vector<double> prices(m.model.nrows); prices[m.balance[0]] = 2;
        if (correction) prices[m.extras[row].row] = -3;
        Pricing search(q, m, budget, s, 64 * 1024 * 1024, 2);
        const int expected = correction ? 3 : 2;
        if (search.run(prices, false).first != expected) return 7;
        int first_arc = 0;
        for (const auto &c : m.paths) if (c.physical && c.edges == std::vector<int>{0}) ++first_arc;
        if (first_arc != 1 || (!correction && !s.reuse)) return 8;
    }
    std::cout << "Global top-1000 selection, terminal collection, upper-load reuse and deduplication passed\n";
}

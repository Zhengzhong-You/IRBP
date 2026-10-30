/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search.hpp"
#include <iostream>
using namespace RouteOpt::Application::IRBP;

int main() try {
    // Two four-unit demands must retain a longer shared split when their donor moves.
    Problem p;
    p.name = "incumbent-block-split-coverage";
    constexpr int A = 0, D = 1, E = 2, T = 3, O = 4, H = 5, X = 6, Y = 7, Z = 8;
    p.nodes.resize(9);
    for (auto &node : p.nodes) {
        node.yard = true; node.tracks = 100; node.handling_capacity = 1000;
    }
    auto block = [&](int from, int to, double lower, double unit) {
        const int id = int(p.blocks.size());
        p.arcs.push_back({from, to, id, 1., unit});
        p.capacities.push_back(1000.);
        p.blocks.push_back({from, to, 1, lower, 1000., 100., 1., 0., 1., {}});
        return id;
    };
    const int ad = block(A, D, 12, 9), de = block(D, E, 4, 0), et = block(E, T, 4, 0);
    const int oe = block(O, E, 4, 0), ea = block(E, A, 4, 10);
    const int ah = block(A, H, 8, 5), hd = block(H, D, 8, 4);
    const int ex = block(E, X, 8, 0), xd = block(X, D, 8, 0);
    block(A, Y, 8, 1); block(Y, T, 8, 1);
    block(O, Z, 8, 1); block(Z, D, 8, 1);
    p.demands = {{A, T, 4, 1000, {}}, {O, D, 4, 1000, {}}, {E, D, 8, 1000, {}}};
    for (auto &d : p.demands) {
        for (int b = 0; b < int(p.blocks.size()); ++b) d.blocks.push_back(b);
        p.rejection += d.quantity * d.penalty;
    }
    std::vector<int> from, to;
    for (const auto &a : p.arcs) { from.push_back(a.from); to.push_back(a.to); }
    p.physical.build(p.nodes.size(), from, to); p.reverse_physical.build(p.nodes.size(), to, from);
    from.clear(); to.clear();
    for (const auto &b : p.blocks) { from.push_back(b.from); to.push_back(b.to); }
    p.logical.build(p.nodes.size(), from, to); p.indexCompatibility();
    Plan incumbent = Plan::empty(p);
    incumbent.logical = {{ad, de, et}, {oe, ea, ad}, {ea, ad}};
    incumbent.served = {4, 4, 8};
    for (int b : {ad, de, et, oe, ea}) incumbent.physical[b] = {b};
    const double old_cost = incumbent.validate(p);
    LocalSearchOptions options;
    options.paths_per_group = 1; options.base_logical_paths = 1;
    options.guidance_max_seconds = 0; options.support_search_seconds = 0;
    options.physical_alternative_seconds = 0; options.pool_capture_only = true;
    options.joint_phase_max_seconds = 1; options.joint_mip_max_seconds = .1;
    Budget budget; budget.seconds = 10;
    auto result = solveLocalSearch(p, budget, options, &incumbent);
    const auto &pool = result.profile.at("candidate_pool");
    const std::vector<int> first{ah, hd, de, et}, second{oe, ea, ah, hd};
    auto contains = [&](int k, const std::vector<int> &path) {
        auto paths = pool.at("logical_paths").at(k).get<PathPool::value_type>();
        return std::find(paths.begin(), paths.end(), path) != paths.end();
    };
    if (!contains(0, first) || !contains(1, second))
        throw std::runtime_error("A block split was lost to unrelated cheaper bridge candidates");
    Plan witness = incumbent;
    witness.logical = {first, second, {ex, xd}};
    witness.physical[ad].clear();
    for (int b : {ah, hd, ex, xd}) witness.physical[b] = {b};
    if (!(witness.validate(p) < old_cost))
        throw std::runtime_error("Regression fixture must contain an improving feasible split");
    std::cout << "BLOCK_SPLIT_COVERAGE_OK\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n'; return 1;
}

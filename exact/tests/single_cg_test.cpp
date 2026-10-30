/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/single_cg.hpp"
#include <iostream>
#include <random>
using namespace RouteOpt;
using namespace RouteOpt::Application::IRBP;
using namespace RouteOpt::Application::IRBP::SingleCG;

static void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
static void graph(Problem &p) {
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
    p.block_demands.assign(p.blocks.size(), {});
    p.rejection = 0;
    for (int k = 0; k < (int)p.demands.size(); ++k) {
        for (int b : p.demands[k].blocks)
            p.block_demands[b].push_back(k);
        p.rejection += p.demands[k].quantity * p.demands[k].penalty;
    }
}
static Problem toy() {
    Problem p;
    p.name = "capacity-pricing-test";
    p.nodes = {{true, 1, 100, 0}, {false, 0, 0, 0}, {true, 1, 100, 0}};
    p.capacities = {4, 10, 4, 10, 1};
    p.arcs = {{0, 1, 0, 1, 5}, {0, 1, 1, 3, 0}, {1, 2, 2, 1, 5}, {1, 2, 3, 3, 0}, {0, 2, 4, 5, 0}};
    p.blocks = {{0, 2, 1, 1, 10, 5, 2, 0, 1, {}}};
    p.demands = {{0, 2, 10, 10, {0}}};
    graph(p);
    return p;
}
static std::vector<std::vector<int>> enumerate(const Problem &p, int b) {
    std::vector<std::vector<int>> paths;
    std::vector<int> path;
    std::vector<char> seen(p.nodes.size());
    seen[p.blocks[b].from] = true;
    std::function<void(int, double)> visit = [&](int i, double length) {
        if (i == p.blocks[b].to) {
            paths.push_back(path);
            return;
        }
        for (int z = p.physical.offsets[i]; z < p.physical.offsets[i + 1]; ++z) {
            int a = p.physical.edge_ids[z];
            const auto &e = p.arcs[a];
            if (seen[e.to] || length + e.length > p.blocks[b].length_limit)
                continue;
            seen[e.to] = true;
            path.push_back(a);
            visit(e.to, length + e.length);
            path.pop_back();
            seen[e.to] = false;
        }
    };
    visit(p.blocks[b].from, 0);
    return paths;
}
static double upper(const Problem &p, int b, const std::vector<int> &path, bool enabled) {
    double value = p.blocks[b].upper;
    if (enabled)
        for (int a : path)
            value = std::min(value, p.capacities[p.arcs[a].link]);
    return value;
}
static double oracle(const Master &m, int b, const std::vector<int> &path, const std::vector<double> &dual) {
    if (m.closed_blocks[b])
        return 1e100;
    double u = upper(m.p, b, path, m.capacity_bounds);
    if (u < m.p.blocks[b].lower)
        return 1e100;
    double cost = 0;
    for (int a : path)
        cost += (m.phase_one ? 0 : m.p.transport(b, a)) - dual[m.link[m.p.arcs[a].link]];
    double alpha = dual[m.select[b]], beta = dual[m.volume[b]];
    for (auto [arc, row] : m.incidence_rows[b])
        if (std::find(path.begin(), path.end(), arc) != path.end())
            alpha += dual[row];
    return std::min(-alpha + m.p.blocks[b].lower * (cost - beta), -alpha + u * (cost - beta));
}
static void rootCheck(const Problem &p, bool enabled, double expected) {
    Budget budget;
    budget.seconds = 30;
    Master all(p, budget, 1, enabled);
    for (int b = 0; b < (int)p.blocks.size(); ++b)
        for (const auto &path : enumerate(p, b)) {
            bool added = all.add(b, path);
            require(added == (upper(p, b, path, enabled) >= p.blocks[b].lower),
                    "Infeasible path screening mismatch");
            if (added) {
                check(all.m.solver.updateModel());
                const auto &column = all.paths.back();
                double coefficient;
                check(all.m.solver.getCoeff(column.upper_row, column.x, coefficient));
                require(std::abs(coefficient + upper(p, b, path, enabled)) < 1e-12,
                        "Path capacity row mismatch");
            }
        }
    require(all.m.optimize(budget) == SOLVER_OPTIMAL, "Complete master failed");
    require(std::abs(all.m.objective() - expected) < 1e-6, "Complete-master reference mismatch");
    for (int workers : {1, 4}) {
        Master m(p, budget, 1, enabled);
        Pricing pricing(m, budget, workers, 16 * 1024 * 1024);
        pricing.seed();
        bool closed = false;
        for (int iteration = 0; iteration < 30; ++iteration) {
            require(m.m.optimize(budget) == SOLVER_OPTIMAL, "Restricted master failed");
            double lp = m.m.objective();
            auto [added, minimum, correction] = pricing.run();
            require(lp + correction <= expected + 1e-6, "Invalid pricing lower bound");
            if (!added) {
                require(std::abs(lp - expected) < 1e-6, "CG differs from complete master");
                require(lp - (lp + correction) < 1e-6, "CG certificate not closed");
                closed = true;
                break;
            }
        }
        require(closed, "CG did not close");
        if (!m.paths.empty()) {
            bool rejected = false;
            const auto path = m.paths.front();
            try {
                m.add(path.owner, path.edges);
            } catch (const std::runtime_error &) {
                rejected = true;
            }
            require(rejected, "Duplicate insertion was not rejected");
        }
    }
}

int main() {
    try {
        auto p = toy();
        Budget expired;
        expired.seconds = 0;
        require(solveSingleCg(p, expired, 1, 16 * 1024 * 1024).status == "time_limit",
                "Construction timeout was not reported");
        rootCheck(p, false, 51.8);
        rootCheck(p, true, 82);
        p.blocks[0].length_limit = 6;
        rootCheck(p, true, 2);
        p = toy();
        p.blocks[0].lower = p.blocks[0].upper = 4;
        rootCheck(p, true, 82);
        p = toy();
        p.blocks[0].lower = 5;
        rootCheck(p, true, 100);
        p = toy();
        p.blocks[0].lower = 0;
        std::fill(p.capacities.begin(), p.capacities.end(), 0);
        rootCheck(p, true, 100);

        std::mt19937 random(17);
        for (int trial = 0; trial < 80; ++trial) {
            auto q = toy();
            const int n = 4 + random() % 3;
            q.nodes.assign(n, {true, 5, 100, 0});
            q.arcs.clear();
            q.capacities.clear();
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    if (i != j && random() % 3 == 0) {
                        q.arcs.push_back(
                            {i, j, int(q.capacities.size()), double(1 + random() % 4), double(random() % 8)});
                        const double levels[] = {0, 2, 5, 10};
                        q.capacities.push_back(levels[random() % 4]);
                    }
            q.blocks = {{0, n - 1, 1, double(trial % 4), 10, double(3 + trial % 10), 2, 0, 1, {}},
                        {0, n - 1, 1, 2, 5, double(3 + trial % 10), 3, 0, 1, {}},
                        {0, n - 1, 1, 0, 3, double(3 + trial % 10), 1, 0, 2, {}}};
            q.demands = {{0, n - 1, 10, 20, {0, 1, 2}}};
            graph(q);
            for (bool enabled : {false, true}) {
                Budget b;
                b.seconds = 20;
                Master m(q, b, 1, enabled);
                Pricing pricing(m, b, 3, 16 * 1024 * 1024);
                m.phase_one = trial % 3 == 0;
                m.closed_blocks[2] = trial % 5 == 0;
                if (!q.arcs.empty())
                    for (int block = 0; block < 2; ++block)
                        m.incidence_rows[block].emplace_back(random() % q.arcs.size(), m.m.row({}, '=', 0));
                std::vector<double> dual(m.m.nrows);
                for (int i : m.select)
                    dual[i] = int(random() % 41) - 20;
                for (int i : m.volume)
                    dual[i] = int(random() % 31) - 5;
                for (int i : m.link)
                    dual[i] = -double(random() % 8);
                for (const auto &rows : m.incidence_rows)
                    for (auto [a, row] : rows)
                        dual[row] = int(random() % 41) - 20;
                auto result = pricing.search(dual, 1e-10);
                double correction = 0;
                for (int block = 0; block < (int)q.blocks.size(); ++block) {
                    double expected = 1e100;
                    for (const auto &path : enumerate(q, block))
                        expected = std::min(expected, oracle(m, block, path, dual));
                    require((expected > 1e90 && result.by_block[block] > 1e90) ||
                                std::abs(expected - result.by_block[block]) < 1e-8,
                            "Capacity pricing differs from enumeration");
                    correction += std::min(0., expected);
                }
                require(std::abs(correction - result.correction) < 1e-8, "Pricing correction mismatch");
                std::set<std::pair<int, std::vector<int>>> unique;
                for (const auto &c : result.columns) {
                    require(unique.emplace(c.block, c.edges).second,
                            "Duplicate candidate across capacity levels");
                    require(std::abs(c.cost - oracle(m, c.block, c.edges, dual)) < 1e-8 && c.cost < 0,
                            "Candidate score mismatch");
                }
            }
        }
        // More than 1,000 distinct negative paths, with repeats between capacity layers.
        auto many = toy();
        constexpr int layers = 11;
        const int sink = 2 * layers + 1;
        many.nodes.assign(sink + 1, {false, 0, 0, 0});
        many.nodes[0] = many.nodes[sink] = {true, 5, 100, 0};
        many.arcs.clear();
        many.capacities.clear();
        auto edge = [&](int i, int j, double cap) {
            int id = many.capacities.size();
            many.arcs.push_back({i, j, id, 1, 1});
            many.capacities.push_back(cap);
        };
        edge(0, 1, 10);
        edge(0, 2, 4);
        for (int layer = 0; layer < layers - 1; ++layer)
            for (int a = 0; a < 2; ++a)
                for (int b = 0; b < 2; ++b)
                    edge(1 + 2 * layer + a, 1 + 2 * (layer + 1) + b, b ? 4 : 10);
        edge(2 * layers - 1, sink, 10);
        edge(2 * layers, sink, 4);
        many.blocks = {{0, sink, 1, 1, 10, double(layers + 1), 2, 0, 1, {}},
                       {0, sink, 1, 2, 7, double(layers + 1), 2, 0, 1, {}}};
        many.demands = {{0, sink, 10, 20, {0, 1}}};
        graph(many);
        Budget limit;
        limit.seconds = 30;
        Master m(many, limit);
        Pricing pricing(m, limit, 4, 32 * 1024 * 1024);
        std::vector<double> dual(m.m.nrows);
        dual[m.volume[0]] = 100;
        dual[m.volume[1]] = 120;
        auto selected = pricing.search(dual, 1e-9);
        std::vector<Candidate> exhaustive;
        for (int b = 0; b < 2; ++b)
            for (const auto &path : enumerate(many, b))
                exhaustive.push_back({b, path, oracle(m, b, path, dual)});
        std::sort(exhaustive.begin(), exhaustive.end(), [](const Candidate &a, const Candidate &b) {
            return std::tie(a.cost, a.block, a.edges) < std::tie(b.cost, b.block, b.edges);
        });
        require(selected.columns.size() == 1000, "Candidate batch size mismatch");
        for (int i = 0; i < 1000; ++i)
            require(selected.columns[i].block == exhaustive[i].block &&
                        selected.columns[i].edges == exhaustive[i].edges,
                    "Streaming best-1000 differs from full sorting");
        std::cout << "Capacity pricing: enumeration, zero/equal bounds, infeasible paths, parallel roots and "
                     "duplicate checks passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

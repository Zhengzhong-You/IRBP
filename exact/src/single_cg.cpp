/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#include "irbp/single_cg.hpp"
#include <numeric>
#include <queue>

namespace RouteOpt::Application::IRBP::SingleCG {
    Master::Master(const Problem &problem, Budget &budget, int threads, bool use_capacity_bounds)
        : p(problem), capacity_bounds(use_capacity_bounds), m(threads, "IRBP-SINGLE-CG") {
        const int N = p.nodes.size(), B = p.blocks.size(), K = p.demands.size();
        incidence_rows.resize(B);
        closed_blocks.assign(B, false);
        m.constant(p.rejection);
        y.resize(B);
        f.resize(B);
        z.resize(K);
        v.resize(K);
        h.resize(N);
        xd.assign(K, std::vector<int>(B, -1));
        g = xd;
        ld.assign(K, std::vector<int>(N, -1));
        for (int b = 0; b < B; ++b) {
            y[b] = m.var(p.blocks[b].opening, 1);
            f[b] = m.var(p.blocks[b].interchange, p.blocks[b].upper);
        }
        int Y = 0;
        for (auto &n : p.nodes)
            Y += n.yard;
        for (int k = 0; k < K; ++k) {
            auto &d = p.demands[k];
            z[k] = m.var(0, 1, SOLVER_CONTINUOUS);
            v[k] = m.var(-d.penalty, d.quantity);
            for (int b : d.blocks) {
                xd[k][b] = m.var(0, 1, SOLVER_CONTINUOUS);
                g[k][b] = m.var(0, d.quantity);
            }
            for (int i = 0; i < N; ++i)
                if (p.nodes[i].yard)
                    ld[k][i] = m.var(0, Y - 1);
        }
        for (int i = 0; i < N; ++i)
            h[i] = m.var(p.nodes[i].handling_cost, p.nodes[i].handling_capacity);
        auto row = [&](std::initializer_list<std::pair<int, double>> items, char sense, double rhs) {
            Sparse s;
            for (auto [i, c] : items)
                s.add(i, c);
            m.row(s, sense, rhs);
        };
        for (int k = 0; k < K; ++k) {
            budget.check();
            auto &d = p.demands[k];
            row({{v[k], 1}, {z[k], -d.quantity}}, '<', 0);
            for (int i = 0; i < N; ++i)
                if (p.nodes[i].yard) {
                    Sparse bal, flow, out, in;
                    for (int b : d.blocks) {
                        if (p.blocks[b].from == i) {
                            bal.add(xd[k][b], 1);
                            flow.add(g[k][b], 1);
                            out.add(xd[k][b], 1);
                        }
                        if (p.blocks[b].to == i) {
                            bal.add(xd[k][b], -1);
                            flow.add(g[k][b], -1);
                            in.add(xd[k][b], 1);
                        }
                    }
                    int delta = (i == d.from) - (i == d.to);
                    bal.add(z[k], -delta);
                    flow.add(v[k], -delta);
                    out.add(z[k], -1);
                    in.add(z[k], -1);
                    m.row(bal, '=', 0);
                    m.row(flow, '=', 0);
                    m.row(out, '<', 0);
                    m.row(in, '<', 0);
                }
            for (int b : d.blocks) {
                auto &c = p.blocks[b];
                row({{g[k][b], 1}, {xd[k][b], -d.quantity}}, '<', 0);
                row({{xd[k][b], 1}, {y[b], -1}}, '<', 0);
                row({{ld[k][c.to], 1}, {ld[k][c.from], -1}, {xd[k][b], -double(Y)}}, '>', 1 - Y);
            }
        }

        for (int b = 0; b < B; ++b) {
            Sparse flow;
            flow.add(f[b], 1);
            for (int k : p.block_demands[b])
                flow.add(g[k][b], -1);
            m.row(flow, '=', 0);
            row({{f[b], 1}, {y[b], -p.blocks[b].upper}}, '<', 0);
            row({{f[b], 1}, {y[b], -p.blocks[b].lower}}, '>', 0);
            Sparse sx;
            sx.add(y[b], -1);
            select.push_back(m.row(sx, '=', 0));
            Sparse sv;
            sv.add(f[b], -1);
            volume.push_back(m.row(sv, '=', 0));
        }
        for (int i = 0; i < N; ++i) {
            Sparse handle, track;
            handle.add(h[i], 1);
            for (int k = 0; k < K; ++k) {
                if (i == p.demands[k].from)
                    handle.add(v[k], 1);
                for (int b : p.demands[k].blocks)
                    if (p.blocks[b].from == i)
                        handle.add(g[k][b], -1);
            }
            for (int b = 0; b < B; ++b)
                if (p.blocks[b].from == i)
                    track.add(y[b], p.blocks[b].slots);
            m.row(handle, '=', 0);
            m.row(track, '<', p.nodes[i].tracks);
        }

        for (double cap : p.capacities)
            link.push_back(m.row({}, '<', cap));
        binary = y;
        for (int k = 0; k < K; ++k) {
            binary.push_back(z[k]);
            for (int b : p.demands[k].blocks)
                binary.push_back(xd[k][b]);
        }
    }
    bool Master::add(int b, std::vector<int> edges) {
        const double upper_load = pathUpper(b, edges);
        if (upper_load < p.blocks[b].lower)
            return false;
        std::uint64_t hash = 1469598103934665603ULL;
        auto mix = [&](int a) {
            hash ^= static_cast<std::uint64_t>(a);
            hash *= 1099511628211ULL;
        };
        mix(b);
        for (int a : edges)
            mix(a);
        auto range = known.equal_range(hash);
        for (auto it = range.first; it != range.second; ++it)
            if (paths[it->second].owner == b && paths[it->second].edges == edges)
                throw std::runtime_error("Duplicate physical path in hybrid master");
        if (paths.size() >= 400000)
            throw LimitReached("path_pool_limit");
        Sparse sx, sv;
        sx.add(select[b], 1);
        sv.add(volume[b], 1);
        double cost = 0;
        for (int a : edges) {
            cost += p.transport(b, a);
            sv.add(link[p.arcs[a].link], 1);
        }
        for (auto [arc, row] : incidence_rows[b])
            if (std::find(edges.begin(), edges.end(), arc) != edges.end())
                sx.add(row, 1);
        Path path{true, b, std::move(edges)};
        path.x = m.var(0, SOLVER_INFINITY, SOLVER_CONTINUOUS, sx);
        path.v = m.var(phase_one ? 0 : cost, SOLVER_INFINITY, SOLVER_CONTINUOUS, sv);
        Sparse upper;
        upper.add(path.v, 1);
        upper.add(path.x, -upper_load);
        path.upper_row = m.row(upper, '<', 0);
        Sparse lower;
        lower.add(path.v, -1);
        lower.add(path.x, p.blocks[b].lower);
        path.lower_row = m.row(lower, '<', 0);
        binary.push_back(path.x);
        known.emplace(hash, static_cast<int>(paths.size()));
        paths.push_back(std::move(path));
        return true;
    }
    Plan Master::recover() const {
        return recover(m.values());
    }
    Plan Master::recover(const std::vector<double> &x) const {
        Plan out = Plan::empty(p);
        for (int k = 0; k < (int)p.demands.size(); ++k) {
            out.served[k] = x[v[k]];
            if (x[z[k]] < .5)
                continue;
            int node = p.demands[k].from;
            for (int step = 0; node != p.demands[k].to && step < (int)p.nodes.size(); ++step) {
                int chosen = -1;
                for (int b : p.demands[k].blocks)
                    if (p.blocks[b].from == node && x[xd[k][b]] > .5) {
                        chosen = b;
                        break;
                    }
                if (chosen < 0)
                    throw std::runtime_error("Broken hybrid logical path");
                out.logical[k].push_back(chosen);
                node = p.blocks[chosen].to;
            }
            if (node != p.demands[k].to)
                throw std::runtime_error("Cyclic hybrid logical path");
        }
        for (const auto &path : paths)
            if (x[path.x] > .5)
                out.physical[path.owner] = path.edges;
        out.validate(p);
        return out;
    }
    double Master::pathUpper(int block, const std::vector<int> &edges) const {
        double upper = p.blocks[block].upper;
        if (capacity_bounds)
            for (int a : edges)
                upper = std::min(upper, p.capacities[p.arcs[a].link]);
        return upper;
    }

    Pricing::Pricing(Master &m, Budget &b, int worker_count, std::size_t label_bytes)
        : p(m.p), master(m), budget(b), team(worker_count) {
        if (worker_count < 1)
            throw std::runtime_error("Positive pricing worker count required");
        workers.reserve(worker_count);
        for (int i = 0; i < worker_count; ++i)
            workers.emplace_back(label_bytes / worker_count, (64 * 1024 * 1024) / worker_count);
        for (const auto &a : p.arcs)
            length.push_back(a.length);
        using Key = std::tuple<int, int, double, double, std::map<int, double>>;
        std::map<Key, int> ids;
        for (int block = 0; block < (int)p.blocks.size(); ++block) {
            const auto &v = p.blocks[block];
            Key key{v.from, v.to, v.length_limit, v.transport_factor, v.arc_cost};
            auto [it, inserted] = ids.emplace(std::move(key), int(groups.size()));
            if (inserted)
                groups.emplace_back();
            groups[it->second].blocks.push_back(block);
        }
        auto capacities = p.capacities;
        std::sort(capacities.begin(), capacities.end());
        capacities.erase(std::unique(capacities.begin(), capacities.end()), capacities.end());
        for (auto &group : groups) {
            std::map<double, std::vector<Request>> layers;
            for (int block : group.blocks) {
                const auto &v = p.blocks[block];
                std::vector<double> loads{v.lower, v.upper};
                if (master.capacity_bounds)
                    for (auto it = std::lower_bound(capacities.begin(), capacities.end(), v.lower);
                         it != capacities.end() && *it < v.upper; ++it)
                        loads.push_back(*it);
                std::sort(loads.begin(), loads.end());
                loads.erase(std::unique(loads.begin(), loads.end()), loads.end());
                for (double load : loads) {
                    double threshold = 0;
                    if (master.capacity_bounds) {
                        auto it = std::lower_bound(capacities.begin(), capacities.end(), load);
                        if (it == capacities.end())
                            continue;
                        threshold = *it;
                    }
                    layers[threshold].push_back({block, load});
                }
            }
            for (auto &[capacity, requests] : layers) {
                std::sort(requests.begin(), requests.end(), [](const Request &a, const Request &b) {
                    return std::tie(a.block, a.load) < std::tie(b.block, b.load);
                });
                group.layers.push_back({capacity, std::move(requests)});
            }
        }
    }

    const Pricing::Completion &Pricing::completion(Worker &worker, int destination, double capacity) {
        const auto key = std::make_pair(destination, capacity);
        auto found = worker.cache.find(key);
        if (found != worker.cache.end())
            return found->second;
        const auto bytes = p.nodes.size() * (sizeof(double) + sizeof(int));
        if (bytes > worker.cache_limit)
            throw LimitReached("completion_memory_limit");
        if (worker.cache_bytes + bytes > worker.cache_limit) {
            worker.cache.clear();
            worker.cache_bytes = 0;
        }
        Completion result{std::vector<double>(p.nodes.size(), 1e100), std::vector<int>(p.nodes.size(), -1)};
        using Entry = std::pair<double, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
        result.distance[destination] = 0;
        queue.emplace(0, destination);
        while (!queue.empty()) {
            budget.check();
            auto [distance, node] = queue.top();
            queue.pop();
            if (distance != result.distance[node])
                continue;
            for (int z = p.reverse_physical.offsets[node]; z < p.reverse_physical.offsets[node + 1]; ++z) {
                int a = p.reverse_physical.edge_ids[z];
                const auto &edge = p.arcs[a];
                if (p.capacities[edge.link] < capacity)
                    continue;
                double next = distance + edge.length;
                if (next < result.distance[edge.from]) {
                    result.distance[edge.from] = next;
                    result.next[edge.from] = a;
                    queue.emplace(next, edge.from);
                }
            }
        }
        worker.cache_bytes += bytes;
        return worker.cache.emplace(key, std::move(result)).first->second;
    }

    void Pricing::retain(Worker &worker, Candidate candidate) {
        // Each block belongs to one worker in a round. Keeping its worker's best
        // 1,000 distinct paths preserves the global best 1,000 with bounded memory.
        if (worker.columns.size() == 1000 && !CandidateLess{}(candidate, *worker.columns.rbegin()))
            return;
        worker.columns.insert(std::move(candidate));
        if (worker.columns.size() > 1000)
            worker.columns.erase(std::prev(worker.columns.end()));
    }

    PricingResult Pricing::search(const std::vector<double> &dual, double tolerance, bool heuristic) {
        if (dual.size() != std::size_t(master.m.nrows))
            throw std::runtime_error("Single-CG dual dimension mismatch");
        const auto began = std::chrono::steady_clock::now();
        PricingResult result;
        result.by_block.assign(p.blocks.size(), 1e100);
        for (auto &worker : workers) {
            worker.stats = {};
            worker.columns.clear();
        }
        auto account = [&] {
            std::size_t bytes = 0, candidate_bytes = 0;
            for (const auto &worker : workers) {
                stats.searches += worker.stats.searches;
                stats.heuristic_searches += worker.stats.heuristic_searches;
                stats.labels += worker.stats.labels;
                stats.dominated += worker.stats.dominated;
                stats.label_work_seconds += worker.stats.pricing_seconds;
                stats.peak_labels = std::max(stats.peak_labels, worker.stats.peak_labels);
                bytes += worker.stats.peak_label_bytes;
                for (const auto &c : worker.columns)
                    candidate_bytes +=
                        sizeof(Candidate) + 4 * sizeof(void *) + c.edges.capacity() * sizeof(int);
            }
            stats.peak_label_bytes = std::max(stats.peak_label_bytes, bytes);
            stats.peak_candidate_bytes = std::max(stats.peak_candidate_bytes, candidate_bytes);
            stats.pricing_seconds +=
                std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
        };
        try {
            team.run(groups.size(), [&](int worker_id, int group_id) {
                auto &worker = workers[worker_id];
                const auto &group = groups[group_id];
                const int representative = group.blocks.front();
                const auto &block = p.blocks[representative];
                std::vector<double> cost(p.arcs.size());
                for (int a = 0; a < (int)p.arcs.size(); ++a)
                    cost[a] = (master.phase_one ? 0 : p.transport(representative, a)) -
                              dual[master.link[p.arcs[a].link]];
                std::map<int, std::vector<double>> corrections;
                for (int b : group.blocks) {
                    if (master.closed_blocks[b])
                        continue;
                    for (auto [a, row] : master.incidence_rows[b])
                        if (dual[row] != 0) {
                            auto [it, inserted] = corrections.try_emplace(b, p.arcs.size(), 0.);
                            it->second[a] -= dual[row];
                        }
                }
                for (const auto &layer : group.layers) {
                    budget.check();
                    if (std::none_of(layer.requests.begin(), layer.requests.end(),
                                     [&](const Request &r) { return !master.closed_blocks[r.block]; }))
                        continue;
                    const auto &bound = completion(worker, block.to, layer.capacity);
                    if (bound.distance[block.from] > block.length_limit + 1e-10)
                        continue;
                    auto collectFor = [&](int b, const std::vector<int> &edges) {
                        double unit_cost = 0;
                        for (int a : edges)
                            unit_cost += cost[a];
                        double upper = master.pathUpper(b, edges);
                        if (upper < p.blocks[b].lower)
                            throw std::runtime_error("Capacity-layer path below minimum load");
                        double slope = unit_cost - dual[master.volume[b]], initial = -dual[master.select[b]];
                        auto found = corrections.find(b);
                        if (found != corrections.end())
                            for (int a : edges)
                                initial += found->second[a];
                        double value = std::min(initial + p.blocks[b].lower * slope, initial + upper * slope);
                        if (value < -tolerance)
                            retain(worker, {b, edges, value});
                    };
                    worker.allowed.resize(p.arcs.size());
                    for (int a = 0; a < (int)p.arcs.size(); ++a)
                        worker.allowed[a] = p.capacities[p.arcs[a].link] >= layer.capacity;
                    std::vector<Request> shared;
                    for (const auto &request : layer.requests) {
                        int b = request.block;
                        if (master.closed_blocks[b])
                            continue;
                        auto correction = corrections.find(b);
                        if (correction == corrections.end()) {
                            shared.push_back(request);
                            continue;
                        }
                        // Incidence branch prices are independent of load: only affected blocks need these
                        // searches.
                        std::vector<double> weighted(cost.size());
                        for (int a = 0; a < (int)cost.size(); ++a)
                            weighted[a] = request.load * cost[a] + correction->second[a];
                        double constant = -dual[master.select[b]] - request.load * dual[master.volume[b]];
                        auto path = worker.labeling.run(
                            p.physical, block.from, block.to, weighted, length, block.length_limit,
                            bound.distance, worker.allowed, budget, worker.stats, heuristic,
                            -constant - tolerance, [&](PricedPath found) { collectFor(b, found.edges); });
                        if (path.cost < 1e90) {
                            result.by_block[b] = std::min(result.by_block[b], constant + path.cost);
                            collectFor(b, path.edges);
                        }
                    }
                    if (shared.empty())
                        continue;
                    auto collect = [&](const std::vector<int> &edges) {
                        int previous = -1;
                        for (const auto &request : shared)
                            if (request.block != previous) {
                                previous = request.block;
                                collectFor(previous, edges);
                            }
                    };
                    double threshold = -1e100;
                    bool positive = false, zero_negative = false;
                    for (const auto &request : shared) {
                        int b = request.block;
                        if (request.load == 0) {
                            result.by_block[b] = std::min(result.by_block[b], -dual[master.select[b]]);
                            zero_negative |= -dual[master.select[b]] < -tolerance;
                        } else {
                            positive = true;
                            threshold =
                                std::max(threshold, dual[master.volume[b]] +
                                                        (dual[master.select[b]] - tolerance) / request.load);
                        }
                    }
                    if (zero_negative) {
                        std::vector<int> edges;
                        int i = block.from;
                        while (i != block.to && edges.size() < p.nodes.size()) {
                            int a = bound.next[i];
                            if (a < 0)
                                throw std::runtime_error("Broken capacity completion path");
                            edges.push_back(a);
                            i = p.arcs[a].to;
                        }
                        if (i != block.to)
                            throw std::runtime_error("Cyclic capacity completion path");
                        collect(edges);
                    }
                    if (!positive)
                        continue;
                    auto path = worker.labeling.run(p.physical, block.from, block.to, cost, length,
                                                    block.length_limit, bound.distance, worker.allowed,
                                                    budget, worker.stats, heuristic, threshold,
                                                    [&](PricedPath found) { collect(found.edges); });
                    if (path.cost >= 1e90)
                        continue;
                    for (const auto &request : shared) {
                        int b = request.block;
                        double value =
                            -dual[master.select[b]] + request.load * (path.cost - dual[master.volume[b]]);
                        result.by_block[b] = std::min(result.by_block[b], value);
                    }
                    // The retained minimum is needed even when terminal collection is suppressed by a strict
                    // threshold.
                    collect(path.edges);
                }
            });
            for (double value : result.by_block) {
                result.minimum = std::min(result.minimum, value);
                result.correction += std::min(0., value);
            }
            for (const auto &worker : workers)
                for (const auto &candidate : worker.columns)
                    result.columns.push_back(candidate);
            std::sort(result.columns.begin(), result.columns.end(), CandidateLess{});
            if (result.columns.size() > 1000)
                result.columns.resize(1000);
            account();
            return result;
        } catch (...) {
            account();
            throw;
        }
    }

    std::tuple<int, double, double> Pricing::run(bool heuristic) {
        auto result = search(master.m.dual(), pricingTolerance(master.m.objective()), heuristic);
        for (auto &candidate : result.columns)
            if (!master.add(candidate.block, std::move(candidate.edges)))
                throw std::runtime_error("Pricing generated a path below minimum load");
        stats.paths = master.paths.size();
        stats.paths_generated += result.columns.size();
        stats.max_paths_per_round = std::max(stats.max_paths_per_round, result.columns.size());
        return {int(result.columns.size()), result.minimum, result.correction};
    }

    void Pricing::seed() {
        for (const auto &group : groups)
            for (const auto &layer : group.layers)
                for (const auto &request : layer.requests) {
                    const auto &block = p.blocks[request.block];
                    if (request.load != block.lower)
                        continue;
                    const auto &bound = completion(workers.front(), block.to, layer.capacity);
                    if (bound.distance[block.from] > block.length_limit + 1e-10)
                        continue;
                    std::vector<int> edges;
                    int i = block.from;
                    while (i != block.to && edges.size() < p.nodes.size()) {
                        int a = bound.next[i];
                        if (a < 0)
                            throw std::runtime_error("Broken seed path");
                        edges.push_back(a);
                        i = p.arcs[a].to;
                    }
                    if (i != block.to)
                        throw std::runtime_error("Cyclic seed path");
                    master.add(request.block, std::move(edges));
                }
    }
} // namespace RouteOpt::Application::IRBP::SingleCG

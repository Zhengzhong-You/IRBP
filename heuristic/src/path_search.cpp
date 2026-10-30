/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/path_search.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
namespace RouteOpt::Application::IRBP {
    std::vector<PricedPath> shortestSimplePaths(const Graph &g, int origin, int destination,
                                                const std::vector<double> &cost,
                                                const std::vector<char> &allowed, Budget &budget, int keep,
                                                bool *completed) {
        std::vector<PricedPath> result;
        if (completed) *completed = false;
        if (keep <= 0) {
            if (completed) *completed = true;
            return result;
        }
        std::vector<char> blocked_node(g.size()), blocked_edge(g.to.size());
        auto shortest = [&](int start) {
            PricedPath path;
            std::vector<double> distance(g.size(), 1e100);
            std::vector<int> previous(g.size(), -1);
            using Item = std::pair<double, int>;
            std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
            distance[start] = 0;
            queue.emplace(0, start);
            while (!queue.empty()) {
                budget.check();
                auto [value, i] = queue.top();
                queue.pop();
                if (value != distance[i]) continue;
                if (i == destination) break;
                for (int z = g.offsets[i]; z < g.offsets[i + 1]; ++z) {
                    int a = g.edge_ids[z], j = g.to[a];
                    if (!allowed[a] || blocked_edge[a] || blocked_node[j]) continue;
                    if (value + cost[a] < distance[j]) {
                        distance[j] = value + cost[a];
                        previous[j] = a;
                        queue.emplace(distance[j], j);
                    }
                }
            }
            if (distance[destination] >= 1e99) return path;
            path.cost = distance[destination];
            for (int i = destination; i != start; i = g.from[previous[i]])
                path.edges.push_back(previous[i]);
            std::reverse(path.edges.begin(), path.edges.end());
            return path;
        };
        try {
            for (int a = 0; a < int(g.to.size()); ++a) {
                budget.check();
                if (allowed[a] && (!std::isfinite(cost[a]) || cost[a] < 0))
                    throw std::invalid_argument("SPP alternatives require finite nonnegative costs");
            }
            auto first = shortest(origin);
            if (first.edges.empty()) {
                if (completed) *completed = true;
                return result;
            }
            result.push_back(std::move(first));
            if (keep == 1) {
                if (completed) *completed = true;
                return result;
            }
            using Ranked = std::pair<double, std::vector<int>>;
            std::set<Ranked> pending;
            std::set<std::vector<int>> seen{result.front().edges};
            while (int(result.size()) < keep) {
                const auto root = result.back().edges;
                std::fill(blocked_node.begin(), blocked_node.end(), false);
                double prefix = 0;
                int at = origin;
                for (std::size_t pos = 0; pos < root.size(); ++pos) {
                    budget.check();
                    std::fill(blocked_edge.begin(), blocked_edge.end(), false);
                    for (const auto &path : result)
                        if (path.edges.size() > pos &&
                            std::equal(root.begin(), root.begin() + pos, path.edges.begin()))
                            blocked_edge[path.edges[pos]] = true;
                    auto suffix = shortest(at);
                    if (!suffix.edges.empty()) {
                        std::vector<int> path(root.begin(), root.begin() + pos);
                        path.insert(path.end(), suffix.edges.begin(), suffix.edges.end());
                        if (seen.insert(path).second)
                            pending.emplace(prefix + suffix.cost, std::move(path));
                    }
                    blocked_node[at] = true;
                    prefix += cost[root[pos]];
                    at = g.to[root[pos]];
                }
                if (pending.empty()) break;
                result.push_back({pending.begin()->first, pending.begin()->second});
                pending.erase(pending.begin());
            }
            if (completed) *completed = true;
        } catch (const LimitReached &) {
        }
        return result;
    }

    std::vector<double> shortestLengths(const Problem &p, int dest, Budget &budget) {
        std::vector<double> d(p.nodes.size(), 1e100);
        d[dest] = 0;
        using Item = std::pair<double, int>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
        q.push({0, dest});
        while (!q.empty()) {
            budget.check();
            auto [c, i] = q.top();
            q.pop();
            if (c != d[i])
                continue;
            for (int z = p.reverse_physical.offsets[i]; z < p.reverse_physical.offsets[i + 1]; ++z) {
                int a = p.reverse_physical.edge_ids[z], j = p.arcs[a].from;
                double n = c + p.arcs[a].length;
                if (n < d[j]) {
                    d[j] = n;
                    q.push({n, j});
                }
            }
        }
        return d;
    }
    std::vector<std::vector<std::vector<int>>>
    supportPaths(const Problem &p, const Plan &plan, const std::vector<double> &minimum_unit_cost,
                 const std::vector<std::vector<std::vector<int>>> &candidates, Budget &budget, Stats &stats,
                 std::size_t label_bytes, int paths_per_demand) {
        const int K = p.demands.size(), B = p.blocks.size();
        std::vector<std::vector<std::vector<int>>> result(K);
        std::vector<int> users(B), sole_user(B, -1);
        std::vector<double> current_unit = minimum_unit_cost;
        for (int k = 0; k < K; ++k)
            if (plan.served[k] > FeasTol)
                for (int b : plan.logical[k]) {
                    ++users[b];
                    sole_user[b] = k;
                }
        for (int b = 0; b < B; ++b)
            if (!plan.physical[b].empty()) {
                current_unit[b] = p.blocks[b].interchange;
                for (int a : plan.physical[b])
                    current_unit[b] += p.transport(b, a);
            }
        struct Anchor {
            int demand;
            double gain;
            std::vector<int> deficits;
        };
        std::vector<Anchor> anchors;
        auto unit = [&](int k, const std::vector<int> &path, const std::vector<double> &cost) {
            double value = 0;
            for (int b : path)
                value +=
                    cost[b] +
                    (p.blocks[b].from == p.demands[k].from ? 0 : p.nodes[p.blocks[b].from].handling_cost);
            return value;
        };
        for (int j = 0; j < K; ++j) {
            if (budget.remaining() == 0)
                return result;
            const auto &d = p.demands[j];
            const double current = (d.quantity - plan.served[j]) * d.penalty +
                                   plan.served[j] * unit(j, plan.logical[j], current_unit);
            for (const auto &path : candidates[j]) {
                if (budget.remaining() == 0)
                    return result;
                double opening = 0, released = 0;
                std::vector<int> deficits;
                for (int b : path)
                    if (plan.physical[b].empty()) {
                        opening += p.blocks[b].opening;
                        if (d.quantity + FeasTol < p.blocks[b].lower)
                            deficits.push_back(b);
                    }
                if (deficits.empty())
                    continue;
                for (int b : plan.logical[j])
                    if (users[b] == 1 && sole_user[b] == j &&
                        std::find(path.begin(), path.end(), b) == path.end())
                        released += p.blocks[b].opening;
                const double gain =
                    current + released - d.quantity * unit(j, path, minimum_unit_cost) - opening;
                if (gain > FeasTol)
                    anchors.push_back({j, gain, std::move(deficits)});
            }
        }
        Labeling labeling(label_bytes);
        std::vector<double> reward(B), cost(B);
        std::vector<char> allowed(B);
        for (int k = 0; k < K && budget.remaining() > 0; ++k) {
            std::fill(reward.begin(), reward.end(), 0);
            std::fill(allowed.begin(), allowed.end(), false);
            const auto &d = p.demands[k];
            for (int b : p.compatibleBlocks(k))
                allowed[b] = minimum_unit_cost[b] < 1e99;
            for (const auto &a : anchors) {
                if (budget.remaining() == 0)
                    return result;
                if (a.demand != k)
                    for (int b : a.deficits)
                        if (allowed[b] &&
                            p.demands[a.demand].quantity + d.quantity + FeasTol >= p.blocks[b].lower)
                            reward[b] = std::max(reward[b], a.gain / a.deficits.size());
            }
            if (std::none_of(reward.begin(), reward.end(), [](double x) { return x > 0; }))
                continue;
            for (int b : p.compatibleBlocks(k)) {
                const auto &block = p.blocks[b];
                cost[b] = d.quantity * (minimum_unit_cost[b] +
                                        (block.from == d.from ? 0 : p.nodes[block.from].handling_cost)) -
                          reward[b];
                // A positive reward already charges the partner's proposed opening costs.
                if (plan.physical[b].empty() && reward[b] == 0)
                    cost[b] += block.opening;
            }
            std::set<std::pair<double, std::vector<int>>> best;
            auto collect = [&](PricedPath path) {
                if (std::find(candidates[k].begin(), candidates[k].end(), path.edges) != candidates[k].end())
                    return;
                best.emplace(path.cost, std::move(path.edges));
                if ((int)best.size() > paths_per_demand)
                    best.erase(std::prev(best.end()));
            };
            Budget search;
            search.start = budget.start;
            search.seconds = budget.elapsed() + budget.remaining() / (K - k);
            try {
                labeling.run(p.logical, d.from, d.to, cost, {}, 1e100, {}, allowed, search, stats, true, 1e99,
                             collect);
            } catch (const LimitReached &) {
            }
            for (const auto &entry : best)
                result[k].push_back(entry.second);
        }
        return result;
    }

    std::vector<PricedPath> physicalAlternatives(const Problem &p, int b,
                                                 const std::vector<std::vector<int>> &existing,
                                                 Budget &budget, Stats &stats, int keep,
                                                 const std::vector<double> &length_weights) {
        const auto &block = p.blocks[b];
        std::set<std::vector<int>> seen(existing.begin(), existing.end());
        std::set<std::pair<double, std::vector<int>>> best;
        if (keep <= 0) return {};
        std::vector<double> costs(p.arcs.size()), lengths, weighted;
        std::vector<char> blocked(p.nodes.size()), allowed(p.arcs.size());
        try {
            for (int a = 0; a < (int)p.arcs.size(); ++a) {
                budget.check();
                costs[a] = p.transport(b, a);
            }
            for (const auto &root : existing) {
                std::fill(blocked.begin(), blocked.end(), false);
                int at = block.from;
                for (std::size_t pos = 0; pos < root.size(); ++pos) {
                    budget.check();
                    for (int a = 0; a < (int)p.arcs.size(); ++a) {
                        budget.check();
                        const auto &arc = p.arcs[a];
                        allowed[a] = !blocked[arc.from] && !blocked[arc.to] &&
                                     p.capacities[arc.link] + FeasTol >= block.lower;
                    }
                    // Keep the prefix; exclude every already retained continuation at this fork.
                    for (const auto &path : existing)
                        if (path.size() > pos && std::equal(root.begin(), root.begin() + pos, path.begin()))
                            allowed[path[pos]] = false;
                    enum class SppStatus { no_path, rejected, accepted };
                    auto search = [&](const std::vector<double> &metric) {
                        ++stats.searches;
                        auto suffixes = shortestSimplePaths(p.physical, at, block.to, metric, allowed,
                                                            budget, 1);
                        if (suffixes.empty()) return SppStatus::no_path;
                        std::vector<int> path(root.begin(), root.begin() + pos);
                        path.insert(path.end(), suffixes.front().edges.begin(), suffixes.front().edges.end());
                        double length = 0, cost = 0;
                        std::map<int, int> links;
                        std::set<int> visited{block.from};
                        int node = block.from;
                        for (int a : path) {
                            budget.check();
                            if (p.arcs[a].from != node || !visited.insert(p.arcs[a].to).second)
                                return SppStatus::rejected;
                            node = p.arcs[a].to;
                            length += p.arcs[a].length;
                            cost += costs[a];
                            ++links[p.arcs[a].link];
                        }
                        if (node != block.to || length > block.length_limit) return SppStatus::rejected;
                        for (auto [e, count] : links)
                            if (count * block.lower > p.capacities[e] + FeasTol) return SppStatus::rejected;
                        if (seen.insert(path).second) {
                            best.emplace(cost, std::move(path));
                            if (int(best.size()) > keep) best.erase(std::prev(best.end()));
                        }
                        return SppStatus::accepted;
                    };
                    if (search(costs) == SppStatus::rejected) {
                        if (lengths.empty()) {
                            lengths.resize(p.arcs.size());
                            for (int a = 0; a < int(p.arcs.size()); ++a) {
                                budget.check();
                                lengths[a] = p.arcs[a].length;
                            }
                        }
                        for (double weight : length_weights)
                            if (weight > 0) {
                                weighted.resize(p.arcs.size());
                                for (int a = 0; a < int(p.arcs.size()); ++a) {
                                    budget.check();
                                    weighted[a] = costs[a] + weight * lengths[a];
                                }
                                search(weighted);
                            }
                        search(lengths);
                    }
                    blocked[at] = true;
                    at = p.arcs[root[pos]].to;
                }
            }
        } catch (const LimitReached &) {
        }
        std::vector<PricedPath> result;
        for (const auto &[cost, path] : best)
            result.push_back({cost, path});
        return result;
    }

    PricedPath Labeling::run(const Graph &g, int origin, int destination, const std::vector<double> &cost,
                             const std::vector<double> &length, double max_length,
                             const std::vector<double> &completion, const std::vector<char> &allowed,
                             Budget &budget, Stats &stats, bool heuristic, double threshold,
                             const std::function<void(PricedPath)> &collect) {
        auto start = std::chrono::steady_clock::now();
        ++stats.searches;
        stats.heuristic_searches += heuristic;
        labels.clear();
        visited.clear();
        heads.assign(g.size(), -1);
        const std::size_t words = (g.size() + 63) / 64;
        const std::size_t fixed_bytes = heads.capacity() * sizeof(int) + words * 8;
        auto account = [&] {
            stats.labels += labels.size();
            stats.peak_labels = std::max(stats.peak_labels, labels.size());
            stats.peak_label_bytes = std::max(stats.peak_label_bytes, labels.capacity() * sizeof(Label) +
                                             visited.capacity() * 8 + fixed_bytes);
            stats.search_seconds +=
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        };
        struct AccountOnExit {
            const decltype(account) &save;
            ~AccountOnExit() { save(); }
        } accounting{account};
        if (limit <= fixed_bytes)
            throw LimitReached("label_memory_limit");
        const std::size_t max_labels = (limit - fixed_bytes) / (sizeof(Label) + words * 8);
        auto reserve = [&](std::size_t count) {
            if (count > max_labels)
                throw LimitReached("label_memory_limit");
            if (count > labels.capacity() || count * words > visited.capacity()) {
                const auto capacity =
                    std::min(max_labels, std::max(count, std::max(std::size_t{64}, labels.capacity() * 2)));
                labels.reserve(capacity);
                visited.reserve(capacity * words);
            }
        };
        reserve(1);
        labels.push_back({origin, -1, -1, -1, 0, 0, true});
        visited.resize(words);
        visited[origin / 64] |= std::uint64_t{1} << (origin % 64);
        heads[origin] = 0;
        std::vector<std::uint64_t> next(words);
        int best = -1;
        double value = 1e100;
        for (std::size_t idx = 0; idx < labels.size(); ++idx) {
            if ((idx & 255) == 0)
                budget.check();
            if (!labels[idx].alive)
                continue;
            const Label cur = labels[idx];
            if (cur.node == destination) {
                if (cur.cost < value) {
                    value = cur.cost;
                    best = idx;
                }
                continue;
            }
            for (int z = g.offsets[cur.node]; z < g.offsets[cur.node + 1]; ++z) {
                int a = g.edge_ids[z], j = g.to[a];
                if (!allowed.empty() && !allowed[a])
                    continue;
                if (visited[idx * words + j / 64] & (std::uint64_t{1} << (j % 64)))
                    continue;
                double len = cur.length + (length.empty() ? 0 : length[a]);
                if (len > max_length || (!completion.empty() && len + completion[j] > max_length + 1e-10))
                    continue;
                double c = cur.cost + cost[a];
                // Record every feasible terminal extension before terminal dominance discards alternatives.
                if (collect && j == destination && c < threshold) {
                    PricedPath path{c, {a}};
                    for (int at = idx; labels[at].parent >= 0; at = labels[at].parent)
                        path.edges.push_back(labels[at].edge);
                    std::reverse(path.edges.begin(), path.edges.end());
                    collect(std::move(path));
                }
                std::copy_n(visited.begin() + idx * words, words, next.begin());
                next[j / 64] |= std::uint64_t{1} << (j % 64);
                bool dominated = false;
                for (int old = heads[j]; old >= 0; old = labels[old].next) {
                    auto &l = labels[old];
                    if (!l.alive || l.cost > c || l.length > len)
                        continue;
                    bool subset = true;
                    for (std::size_t w = 0; w < words; ++w)
                        if (!heuristic && (visited[old * words + w] & ~next[w])) {
                            subset = false;
                            break;
                        }
                    if (subset) {
                        dominated = true;
                        break;
                    }
                }
                if (dominated) {
                    ++stats.dominated;
                    continue;
                }
                int *cursor = &heads[j];
                while (*cursor >= 0) {
                    auto &old = labels[*cursor];
                    bool subset = c <= old.cost && len <= old.length;
                    for (std::size_t w = 0; subset && !heuristic && w < words; ++w)
                        if (next[w] & ~visited[*cursor * words + w])
                            subset = false;
                    if (subset) {
                        old.alive = false;
                        ++stats.dominated;
                        *cursor = old.next;
                    } else
                        cursor = &old.next;
                }
                reserve(labels.size() + 1);
                const int index = labels.size();
                labels.push_back({j, (int)idx, a, heads[j], c, len, true});
                heads[j] = index;
                visited.insert(visited.end(), next.begin(), next.end());
            }
        }
        PricedPath result;
        result.cost = value;
        if (best >= 0) {
            for (int at = best; labels[at].parent >= 0; at = labels[at].parent)
                result.edges.push_back(labels[at].edge);
            std::reverse(result.edges.begin(), result.edges.end());
        }
        return result;
    }
} // namespace RouteOpt::Application::IRBP

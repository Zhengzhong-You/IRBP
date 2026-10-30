/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/pricing.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
namespace RouteOpt::Application::IRBP {
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
        stats.labels += labels.size();
        stats.peak_labels = std::max(stats.peak_labels, labels.size());
        stats.peak_label_bytes = std::max(stats.peak_label_bytes, labels.capacity() * sizeof(Label) +
                                                                      visited.capacity() * 8 + fixed_bytes);
        PricedPath result;
        result.cost = value;
        if (best >= 0) {
            for (int at = best; labels[at].parent >= 0; at = labels[at].parent)
                result.edges.push_back(labels[at].edge);
            std::reverse(result.edges.begin(), result.edges.end());
        }
        stats.pricing_seconds +=
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        return result;
    }
} // namespace RouteOpt::Application::IRBP

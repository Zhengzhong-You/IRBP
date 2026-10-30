/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/pool_reduction.hpp"
#include <algorithm>
#include <limits>
#include <numeric>
#include <set>

namespace RouteOpt::Application::IRBP {
    PathPool selectGuidancePaths(
        const PathPool &full, const Plan &incumbent, int limit,
        const std::function<double(int, const std::vector<int> &)> &unit_cost) {
        PathPool kept = full;
        if (!limit) return kept;
        for (int k = 0; k < int(full.size()); ++k) {
            if (int(full[k].size()) <= limit) continue;
            kept[k].clear();
            if (!incumbent.logical[k].empty()) kept[k].push_back(incumbent.logical[k]);
            std::vector<std::pair<double, int>> order;
            for (int j = 0; j < int(full[k].size()); ++j)
                order.emplace_back(unit_cost(k, full[k][j]), j);
            std::sort(order.begin(), order.end());
            for (const auto &[cost, j] : order) {
                (void)cost;
                if (int(kept[k].size()) == limit) break;
                if (std::find(kept[k].begin(), kept[k].end(), full[k][j]) == kept[k].end())
                    kept[k].push_back(full[k][j]);
            }
        }
        return kept;
    }

    std::vector<char> selectRepairDemands(
        const Problem &p, const Plan &incumbent, const PathPool &paths, int limit,
        const std::function<double(int, const std::vector<int> &)> &unit_cost, Json &audit) {
        const int K = p.demands.size(), B = p.blocks.size();
        if (limit < 0 || int(paths.size()) != K)
            throw std::invalid_argument("Invalid joint-repair demand limit or pool size");
        std::vector<char> fixed(K, false);
        audit = {{"limit", limit}, {"applied", limit > 0 && K > limit}, {"free_count", K}};
        if (!limit || K <= limit) return fixed;
        std::vector<double> load(B), score(K);
        std::vector<std::vector<int>> users(B), best_users(B);
        for (int k = 0; k < K; ++k)
            if (incumbent.served[k] > 1e-6)
                for (int b : incumbent.logical[k]) {
                    load[b] += incumbent.served[k];
                    users[b].push_back(k);
                }
        auto closure = [&](std::set<int> group) {
            for (;;) {
                std::map<int, double> removed;
                for (int k : group)
                    for (int b : incumbent.logical[k]) removed[b] += incumbent.served[k];
                const auto size = group.size();
                for (auto [b, value] : removed) {
                    const double remaining = load[b] - value;
                    if (remaining > 1e-6 && remaining < p.blocks[b].lower - 1e-6)
                        group.insert(users[b].begin(), users[b].end());
                }
                if (group.size() == size) return group;
            }
        };
        int excluded = 0;
        for (int k = 0; k < K; ++k) {
            const auto &d = p.demands[k];
            const double v = incumbent.served[k];
            double best = std::numeric_limits<double>::infinity();
            const std::vector<int> *best_path = nullptr;
            for (const auto &path : paths[k]) {
                const double cost = unit_cost(k, path);
                if (!best_path || cost < best) { best = cost; best_path = &path; }
            }
            score[k] = (d.quantity - v) * d.penalty;
            if (!incumbent.logical[k].empty()) {
                score[k] += v * std::max(0., unit_cost(k, incumbent.logical[k]) - best);
                for (int b : incumbent.logical[k])
                    if (load[b] > 1e-6) score[k] += p.blocks[b].opening * v / load[b];
            }
            if (best_path)
                for (int b : *best_path) best_users[b].push_back(k);
            else ++excluded;
        }
        std::set<std::vector<int>> unique;
        auto add = [&](const std::vector<int> &members) {
            auto group = closure({members.begin(), members.end()});
            if (!group.empty() && int(group.size()) <= limit)
                unique.emplace(group.begin(), group.end());
        };
        for (int b = 0; b < B; ++b) {
            if (!users[b].empty()) add(users[b]);
            double supply = 0;
            for (int k : best_users[b]) supply += p.demands[k].quantity;
            if (best_users[b].size() > 1 && supply >= p.blocks[b].lower - 1e-6) add(best_users[b]);
        }
        for (int k = 0; k < K; ++k) if (!paths[k].empty()) add({k});
        std::map<int, double> link_load;
        std::map<int, std::vector<int>> link_users;
        for (int b = 0; b < B; ++b) if (load[b] > 1e-6)
            for (int a : incumbent.physical[b]) {
                const int e = p.arcs[a].link;
                link_load[e] += load[b];
                link_users[e].insert(link_users[e].end(), users[b].begin(), users[b].end());
            }
        for (auto [e, value] : link_load)
            if (p.capacities[e] - value <= 1e-6) add(link_users[e]);
        std::vector<double> used_tracks(p.nodes.size());
        for (int b = 0; b < B; ++b) if (!incumbent.physical[b].empty())
            used_tracks[p.blocks[b].from] += p.blocks[b].slots;
        std::vector<int> direct_slots(K, std::numeric_limits<int>::max()), anchor(p.nodes.size(), -1);
        for (int k = 0; k < K; ++k) if (p.demands[k].quantity - incumbent.served[k] > 1e-6) {
            const int i = p.demands[k].from;
            for (const auto &path : paths[k]) if (path.size() == 1) {
                const int b = path.front();
                const auto &block = p.blocks[b];
                if (incumbent.physical[b].empty() && block.from == i && block.to == p.demands[k].to &&
                    used_tracks[i] + block.slots > p.nodes[i].tracks + 1e-6)
                    direct_slots[k] = std::min(direct_slots[k], block.slots);
            }
            if (direct_slots[k] != std::numeric_limits<int>::max()) {
                auto &current = anchor[i];
                if (current < 0 || score[k] > score[current] ||
                    (score[k] == score[current] && k < current)) current = k;
            }
        }
        for (int b = 0; b < B; ++b) if (!incumbent.physical[b].empty() && !users[b].empty()) {
            const int i = p.blocks[b].from, k = anchor[i];
            if (k >= 0 && used_tracks[i] - p.blocks[b].slots + direct_slots[k] <= p.nodes[i].tracks + 1e-6) {
                auto members = users[b];
                members.push_back(k);
                add(members);
            }
        }
        struct Group { std::vector<int> members; double total; };
        std::vector<Group> groups;
        for (const auto &members : unique) {
            CostSum total;
            for (int k : members) total += score[k];
            groups.push_back({members, total.value});
        }
        std::sort(groups.begin(), groups.end(), [](const Group &a, const Group &b) {
            const double x = a.total / a.members.size(), y = b.total / b.members.size();
            if (x != y) return x > y;
            if (a.total != b.total) return a.total > b.total;
            return a.members < b.members;
        });
        std::set<int> selected;
        int skipped = 0;
        for (const auto &group : groups) {
            auto combined = selected;
            combined.insert(group.members.begin(), group.members.end());
            combined = closure(std::move(combined));
            if (int(combined.size()) > limit) { ++skipped; continue; }
            selected = std::move(combined);
            if (int(selected.size()) == limit) break;
        }
        std::fill(fixed.begin(), fixed.end(), true);
        for (int k : selected) fixed[k] = false;
        audit["free_count"] = selected.size();
        audit["free_demands"] = std::vector<int>(selected.begin(), selected.end());
        audit["excluded_no_logical_path"] = excluded;
        audit["skipped_oversize_groups"] = skipped;
        return fixed;
    }

    PoolReduction reducePool(const Problem &p, const Plan &incumbent, const PathPool &paths,
                             const PoolLPInfo &lp, const LocalSearchOptions &options) {
        options.validate();
        const std::size_t K = p.demands.size(), B = p.blocks.size();
        if (!lp.optimal || !std::isfinite(lp.objective) || paths.size() != K || lp.selection.size() != K ||
            lp.volume.size() != K || lp.jrc.size() != K)
            throw std::invalid_argument("Pool reduction needs a matching optimal LP");
        PoolReduction result;
        result.fixed.assign(K, false);
        std::vector<std::vector<char>> keep(K);
        std::vector<std::vector<int>> order(K);
        std::vector<int> current(K, -1), eligible;
        std::vector<double> margin(K);
        const double weight_tol = options.pool_lp_weight_tolerance;
        const double rc_tol = options.pool_jrc_relative_tolerance * std::max(1., std::abs(lp.objective));
        std::size_t before = 0;
        for (std::size_t k = 0; k < K; ++k) {
            const auto n = paths[k].size();
            if (lp.selection[k].size() != n || lp.volume[k].size() != n || lp.jrc[k].size() != n)
                throw std::invalid_argument("Pool LP path dimensions do not match");
            keep[k].assign(n, false);
            order[k].resize(n);
            std::iota(order[k].begin(), order[k].end(), 0);
            double served = 0, alternative = std::numeric_limits<double>::infinity();
            for (std::size_t j = 0; j < n; ++j) {
                if (!std::isfinite(lp.selection[k][j]) || !std::isfinite(lp.volume[k][j]) ||
                    !std::isfinite(lp.jrc[k][j]))
                    throw std::invalid_argument("Nonfinite pool LP information");
                served += lp.volume[k][j];
                if (paths[k][j] == incumbent.logical[k])
                    current[k] = int(j);
                else
                    alternative = std::min(alternative, lp.jrc[k][j]);
            }
            if (incumbent.served[k] > 1e-8 && current[k] < 0)
                throw std::invalid_argument("Incumbent logical path missing from reduction pool");
            std::stable_sort(order[k].begin(), order[k].end(), [&](int a, int b) {
                return lp.jrc[k][a] < lp.jrc[k][b];
            });
            if (incumbent.served[k] > 1e-8 && served > weight_tol && current[k] >= 0 &&
                lp.selection[k][current[k]] >= 1 - weight_tol && alternative > rc_tol) {
                eligible.push_back(int(k));
                margin[k] = alternative / std::max(1., p.demands[k].quantity * p.demands[k].penalty);
            }
            before += n;
        }
        std::stable_sort(eligible.begin(), eligible.end(),
                         [&](int a, int b) { return margin[a] > margin[b]; });
        const std::size_t limit = std::size_t(std::floor(options.pool_fix_fraction * K));
        for (std::size_t j = 0; j < std::min(limit, eligible.size()); ++j)
            result.fixed[eligible[j]] = true;
        const auto initial_fixed = result.fixed;
        auto retain_free = [&](int k) {
            if (options.pool_extra_paths == 0) {
                std::fill(keep[k].begin(), keep[k].end(), true);
                return;
            }
            if (current[k] >= 0)
                keep[k][current[k]] = true;
            for (std::size_t j = 0; j < paths[k].size(); ++j)
                if (lp.selection[k][j] > weight_tol)
                    keep[k][j] = true;
            int extras = 0;
            for (int j : order[k])
                if (!keep[k][j] && extras < options.pool_extra_paths) {
                    keep[k][j] = true;
                    ++extras;
                }
        };
        for (std::size_t k = 0; k < K; ++k)
            if (result.fixed[k])
                keep[k][current[k]] = true;
            else
                retain_free(int(k));

        std::size_t support_paths = 0;
        bool changed = true;
        while (changed) {
            changed = false;
            for (std::size_t b = 0; b < B; ++b) {
                if (!incumbent.physical[b].empty())
                    continue;
                std::vector<char> supplied(K);
                double supply = 0;
                bool retained_block = false;
                for (std::size_t k = 0; k < K; ++k)
                    for (std::size_t j = 0; j < paths[k].size(); ++j)
                        if (keep[k][j] &&
                            std::find(paths[k][j].begin(), paths[k][j].end(), int(b)) != paths[k][j].end()) {
                            retained_block = true;
                            if (!supplied[k]) {
                                supplied[k] = true;
                                supply += p.demands[k].quantity;
                            }
                        }
                if (!retained_block || supply >= p.blocks[b].lower - 1e-8)
                    continue;
                struct Support { int k, path; double rank; };
                std::vector<Support> candidates;
                for (std::size_t k = 0; k < K; ++k)
                    if (!supplied[k])
                        for (int j : order[k])
                            if (std::find(paths[k][j].begin(), paths[k][j].end(), int(b)) != paths[k][j].end()) {
                                candidates.push_back({int(k), j,
                                    lp.jrc[k][j] / std::max(1., p.demands[k].quantity)});
                                break;
                            }
                std::stable_sort(candidates.begin(), candidates.end(),
                                 [](const Support &a, const Support &b) { return a.rank < b.rank; });
                for (const auto &support : candidates) {
                    if (supply >= p.blocks[b].lower - 1e-8)
                        break;
                    if (result.fixed[support.k]) {
                        result.fixed[support.k] = false;
                        retain_free(support.k);
                    }
                    if (!keep[support.k][support.path]) {
                        keep[support.k][support.path] = true;
                        ++support_paths;
                    }
                    supply += p.demands[support.k].quantity;
                    changed = true;
                }
            }
        }
        result.logical.resize(K);
        Json indices = Json::array(), fixed_ids = Json::array(), restored_ids = Json::array();
        std::size_t after = 0;
        for (std::size_t k = 0; k < K; ++k) {
            Json selected = Json::array();
            for (std::size_t j = 0; j < paths[k].size(); ++j)
                if (keep[k][j]) {
                    result.logical[k].push_back(paths[k][j]);
                    selected.push_back(j);
                    ++after;
                }
            indices.push_back(std::move(selected));
            if (result.fixed[k])
                fixed_ids.push_back(k);
            if (initial_fixed[k] && !result.fixed[k])
                restored_ids.push_back(k);
        }
        result.audit = {{"eligible_demands", eligible}, {"fixed_demands", fixed_ids},
                        {"unfixed_for_support", restored_ids}, {"kept_path_indices", indices},
                        {"logical_before", before}, {"logical_after", after},
                        {"support_paths_restored", support_paths}, {"jrc_tolerance", rc_tol},
                        {"lp_objective", lp.objective}};
        return result;
    }
}

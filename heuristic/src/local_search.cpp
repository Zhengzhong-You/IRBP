/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search.hpp"
#include "irbp/model.hpp"
#include "irbp/path_search.hpp"
#include "irbp/pool_reduction.hpp"
#include "irbp/pricing_workers.hpp"
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <thread>
#include <tuple>

namespace RouteOpt::Application::IRBP {
    namespace {
        constexpr double Tol = 1e-8;
        // Equal capacity left-hand sides need only the tightest right-hand side.
        // Lift its dual to ONE original resource, not to every member of the class.
        template <class Capacity>
        std::vector<int> addLinkCapacityRows(Model &model, std::vector<Sparse> &rows, Capacity capacity,
                                             Json &statistics) {
            using Key = std::pair<std::vector<int>, std::vector<double>>;
            std::map<Key, int> classes;
            std::vector<int> representative;
            std::vector<double> rhs;
            int empty = 0;
            for (int e = 0; e < int(rows.size()); ++e) {
                auto &row = rows[e];
                row.canonicalize();
                if (row.ind.empty()) {
                    ++empty;
                    continue;
                }
                auto [it, added] = classes.emplace(Key{row.ind, row.val}, int(representative.size()));
                if (added) {
                    representative.push_back(e);
                    rhs.push_back(capacity(e));
                } else if (capacity(e) < rhs[it->second]) {
                    representative[it->second] = e;
                    rhs[it->second] = capacity(e);
                }
            }
            std::vector<int> dual_rows(rows.size(), -1);
            for (int g = 0; g < int(representative.size()); ++g) {
                int e = representative[g];
                dual_rows[e] = model.row(rows[e], '<', rhs[g]);
            }
            statistics = {{"original", rows.size()},
                          {"empty", empty},
                          {"retained", representative.size()},
                          {"duplicate", rows.size() - empty - representative.size()}};
            return dual_rows;
        }
        struct Route {
            std::vector<int> arcs;
            std::vector<std::pair<int, int>> links;
            double unit{}, capacity{};
        };
        struct Change {
            int demand;
            std::vector<int> path;
            double target;
        };
        struct Move {
            std::vector<Change> changes;
            std::vector<std::pair<int, int>> routes;
            double alpha{}, cost = std::numeric_limits<double>::infinity();
        };
        struct State {
            Plan plan;
            std::vector<int> route;
            std::vector<double> load, link, handling, tracks;
            std::vector<int> strict_bad_yards, strict_bad_links;
            double cost;
            explicit State(const Problem &p)
                : plan(Plan::empty(p)), route(p.blocks.size(), -1), load(p.blocks.size()),
                  link(p.capacities.size()), handling(p.nodes.size()), tracks(p.nodes.size()),
                  cost(p.rejection) {}
        };
        Graph compactLogicalGraph(const Problem &p, std::vector<int> &node_ids) {
            node_ids.assign(p.nodes.size(), -1);
            for (const auto &block : p.blocks) node_ids[block.from] = node_ids[block.to] = 0;
            for (const auto &demand : p.demands) node_ids[demand.from] = node_ids[demand.to] = 0;
            int count = 0;
            // Number the complete endpoint union in ascending original-node order.
            for (int &id : node_ids) if (id == 0) id = count++;
            std::vector<int> from, to;
            from.reserve(p.blocks.size());
            to.reserve(p.blocks.size());
            // Keep block edge IDs and each adjacency's extension order unchanged.
            for (const auto &block : p.blocks) {
                from.push_back(node_ids[block.from]);
                to.push_back(node_ids[block.to]);
            }
            Graph graph;
            graph.build(count, from, to);
            return graph;
        }
        std::vector<int> logicalPricingNeighborhood(const Problem &p, const Graph &graph, int k,
                                                   const Plan &incumbent,
                                                   const std::vector<double> &reference_length, int neighbors) {
            std::vector<std::map<int, double>> distance(graph.size());
            for (int b : p.compatibleBlocks(k)) if (std::isfinite(reference_length[b])) {
                const int i = graph.from[b], j = graph.to[b];
                if (i == j) continue;
                auto [it, added] = distance[i].emplace(j, reference_length[b]);
                if (!added) it->second = std::min(it->second, reference_length[b]);
            }
            std::vector<std::vector<int>> nearest(graph.size());
            for (int i = 0; i < int(graph.size()); ++i) {
                std::vector<std::pair<double, int>> order;
                for (auto [j, length] : distance[i]) order.emplace_back(length, j);
                std::sort(order.begin(), order.end());
                for (int n = 0; n < std::min(neighbors, int(order.size())); ++n)
                    nearest[i].push_back(order[n].second);
            }
            std::vector<int> allowed;
            const auto &d = p.demands[k];
            for (int b : p.compatibleBlocks(k)) if (std::isfinite(reference_length[b])) {
                const auto &heads = nearest[graph.from[b]];
                if (!neighbors || (p.blocks[b].from == d.from && p.blocks[b].to == d.to) ||
                    std::find(incumbent.logical[k].begin(), incumbent.logical[k].end(), b) != incumbent.logical[k].end() ||
                    std::find(heads.begin(), heads.end(), graph.to[b]) != heads.end())
                    allowed.push_back(b);
            }
            return allowed;
        }
        std::vector<int> selectDiversePaths(const std::vector<double> &scores,
                                           const std::vector<std::vector<int>> &resources,
                                           int limit, int low_cost_paths) {
            std::vector<int> selected;
            std::vector<char> used(scores.size());
            auto distance = [&](int a, int b) {
                const auto &x = resources[a], &y = resources[b];
                std::size_t i = 0, j = 0, common = 0;
                while (i < x.size() && j < y.size()) {
                    if (x[i] == y[j]) { ++i; ++j; ++common; }
                    else if (x[i] < y[j]) ++i;
                    else ++j;
                }
                const auto total = x.size() + y.size() - common;
                return total ? 1. - double(common) / total : 0.;
            };
            for (int pass = 0; pass < limit && pass < int(scores.size()); ++pass) {
                int best = -1;
                double best_distance = -1;
                for (int j = 0; j < int(scores.size()); ++j) if (!used[j]) {
                    double separation = selected.empty() ? 1. : 2.;
                    if (pass >= low_cost_paths && selected.size() + 1 < scores.size())
                        for (int i : selected) separation = std::min(separation, distance(i, j));
                    if (best < 0 || (pass < low_cost_paths ? scores[j] < scores[best]
                        : separation > best_distance + 1e-8 ||
                          (std::abs(separation - best_distance) <= 1e-8 && scores[j] < scores[best]))) {
                        best = j;
                        best_distance = separation;
                    }
                }
                used[best] = true;
                selected.push_back(best);
            }
            return selected;
        }

        class Search {
          public:
            LocalSearchOptions options;
            const Problem &p;
            Budget &budget;
            std::vector<std::vector<Route>> pool;
            std::vector<int> yards, group_yards, bundle_yards;
            std::vector<char> seeded;
            std::vector<double> capacity_levels;
            bool proportional_costs{true};
            struct PhysicalTree {
                std::vector<int> previous;
                std::vector<double> distance;
            };
            std::map<std::pair<int, int>, PhysicalTree> physical_trees;
            double transport_length_ratio{};
            std::uint64_t physical_estimates{}, physical_route_recoveries{};
            std::deque<std::pair<int, int>> tree_order;
            std::uint64_t physical_searches{}, physical_cache_hits{};
            // Reused work arrays; no model construction or solver calls in a move.
            std::vector<double> b0, bq, h0, hq, l0, lq, track;
            std::vector<char> affected, yard_affected, link_affected;
            std::vector<int> touched_blocks, touched_yards, touched_links;
            std::uint64_t evaluated_block_cells{}, evaluated_yard_cells{}, evaluated_link_cells{};
            std::uint64_t evaluated{}, infeasible{}, accepted{}, destroys{};
            double seed_time{}, move_time{}, repair_time{};
            std::size_t seed_blocks_completed{};
            double mip_seconds{}, mip_build_seconds{};
            int pool_mip_calls{}, pool_mip_status{};
            int local_integrality_focus{}, joint_integrality_focus{};
            mutable double bridge_generation_seconds{};
            std::size_t joint_pool_logical_paths{}, joint_pool_physical_paths{};
            double pool_mip_solver_seconds{};
            double pool_mip_seconds{}, pool_mip_build_seconds{}, pool_mip_bound{}, pool_mip_nodes{};
            std::vector<std::vector<std::vector<int>>> guided;
            std::vector<std::uint64_t> open_path_generation;
            std::uint64_t open_path_searches{}, open_paths_added{};
            std::uint64_t open_path_cache_hits{}, open_path_interruptions{}, open_path_skipped{};
            std::uint64_t open_path_k_reached{}, open_path_exhausted{};
            std::size_t open_path_cursor{};
            double open_path_seconds{};
            double guidance_seconds{}, guidance_lp_seconds{}, guidance_build_seconds{}, guidance_pricing_seconds{}, support_seconds{},
                pool_objective{};
            int guidance_lp_calls{};
            std::uint64_t rejected_candidate_checks{};
            int support_paths_added{};
            double physical_alternative_seconds{};
            int physical_alternatives_added{};
            int guided_logical{}, guidance_status{};
            int mip_calls{}, mip_columns{}, mip_rows{}, mip_round{}, stale_repairs{}, expanded_repairs{};
            int pool_mip_continuations{};
            Json candidate_pool, initial_candidate_pool;
            Json pool_reduction_audit;
            Json fixed_assignment_audit;
            Json pool_model_history = Json::array();
            Json capacity_row_history = Json::array();
            Json path_generation_history = Json::array();
            std::uint64_t component_joint_calls{}, component_joint_skips{}, component_prep_calls{};
            std::uint64_t component_pg_calls{}, component_pg_skips{}, component_local_calls{}, component_local_skips{};
            std::function<void(const Search &)> component_ctor_observer;
            double component_ctor_complete_seconds{};
            std::uint64_t component_joint_arc_enabled{}, component_joint_arc_disabled{};
            int path_model_columns{}, path_model_rows{};
            LocalSearchResult &out;
            bool demand_driven;
            bool incremental_active{};
            double incremental_begin{}, last_global_improvement{}, last_progress_log{};
            double incumbent_restore_seconds{}, incremental_seconds{}, group_phase_seconds{};
            std::string incremental_stop_reason{"completed"};
            std::uint64_t sweeps{}, incumbent_generation{}, group_cache_generation{};
            std::size_t repair_index{}, repair_total{};
            int exhausted_group_skips{};
            std::map<std::string, int> exhausted_groups;
            Json group_history = Json::array();
            std::uint64_t search_cycles{};
            std::uint64_t destroy_round{}, pricing_rounds{};
            Json schedule_history = Json::array();
            int pool_repeat_skips{}, lp_repeat_skips{};
            std::mt19937 random;
            std::string exhausted_pool_key, priced_pool_key;
            void progressLog(const char *phase) {
                const double now = budget.elapsed();
                if (options.progress_log_seconds <= 0 || now - last_progress_log < options.progress_log_seconds)
                    return;
                last_progress_log = now;
                std::clog << "SEARCH_PROGRESS phase=" << phase << " seconds=" << now
                          << " best=" << std::setprecision(17) << out.objective
                          << " since_best=" << now - last_global_improvement
                          << " sweeps=" << sweeps << " repair=" << repair_index << '/' << repair_total
                          << " evaluated=" << evaluated << " physical_searches=" << physical_searches
                          << " physical_cache_hits=" << physical_cache_hits << '\n';
            }
            std::vector<int> original_blocks;
            void notify() {
                ++incumbent_generation;
                last_global_improvement = budget.elapsed();
            }

            std::vector<int> physicalPath(int b, double level, double weight, int forbidden = -1,
                                          bool distance_only = false) {
                budget.check();
                const auto &block = p.blocks[b];
                const bool shared =
                    forbidden < 0 && (weight < 0 || (proportional_costs && block.arc_cost.empty()));
                const int capacity_class =
                    std::lower_bound(capacity_levels.begin(), capacity_levels.end(), level - Tol) -
                    capacity_levels.begin();
                const std::pair<int, int> key{block.from, capacity_class};
                std::vector<int> previous;
                auto cached = physical_trees.find(key);
                if (shared && cached != physical_trees.end()) {
                    ++physical_cache_hits;
                    if (distance_only)
                        return {};
                    previous = cached->second.previous;
                } else {
                    ++physical_searches;
                    std::vector<double> distance(p.nodes.size(), std::numeric_limits<double>::infinity());
                    previous.assign(p.nodes.size(), -1);
                    using Item = std::pair<double, int>;
                    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
                    distance[block.from] = 0;
                    queue.emplace(0, block.from);
                    int visits = 0;
                    while (!queue.empty()) {
                        if ((++visits & 1023) == 0)
                            budget.check();
                        auto [d, i] = queue.top();
                        queue.pop();
                        if (d != distance[i])
                            continue;
                        if (!shared && i == block.to)
                            break;
                        for (int e = p.physical.offsets[i]; e < p.physical.offsets[i + 1]; ++e) {
                            int a = p.physical.edge_ids[e];
                            const auto &arc = p.arcs[a];
                            if (a == forbidden || p.capacities[arc.link] + Tol < level)
                                continue;
                            double cost =
                                shared || weight < 0 ? arc.length : p.transport(b, a) + weight * arc.length;
                            if (d + cost < distance[arc.to]) {
                                distance[arc.to] = d + cost;
                                previous[arc.to] = a;
                                queue.emplace(distance[arc.to], arc.to);
                            }
                        }
                    }
                    if (shared) {
                        if (physical_trees.size() >= std::size_t(options.physical_tree_cache_size)) {
                            physical_trees.erase(tree_order.front());
                            tree_order.pop_front();
                        }
                        physical_trees[key] = {previous, std::move(distance)};
                        tree_order.push_back(key);
                    }
                }
                if (distance_only)
                    return {};
                ++physical_route_recoveries;
                std::vector<int> path;
                for (int i = block.to; i != block.from;) {
                    int a = previous[i];
                    if (a < 0)
                        return {};
                    path.push_back(a);
                    i = p.arcs[a].from;
                    if (path.size() >= p.nodes.size())
                        throw std::runtime_error("Heuristic physical cycle");
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            void ensureRoutes(int b) {
                if (seeded[b])
                    return;
                budget.check();
                const double start = budget.elapsed();
                struct Timer {
                    double &value;
                    const Budget &clock;
                    double start;
                    ~Timer() {
                        value += clock.elapsed() - start;
                    }
                } timer{seed_time, budget, start};
                {
                    const auto &block = p.blocks[b];
                    if (block.upper + Tol < block.lower) {
                        seeded[b] = true;
                        return;
                    }
                    double q = block.lower;
                    for (int k : p.compatibleDemands(b))
                        q = std::max(q, p.demands[k].quantity);
                    std::set<double> levels{block.lower, std::min(block.upper, q),
                                            std::min(block.upper, options.seed_load_multiple * q)};
                    std::set<std::vector<int>> seen;
                    for (const auto &route : pool[b]) seen.insert(route.arcs);
                    auto retain = [&](std::vector<int> path) {
                        if (path.empty() || !seen.insert(path).second)
                            return;
                        double length = 0;
                        Route r;
                        r.arcs = path;
                        r.unit = block.interchange;
                        std::map<int, int> links;
                        for (int a : path) {
                            length += p.arcs[a].length;
                            r.unit += p.transport(b, a);
                            ++links[p.arcs[a].link];
                        }
                        if (length > block.length_limit + Tol)
                            return;
                        r.capacity = block.upper;
                        for (auto [e, n] : links) {
                            r.links.emplace_back(e, n);
                            r.capacity = std::min(r.capacity, p.capacities[e] / n);
                        }
                        if (r.capacity + Tol >= block.lower)
                            pool[b].push_back(std::move(r));
                    };
                    std::set<int> filtered_graphs;
                    const bool single_metric = proportional_costs && block.arc_cost.empty();
                    for (double level : levels) {
                        int graph =
                            std::lower_bound(capacity_levels.begin(), capacity_levels.end(), level - Tol) -
                            capacity_levels.begin();
                        if (single_metric && !filtered_graphs.insert(graph).second)
                            continue;
                        auto shortest = physicalPath(b, level, -1.);
                        double shortest_length = 0;
                        for (int a : shortest) shortest_length += p.arcs[a].length;
                        if (shortest.empty() || shortest_length > block.length_limit + Tol) break;
                        retain(std::move(shortest));
                        if (single_metric)
                            continue;
                        auto cheapest = physicalPath(b, level, 0.);
                        double path_length = 0;
                        for (int a : cheapest)
                            path_length += p.arcs[a].length;
                        retain(cheapest);
                        if (!cheapest.empty() && path_length > block.length_limit + Tol &&
                            options.initial_physical_spp_paths > 0) {
                            std::vector<double> cost(p.arcs.size());
                            std::vector<char> allowed(p.arcs.size());
                            for (int a = 0; a < (int)p.arcs.size(); ++a) {
                                budget.check();
                                cost[a] = p.transport(b, a);
                                allowed[a] = p.capacities[p.arcs[a].link] + Tol >= level;
                            }
                            for (auto &path : shortestSimplePaths(p.physical, block.from, block.to, cost,
                                                                 allowed, budget,
                                                                 options.initial_physical_spp_paths))
                                retain(std::move(path.edges));
                        }
                        for (double weight : options.physical_length_weights)
                            if (weight >= 0)
                                retain(physicalPath(b, level, weight));
                    }
                    std::stable_sort(pool[b].begin(), pool[b].end(),
                                     [](const Route &a, const Route &b) { return a.unit < b.unit; });
                    ++seed_blocks_completed;
                    seeded[b] = true;
                }
            }
            double blockScore(const State &s, int b, int k, bool generate_routes = true) {
                if (generate_routes) ensureRoutes(b);
                if (pool[b].empty())
                    return std::numeric_limits<double>::infinity();
                double unit = s.route[b] >= 0
                                  ? pool[b][s.route[b]].unit
                                  : std::min_element(pool[b].begin(), pool[b].end(),
                                                     [](const Route &a, const Route &c) {
                                                         return a.unit < c.unit;
                                                     })->unit;
                return unit + (s.route[b] < 0 ? p.blocks[b].opening / p.demands[k].quantity : 0.);
            }
            double blockEstimate(const State &s, int b, int k) {
                if (s.route[b] >= 0 || seeded[b] || !proportional_costs || !p.blocks[b].arc_cost.empty())
                    return blockScore(s, b, k);
                const auto &block = p.blocks[b];
                if (block.upper + Tol < block.lower)
                    return std::numeric_limits<double>::infinity();
                ++physical_estimates;
                physicalPath(b, block.lower, -1., -1, true);
                int capacity_class =
                    std::lower_bound(capacity_levels.begin(), capacity_levels.end(), block.lower - Tol) -
                    capacity_levels.begin();
                double length = physical_trees.at({block.from, capacity_class}).distance[block.to];
                if (!std::isfinite(length) || length > block.length_limit + Tol)
                    return std::numeric_limits<double>::infinity();
                return block.interchange + length * transport_length_ratio * block.transport_factor +
                       block.opening / p.demands[k].quantity;
            }
            int connection(const State &s, int k, int from, int to) {
                int unique = -1;
                bool multiple = false;
                for (int e = p.logical.offsets[from]; e < p.logical.offsets[from + 1]; ++e) {
                    int b = p.logical.edge_ids[e];
                    if (p.blocks[b].to != to || !p.compatible(k, b))
                        continue;
                    if (unique >= 0) {
                        multiple = true;
                        break;
                    }
                    unique = b;
                }
                if (!multiple) {
                    if (unique < 0)
                        return -1;
                    ensureRoutes(unique);
                    return pool[unique].empty() ? -1 : unique;
                }
                std::vector<std::pair<double, int>> candidates;
                for (int e = p.logical.offsets[from]; e < p.logical.offsets[from + 1]; ++e) {
                    int b = p.logical.edge_ids[e];
                    if (p.blocks[b].to == to && p.compatible(k, b))
                        candidates.emplace_back(blockEstimate(s, b, k), b);
                }
                std::sort(candidates.begin(), candidates.end());
                int best = -1;
                double value = std::numeric_limits<double>::infinity();
                for (auto [estimate, b] : candidates) {
                    if (estimate >= value)
                        break;
                    double cost = blockScore(s, b, k);
                    if (cost < value) {
                        best = b;
                        value = cost;
                    }
                }
                return best;
            }
            std::vector<int> via(const State &s, int k, int hub) {
                const auto &d = p.demands[k];
                if (hub == d.from || hub == d.to)
                    return {};
                int first = connection(s, k, d.from, hub), last = connection(s, k, hub, d.to);
                return first < 0 || last < 0 ? std::vector<int>{} : std::vector<int>{first, last};
            }
            std::vector<int> corridor(const State &s, int k, int first, int second) {
                std::vector<int> nodes{p.demands[k].from};
                for (int i : {first, second, p.demands[k].to})
                    if (i != nodes.back())
                        nodes.push_back(i);
                auto unique = nodes;
                std::sort(unique.begin(), unique.end());
                if (std::adjacent_find(unique.begin(), unique.end()) != unique.end())
                    return {};
                std::vector<int> path;
                for (std::size_t j = 1; j < nodes.size(); ++j) {
                    int b = connection(s, k, nodes[j - 1], nodes[j]);
                    if (b < 0)
                        return {};
                    path.push_back(b);
                }
                return path;
            }
            std::vector<int> demandHubs(int k) const {
                std::vector<int> hubs;
                int from = p.demands[k].from;
                for (int e = p.logical.offsets[from]; e < p.logical.offsets[from + 1]; ++e) {
                    int b = p.logical.edge_ids[e];
                    if (p.compatible(k, b) && p.blocks[b].to != p.demands[k].to)
                        hubs.push_back(p.blocks[b].to);
                }
                std::sort(hubs.begin(), hubs.end());
                hubs.erase(std::unique(hubs.begin(), hubs.end()), hubs.end());
                return hubs;
            }
            std::vector<int> logicalPath(const State &s, int k, bool opened_only, bool joint = false) {
                const auto &d = p.demands[k];
                std::vector<double> distance(p.nodes.size(), 1e100);
                std::vector<int> previous(p.nodes.size(), -1);
                using Item = std::pair<double, int>;
                std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
                distance[d.from] = 0;
                queue.emplace(0, d.from);
                while (!queue.empty()) {
                    auto [value, i] = queue.top();
                    queue.pop();
                    if (value != distance[i])
                        continue;
                    if (i == d.to)
                        break;
                    for (int e = p.logical.offsets[i]; e < p.logical.offsets[i + 1]; ++e) {
                        int b = p.logical.edge_ids[e];
                        if (!p.compatible(k, b))
                            continue;
                        if (!demand_driven && !opened_only)
                            ensureRoutes(b);
                        if (pool[b].empty() || (opened_only && s.route[b] < 0))
                            continue;
                        if (!joint) {
                            double outside = s.load[b];
                            if (std::find(s.plan.logical[k].begin(), s.plan.logical[k].end(), b) !=
                                s.plan.logical[k].end())
                                outside -= s.plan.served[k];
                            if (outside + d.quantity < p.blocks[b].lower - Tol)
                                continue;
                        }
                        int j = p.blocks[b].to;
                        double cost = blockScore(s, b, k) + (i == d.from ? 0 : p.nodes[i].handling_cost);
                        if (value + cost < distance[j]) {
                            distance[j] = value + cost;
                            previous[j] = b;
                            queue.emplace(distance[j], j);
                        }
                    }
                }
                std::vector<int> path;
                for (int i = d.to; i != d.from;) {
                    int b = previous[i];
                    if (b < 0)
                        return {};
                    path.push_back(b);
                    i = p.blocks[b].from;
                    if (path.size() >= yards.size())
                        throw std::runtime_error("Heuristic logical cycle");
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            double pathUnit(const State &s, int k, const std::vector<int> &path, bool generate_routes = true) {
                double c = 0;
                for (int b : path)
                    c +=
                        blockScore(s, b, k, generate_routes) +
                        (p.blocks[b].from == p.demands[k].from ? 0 : p.nodes[p.blocks[b].from].handling_cost);
                return c;
            }
            Move evaluate(const State &s, const std::vector<Change> &changes, bool wide) {
                budget.check();
                ++evaluated;
                if (incremental_active) progressLog("incremental");
                const double begin = budget.elapsed();
                for (int b : touched_blocks)
                    affected[b] = false;
                for (int i : touched_yards)
                    yard_affected[i] = false;
                for (int e : touched_links)
                    link_affected[e] = false;
                touched_blocks.clear();
                touched_yards.clear();
                touched_links.clear();
                auto touchYard = [&](int i) {
                    if (!yard_affected[i]) {
                        yard_affected[i] = true;
                        touched_yards.push_back(i);
                        h0[i] = s.handling[i];
                        hq[i] = 0;
                        track[i] = s.tracks[i];
                    }
                };
                auto touchBlock = [&](int b) {
                    if (!affected[b]) {
                        affected[b] = true;
                        touched_blocks.push_back(b);
                        b0[b] = s.load[b];
                        bq[b] = 0;
                        touchYard(p.blocks[b].from);
                    }
                };
                auto touchLink = [&](int e) {
                    if (!link_affected[e]) {
                        link_affected[e] = true;
                        touched_links.push_back(e);
                        l0[e] = 0;
                        lq[e] = 0;
                    }
                };
                // Preserve the former strict move checks even for an incumbent with a
                // resource residual inside the final validator's numerical allowance.
                for (int i : s.strict_bad_yards)
                    touchYard(i);
                for (int e : s.strict_bad_links)
                    touchLink(e);
                auto account = [&] {
                    evaluated_block_cells += touched_blocks.size();
                    evaluated_yard_cells += touched_yards.size();
                    evaluated_link_cells += touched_links.size();
                    move_time += budget.elapsed() - begin;
                };
                // Restoring served traffic subtracts large rejection terms again. Compensated
                // sums keep this cancellation consistent with the full-plan cost check.
                CostSum constant, slope;
                constant += s.cost;
                double lo = 0, hi = 1;
                Move m;
                m.changes = changes;
                auto reject = [&]() {
                    ++infeasible;
                    account();
                    return Move{};
                };
                for (const auto &c : changes) {
                    const int k = c.demand;
                    const double old = s.plan.served[k];
                    constant += old * p.demands[k].penalty;
                    slope += -c.target * p.demands[k].penalty;
                    for (int b : s.plan.logical[k]) {
                        touchBlock(b);
                        b0[b] -= old;
                        int i = p.blocks[b].from;
                        if (i != p.demands[k].from) {
                            h0[i] -= old;
                            constant += -old * p.nodes[i].handling_cost;
                        }
                    }
                    for (int b : c.path) {
                        touchBlock(b);
                        bq[b] += c.target;
                        int i = p.blocks[b].from;
                        if (i != p.demands[k].from) {
                            hq[i] += c.target;
                            slope += c.target * p.nodes[i].handling_cost;
                        }
                    }
                }
                std::sort(touched_blocks.begin(), touched_blocks.end());
                for (int b : touched_blocks) {
                    const auto &block = p.blocks[b];
                    if (std::abs(b0[b]) < Tol)
                        b0[b] = 0;
                    if (bq[b] > Tol) {
                        lo = std::max(lo, (block.lower - b0[b]) / bq[b]);
                        hi = std::min(hi, (block.upper - b0[b]) / bq[b]);
                    } else if (b0[b] > Tol && b0[b] < block.lower - Tol)
                        return reject();
                    if (s.route[b] >= 0) {
                        const auto &r = pool[b][s.route[b]];
                        constant += -(block.opening + s.load[b] * r.unit);
                        track[block.from] -= block.slots;
                        for (auto [e, n] : r.links) {
                            touchLink(e);
                            l0[e] -= n * s.load[b];
                        }
                    }
                    if (b0[b] + bq[b] > Tol)
                        track[block.from] += block.slots;
                }
                for (int i : touched_yards) {
                    if (track[i] > p.nodes[i].tracks + Tol)
                        return reject();
                    if (hq[i] > Tol)
                        hi = std::min(hi, (p.nodes[i].handling_capacity - h0[i]) / hq[i]);
                    else if (h0[i] > p.nodes[i].handling_capacity + Tol)
                        return reject();
                }
                lo = std::max(0., lo);
                if (lo > hi + Tol)
                    return reject();
                // Retain existing physical paths; choose new paths against joint residual
                // capacities.
                std::vector<int> ordered;
                for (int b : touched_blocks)
                    if (b0[b] + bq[b] > Tol && s.route[b] >= 0)
                        ordered.push_back(b);
                for (int b : touched_blocks)
                    if (b0[b] + bq[b] > Tol && s.route[b] < 0)
                        ordered.push_back(b);
                for (int b : ordered) {
                    int selected = s.route[b];
                    double best = -1;
                    if (selected < 0)
                        for (int j = 0; j < (int)pool[b].size(); ++j) {
                            const auto &r = pool[b][j];
                            double limit = hi;
                            bool feasible = true;
                            for (auto [e, n] : r.links) {
                                touchLink(e);
                                double base = s.link[e] + l0[e] + n * b0[b], rate = lq[e] + n * bq[b];
                                if (base + lo * rate > p.capacities[e] + Tol) {
                                    feasible = false;
                                    break;
                                }
                                if (rate > Tol)
                                    limit = std::min(limit, (p.capacities[e] - base) / rate);
                            }
                            if (!feasible)
                                continue;
                            if (!wide) {
                                if (selected < 0 || r.unit < pool[b][selected].unit - Tol)
                                    selected = j;
                                continue;
                            }
                            if (limit > best + Tol ||
                                (std::abs(limit - best) <= Tol &&
                                 (selected < 0 || r.unit < pool[b][selected].unit - Tol))) {
                                selected = j;
                                best = limit;
                            }
                        }
                    if (selected < 0)
                        return reject();
                    const auto &r = pool[b][selected];
                    m.routes.emplace_back(b, selected);
                    constant += p.blocks[b].opening + b0[b] * r.unit;
                    slope += bq[b] * r.unit;
                    for (auto [e, n] : r.links) {
                        touchLink(e);
                        l0[e] += n * b0[b];
                        lq[e] += n * bq[b];
                        // Later path choices must respect bottlenecks already selected.
                        const double base = s.link[e] + l0[e];
                        if (lq[e] > Tol)
                            hi = std::min(hi, (p.capacities[e] - base) / lq[e]);
                        else if (base > p.capacities[e] + Tol)
                            return reject();
                    }
                    if (lo > hi + Tol || hi < -Tol)
                        return reject();
                }
                for (int e : touched_links) {
                    double base = s.link[e] + l0[e];
                    if (lq[e] > Tol)
                        hi = std::min(hi, (p.capacities[e] - base) / lq[e]);
                    else if (base > p.capacities[e] + Tol)
                        return reject();
                }
                if (lo > hi + Tol || hi < -Tol)
                    return reject();
                m.alpha = std::clamp(slope.value < 0 ? hi : lo, 0., 1.);
                if (m.alpha <= Tol) {
                    // Rejection is permitted: price a zero endpoint with its actual
                    // block openings, rather than the unused proposed paths.
                    for (const auto &c : changes)
                        if (c.target > Tol) {
                            std::vector<Change> removals;
                            for (const auto &change : changes)
                                removals.push_back({change.demand, {}, 0.});
                            account();
                            return evaluate(s, removals, wide);
                        }
                }
                m.cost = constant.value + m.alpha * slope.value;
                account();
                return m;
            }
            void rebuild(State &s) const {
                std::fill(s.load.begin(), s.load.end(), 0);
                std::fill(s.link.begin(), s.link.end(), 0);
                std::fill(s.handling.begin(), s.handling.end(), 0);
                std::fill(s.tracks.begin(), s.tracks.end(), 0);
                for (int k = 0; k < (int)p.demands.size(); ++k)
                    for (int b : s.plan.logical[k]) {
                        s.load[b] += s.plan.served[k];
                        int i = p.blocks[b].from;
                        if (i != p.demands[k].from)
                            s.handling[i] += s.plan.served[k];
                    }
                for (int b = 0; b < (int)p.blocks.size(); ++b) {
                    if (s.load[b] <= Tol) {
                        s.route[b] = -1;
                        s.plan.physical[b].clear();
                        continue;
                    }
                    if (s.route[b] < 0)
                        throw std::runtime_error("Loaded block lacks a physical path");
                    const auto &r = pool[b][s.route[b]];
                    s.plan.physical[b] = r.arcs;
                    s.tracks[p.blocks[b].from] += p.blocks[b].slots;
                    for (auto [e, n] : r.links)
                        s.link[e] += n * s.load[b];
                }
                s.cost = s.plan.validate(p);
                s.strict_bad_yards.clear();
                s.strict_bad_links.clear();
                for (int i : yards)
                    if (s.tracks[i] > p.nodes[i].tracks + Tol ||
                        s.handling[i] > p.nodes[i].handling_capacity + Tol)
                        s.strict_bad_yards.push_back(i);
                for (int e = 0; e < int(p.capacities.size()); ++e)
                    if (s.link[e] > p.capacities[e] + Tol)
                        s.strict_bad_links.push_back(e);
            }
            void commit(State &s, const Move &m) {
                for (const auto &c : m.changes) {
                    s.plan.logical[c.demand] = c.path;
                    s.plan.served[c.demand] = m.alpha * c.target;
                    if (s.plan.served[c.demand] <= Tol)
                        s.plan.logical[c.demand].clear();
                }
                for (auto [b, r] : m.routes)
                    s.route[b] = r;
                rebuild(s);
                if (!objectiveEqual(s.cost, m.cost))
                    throw std::runtime_error(
                        "Incremental objective mismatch: validated=" + std::to_string(s.cost) +
                        " predicted=" + std::to_string(m.cost) + " delta=" + std::to_string(s.cost - m.cost));
                ++accepted;
            }
            void save(const State &s, const char *stage) {
                if (s.cost < out.objective - Tol) {
                    out.objective = s.cost;
                    out.plan = s.plan;
                    out.history.push_back(
                        {{"seconds", budget.elapsed()}, {"objective", s.cost}, {"stage", stage}});
                    notify();
                    if (out.history.size() == 1 || out.history.size() % 50 == 0)
                        std::clog << "IMPROVE stage=" << stage << " seconds=" << budget.elapsed()
                                  << " objective=" << s.cost << " served="
                                  << std::accumulate(s.plan.served.begin(), s.plan.served.end(), 0.) << '\n';
                }
            }
            void consider(const State &s, const std::vector<Change> &changes, Move &best) {
                for (bool wide : {false, true}) {
                    Move m = evaluate(s, changes, wide);
                    if (m.cost < best.cost - objectiveTolerance(m.cost, best.cost))
                        best = std::move(m);
                }
            }
            std::vector<Change> supported(const State &s, std::vector<Change> changes) const {
                for (;;) {
                    budget.check();
                    auto loads = s.load;
                    for (const auto &c : changes) {
                        for (int b : s.plan.logical[c.demand])
                            loads[b] -= s.plan.served[c.demand];
                        for (int b : c.path)
                            loads[b] += c.target;
                    }
                    auto bad = [&](int b) { return loads[b] > Tol && loads[b] < p.blocks[b].lower - Tol; };
                    auto last = std::remove_if(changes.begin(), changes.end(), [&](const Change &c) {
                        return std::any_of(c.path.begin(), c.path.end(), bad) ||
                               std::any_of(s.plan.logical[c.demand].begin(), s.plan.logical[c.demand].end(),
                                           bad);
                    });
                    if (last == changes.end())
                        return changes;
                    changes.erase(last, changes.end());
                }
            }
            void group(const State &s, std::vector<Change> changes, Move &best) {
                std::vector<std::pair<double, std::size_t>> rank;
                for (std::size_t i = 0; i < changes.size(); ++i) {
                    const auto &c = changes[i];
                    const auto &d = p.demands[c.demand];
                    const double volume = s.plan.served[c.demand];
                    const double current = volume * pathUnit(s, c.demand, s.plan.logical[c.demand], false) +
                                           (d.quantity - volume) * d.penalty;
                    rank.emplace_back(current / d.quantity - pathUnit(s, c.demand, c.path), i);
                }
                std::stable_sort(rank.begin(), rank.end(), [](const auto &a, const auto &b) {
                    return a.first > b.first;
                });
                std::vector<Change> sorted;
                sorted.reserve(changes.size());
                for (auto [score, i] : rank) sorted.push_back(std::move(changes[i]));
                changes = std::move(sorted);
                std::vector<Change> selected;
                for (auto &c : changes) {
                    budget.check();
                    selected.push_back(std::move(c));
                    if (selected.size() > 1) {
                        auto feasible_support = supported(s, selected);
                        if (!feasible_support.empty())
                            consider(s, feasible_support, best);
                    }
                }
            }
            enum class CandidateKind { Single, Direct, Corridor };
            struct DemandCandidate {
                CandidateKind kind;
                std::vector<int> demands;
                int first = -1, second = -1;
            };
            bool canConnect(int k, int from, int to) const {
                for (int e = p.logical.offsets[from]; e < p.logical.offsets[from + 1]; ++e) {
                    int b = p.logical.edge_ids[e];
                    if (p.blocks[b].to == to && p.compatible(k, b)) return true;
                }
                return false;
            }
            std::vector<int> prescribedYards(int k, int first, int second = -1) const {
                std::vector<int> nodes{p.demands[k].from};
                std::vector<int> stops{first};
                if (second >= 0) stops.push_back(second);
                stops.push_back(p.demands[k].to);
                for (int i : stops) if (i != nodes.back()) nodes.push_back(i);
                auto unique = nodes;
                std::sort(unique.begin(), unique.end());
                if (std::adjacent_find(unique.begin(), unique.end()) != unique.end()) return {};
                return nodes;
            }
            std::vector<DemandCandidate> constructionCandidates() {
                std::vector<DemandCandidate> candidates;
                auto add = [&](DemandCandidate c) {
                    const bool single = c.kind == CandidateKind::Single;
                    if (c.demands.size() >= (single ? 1u : 2u)) candidates.push_back(std::move(c));
                };
                for (int k = 0; k < int(p.demands.size()); ++k)
                    add({CandidateKind::Single, {k}});
                for (int b = 0; b < int(p.blocks.size()); ++b) {
                    budget.check();
                    DemandCandidate c{CandidateKind::Direct, {}, b};
                    for (int k : p.compatibleDemands(b))
                        if (p.demands[k].from == p.blocks[b].from && p.demands[k].to == p.blocks[b].to)
                            c.demands.push_back(k);
                    add(std::move(c));
                }
                auto bundle = [&](int first, int second) {
                    DemandCandidate c{CandidateKind::Corridor, {}, first, second};
                    for (int k = 0; k < int(p.demands.size()); ++k) {
                        budget.check();
                        const auto nodes = prescribedYards(k, first, second);
                        bool compatible = !nodes.empty();
                        for (std::size_t j = 1; j < nodes.size() && compatible; ++j)
                            compatible = canConnect(k, nodes[j - 1], nodes[j]);
                        if (compatible) c.demands.push_back(k);
                    }
                    add(std::move(c));
                };
                for (int h : bundle_yards) bundle(h, h);
                for (int first : group_yards)
                    for (int second : group_yards)
                        if (first != second) bundle(first, second);
                std::vector<std::pair<double, int>> anchors;
                for (int b = 0; b < int(p.blocks.size()); ++b) {
                    double loss = 0;
                    for (int k : p.compatibleDemands(b)) {
                        const auto &d = p.demands[k];
                        if (d.from == p.blocks[b].from && d.to == p.blocks[b].to)
                            loss += d.quantity * d.penalty;
                    }
                    if (loss > 0) anchors.emplace_back(-loss, b);
                }
                std::sort(anchors.begin(), anchors.end());
                for (auto [score, b] : anchors) {
                    const auto &block = p.blocks[b];
                    if (std::find(group_yards.begin(), group_yards.end(), block.from) != group_yards.end() &&
                        std::find(group_yards.begin(), group_yards.end(), block.to) != group_yards.end())
                        continue;
                    bundle(block.from, block.to);
                }
                return candidates;
            }
            std::set<std::vector<int>> singlePaths(const State &s, int k) {
                const auto &d = p.demands[k];
                std::set<std::vector<int>> paths;
                for (int b : p.compatibleBlocks(k))
                    if (p.blocks[b].from == d.from && p.blocks[b].to == d.to) {
                        ensureRoutes(b);
                        if (!pool[b].empty()) paths.insert({b});
                    }
                for (int h : demandHubs(k)) {
                    auto path = via(s, k, h);
                    if (!path.empty()) paths.insert(std::move(path));
                }
                for (bool opened : {false, true}) {
                    auto path = logicalPath(s, k, opened);
                    if (!path.empty()) paths.insert(std::move(path));
                }
                return paths;
            }
            std::vector<Change> candidateAssignments(const State &s, const DemandCandidate &c) {
                std::vector<Change> assignments;
                if (c.kind == CandidateKind::Single) {
                    int k = c.demands.front();
                    for (const auto &path : singlePaths(s, k))
                        assignments.push_back({k, path, p.demands[k].quantity});
                } else {
                    if (c.kind == CandidateKind::Direct) ensureRoutes(c.first);
                    for (int k : c.demands) {
                        budget.check();
                        std::vector<int> path;
                        switch (c.kind) {
                        case CandidateKind::Direct:
                            if (!pool[c.first].empty()) path = {c.first};
                            break;
                        case CandidateKind::Corridor: path = corridor(s, k, c.first, c.second); break;
                        case CandidateKind::Single: break;
                        }
                        if (!path.empty()) assignments.push_back({k, std::move(path), p.demands[k].quantity});
                    }
                }
                return assignments;
            }
            bool sweep(State &s) {
                ++sweeps;
                progressLog("incremental");
                bool changed = false;
                try {
                    // Fix demand membership first; costs and paths are computed only when processing it.
                    const auto candidates = constructionCandidates();
                    for (const auto &c : candidates) {
                        budget.check();
                        Move best;
                        best.cost = s.cost;
                        try {
                            auto assignments = candidateAssignments(s, c);
                            if (c.kind == CandidateKind::Single) {
                                for (const auto &assignment : assignments) consider(s, {assignment}, best);
                            } else if (assignments.size() > 1) group(s, std::move(assignments), best);
                        } catch (const LimitReached &) {
                            if (best.cost < s.cost - objectiveTolerance(best.cost, s.cost)) {
                                commit(s, best);
                                save(s, "incremental");
                            }
                            throw;
                        }
                        if (best.cost < s.cost - objectiveTolerance(best.cost, s.cost)) {
                            commit(s, best);
                            save(s, "incremental");
                            changed = true;
                        }
                    }
                } catch (const LimitReached &) {}
                return changed;
            }
            void descend(State &s, const char *stage) {
                for (;;) {
                    budget.check();
                    const double objective_before = s.cost;
                    bool changed = sweep(s);
                    save(s, stage);
                    if (budget.remaining() <= 0)
                        return;
                    const double relative_improvement =
                        (objective_before - s.cost) / std::max(1., std::abs(objective_before));
                    std::clog << "SWEEP_COMPLETE sweep=" << sweeps << " objective=" << s.cost
                              << " relative_improvement=" << relative_improvement << '\n';
                    if (!changed || relative_improvement <= options.construction_relative_tolerance) {
                        incremental_stop_reason = changed ? "relative_sweep_tailoff" : "no_improving_sweep";
                        break;
                    }
                }
            }
            void destroy(State &s, int block) {
                std::vector<char> remove(p.demands.size());
                for (int k = 0; k < (int)remove.size(); ++k)
                    remove[k] = std::find(s.plan.logical[k].begin(), s.plan.logical[k].end(), block) !=
                                s.plan.logical[k].end();
                bool changed = true;
                while (changed) {
                    changed = false;
                    std::vector<double> remaining = s.load;
                    for (int k = 0; k < (int)remove.size(); ++k)
                        if (remove[k])
                            for (int b : s.plan.logical[k])
                                remaining[b] -= s.plan.served[k];
                    for (int b = 0; b < (int)remaining.size(); ++b)
                        if (remaining[b] > Tol && remaining[b] < p.blocks[b].lower - Tol)
                            for (int k : p.compatibleDemands(b))
                                if (!remove[k] &&
                                    std::find(s.plan.logical[k].begin(), s.plan.logical[k].end(), b) !=
                                        s.plan.logical[k].end()) {
                                    remove[k] = true;
                                    changed = true;
                                }
                }
                for (int k = 0; k < (int)remove.size(); ++k)
                    if (remove[k]) {
                        s.plan.logical[k].clear();
                        s.plan.served[k] = 0;
                    }
                rebuild(s);
            }

            std::vector<std::vector<int>> bridgePaths(const State &s, int k, bool protect_splits) {
                struct Timer {
                    double &total;
                    const Budget &budget;
                    double start;
                    ~Timer() {
                        total += budget.elapsed() - start;
                    }
                } timer{bridge_generation_seconds, budget, budget.elapsed()};
                const auto &d = p.demands[k];
                const int Y = yards.size(), N = p.nodes.size();
                std::vector<int> yard_index(N, -1);
                for (int i = 0; i < Y; ++i)
                    yard_index[yards[i]] = i;
                std::vector<std::vector<int>> open_path(Y * Y);
                std::vector<char> reachable(Y * Y);
                using Item = std::pair<double, int>;
                for (int root = 0; root < Y; ++root) {
                    budget.check();
                    std::vector<double> dist(N, 1e100);
                    std::vector<int> prev(N, -1);
                    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
                    dist[yards[root]] = 0;
                    queue.emplace(0, yards[root]);
                    while (!queue.empty()) {
                        auto [value, i] = queue.top();
                        queue.pop();
                        if (value != dist[i])
                            continue;
                        for (int e = p.logical.offsets[i]; e < p.logical.offsets[i + 1]; ++e) {
                            int b = p.logical.edge_ids[e];
                            if (!p.compatible(k, b) || s.route[b] < 0)
                                continue;
                            int j = p.blocks[b].to;
                            double cost = pool[b][s.route[b]].unit + p.nodes[i].handling_cost;
                            if (value + cost < dist[j]) {
                                dist[j] = value + cost;
                                prev[j] = b;
                                queue.emplace(dist[j], j);
                            }
                        }
                    }
                    for (int end = 0; end < Y; ++end)
                        if (dist[yards[end]] < 1e99) {
                            int id = root * Y + end;
                            reachable[id] = true;
                            for (int i = yards[end]; i != yards[root]; i = p.blocks[prev[i]].from)
                                open_path[id].push_back(prev[i]);
                            std::reverse(open_path[id].begin(), open_path[id].end());
                        }
                }
                auto index = [&](int a, int b) { return yard_index[a] * Y + yard_index[b]; };
                using Ranked = std::pair<double, std::vector<int>>;
                std::vector<std::set<Ranked>> best(2 * options.max_new_blocks);
                std::set<std::vector<int>> coverage_paths;
                std::vector<int> path;
                std::vector<char> visited(N);
                visited[d.from] = true;
                auto undo = [&](std::size_t mark) {
                    while (path.size() > mark) {
                        visited[p.blocks[path.back()].to] = false;
                        path.pop_back();
                    }
                };
                auto append = [&](const std::vector<int> &segment, bool terminal) {
                    int at = path.empty() ? d.from : p.blocks[path.back()].to;
                    for (int b : segment) {
                        const auto &block = p.blocks[b];
                        if (block.from != at || visited[block.to] || (!terminal && block.to == d.to))
                            return false;
                        path.push_back(b);
                        visited[block.to] = true;
                        at = block.to;
                    }
                    return true;
                };
                auto retain = [&](int count) {
                    bool supported = true;
                    for (int b : path) {
                        double outside = s.load[b];
                        if (std::find(s.plan.logical[k].begin(), s.plan.logical[k].end(), b) !=
                            s.plan.logical[k].end())
                            outside -= s.plan.served[k];
                        supported = supported && outside + d.quantity >= p.blocks[b].lower - Tol;
                    }
                    // Preserve a complete incumbent chain when one block is split into two.
                    const auto &incumbent = s.plan.logical[k];
                    bool single_block_split = false;
                    if (protect_splits && count == 2 && path.size() == incumbent.size() + 1) {
                        std::size_t at = 0;
                        while (at < incumbent.size() && path[at] == incumbent[at]) ++at;
                        single_block_split = at < incumbent.size() &&
                            p.blocks[path[at]].from == p.blocks[incumbent[at]].from &&
                            p.blocks[path[at + 1]].to == p.blocks[incumbent[at]].to &&
                            std::equal(incumbent.begin() + at + 1, incumbent.end(),
                                       path.begin() + at + 2);
                    }
                    if (count == 1 || single_block_split)
                        coverage_paths.insert(path);
                    auto &bucket = best[2 * (count - 1) + supported];
                    bucket.emplace(pathUnit(s, k, path), path);
                    if (options.paths_per_group > 0 && (int)bucket.size() > options.paths_per_group)
                        bucket.erase(std::prev(bucket.end()));
                };
                std::vector<int> closed;
                for (int b : p.compatibleBlocks(k))
                    if (s.route[b] < 0) {
                        ensureRoutes(b);
                        if (!pool[b].empty())
                            closed.push_back(b);
                    }
                std::function<void(int, int)> extend = [&](int at, int count) {
                    budget.check();
                    if (at == d.to) {
                        if (count)
                            retain(count);
                        return;
                    }
                    if (count && reachable[index(at, d.to)]) {
                        auto mark = path.size();
                        if (append(open_path[index(at, d.to)], true))
                            retain(count);
                        undo(mark);
                    }
                    if (count == options.max_new_blocks)
                        return;
                    for (int b : closed) {
                        int connector = index(at, p.blocks[b].from);
                        if (!reachable[connector])
                            continue;
                        auto mark = path.size();
                        if (append(open_path[connector], false) && append(std::vector<int>{b}, true))
                            extend(p.blocks[b].to, count + 1);
                        undo(mark);
                    }
                };
                extend(d.from, 0);
                std::vector<std::vector<int>> paths;
                for (const auto &bucket : best)
                    for (const auto &entry : bucket)
                        paths.push_back(entry.second);
                for (const auto &path : coverage_paths)
                    if (std::find(paths.begin(), paths.end(), path) == paths.end())
                        paths.push_back(path);
                return paths;
            }

            std::vector<int> pricingResources(const State &s, const std::vector<int> &path) const {
                std::vector<int> resources;
                const int B = p.blocks.size(), N = p.nodes.size();
                for (int b : path) {
                    resources.push_back(b);
                    resources.push_back(B + p.blocks[b].from);
                    if (pool[b].empty()) continue;
                    int r = s.route[b];
                    if (r < 0)
                        r = int(std::min_element(pool[b].begin(), pool[b].end(),
                                                [](const Route &a, const Route &c) { return a.unit < c.unit; })
                                - pool[b].begin());
                    for (auto [e, n] : pool[b][r].links) resources.push_back(B + N + e);
                }
                std::sort(resources.begin(), resources.end());
                resources.erase(std::unique(resources.begin(), resources.end()), resources.end());
                return resources;
            }

            int repairIntegralityFocus(Model &model, const char *stage) {
                int value = -1;
                check(model.solver.setEnvIntegralityFocus(1));
                check(model.solver.getEnvIntegralityFocus(&value));
                if (value != 1) throw std::runtime_error("Heuristic MIP IntegralityFocus readback mismatch");
                std::clog << "HEURISTIC_MIP_PARAMETERS stage=" << stage << " IntegralityFocus=" << value << '\n';
                return value;
            }

            template<class Details>
            std::optional<double> validateRepairCandidate(Plan &plan, const char *stage, Details details) {
                try {
                    const double raw_cost = plan.validate(p);
                    bool changed = false;
                    for (int k = 0; k < int(p.demands.size()); ++k) {
                        const double value = std::clamp(plan.served[k], 0., p.demands[k].quantity);
                        if (value == plan.served[k]) continue;
                        std::clog << "MIP_QUANTITY_BOUND_NORMALIZATION stage=" << stage << " demand=" << k
                                  << " raw=" << std::setprecision(17) << plan.served[k]
                                  << " normalized=" << value << " quantity=" << p.demands[k].quantity << '\n';
                        plan.served[k] = value;
                        changed = true;
                    }
                    if (!changed) return raw_cost;
                    try {
                        return plan.validate(p);
                    } catch (const std::exception &error) {
                        ++rejected_candidate_checks;
                        std::clog << "MIP_CANDIDATE_REJECTED stage=" << stage
                                  << " reason=quantity_normalization error=" << error.what()
                                  << " checks=" << rejected_candidate_checks << '\n';
                        return std::nullopt;
                    }
                } catch (const std::exception &error) {
                    std::clog << "MIP_CANDIDATE_VALIDATION_FAILURE stage=" << stage
                              << " error=" << error.what() << '\n';
                    if (std::string(error.what()) != "Minimum block load") throw;
                    std::vector<double> loads(p.blocks.size());
                    for (int k = 0; k < int(p.demands.size()); ++k)
                        for (int b : plan.logical[k]) loads[b] += plan.served[k];
                    bool near_numeric = true, has_violation = false;
                    for (int b = 0; b < int(p.blocks.size()); ++b)
                        if (!plan.physical[b].empty() && loads[b] + 1e-6 < p.blocks[b].lower) {
                            has_violation = true;
                            near_numeric = near_numeric && p.blocks[b].lower - loads[b] <=
                                                               1e-6 * std::max(1., p.blocks[b].lower);
                            Json record = {{"stage", stage}, {"error", error.what()}, {"block", b},
                                           {"original_block", original_blocks.empty() ? b : original_blocks[b]},
                                           {"decoded_load", loads[b]}, {"lower", p.blocks[b].lower},
                                           {"deficit", p.blocks[b].lower - loads[b]}, {"decoded_demands", Json::array()}};
                            for (int k = 0; k < int(p.demands.size()); ++k)
                                if (std::find(plan.logical[k].begin(), plan.logical[k].end(), b) != plan.logical[k].end())
                                    record["decoded_demands"].push_back({{"k", k}, {"served", plan.served[k]},
                                                                         {"quantity", p.demands[k].quantity}, {"path", plan.logical[k]}});
                            details(b, record);
                            std::clog << "MIP_MINIMUM_LOAD_DIAGNOSTIC " << record.dump() << '\n';
                        }
                    if (has_violation && near_numeric) {
                        ++rejected_candidate_checks;
                        std::clog << "MIP_CANDIDATE_REJECTED stage=" << stage
                                  << " reason=near_numerical_minimum_load checks=" << rejected_candidate_checks << '\n';
                        return std::nullopt;
                    }
                    throw;
                }
            }

            void solvePathModel(bool integer_finish = false, const PathPool *frozen_paths = nullptr,
                                PoolLPInfo *lp_info = nullptr) {
                if ((!integer_finish && !options.benchmark_path_generation) ||
                    budget.remaining() < options.min_pool_seconds)
                    return;
                const double begin = budget.elapsed();
                Budget local;
                // The LP uses the case's remaining global budget, never a short pricing cap.
                local.seconds =
                    integer_finish ? std::max(0., budget.remaining() - options.final_return_reserve)
                                   : local.elapsed() + std::max(0., budget.remaining());
                const int K = p.demands.size(), B = p.blocks.size(), N = p.nodes.size(),
                          E = p.capacities.size();
                try {
                    State s(p);
                    s.plan = out.plan;
                    for (int b = 0; b < B; ++b)
                        if (!s.plan.physical[b].empty())
                            for (int r = 0; r < (int)pool[b].size(); ++r)
                                if (pool[b][r].arcs == s.plan.physical[b])
                                    s.route[b] = r;
                    rebuild(s);
                    if (!integer_finish && !frozen_paths && !lp_info)
                        generateOpenAlternatives(s);
                    PathPool full_paths, reduced_paths;
                    const PathPool *effective_paths = frozen_paths;
                    std::vector<char> fixed(K);
                    std::vector<char> fixed_assignment(K);
                    if (integer_finish && (options.pool_fix_fraction > 0 || options.pool_extra_paths > 0)) {
                        full_paths.resize(K);
                        for (int k = 0; k < K; ++k)
                            full_paths[k] = frozen_paths ? frozen_paths->at(k) : jointPaths(s, k, false);
                        PoolLPInfo information;
                        const double lp_begin = budget.elapsed();
                        solvePathModel(false, &full_paths, &information);
                        information.seconds = budget.elapsed() - lp_begin;
                        effective_paths = &full_paths;
                        if (information.optimal) {
                            const double reduction_begin = budget.elapsed();
                            auto reduction = reducePool(p, out.plan, full_paths, information, options);
                            reduced_paths = std::move(reduction.logical);
                            fixed = std::move(reduction.fixed);
                            effective_paths = &reduced_paths;
                            pool_reduction_audit = std::move(reduction.audit);
                            pool_reduction_audit["applied"] = true;
                            pool_reduction_audit["lp_seconds"] = information.seconds;
                            pool_reduction_audit["lp_status"] = guidance_status;
                            pool_reduction_audit["selection_seconds"] = budget.elapsed() - reduction_begin;
                            if (options.record_candidate_pool)
                                pool_reduction_audit["lp_solution"] = {
                                    {"path_weights", information.selection},
                                    {"volumes", information.volume},
                                    {"jrc", information.jrc}};
                        } else
                            pool_reduction_audit = {{"applied", false},
                                                    {"lp_seconds", information.seconds},
                                                    {"lp_status", guidance_status}};
                    }
                    const double model_begin = budget.elapsed();
                    if (integer_finish && (options.pool_fix_fraction > 0 || options.pool_extra_paths > 0)) {
                        local.start = std::chrono::steady_clock::now();
                        local.seconds = std::max(0., budget.remaining() - options.final_return_reserve);
                    }
                    PathPool paths(K);
                    for (int k = 0; k < K; ++k)
                        paths[k] = effective_paths ? effective_paths->at(k)
                                   : (integer_finish && options.benchmark_pool_only_joint ? poolPaths(s, k)
                                                                                       : jointPaths(s, k, false));
                    const PathPool known_paths = paths;
                    if (!integer_finish && !lp_info && options.guidance_logical_paths_per_demand > 0)
                        paths = selectGuidancePaths(known_paths, s.plan,
                                                   options.guidance_logical_paths_per_demand,
                                                   [&](int k, const auto &path) { return pathUnit(s, k, path, false); });
                    if (integer_finish) {
                        Json selection;
                        fixed_assignment = selectRepairDemands(
                            p, s.plan, paths, options.joint_repair_max_demands,
                            [&](int k, const auto &path) {
                                budget.check();
                                local.check();
                                return pathUnit(s, k, path, false);
                            }, selection);
                        pool_reduction_audit["demand_cap"] = std::move(selection);
                        for (int k = 0; k < K; ++k) fixed[k] = fixed[k] || fixed_assignment[k];
                        if (std::all_of(fixed_assignment.begin(), fixed_assignment.end(), [](char x) { return x; })) {
                            pool_reduction_audit["demand_cap"]["skipped_no_admissible_group"] = true;
                            return;
                        }
                    }
                    const bool conditional = std::any_of(fixed_assignment.begin(), fixed_assignment.end(),
                                                         [](char x) { return x; });
                    std::vector<char> active(B, !conditional);
                    if (conditional)
                        for (int k = 0; k < K; ++k) if (!fixed_assignment[k]) {
                            for (const auto &path : paths[k]) for (int b : path) active[b] = true;
                        }
                    std::vector<int> arc_seeds;
                    Json arc_guard = {{"enabled", false}, {"limit", options.joint_arc_max_model_size}};
                    if (integer_finish && options.joint_arc_max_model_size > 0) {
                        auto expanded_active = active;
                        std::vector<int> seen_blocks(B, -1), eligible;
                        std::size_t logical_count = 0, pair_count = 0, free_count = 0;
                        std::size_t arc_variables = 0, arc_rows = 1;
                        const std::size_t Y = yards.size();
                        for (int k = 0; k < K; ++k) if (!fixed_assignment[k]) {
                            local.check();
                            ++free_count;
                            logical_count += paths[k].size();
                            auto mark = [&](int b) {
                                if (seen_blocks[b] != k) { seen_blocks[b] = k; ++pair_count; }
                            };
                            for (const auto &path : paths[k]) for (int b : path) mark(b);
                            if (!fixed[k]) {
                                eligible.push_back(k);
                                arc_variables += 2 + Y;
                                arc_rows += 1 + 4 * Y;
                                for (int b : p.compatibleBlocks(k)) if (!pool[b].empty()) {
                                    expanded_active[b] = true;
                                    mark(b);
                                    arc_variables += 2;
                                    arc_rows += 4;
                                }
                            }
                        }
                        std::size_t physical_count = 0, active_count = 0;
                        for (int b = 0; b < B; ++b) if (expanded_active[b]) {
                            ++active_count;
                            physical_count += pool[b].size();
                        }
                        const std::size_t variables = 2 * (physical_count + logical_count) + arc_variables;
                        // Three rows per active block cover balance, selection and any fixed outside load.
                        // All resource rows are counted before empty/duplicate rows are removed.
                        const std::size_t rows = 2 * physical_count + logical_count + 3 * active_count +
                            free_count + pair_count + 2 * Y + E + arc_rows;
                        const bool enabled = !eligible.empty() &&
                            variables + rows <= std::size_t(options.joint_arc_max_model_size);
                        arc_guard.update({{"estimated_variables", variables}, {"estimated_rows_upper", rows},
                                          {"estimated_total_upper", variables + rows}, {"enabled", enabled}});
                        if (enabled) {
                            arc_seeds = std::move(eligible);
                            active = std::move(expanded_active);
                        }
                    }
                    if (integer_finish)
                        std::clog << "JOINT_ARC_SIZE " << arc_guard.dump() << '\n';
                    CostSum constant;
                    constant += p.rejection;
                    const bool resource_audit = conditional || (integer_finish && options.record_candidate_pool);
                    std::vector<double> outside(B), fixed_handling(resource_audit ? N : 0),
                        fixed_tracks(resource_audit ? N : 0), fixed_links(resource_audit ? E : 0);
                    if (conditional) {
                        for (int k = 0; k < K; ++k) if (fixed_assignment[k]) {
                            const double v = s.plan.served[k];
                            constant += -v * p.demands[k].penalty;
                            for (int b : s.plan.logical[k]) {
                                outside[b] += v;
                                const int i = p.blocks[b].from;
                                if (i != p.demands[k].from) {
                                    fixed_handling[i] += v;
                                    constant += v * p.nodes[i].handling_cost;
                                }
                            }
                        }
                        for (int b = 0; b < B; ++b) if (!active[b] && s.route[b] >= 0) {
                            const auto &route = pool[b][s.route[b]];
                            constant += p.blocks[b].opening + s.load[b] * route.unit;
                            fixed_tracks[p.blocks[b].from] += p.blocks[b].slots;
                            for (auto [e, n] : route.links) fixed_links[e] += n * s.load[b];
                        }
                    }
                    if (integer_finish) {
                        fixed_assignment_audit = {{"conditional", conditional},
                            {"fixed_demands", fixed_assignment}, {"active_blocks", active},
                            {"objective_constant", constant.value}};
                        if (options.record_candidate_pool) {
                            fixed_assignment_audit["outside_load"] = outside;
                            fixed_assignment_audit["fixed_handling"] = fixed_handling;
                            fixed_assignment_audit["fixed_tracks"] = fixed_tracks;
                            fixed_assignment_audit["fixed_links"] = fixed_links;
                        }
                    }
                    std::string pool_key;
                    if (!frozen_paths && !lp_info) {
                        Json signature = {{"generation", incumbent_generation}, {"logical", paths},
                                          {"fixed", fixed}, {"fixed_assignments", fixed_assignment},
                                          {"physical", Json::array()}};
                        if (!integer_finish) signature["full_logical"] = known_paths;
                        if (!arc_seeds.empty()) signature["implicit_logical_demands"] = arc_seeds;
                        // Route values are immutable. A pending seed may still finish its initial stable sort.
                        // Existing sizes and seeded flags capture append/sort changes without serializing arcs.
                        for (int b = 0; b < B; ++b)
                            signature["physical"].push_back({pool[b].size(), bool(seeded[b])});
                        pool_key = signature.dump();
                        if (integer_finish && pool_key == exhausted_pool_key) {
                            ++pool_repeat_skips;
                            std::clog << "PATH_MIP_SKIP reason=same_exhausted_pool\n";
                            return;
                        }
                        if (!integer_finish && pool_key == priced_pool_key) {
                            ++lp_repeat_skips;
                            std::clog << "PATH_LP_SKIP reason=unchanged_priced_pool\n";
                            return;
                        }
                    }
                    Model model(options.solver_threads, "IRBP-joint-path-model");
                    model.constant(constant.value);
                    struct Column {
                        int owner, path, x, v;
                    };
                    struct ArcDemand {
                        int owner, choice, volume;
                        std::vector<int> edges, flows;
                    };
                    std::vector<Column> physical, logical;
                    std::vector<ArcDemand> arc_demands;
                    std::vector<Sparse> balance(B), open(B), demand(K), handling(N), slots(N), links(E);
                    std::vector<std::map<int, Sparse>> activation(K);
                    for (int b = 0; b < B; ++b) {
                        local.check();
                        if (!active[b]) continue;
                        for (int r = 0; r < (int)pool[b].size(); ++r) {
                            const auto &path = pool[b][r];
                            int x = model.var(p.blocks[b].opening), v = model.var(path.unit);
                            Sparse lower, upper;
                            lower.add(v, 1);
                            lower.add(x, -p.blocks[b].lower);
                            upper.add(v, 1);
                            upper.add(x, -path.capacity);
                            model.row(lower, '>', 0);
                            model.row(upper, '<', 0);
                            physical.push_back({b, r, x, v});
                            balance[b].add(v, 1);
                            open[b].add(x, 1);
                            slots[p.blocks[b].from].add(x, p.blocks[b].slots);
                            for (auto [e, n] : path.links)
                                links[e].add(v, n);
                        }
                    }
                    for (int k = 0; k < K; ++k) {
                        local.check();
                        if (fixed_assignment[k]) continue;
                        for (int r = 0; r < (int)paths[k].size(); ++r) {
                            double unit = -p.demands[k].penalty;
                            for (int b : paths[k][r])
                                if (p.blocks[b].from != p.demands[k].from)
                                    unit += p.nodes[p.blocks[b].from].handling_cost;
                            int x = model.var(), v = model.var(unit);
                            Sparse upper;
                            upper.add(v, 1);
                            upper.add(x, -p.demands[k].quantity);
                            model.row(upper, '<', 0);
                            logical.push_back({k, r, x, v});
                            demand[k].add(x, 1);
                            for (int b : paths[k][r]) {
                                balance[b].add(v, -1);
                                activation[k][b].add(x, 1);
                                if (p.blocks[b].from != p.demands[k].from)
                                    handling[p.blocks[b].from].add(v, 1);
                            }
                        }
                    }
                    if (!arc_seeds.empty()) {
                        Sparse implicit_paths;
                        const double Y = yards.size();
                        for (int k : arc_seeds) {
                            local.check();
                            const auto &d = p.demands[k];
                            arc_demands.push_back({k, model.var(0, 1), model.var(-d.penalty, d.quantity),
                                                   std::vector<int>(B, -1), std::vector<int>(B, -1)});
                            auto &arc = arc_demands.back();
                            implicit_paths.add(arc.choice, 1);
                            demand[k].add(arc.choice, 1);
                            Sparse volume;
                            volume.add(arc.volume, 1);
                            volume.add(arc.choice, -d.quantity);
                            model.row(volume, '<', 0);
                            std::vector<int> order(N, -1);
                            std::vector<Sparse> path_balance(N), flow_balance(N), incoming(N), outgoing(N);
                            for (int i : yards)
                                order[i] = model.var(0, Y - 1);
                            for (int b : p.compatibleBlocks(k)) {
                                if (pool[b].empty())
                                    continue;
                                const auto &block = p.blocks[b];
                                const int x = model.var(0, 1);
                                const int g = model.var(
                                    block.from == d.from ? 0 : p.nodes[block.from].handling_cost, d.quantity);
                                arc.edges[b] = x;
                                arc.flows[b] = g;
                                activation[k][b].add(x, 1);
                                balance[b].add(g, -1);
                                if (block.from != d.from)
                                    handling[block.from].add(g, 1);
                                path_balance[block.from].add(x, 1);
                                path_balance[block.to].add(x, -1);
                                flow_balance[block.from].add(g, 1);
                                flow_balance[block.to].add(g, -1);
                                outgoing[block.from].add(x, 1);
                                incoming[block.to].add(x, 1);
                                Sparse upper, flow_upper, flow_lower, elementary;
                                upper.add(g, 1);
                                upper.add(x, -d.quantity);
                                model.row(upper, '<', 0);
                                flow_upper.add(g, 1);
                                flow_upper.add(arc.volume, -1);
                                model.row(flow_upper, '<', 0);
                                flow_lower.add(g, 1);
                                flow_lower.add(arc.volume, -1);
                                flow_lower.add(arc.choice, d.quantity);
                                flow_lower.add(x, -d.quantity);
                                model.row(flow_lower, '>', 0);
                                elementary.add(order[block.to], 1);
                                elementary.add(order[block.from], -1);
                                elementary.add(x, -Y);
                                model.row(elementary, '>', 1 - Y);
                            }
                            for (int i : yards) {
                                const int delta = (i == d.from) - (i == d.to);
                                path_balance[i].add(arc.choice, -delta);
                                flow_balance[i].add(arc.volume, -delta);
                                incoming[i].add(arc.choice, -1);
                                outgoing[i].add(arc.choice, -1);
                                model.row(path_balance[i], '=', 0);
                                model.row(flow_balance[i], '=', 0);
                                model.row(incoming[i], '<', 0);
                                model.row(outgoing[i], '<', 0);
                            }
                        }
                        model.row(implicit_paths, '<', 1);
                    }
                    std::vector<int> br(B, -1), yr(B, -1), kr(K, -1), hr(N, -1), tr(N, -1), er(E);
                    std::vector<std::map<int, int>> gr(K);
                    for (int b = 0; b < B; ++b) if (active[b]) {
                        br[b] = model.row(balance[b], '=', outside[b]);
                        yr[b] = model.row(open[b], '<', 1);
                        if (outside[b] > Tol) model.row(open[b], '>', 1);
                    }
                    for (int k = 0; k < K; ++k) {
                        if (fixed_assignment[k]) continue;
                        kr[k] = model.row(demand[k], '<', 1);
                        // An unused pair has only -sum(y_b) <= 0, implied by y_b >= 0.
                        // Its omitted row can be extended with zero dual; block balance stays explicit.
                        for (const auto &[b, expression] : activation[k]) {
                            auto row = expression;
                            for (int x : open[b].ind)
                                row.add(x, -1);
                            gr[k][b] = model.row(row, '<', 0);
                        }
                    }
                    for (int i : yards) {
                        if (!conditional || !handling[i].ind.empty())
                            hr[i] = model.row(handling[i], '<', std::max(0., p.nodes[i].handling_capacity - (conditional ? fixed_handling[i] : 0.)));
                        if (!conditional || !slots[i].ind.empty())
                            tr[i] = model.row(slots[i], '<', std::max(0., p.nodes[i].tracks - (conditional ? fixed_tracks[i] : 0.)));
                    }
                    Json capacity_rows;
                    er = addLinkCapacityRows(
                        model, links, [&](int e) { return std::max(0., p.capacities[e] - (conditional ? fixed_links[e] : 0.)); }, capacity_rows);
                    capacity_rows["stage"] = integer_finish ? "pool_mip" : "pool_lp";
                    capacity_rows["model_columns"] = model.nvars;
                    capacity_rows["model_rows"] = model.nrows;
                    capacity_rows["logical_paths"] = logical.size();
                    capacity_rows["physical_paths"] = physical.size();
                    capacity_rows["logical_paths_before"] = 0;
                    for (const auto &candidates : known_paths)
                        capacity_rows["logical_paths_before"] =
                            capacity_rows["logical_paths_before"].get<std::size_t>() + candidates.size();
                    capacity_row_history.push_back(capacity_rows);
                    if (integer_finish) {
                        if (!arc_seeds.empty() &&
                            (std::size_t(model.nvars) != arc_guard["estimated_variables"].get<std::size_t>() ||
                             std::size_t(model.nrows) > arc_guard["estimated_rows_upper"].get<std::size_t>()))
                            throw std::runtime_error("Joint ARC size estimate does not bound the constructed model");
                        check(model.solver.updateModel());
                        std::vector<char> types(model.nvars, SOLVER_CONTINUOUS);
                        std::vector<double> start(model.nvars), loads(B);
                        for (int k = 0; k < K; ++k)
                            for (int b : out.plan.logical[k])
                                loads[b] += out.plan.served[k];
                        for (const auto &c : logical) {
                            types[c.x] = SOLVER_BINARY;
                            check(model.solver.setColUpper(c.x, 1));
                            if (paths[c.owner][c.path] == out.plan.logical[c.owner]) {
                                start[c.x] = 1;
                                start[c.v] = out.plan.served[c.owner];
                            }
                        }
                        for (const auto &arc : arc_demands) {
                            types[arc.choice] = SOLVER_BINARY;
                            for (int x : arc.edges)
                                if (x >= 0)
                                    types[x] = SOLVER_BINARY;
                        }
                        for (const auto &c : physical) {
                            types[c.x] = SOLVER_BINARY;
                            check(model.solver.setColUpper(c.x, 1));
                            if (pool[c.owner][c.path].arcs == out.plan.physical[c.owner]) {
                                start[c.x] = 1;
                                start[c.v] = loads[c.owner];
                            }
                        }
                        check(model.solver.setVTypeArray(0, model.nvars, types.data()));
                        check(model.solver.setMIPStart(0, model.nvars, start.data()));
                        check(model.solver.setEnvCutoff(out.objective));
                        check(model.solver.setEnvMIPFocus(options.final_mip_focus));
                        pool_mip_build_seconds += budget.elapsed() - model_begin;
                        if (budget.remaining() <= options.final_return_reserve + options.solver_start_reserve)
                            return;
                        check(model.solver.setEnvTimeLimit(
                            std::min(options.pool_mip_max_seconds,
                                     budget.remaining() - options.final_return_reserve)));
                        ++pool_mip_calls;
                        std::clog << "PATH_MIP iteration=" << pool_mip_calls << " columns=" << model.nvars
                                  << " rows=" << model.nrows << " cutoff=" << out.objective << '\n';
                        joint_pool_logical_paths = logical.size();
                        joint_pool_physical_paths = physical.size();
                        if (options.record_candidate_pool) {
                            candidate_pool = {{"scope", "Last joint MIP candidate pool"},
                                              {"iteration", pool_mip_calls},
                                              {"incumbent_objective", out.objective},
                                              {"incumbent", out.plan.json()},
                                              {"implicit_logical_demands", arc_seeds},
                                              {"implicit_logical_limit", arc_seeds.empty() ? 0 : 1},
                                              {"logical_paths", paths},
                                              {"physical_paths", Json::array()}};
                            for (int b = 0; b < B; ++b) {
                                Json arcs = Json::array();
                                for (const auto &route : pool[b])
                                    arcs.push_back(route.arcs);
                                candidate_pool["physical_paths"].push_back(std::move(arcs));
                            }
                        }
                        path_model_columns = model.nvars;
                        path_model_rows = model.nrows;
                        pool_model_history.push_back({{"cutoff", out.objective},
                                                      {"free_demands", K - std::count(fixed_assignment.begin(), fixed_assignment.end(), true)},
                                                      {"columns", model.nvars},
                                                      {"rows", model.nrows},
                                                      {"logical_paths", logical.size()},
                                                      {"physical_paths", physical.size()},
                                                      {"implicit_demands", arc_seeds},
                                                      {"implicit_selected", Json::array()},
                                                      {"arc_size_guard", arc_guard}});
                        const bool component_arc_on = arc_guard.is_object() && arc_guard.value("enabled", false);
                        if (component_arc_on) ++component_joint_arc_enabled; else ++component_joint_arc_disabled;
                        std::clog << "COMPONENT_JOINT_DOMAIN " << Json{{"arc_enabled", component_arc_on}, {"guard", arc_guard}, {"free_demands", K - std::count(fixed_assignment.begin(), fixed_assignment.end(), true)}, {"implicit_demands", arc_seeds}}.dump() << std::endl;
                        const auto rejects_before = rejected_candidate_checks;
                        auto accept_solution = [&](double objective, const std::vector<double> &values) {
                            Plan plan = conditional ? s.plan : Plan::empty(p);
                            if (conditional) {
                                for (int k = 0; k < K; ++k) if (!fixed_assignment[k]) {
                                    plan.logical[k].clear(); plan.served[k] = 0;
                                }
                                for (int b = 0; b < B; ++b) if (active[b]) plan.physical[b].clear();
                            }
                            for (const auto &c : logical)
                                if (values[c.x] > .5 && values[c.v] > Tol) {
                                    plan.logical[c.owner] = paths[c.owner][c.path];
                                    plan.served[c.owner] = values[c.v];
                                }
                            for (const auto &c : physical)
                                if (values[c.x] > .5)
                                    plan.physical[c.owner] = pool[c.owner][c.path].arcs;
                            for (const auto &arc : arc_demands)
                                if (values[arc.choice] > .5 && values[arc.volume] > Tol) {
                                    const auto &d = p.demands[arc.owner];
                                    auto &path = plan.logical[arc.owner];
                                    path.clear();
                                    int at = d.from;
                                    for (std::size_t step = 0; step < yards.size() && at != d.to; ++step) {
                                        int next = -1;
                                        for (int b : p.compatibleBlocks(arc.owner))
                                            if (p.blocks[b].from == at && arc.edges[b] >= 0 &&
                                                values[arc.edges[b]] > .5) {
                                                next = b;
                                                break;
                                            }
                                        if (next < 0)
                                            throw std::runtime_error("Incomplete implicit logical path");
                                        path.push_back(next);
                                        at = p.blocks[next].to;
                                    }
                                    if (at != d.to)
                                        throw std::runtime_error("Cyclic implicit logical path");
                                    plan.served[arc.owner] = values[arc.volume];
                                }
                            const auto validated = validateRepairCandidate(plan, "joint", [&](int b, Json &record) {
                                CostSum raw_physical, raw_logical;
                                record["outside_load"] = outside[b];
                                record["physical_columns"] = record["logical_columns"] = Json::array();
                                for (const auto &c : physical) if (c.owner == b) {
                                    raw_physical += values[c.v];
                                    record["physical_columns"].push_back({{"path_index", c.path}, {"x", values[c.x]},
                                        {"v", values[c.v]}, {"min_row_residual", values[c.v] - p.blocks[b].lower * values[c.x]}});
                                }
                                for (const auto &c : logical)
                                    if (std::find(paths[c.owner][c.path].begin(), paths[c.owner][c.path].end(), b) != paths[c.owner][c.path].end()) {
                                        raw_logical += values[c.v];
                                        if (values[c.v] != 0.) record["logical_columns"].push_back({{"k", c.owner},
                                            {"x", values[c.x]}, {"v", values[c.v]}, {"path", paths[c.owner][c.path]},
                                            {"volume_link_slack", p.demands[c.owner].quantity * values[c.x] - values[c.v]}});
                                    }
                                for (const auto &c : arc_demands) if (c.flows[b] >= 0) {
                                    raw_logical += values[c.flows[b]];
                                    if (values[c.flows[b]] != 0.) record["logical_columns"].push_back({{"k", c.owner},
                                        {"implicit", true}, {"x", values[c.choice]}, {"v", values[c.volume]}, {"block_flow", values[c.flows[b]]}});
                                }
                                record["raw_physical_volume"] = raw_physical.value;
                                record["raw_free_logical_volume"] = raw_logical.value;
                                record["balance_residual"] = raw_physical.value - outside[b] - raw_logical.value;
                            });
                            if (!validated) return;
                            const double cost = *validated;
                            // Replay the decoded feasible plan, including exact block-load balance.
                            std::vector<double> replay(model.nvars), loads(B);
                            for (int k = 0; k < K; ++k)
                                for (int b : plan.logical[k]) loads[b] += plan.served[k];
                            for (const auto &c : logical)
                                if (values[c.x] > .5 && values[c.v] > Tol) {
                                    replay[c.x] = 1;
                                    replay[c.v] = plan.served[c.owner];
                                }
                            for (const auto &c : physical)
                                if (values[c.x] > .5) {
                                    replay[c.x] = 1;
                                    replay[c.v] = loads[c.owner];
                                }
                            for (const auto &arc : arc_demands)
                                if (values[arc.choice] > .5 && values[arc.volume] > Tol) {
                                    replay[arc.choice] = 1;
                                    replay[arc.volume] = plan.served[arc.owner];
                                    for (int b : plan.logical[arc.owner]) {
                                        replay[arc.edges[b]] = 1;
                                        replay[arc.flows[b]] = plan.served[arc.owner];
                                    }
                                }
                            const double decoded_objective = model.objective(replay);
                            if (!objectiveEqual(cost, decoded_objective))
                                throw std::runtime_error("Pool decoded-model objective mismatch: validated=" +
                                                         std::to_string(cost) + " model=" +
                                                         std::to_string(decoded_objective) + " reported=" +
                                                         std::to_string(objective));
                            if (!objectiveEqual(cost, objective))
                                std::clog << "POOL_OBJECTIVE_REPLAY reported=" << std::setprecision(17)
                                          << objective << " decoded_model=" << decoded_objective
                                          << " validated=" << cost << '\n';
                            if (cost < out.objective - Tol) {
                                auto &selected_arc = pool_model_history.back()["implicit_selected"];
                                selected_arc = Json::array();
                                for (const auto &arc : arc_demands)
                                    if (values[arc.choice] > .5 && values[arc.volume] > Tol)
                                        selected_arc.push_back(arc.owner);
                                for (const auto &arc : arc_demands)
                                    if (values[arc.choice] > .5 && values[arc.volume] > Tol &&
                                        std::find(guided[arc.owner].begin(), guided[arc.owner].end(),
                                                  plan.logical[arc.owner]) == guided[arc.owner].end())
                                        guided[arc.owner].push_back(plan.logical[arc.owner]);
                                out.plan = std::move(plan);
                                out.objective = cost;
                                out.history.push_back({{"seconds", budget.elapsed()},
                                                       {"objective", cost},
                                                       {"stage", "pool_mip"}});
                                notify();
                            }
                        };
                        const double objective_before_solve = out.objective;
                        check(model.solver.setSolutionCallback(model.nvars, out.objective, accept_solution, &out.objective));
                        const double solve_begin = budget.elapsed();
                        joint_integrality_focus = repairIntegralityFocus(model, "joint");
                        check(model.solver.mipOptimize());
                        check(model.solver.getStatus(&pool_mip_status));
                        int count;
                        check(model.solver.getSolCount(&count));
                        if (count) accept_solution(model.objective(), model.values());
                        auto &round = pool_model_history.back();
                        round["first_status"] = pool_mip_status;
                        round["first_solver_objective"] = count ? Json(model.objective()) : Json(nullptr);
                        round["first_validated_objective"] = out.objective;
                        round["first_application_rejected_candidate_checks"] = rejected_candidate_checks - rejects_before;
                        round["continued"] = false;
                        if (pool_mip_status == SOLVER_TIME_LIMIT &&
                            out.objective >= objective_before_solve -
                                                 objectiveTolerance(out.objective, objective_before_solve) &&
                            budget.remaining() >
                                options.final_return_reserve + options.solver_start_reserve) {
                            // Solver objective roundoff is not a validated improvement.
                            ++pool_mip_continuations;
                            round["continued"] = true;
                            std::clog << "PATH_MIP_CONTINUE iteration=" << pool_mip_calls
                                      << " reason=no_validated_improvement"
                                      << " solver_objective=" << round["first_solver_objective"]
                                      << " validated_objective=" << out.objective << '\n';
                            check(model.solver.setEnvTimeLimit(budget.remaining() -
                                                               options.final_return_reserve));
                            check(model.solver.mipOptimize());
                            check(model.solver.getStatus(&pool_mip_status));
                            check(model.solver.getSolCount(&count));
                        }
                        pool_mip_solver_seconds += budget.elapsed() - solve_begin;
                        check(model.solver.getObjBound(&pool_mip_bound));
                        check(model.solver.getNodeCount(&pool_mip_nodes));
                        if (count) accept_solution(model.objective(), model.values());
                        round["application_rejected_candidate_checks"] = rejected_candidate_checks - rejects_before;
                        std::clog << "PATH_MIP_DONE iteration=" << pool_mip_calls
                                  << " status=" << pool_mip_status
                                  << " solve_seconds=" << budget.elapsed() - solve_begin
                                  << " best=" << out.objective << " conditional_bound=" << pool_mip_bound
                                  << " nodes=" << pool_mip_nodes
                                  << " application_rejected_checks=" << rejected_candidate_checks - rejects_before << '\n';
                        if ((pool_mip_status == SOLVER_OPTIMAL || pool_mip_status == SOLVER_CUTOFF) &&
                            rejected_candidate_checks == rejects_before)
                            exhausted_pool_key = std::move(pool_key);
                        pool_mip_seconds += budget.elapsed() - begin;
                        return;
                    }
                    auto generate = [&] {
                        guidance_build_seconds += budget.elapsed() - model_begin;
                        if (options.guidance_lp_method == SOLVER_BARRIER) {
                            check(model.solver.setEnvCrossOver(options.guidance_crossover ? -1 : 0));
                            check(model.solver.setEnvBarConvTol(options.guidance_barrier_tolerance));
                        }
                        check(model.solver.setEnvOutputFlag(options.guidance_solver_log ? 1 : 0, false));
                        int local_lp_calls = 0;
                        auto solve_lp = [&](bool terminal) {
                            budget.check();
                            local.seconds = local.elapsed() + budget.remaining();
                            const int method = local_lp_calls++ ? SOLVER_DUAL_SIMPLEX : options.guidance_lp_method;
                            std::clog << "PATH_LP columns=" << model.nvars << " rows=" << model.nrows
                                      << " terminal=" << terminal << " method=" << method << '\n';
                            ++guidance_lp_calls;
                            const double lp_start = budget.elapsed();
                            guidance_status = model.optimize(local, false, method);
                            guidance_lp_seconds += budget.elapsed() - lp_start;
                            std::clog << "PATH_LP_DONE status=" << guidance_status
                                      << " seconds=" << budget.elapsed() - lp_start << '\n';
                            if (guidance_status != SOLVER_OPTIMAL) return false;
                            pool_objective = model.objective();
                            return true;
                        };
                        if (!lp_info)
                            path_generation_history.push_back({{"stop", "initial_lp_failure"}, {"batches", 0},
                                {"initial_logical_paths", logical.size()}, {"physical_paths", physical.size()},
                                {"returned", 0}, {"terminal_optimal", false},
                                {"heuristic_pricing", options.guidance_heuristic_pricing}});
                        if (!solve_lp(false)) return;
                        auto dual = model.dual();
                        auto activationDual = [&](int k, int b) {
                            auto it = gr[k].find(b);
                            return it == gr[k].end() ? 0. : dual[it->second];
                        };
                        auto jrc = [&](int k, const std::vector<int> &path) {
                            const auto &d = p.demands[k];
                            double score = -d.quantity * d.penalty - dual[kr[k]];
                            for (int b : path) {
                                const int i = p.blocks[b].from;
                                score += d.quantity * dual[br[b]] - activationDual(k, b) +
                                    (i == d.from ? 0 : d.quantity * (p.nodes[i].handling_cost - dual[hr[i]]));
                            }
                            return score;
                        };
                        if (lp_info) {
                            lp_info->optimal = true;
                            lp_info->objective = pool_objective;
                            lp_info->selection.resize(K);
                            lp_info->volume.resize(K);
                            lp_info->jrc.resize(K);
                            const auto values = model.values();
                            for (int k = 0; k < K; ++k) {
                                lp_info->selection[k].resize(paths[k].size());
                                lp_info->volume[k].resize(paths[k].size());
                                lp_info->jrc[k].resize(paths[k].size());
                            }
                            for (const auto &column : logical) {
                                const int k = column.owner, r = column.path;
                                lp_info->selection[k][r] = values[column.x];
                                lp_info->volume[k][r] = values[column.v];
                                lp_info->jrc[k][r] = jrc(k, paths[k][r]);
                            }
                            return;
                        }
                        auto &audit = path_generation_history.back();
                        std::vector<double> objectives;
                        std::vector<std::vector<double>> solved_scores(K);
                        double solved_objective = 0, solved_violation = 0;
                        int batches = 0, solved_batches = 0;
                        std::vector<double> iteration_seconds;
                        auto save_solved = [&] {
                            std::vector<double> slack(model.nrows), rhs(model.nrows);
                            std::vector<char> sense(model.nrows);
                            check(model.solver.getSlack(0, model.nrows, slack.data()));
                            check(model.solver.getRhs(0, model.nrows, rhs.data()));
                            check(model.solver.getSense(0, model.nrows, sense.data()));
                            double violation = 0;
                            for (int i = 0; i < model.nrows; ++i) {
                                if (!std::isfinite(slack[i]) || !std::isfinite(rhs[i])) return false;
                                const double residual = sense[i] == '=' ? std::abs(slack[i]) :
                                    std::max(0., sense[i] == '<' ? -slack[i] : slack[i]);
                                violation = std::max(violation, residual / std::max(1., std::abs(rhs[i])));
                            }
                            if (violation > FeasTol) return false;
                            std::vector<std::vector<double>> scores(K);
                            for (int k = 0; k < K; ++k)
                                for (const auto &path : paths[k]) {
                                    const double score = jrc(k, path);
                                    if (!std::isfinite(score)) return false;
                                    scores[k].push_back(score);
                                }
                            solved_scores = std::move(scores);
                            solved_objective = pool_objective;
                            solved_violation = violation;
                            solved_batches = batches;
                            return true;
                        };
                        if (!save_solved()) { audit["stop"] = "initial_row_failure"; return; }
                        objectives.push_back(pool_objective);
                        std::string stop;
                        bool completed_no_new = false;
                        const double graph_begin = budget.elapsed();
                        std::vector<int> logical_nodes;
                        const Graph pricing_graph = compactLogicalGraph(p, logical_nodes);
                        const bool restricted_links = options.guidance_heuristic_pricing && options.guidance_neighbors > 0;
                        std::vector<double> reference_length;
                        if (restricted_links) {
                            reference_length.assign(B, std::numeric_limits<double>::infinity());
                            for (int b = 0; b < B; ++b)
                                for (const auto &route : pool[b]) {
                                    double length = 0;
                                    for (int a : route.arcs) length += p.arcs[a].length;
                                    reference_length[b] = std::min(reference_length[b], length);
                                }
                        }
                        std::vector<std::optional<std::vector<int>>> neighborhoods(restricted_links ? K : 0);
                        guidance_build_seconds += budget.elapsed() - graph_begin;
                        audit["original_logical_nodes"] = p.logical.size();
                        audit["compact_logical_nodes"] = pricing_graph.size();
                        using Candidate = std::tuple<double, int, std::vector<int>>;
                        struct Worker {
                            Labeling labels;
                            Stats stats;
                            std::vector<double> cost;
                            std::vector<char> allowed;
                            std::set<Candidate> candidates;
                            std::map<std::string, int> limits;
                            bool completed{true};
                            std::size_t peak_bytes{}, peak_labels{};
                            Worker(std::size_t bytes, int blocks) : labels(bytes), cost(blocks), allowed(blocks) {}
                        };
                        const int worker_count = std::max(1, std::min(options.solver_threads, K));
                        const double worker_begin = budget.elapsed();
                        std::vector<Worker> workers;
                        workers.reserve(worker_count);
                        for (int i = 0; i < worker_count; ++i)
                            workers.emplace_back(std::size_t(options.pricing_label_memory_mib) * 1024 * 1024, B);
                        PricingWorkers team(worker_count); // Existing exact dispatcher; worker0 participates.
                        guidance_build_seconds += budget.elapsed() - worker_begin;
                        audit["pricing_workers"] = worker_count;
                        audit["label_memory_MiB_per_worker"] = options.pricing_label_memory_mib;
                        Stats stats;
                        std::map<std::string, int> pricing_limits;
                        try {
                            while (stop.empty()) {
                                if (batches >= options.guidance_max_iterations) { stop = "iteration_limit"; break; }
                                if (options.guidancePlateau(objectives)) { stop = "lp_plateau"; break; }
                                const double iteration_begin = batches == 0 ? begin : budget.elapsed();
                                Budget pricing;
                                pricing.seconds = std::min(options.guidance_max_seconds,
                                                           options.guidance_budget_fraction * budget.remaining());
                                const double pricing_start = budget.elapsed();
                                const double tolerance = pricingTolerance(pool_objective);
                                std::set<Candidate> candidates;
                                bool completed = true;
                                for (auto &worker : workers) {
                                    worker.stats = {}; worker.candidates.clear(); worker.limits.clear(); worker.completed = true;
                                }
                                team.run(K, [&](int id, int k) {
                                    auto &worker = workers[id];
                                    auto &logical_cost = worker.cost;
                                    auto &allowed = worker.allowed;
                                    if (pricing.remaining() <= 0) { worker.completed = false; return; }
                                    const auto &d = p.demands[k];
                                    const double initial = -d.quantity * d.penalty - dual[kr[k]];
                                    std::fill(allowed.begin(), allowed.end(), false);
                                    if (restricted_links) {
                                        if (!neighborhoods[k])
                                            neighborhoods[k] = logicalPricingNeighborhood(p, pricing_graph, k, s.plan,
                                                reference_length, options.guidance_neighbors);
                                        for (int b : *neighborhoods[k]) allowed[b] = true;
                                    }
                                    if (dual[kr[k]] > 10 * tolerance)
                                        throw std::runtime_error("Positive demand-row dual in logical pricing");
                                    for (int b : p.compatibleBlocks(k)) {
                                        const int i = p.blocks[b].from;
                                        if (!restricted_links) allowed[b] = !pool[b].empty();
                                        if (activationDual(k, b) > 10 * tolerance)
                                            throw std::runtime_error("Positive activation-row dual in logical pricing");
                                        logical_cost[b] = d.quantity * dual[br[b]] - activationDual(k, b) +
                                            (i == d.from ? 0 : d.quantity * (p.nodes[i].handling_cost - dual[hr[i]]));
                                    }
                                    for (const auto &path : paths[k])
                                        if (jrc(k, path) < -10 * tolerance)
                                            throw std::runtime_error("Scratch LP logical JRC coefficient mismatch");
                                    auto collect = [&](PricedPath path) {
                                        const double score = initial + path.cost;
                                        if (score >= -tolerance ||
                                            std::find(paths[k].begin(), paths[k].end(), path.edges) != paths[k].end()) return;
                                        worker.candidates.emplace(score, k, std::move(path.edges));
                                        if (worker.candidates.size() > 1000)
                                            worker.candidates.erase(std::prev(worker.candidates.end()));
                                    };
                                    Budget search;
                                    search.start = pricing.start;
                                    search.seconds = pricing.elapsed() + pricing.remaining() / (K - k);
                                    try {
                                        worker.labels.run(pricing_graph, logical_nodes[d.from], logical_nodes[d.to],
                                                   logical_cost, {}, 1e100, {}, allowed,
                                                   search, worker.stats, options.guidance_heuristic_pricing, -initial - tolerance, collect);
                                    } catch (const LimitReached &limit) {
                                        ++worker.limits[limit.what()];
                                        worker.completed = false;
                                    }
                                });
                                std::size_t peak_bytes = 0, peak_labels = 0;
                                for (auto &worker : workers) {
                                    completed = completed && worker.completed;
                                    for (auto [reason, count] : worker.limits) pricing_limits[reason] += count;
                                    stats.labels += worker.stats.labels;
                                    stats.dominated += worker.stats.dominated;
                                    stats.searches += worker.stats.searches;
                                    stats.heuristic_searches += worker.stats.heuristic_searches;
                                    stats.search_seconds += worker.stats.search_seconds;
                                    worker.peak_bytes = std::max(worker.peak_bytes, worker.stats.peak_label_bytes);
                                    worker.peak_labels = std::max(worker.peak_labels, worker.stats.peak_labels);
                                    peak_bytes += worker.peak_bytes; peak_labels += worker.peak_labels;
                                    candidates.merge(worker.candidates);
                                    while (candidates.size() > 1000) candidates.erase(std::prev(candidates.end()));
                                }
                                stats.peak_label_bytes = std::max(stats.peak_label_bytes, peak_bytes);
                                stats.peak_labels = std::max(stats.peak_labels, peak_labels);
                                guidance_pricing_seconds += budget.elapsed() - pricing_start;
                                audit["pricing_completed"] = completed;
                                if (candidates.empty()) {
                                    completed_no_new = completed;
                                    iteration_seconds.push_back(budget.elapsed() - iteration_begin);
                                    stop = completed ? (options.guidance_heuristic_pricing ?
                                        "heuristic_no_new_columns" : "no_new_columns") : "pricing_limit";
                                    break;
                                }
                                const double add_start = budget.elapsed();
                                for (const auto &[score, k, path] : candidates) {
                                    budget.check();
                                    (void)score;
                                    for (int b : path) if (!gr[k].count(b)) {
                                        Sparse row;
                                        for (const auto &column : logical)
                                            if (column.owner == k && std::find(paths[k][column.path].begin(),
                                                paths[k][column.path].end(), b) != paths[k][column.path].end())
                                                row.add(column.x, 1);
                                        for (const auto &column : physical) if (column.owner == b) row.add(column.x, -1);
                                        gr[k][b] = model.row(row, '<', 0);
                                    }
                                    Sparse xcol, vcol;
                                    xcol.add(kr[k], 1);
                                    double unit = -p.demands[k].penalty;
                                    for (int b : path) {
                                        xcol.add(gr[k].at(b), 1);
                                        vcol.add(br[b], -1);
                                        const int i = p.blocks[b].from;
                                        if (i != p.demands[k].from) {
                                            unit += p.nodes[i].handling_cost;
                                            vcol.add(hr[i], 1);
                                        }
                                    }
                                    const int x = model.var(0, SOLVER_INFINITY, SOLVER_CONTINUOUS, xcol);
                                    const int v = model.var(unit, SOLVER_INFINITY, SOLVER_CONTINUOUS, vcol);
                                    Sparse upper;
                                    upper.add(v, 1); upper.add(x, -p.demands[k].quantity);
                                    model.row(upper, '<', 0);
                                    logical.push_back({k, int(paths[k].size()), x, v});
                                    paths[k].push_back(path);
                                }
                                guidance_build_seconds += budget.elapsed() - add_start;
                                ++batches;
                                audit["batches"] = batches;
                                if (!solve_lp(false)) { stop = "lp_failure"; break; }
                                dual = model.dual();
                                const double previous = objectives.back();
                                const double increase = pool_objective - previous;
                                if (increase > std::max(objectiveTolerance(pool_objective, previous),
                                                        1e-8 * std::max(1., std::abs(previous))))
                                    throw std::runtime_error("Scratch LP objective increased after a complete column batch");
                                if (!save_solved()) { stop = "row_failure"; break; }
                                objectives.push_back(pool_objective);
                                const double iteration_elapsed = budget.elapsed() - iteration_begin;
                                iteration_seconds.push_back(iteration_elapsed);
                                if (options.guidance_iteration_seconds > 0 &&
                                    iteration_elapsed >= options.guidance_iteration_seconds) {
                                    stop = "iteration_time_target";
                                    break;
                                }
                            }
                        } catch (const LimitReached &limit) {
                            stop = limit.what();
                        }
                        audit["stop"] = stop;
                        audit["iteration_seconds"] = iteration_seconds;
                        audit["scratch_logical_paths"] = logical.size();
                        audit["labels"] = stats.labels;
                        audit["peak_labels"] = stats.peak_labels;
                        audit["peak_label_bytes"] = stats.peak_label_bytes;
                        audit["pricing_limits"] = pricing_limits;
                        audit["pricing_searches"] = stats.searches;
                        audit["heuristic_pricing_searches"] = stats.heuristic_searches;
                        bool fresh_terminal = false;
                        // Only a solved, row-feasible prefix may survive a failed terminal attempt.
                        try {
                            if (solve_lp(true)) {
                                dual = model.dual();
                                fresh_terminal = save_solved();
                                if (!fresh_terminal) audit["stop"] = "terminal_row_failure";
                            } else audit["stop"] = "terminal_lp_failure";
                        } catch (const LimitReached &limit) {
                            audit["stop"] = limit.what();
                        }
                        audit["fresh_terminal_solve"] = fresh_terminal;
                        audit["terminal_optimal"] = true;
                        audit["terminal_objective"] = solved_objective;
                        audit["terminal_row_violation"] = solved_violation;
                        audit["solved_batches"] = solved_batches;
                        std::size_t solved_count = 0;
                        for (const auto &scores : solved_scores) solved_count += scores.size();
                        audit["solved_logical_paths"] = solved_count;
                        // Classify the latest solved prefix, independently of limit/time stop priority.
                        std::vector<double> convergence_values = objectives;
                        if (convergence_values.size() >= std::size_t(solved_batches + 1)) {
                            convergence_values.resize(solved_batches + 1);
                            convergence_values.back() = solved_objective;
                        } else if (convergence_values.size() == std::size_t(solved_batches))
                            convergence_values.push_back(solved_objective);
                        const bool converged = completed_no_new || options.guidancePlateau(convergence_values);
                        audit["lp_objectives"] = convergence_values;
                        audit["converged"] = converged;
                        audit["return_mode"] = converged ? "JRC_only" : "mixed";
                        audit["returned_paths"] = Json::array();
                        const double zero = 1e-12 * std::max(1., std::abs(solved_objective));
                        const int jrc_limit = converged ? options.guided_paths_per_demand :
                            std::min(options.guided_jrc_paths_per_demand, options.guided_paths_per_demand);
                        const int diverse_limit = converged ? 0 : std::min(options.guided_diverse_paths_per_demand,
                            options.guided_paths_per_demand - jrc_limit);
                        const int euc_limit = options.guided_paths_per_demand - jrc_limit - diverse_limit;
                        double euc_seconds = 0, selection_seconds = 0;
                        int euc_completed = 0, euc_interrupted = 0;
                        std::map<std::string, int> source_counts;
                        std::vector<double> minimum, current;
                        std::vector<double> logical_cost(euc_limit > 0 ? B : 0);
                        std::vector<char> allowed(euc_limit > 0 ? B : 0);
                        if (euc_limit > 0) {
                            const double prepare = budget.elapsed();
                            minimum.assign(B, std::numeric_limits<double>::infinity());
                            current = minimum;
                            for (int b = 0; b < B; ++b) {
                                for (const auto &route : pool[b]) minimum[b] = std::min(minimum[b], route.unit);
                                if (s.route[b] >= 0) current[b] = pool[b][s.route[b]].unit;
                            }
                            euc_seconds += budget.elapsed() - prepare;
                        }
                        int returned = 0;
                        for (int k = 0; k < K; ++k) {
                            const double select_begin = budget.elapsed();
                            std::set<std::pair<double, std::vector<int>>> ranked;
                            for (int r = 0; r < int(solved_scores[k].size()); ++r) {
                                const auto &path = paths[k][r];
                                if (std::find(known_paths[k].begin(), known_paths[k].end(), path) == known_paths[k].end()) {
                                    const double score = solved_scores[k][r];
                                    ranked.emplace(std::abs(score) <= zero ? 0. : score, path);
                                }
                            }
                            std::set<std::vector<int>> used(known_paths[k].begin(), known_paths[k].end());
                            int kept = 0;
                            auto retain = [&](const std::vector<int> &path, const char *origin, double score, bool euc) {
                                if (kept >= options.guided_paths_per_demand || !used.insert(path).second) return false;
                                guided[k].push_back(path);
                                ++guided_logical;
                                ++returned;
                                Json row = {{"demand", k}, {"rank", ++kept}, {"path", path}, {"source", origin}};
                                row[euc ? "euc" : "jrc"] = score;
                                audit["returned_paths"].push_back(std::move(row));
                                ++source_counts[origin];
                                return true;
                            };
                            if (converged) {
                                for (const auto &[score, path] : ranked) {
                                    retain(path, "JRC", score, false);
                                    if (kept == options.guided_paths_per_demand) break;
                                }
                                selection_seconds += budget.elapsed() - select_begin;
                                continue;
                            }
                            std::vector<double> scores;
                            std::vector<std::vector<int>> choices, resources;
                            for (const auto &[score, path] : ranked) {
                                scores.push_back(score);
                                choices.push_back(path);
                                if (diverse_limit > 0 && ranked.size() > std::size_t(jrc_limit + 1))
                                    resources.push_back(pricingResources(s, path));
                            }
                            const auto diverse = selectDiversePaths(scores, resources, jrc_limit + diverse_limit, jrc_limit);
                            for (int j = 0; j < int(diverse.size()); ++j) {
                                const int r = diverse[j];
                                retain(choices[r], j < jrc_limit ? "JRC" : "resource_diverse", scores[r], false);
                            }
                            double euc_elapsed = 0;
                            if (euc_limit > 0 && kept < options.guided_paths_per_demand) {
                                const double euc_begin = budget.elapsed();
                                const auto &d = p.demands[k];
                                if (!(d.quantity > 0)) throw std::runtime_error("EUC needs positive demand quantity");
                                std::fill(allowed.begin(), allowed.end(), false);
                                for (int b : p.compatibleBlocks(k)) {
                                    if (!std::isfinite(minimum[b])) continue;
                                    const bool open = !s.plan.physical[b].empty();
                                    logical_cost[b] = (open ? current[b] : minimum[b] + p.blocks[b].opening / d.quantity) +
                                        (p.blocks[b].from == d.from ? 0 : p.nodes[p.blocks[b].from].handling_cost);
                                    if (!std::isfinite(logical_cost[b]) || logical_cost[b] < 0)
                                        throw std::runtime_error("EUC graph needs finite nonnegative edge costs");
                                    allowed[b] = true;
                                }
                                if (known_paths[k].size() > std::size_t(std::numeric_limits<int>::max() - options.guided_paths_per_demand))
                                    throw std::runtime_error("Too many C0 paths for EUC enumeration");
                                bool completed = false;
                                const auto euc_ranked = shortestSimplePaths(pricing_graph, logical_nodes[d.from], logical_nodes[d.to],
                                    logical_cost, allowed, budget, int(known_paths[k].size()) + kept + euc_limit, &completed);
                                euc_completed += completed;
                                euc_interrupted += !completed;
                                int taken = 0;
                                for (const auto &path : euc_ranked) {
                                    taken += retain(path.edges, "EUC", path.cost, true);
                                    if (taken == euc_limit) break;
                                }
                                euc_elapsed = budget.elapsed() - euc_begin;
                                euc_seconds += euc_elapsed;
                            }
                            selection_seconds += budget.elapsed() - select_begin - euc_elapsed;
                        }
                        audit["native_euc_seconds"] = euc_seconds;
                        audit["native_selection_seconds"] = selection_seconds;
                        audit["native_euc_completed_searches"] = euc_completed;
                        audit["native_euc_interrupted_searches"] = euc_interrupted;
                        audit["native_source_counts"] = source_counts;
                        audit["returned"] = returned;
                        if (fresh_terminal && audit.value("pricing_completed", false))
                            priced_pool_key = std::move(pool_key);
                        std::clog << "PATH_GENERATION_DONE " << audit.dump() << '\n';
                    };
                    generate();
                } catch (const LimitReached &) {
                }
                if (integer_finish) pool_mip_seconds += budget.elapsed() - begin;
                else guidance_seconds += budget.elapsed() - begin;
            }

            void expandPhysicalPool() {
                if (options.physical_alternative_seconds <= 0)
                    return;
                const double start = budget.elapsed();
                Budget phase;
                phase.seconds = std::min(options.physical_alternative_seconds,
                                         std::max(0., budget.remaining() - options.min_pool_seconds));
                std::vector<int> opened;
                for (int b = 0; b < (int)p.blocks.size(); ++b)
                    if (!out.plan.physical[b].empty())
                        opened.push_back(b);
                Stats stats;
                for (std::size_t j = 0; j < opened.size() && phase.remaining() > 0; ++j) {
                    const int b = opened[j];
                    std::vector<std::vector<int>> existing;
                    for (const auto &route : pool[b])
                        existing.push_back(route.arcs);
                    Budget search;
                    search.start = phase.start;
                    search.seconds = phase.elapsed() + phase.remaining() / (opened.size() - j);
                    auto paths =
                        physicalAlternatives(p, b, existing, search, stats,
                                             options.physical_alternative_paths_per_block,
                                             options.physical_length_weights);
                    for (auto &path : paths) {
                        Route route;
                        route.arcs = std::move(path.edges);
                        route.unit = p.blocks[b].interchange + path.cost;
                        route.capacity = p.blocks[b].upper;
                        std::map<int, int> links;
                        for (int a : route.arcs)
                            ++links[p.arcs[a].link];
                        for (auto [e, n] : links) {
                            route.links.emplace_back(e, n);
                            route.capacity = std::min(route.capacity, p.capacities[e] / n);
                        }
                        if (route.capacity + Tol >= p.blocks[b].lower) {
                            pool[b].push_back(std::move(route));
                            ++physical_alternatives_added;
                        }
                    }
                }
                physical_alternative_seconds += budget.elapsed() - start;
            }

            void preparePoolCandidates() {
                ++component_prep_calls;
                if (options.benchmark_pool_only_joint) return;
                expandPhysicalPool();
                State state(p);
                state.plan = out.plan;
                for (int b = 0; b < (int)p.blocks.size(); ++b)
                    if (!state.plan.physical[b].empty()) {
                        for (int r = 0; r < (int)pool[b].size(); ++r)
                            if (pool[b][r].arcs == state.plan.physical[b])
                                state.route[b] = r;
                        if (state.route[b] < 0)
                            throw std::runtime_error("Joint-pool incumbent route is missing");
                    }
                rebuild(state);
                for (int k = 0; k < (int)p.demands.size(); ++k)
                    for (const auto &path : jointPaths(state, k, true, true))
                        if (std::find(guided[k].begin(), guided[k].end(), path) == guided[k].end())
                            guided[k].push_back(path);
                if (options.support_search_seconds > 0) {
                    const double start = budget.elapsed();
                    std::vector<double> minimum_unit(p.blocks.size(), 1e100);
                    for (int b = 0; b < (int)p.blocks.size(); ++b)
                        for (const auto &route : pool[b])
                            minimum_unit[b] = std::min(minimum_unit[b], route.unit);
                    Budget support_budget;
                    support_budget.seconds =
                        std::min(options.support_search_seconds,
                                 std::max(0., budget.remaining() - options.min_pool_seconds));
                    Stats stats;
                    auto paths = supportPaths(p, out.plan, minimum_unit, guided, support_budget, stats,
                                              std::size_t(options.pricing_label_memory_mib) * 1024 * 1024,
                                              options.support_paths_per_demand);
                    for (int k = 0; k < (int)paths.size(); ++k)
                        for (auto &path : paths[k]) {
                            guided[k].push_back(std::move(path));
                            ++support_paths_added;
                        }
                    support_seconds += budget.elapsed() - start;
                }
            }

            bool improveAllDemands() {
                if (budget.remaining() > options.min_pool_seconds) {
                    auto path_count = [&]() {
                        std::size_t count = 0;
                        for (const auto &paths : guided) count += paths.size();
                        for (const auto &paths : pool) count += paths.size();
                        return count;
                    };
                    const auto before = path_count();
                    try {
                        preparePoolCandidates();
                        if (options.pool_capture_only) {
                            State state(p);
                            state.plan = out.plan;
                            for (int b = 0; b < (int)p.blocks.size(); ++b)
                                for (int r = 0; r < (int)pool[b].size(); ++r)
                                    if (pool[b][r].arcs == state.plan.physical[b])
                                        state.route[b] = r;
                            rebuild(state);
                            PathPool logical(p.demands.size()), physical(p.blocks.size());
                            for (int k = 0; k < (int)p.demands.size(); ++k)
                                logical[k] = jointPaths(state, k, false);
                            for (int b = 0; b < (int)p.blocks.size(); ++b)
                                for (const auto &route : pool[b])
                                    physical[b].push_back(route.arcs);
                            candidate_pool = {{"scope", "First big MIP input; no big MIP solved"},
                                              {"incumbent_objective", out.objective},
                                              {"incumbent", out.plan.json()},
                                              {"logical_paths", logical},
                                              {"physical_paths", physical}};
                            return false;
                        }
                        if (options.benchmark_joint_repair) solvePathModel(true);
                    } catch (const LimitReached &) {
                        return path_count() > before;
                    }
                    return path_count() > before;
                }
                return false;
            }
            void generateOpenAlternatives(const State &s) {
                if (options.open_logical_paths_per_demand == 0 || p.demands.empty()) return;
                const double start = budget.elapsed();
                Budget search;
                search.seconds = std::min(options.open_logical_search_seconds,
                                         std::max(0., budget.remaining() - options.min_pool_seconds));
                const std::size_t K = p.demands.size();
                for (std::size_t visited = 0; visited < K; ++visited) {
                    if (search.remaining() <= 0) {
                        open_path_skipped += K - visited;
                        break;
                    }
                    const int k = open_path_cursor;
                    open_path_cursor = (open_path_cursor + 1) % K;
                    if (open_path_generation[k] == incumbent_generation) {
                        ++open_path_cache_hits;
                        continue;
                    }
                    ++open_path_searches;
                    try {
                        std::vector<double> cost(p.blocks.size());
                        std::vector<char> allowed(p.blocks.size());
                        for (int b : p.compatibleBlocks(k)) {
                            search.check();
                            if (s.route[b] < 0) continue;
                            allowed[b] = true;
                            cost[b] = pool[b][s.route[b]].unit +
                                      (p.blocks[b].from == p.demands[k].from
                                           ? 0 : p.nodes[p.blocks[b].from].handling_cost);
                        }
                        bool completed = false;
                        auto alternatives = shortestSimplePaths(p.logical, p.demands[k].from,
                                                                 p.demands[k].to, cost, allowed, search,
                                                                 options.open_logical_paths_per_demand,
                                                                 &completed);
                        for (auto &path : alternatives)
                            if (std::find(guided[k].begin(), guided[k].end(), path.edges) == guided[k].end()) {
                                guided[k].push_back(std::move(path.edges));
                                ++open_paths_added;
                            }
                        if (completed) {
                            open_path_generation[k] = incumbent_generation;
                            if (alternatives.size() == std::size_t(options.open_logical_paths_per_demand))
                                ++open_path_k_reached;
                            else ++open_path_exhausted;
                        } else ++open_path_interruptions;
                    } catch (const LimitReached &) {
                        ++open_path_interruptions;
                    }
                }
                open_path_seconds += budget.elapsed() - start;
            }

            std::vector<std::vector<int>> poolPaths(const State &s, int k) const {
                // Experiment switch: the incumbent path and the shared logical pool only.
                std::vector<std::vector<int>> paths;
                if (!s.plan.logical[k].empty()) paths.push_back(s.plan.logical[k]);
                for (const auto &path : guided[k])
                    if (std::find(paths.begin(), paths.end(), path) == paths.end()) paths.push_back(path);
                return paths;
            }

            std::vector<std::vector<int>> jointPaths(const State &s, int k, bool expand, bool protect_splits = false) {
                budget.check();
                std::set<std::vector<int>> unique;
                for (int b : p.compatibleBlocks(k))
                    if (p.blocks[b].from == p.demands[k].from && p.blocks[b].to == p.demands[k].to &&
                        p.compatible(k, b)) {
                        ensureRoutes(b);
                        if (!pool[b].empty())
                            unique.insert({b});
                    }
                for (int h : demandHubs(k)) {
                    auto path = via(s, k, h);
                    if (!path.empty())
                        unique.insert(std::move(path));
                }
                for (int h : demandHubs(k))
                    for (int e = p.logical.offsets[h]; e < p.logical.offsets[h + 1]; ++e) {
                        int b = p.logical.edge_ids[e], j = p.blocks[b].to;
                        if (!p.compatible(k, b) || h == j)
                            continue;
                        auto path = corridor(s, k, h, j);
                        if (!path.empty())
                            unique.insert(std::move(path));
                    }
                for (bool opened : {false, true}) {
                    auto path = logicalPath(s, k, opened, true);
                    if (!path.empty())
                        unique.insert(std::move(path));
                }
                std::vector<std::vector<int>> paths;
                paths.assign(unique.begin(), unique.end());
                std::stable_sort(paths.begin(), paths.end(), [&](const auto &a, const auto &b) {
                    return pathUnit(s, k, a) < pathUnit(s, k, b);
                });
                if (paths.size() > std::size_t(options.base_logical_paths))
                    paths.resize(options.base_logical_paths);
                // Preserve long chains that reuse existing blocks around one or two new
                // services.
                if (expand && !demand_driven)
                    for (auto &path : bridgePaths(s, k, protect_splits))
                        if (std::find(paths.begin(), paths.end(), path) == paths.end())
                            paths.push_back(std::move(path));
                if (!s.plan.logical[k].empty() &&
                    std::find(paths.begin(), paths.end(), s.plan.logical[k]) == paths.end())
                    paths.push_back(s.plan.logical[k]);
                for (const auto &path : guided[k])
                    if (std::find(paths.begin(), paths.end(), path) == paths.end())
                        paths.push_back(path);
                return paths;
            }

            void improveDemandGroup(const std::vector<int> &released = {}) {
                if (budget.remaining() < options.min_joint_seconds)
                    return;
                const double begin = budget.elapsed();
                const bool expand = stale_repairs >= options.expand_after_stalls;
                State s(p);
                s.plan = out.plan;
                for (int b = 0; b < (int)p.blocks.size(); ++b)
                    if (!s.plan.physical[b].empty()) {
                        for (int r = 0; r < (int)pool[b].size(); ++r)
                            if (pool[b][r].arcs == s.plan.physical[b])
                                s.route[b] = r;
                        if (s.route[b] < 0)
                            throw std::runtime_error("Incumbent path missing from joint repair");
                    }
                rebuild(s);
                std::vector<int> free(p.demands.size());
                std::iota(free.begin(), free.end(), 0);
                auto loss = [&](int k) {
                    return (p.demands[k].quantity - s.plan.served[k]) * p.demands[k].penalty;
                };
                std::stable_sort(free.begin(), free.end(), [&](int a, int b) { return loss(a) > loss(b); });
                if (free.empty())
                    return;
                const double largest_loss = std::max(1., loss(free.front()));
                int anchor = free[mip_round++ % free.size()];
                if (search_cycles > 1) {
                    anchor = free[std::uniform_int_distribution<std::size_t>(0, free.size() - 1)(random)];
                    std::shuffle(free.begin(), free.end(), random);
                }
                if (!released.empty())
                    anchor = *std::max_element(released.begin(), released.end(),
                                              [&](int a, int b) { return loss(a) < loss(b); });
                auto rank = [&](int k) {
                    double shared = 0;
                    for (int b : s.plan.logical[k])
                        if (p.blocks[b].from == p.demands[anchor].from ||
                            p.blocks[b].to == p.demands[anchor].to)
                            shared += 1;
                    return options.anchor_priority * (k == anchor) +
                           options.origin_priority * (p.demands[k].from == p.demands[anchor].from ||
                                                      p.demands[k].to == p.demands[anchor].to) +
                           options.endpoint_priority * shared + loss(k) / largest_loss;
                };
                std::stable_sort(free.begin(), free.end(), [&](int a, int b) { return rank(a) > rank(b); });
                const std::size_t group_size = expand ? options.large_group_size : options.small_group_size;
                // The destroyed minimum-load closure must be repairable in the same group.
                std::stable_partition(free.begin(), free.end(), [&](int k) {
                    return std::find(released.begin(), released.end(), k) != released.end();
                });
                if (released.size() > group_size)
                    throw std::invalid_argument("Destroyed demand group exceeds the local repair limit");
                if (free.size() > group_size)
                    free.resize(group_size);
                const auto generation_before = incumbent_generation;
                auto members = free;
                std::sort(members.begin(), members.end());
                std::clog << "GROUP_SELECT round=" << mip_round << " seed=" << anchor
                          << " expanded=" << expand << " generation=" << generation_before
                          << " demands=" << Json(members).dump() << '\n';
                std::vector<char> is_free(p.demands.size());
                for (int k : free)
                    is_free[k] = true;
                struct DColumn {
                    int k, x, v;
                    std::vector<int> path;
                };
                struct BColumn {
                    int b, r, x, v;
                };
                std::vector<DColumn> demands;
                std::vector<BColumn> blocks;
                std::vector<std::vector<std::vector<int>>> paths_by_demand(p.demands.size());
                std::vector<char> active(p.blocks.size());
                for (int k : free) {
                    paths_by_demand[k] = options.benchmark_pool_only_local ? poolPaths(s, k) : jointPaths(s, k, expand);
                    for (const auto &path : paths_by_demand[k])
                        for (int b : path)
                            active[b] = true;
                }
                CostSum constant;
                constant += p.rejection;
                std::vector<double> outside(p.blocks.size()), fixed_h(p.nodes.size()),
                    fixed_tracks(p.nodes.size()), fixed_links(p.capacities.size());
                for (int k = 0; k < (int)p.demands.size(); ++k)
                    if (!is_free[k]) {
                        double v = s.plan.served[k];
                        constant += -v * p.demands[k].penalty;
                        for (int b : s.plan.logical[k]) {
                            outside[b] += v;
                            int i = p.blocks[b].from;
                            if (i != p.demands[k].from) {
                                fixed_h[i] += v;
                                constant += v * p.nodes[i].handling_cost;
                            }
                        }
                    }
                for (int b = 0; b < (int)p.blocks.size(); ++b)
                    if (!active[b] && s.route[b] >= 0) {
                        const auto &r = pool[b][s.route[b]];
                        constant += p.blocks[b].opening + s.load[b] * r.unit;
                        fixed_tracks[p.blocks[b].from] += p.blocks[b].slots;
                        for (auto [e, n] : r.links)
                            fixed_links[e] += n * s.load[b];
                    }
                Model model(options.solver_threads, "IRBP-small-demand-group");
                model.constant(constant.value);
                std::vector<Sparse> balance(p.blocks.size()), open(p.blocks.size()), handling(p.nodes.size()),
                    slots(p.nodes.size()), link(p.capacities.size());
                for (int k : free) {
                    Sparse select;
                    for (const auto &path : paths_by_demand[k]) {
                        double cost = -p.demands[k].penalty;
                        for (int b : path)
                            if (p.blocks[b].from != p.demands[k].from)
                                cost += p.nodes[p.blocks[b].from].handling_cost;
                        int x = model.var(0, 1, SOLVER_BINARY), v = model.var(cost, p.demands[k].quantity);
                        Sparse bound;
                        bound.add(v, 1);
                        bound.add(x, -p.demands[k].quantity);
                        model.row(bound, '<', 0);
                        select.add(x, 1);
                        demands.push_back({k, x, v, path});
                        for (int b : path) {
                            balance[b].add(v, -1);
                            int i = p.blocks[b].from;
                            if (i != p.demands[k].from)
                                handling[i].add(v, 1);
                        }
                    }
                    model.row(select, '<', 1);
                }
                for (int b = 0; b < (int)p.blocks.size(); ++b)
                    if (active[b]) {
                        budget.check();
                        std::set<int> routes;
                        for (int r = 0; r < std::min(options.physical_paths_per_block, (int)pool[b].size());
                             ++r)
                            routes.insert(r);
                        if (s.route[b] >= 0)
                            routes.insert(s.route[b]);
                        if (!pool[b].empty())
                            routes.insert(std::max_element(pool[b].begin(), pool[b].end(),
                                                           [](const Route &a, const Route &b) {
                                                               return a.capacity < b.capacity;
                                                           }) -
                                          pool[b].begin());
                        for (int r : routes) {
                            const auto &route = pool[b][r];
                            int x = model.var(p.blocks[b].opening, 1, SOLVER_BINARY),
                                v = model.var(route.unit, route.capacity);
                            Sparse lower;
                            lower.add(v, 1);
                            lower.add(x, -p.blocks[b].lower);
                            model.row(lower, '>', 0);
                            Sparse upper;
                            upper.add(v, 1);
                            upper.add(x, -route.capacity);
                            model.row(upper, '<', 0);
                            blocks.push_back({b, r, x, v});
                            balance[b].add(v, 1);
                            open[b].add(x, 1);
                            slots[p.blocks[b].from].add(x, p.blocks[b].slots);
                            for (auto [e, n] : route.links)
                                link[e].add(v, n);
                        }
                        model.row(open[b], '<', 1);
                        model.row(balance[b], '=', outside[b]);
                    }
                for (int k : free)
                    for (int b : p.compatibleBlocks(k))
                        if (active[b]) {
                            Sparse row;
                            for (const auto &d : demands)
                                if (d.k == k && std::find(d.path.begin(), d.path.end(), b) != d.path.end())
                                    row.add(d.x, 1);
                            if (row.ind.empty())
                                continue;
                            for (const auto &r : blocks)
                                if (r.b == b)
                                    row.add(r.x, -1);
                            model.row(row, '<', 0);
                        }
                for (int i : yards) {
                    if (!handling[i].ind.empty())
                        model.row(handling[i], '<', std::max(0., p.nodes[i].handling_capacity - fixed_h[i]));
                    if (!slots[i].ind.empty())
                        model.row(slots[i], '<', std::max(0., p.nodes[i].tracks - fixed_tracks[i]));
                }
                Json capacity_rows;
                addLinkCapacityRows(
                    model, link, [&](int e) { return std::max(0., p.capacities[e] - fixed_links[e]); },
                    capacity_rows);
                capacity_rows["stage"] = "group_mip";
                capacity_row_history.push_back(capacity_rows);
                check(model.solver.updateModel());
                std::vector<double> start(model.nvars);
                for (const auto &d : demands)
                    if (d.path == s.plan.logical[d.k]) {
                        start[d.x] = 1;
                        start[d.v] = s.plan.served[d.k];
                    }
                for (const auto &r : blocks)
                    if (r.r == s.route[r.b]) {
                        start[r.x] = 1;
                        start[r.v] = s.load[r.b];
                    }
                check(model.solver.setMIPStart(0, model.nvars, start.data()));
                check(model.solver.setEnvCutoff(s.cost));
                if (group_cache_generation != incumbent_generation) {
                    exhausted_groups.clear();
                    group_cache_generation = incumbent_generation;
                }
                Json signature = {{"members", members}, {"logical", Json::array()},
                                  {"physical", Json::array()}};
                for (int k : members) {
                    auto candidates = paths_by_demand[k];
                    std::sort(candidates.begin(), candidates.end());
                    signature["logical"].push_back({k, candidates});
                }
                std::map<int, std::vector<std::vector<int>>> physical_choices;
                for (const auto &r : blocks)
                    physical_choices[r.b].push_back(pool[r.b][r.r].arcs);
                for (auto &[b, candidates] : physical_choices) {
                    std::sort(candidates.begin(), candidates.end());
                    signature["physical"].push_back({b, candidates});
                }
                const std::string group_key = signature.dump();
                auto previous = exhausted_groups.find(group_key);
                if (previous != exhausted_groups.end()) {
                    ++exhausted_group_skips;
                    ++stale_repairs;
                    mip_build_seconds += budget.elapsed() - begin;
                    group_history.push_back({{"seed", anchor}, {"members", members},
                                             {"generation", generation_before}, {"skipped", true},
                                             {"previous_iteration", previous->second}});
                    std::clog << "GROUP_SKIP reason=same_optimal_subproblem seed=" << anchor
                              << " previous_iteration=" << previous->second << '\n';
                    return;
                }
                ++mip_calls;
                std::clog << "GROUP_MIP iteration=" << mip_calls << " demands=" << free.size()
                          << " columns=" << model.nvars << " rows=" << model.nrows << '\n';
                expanded_repairs += expand;
                mip_columns = model.nvars;
                mip_rows = model.nrows;
                mip_build_seconds += budget.elapsed() - begin;
                if (budget.remaining() <= options.joint_return_reserve + options.solver_start_reserve)
                    return;
                check(model.solver.setEnvTimeLimit(std::min(
                    options.joint_mip_max_seconds, budget.remaining() - options.joint_return_reserve)));
                const auto rejects_before = rejected_candidate_checks;
                auto accept_solution = [&](double objective, const std::vector<double> &values) {
                    Plan candidate = s.plan;
                    for (int k : free) {
                        candidate.logical[k].clear();
                        candidate.served[k] = 0;
                    }
                    for (const auto &d : demands)
                        if (values[d.x] > .5 && values[d.v] > Tol) {
                            candidate.logical[d.k] = d.path;
                            candidate.served[d.k] = values[d.v];
                        }
                    for (int b = 0; b < (int)active.size(); ++b)
                        if (active[b])
                            candidate.physical[b].clear();
                    for (const auto &r : blocks)
                        if (values[r.x] > .5)
                            candidate.physical[r.b] = pool[r.b][r.r].arcs;
                    const auto validated = validateRepairCandidate(candidate, "local", [&](int b, Json &record) {
                        CostSum raw_physical, raw_logical;
                        record["outside_load"] = outside[b];
                        record["physical_columns"] = record["logical_columns"] = Json::array();
                        for (const auto &r : blocks) if (r.b == b) {
                            raw_physical += values[r.v];
                            record["physical_columns"].push_back({{"path_index", r.r}, {"x", values[r.x]},
                                {"v", values[r.v]}, {"min_row_residual", values[r.v] - p.blocks[b].lower * values[r.x]}});
                        }
                        for (const auto &d : demands)
                            if (std::find(d.path.begin(), d.path.end(), b) != d.path.end()) {
                                raw_logical += values[d.v];
                                if (values[d.v] != 0.) record["logical_columns"].push_back({{"k", d.k},
                                    {"x", values[d.x]}, {"v", values[d.v]}, {"path", d.path},
                                    {"volume_link_slack", p.demands[d.k].quantity * values[d.x] - values[d.v]}});
                            }
                        record["raw_physical_volume"] = raw_physical.value;
                        record["raw_free_logical_volume"] = raw_logical.value;
                        record["balance_residual"] = raw_physical.value - outside[b] - raw_logical.value;
                    });
                    if (!validated) return;
                    const double cost = *validated;
                    std::vector<double> replay(model.nvars), loads(p.blocks.size());
                    for (int k = 0; k < int(p.demands.size()); ++k)
                        for (int b : candidate.logical[k]) loads[b] += candidate.served[k];
                    for (const auto &d : demands)
                        if (values[d.x] > .5 && values[d.v] > Tol) {
                            replay[d.x] = 1;
                            replay[d.v] = candidate.served[d.k];
                        }
                    for (const auto &r : blocks)
                        if (values[r.x] > .5) {
                            replay[r.x] = 1;
                            replay[r.v] = loads[r.b];
                        }
                    const double decoded_objective = model.objective(replay);
                    if (!objectiveEqual(cost, decoded_objective))
                        throw std::runtime_error("Joint decoded-model objective mismatch: validated=" +
                                                 std::to_string(cost) + " model=" +
                                                 std::to_string(decoded_objective) + " reported=" +
                                                 std::to_string(objective));
                    if (!objectiveEqual(cost, objective))
                        std::clog << "GROUP_OBJECTIVE_REPLAY reported=" << std::setprecision(17)
                                  << objective << " decoded_model=" << decoded_objective
                                  << " validated=" << cost << '\n';
                    if (cost < out.objective - Tol) {
                        out.plan = std::move(candidate);
                        out.objective = cost;
                        out.history.push_back(
                            {{"seconds", budget.elapsed()}, {"objective", cost}, {"stage", "joint_mip"}});
                        notify();
                    }
                };
                check(model.solver.setSolutionCallback(model.nvars, out.objective, accept_solution, &out.objective));
                local_integrality_focus = repairIntegralityFocus(model, "local");
                check(model.solver.mipOptimize());
                int count;
                check(model.solver.getSolCount(&count));
                if (count) accept_solution(model.objective(), model.values());
                int group_status;
                check(model.solver.getStatus(&group_status));
                if (group_status == SOLVER_OPTIMAL && incumbent_generation == generation_before &&
                    rejected_candidate_checks == rejects_before) {
                    if (exhausted_groups.size() >= p.demands.size())
                        exhausted_groups.clear();
                    exhausted_groups.emplace(group_key, mip_calls);
                }
                group_history.push_back({{"seed", anchor}, {"members", members},
                                         {"generation", generation_before}, {"skipped", false},
                                         {"status", group_status}, {"columns", model.nvars},
                                         {"rows", model.nrows}, {"seconds", budget.elapsed() - begin},
                                         {"before", s.cost}, {"after", out.objective}});
                group_history.back()["application_rejected_candidate_checks"] = rejected_candidate_checks - rejects_before;
                std::clog << "GROUP_MIP_DONE iteration=" << mip_calls << " status=" << group_status
                          << " seconds=" << budget.elapsed() - begin << " best=" << out.objective
                          << " application_rejected_checks=" << rejected_candidate_checks - rejects_before << '\n';
                mip_seconds += budget.elapsed() - begin;
                stale_repairs = options.scheduleImproved(s.cost, out.objective, Tol) ? 0 : stale_repairs + 1;
            }

          public:
            Search(const Problem &p, Budget &budget, LocalSearchResult &out,
                   const LocalSearchOptions &settings = {}, bool demand_driven = false,
                   std::vector<int> original_blocks = {})
                : options(settings), p(p), budget(budget), pool(p.blocks.size()), seeded(p.blocks.size()),
                  b0(p.blocks.size()), bq(p.blocks.size()), h0(p.nodes.size()), hq(p.nodes.size()),
                  l0(p.capacities.size()), lq(p.capacities.size()), track(p.nodes.size()),
                  affected(p.blocks.size()), yard_affected(p.nodes.size()),
                  link_affected(p.capacities.size()), out(out), demand_driven(demand_driven),
                  random(settings.random_seed),
                  original_blocks(std::move(original_blocks)) {
                guided.resize(p.demands.size());
                open_path_generation.assign(p.demands.size(), std::numeric_limits<std::uint64_t>::max());
                capacity_levels = p.capacities;
                std::sort(capacity_levels.begin(), capacity_levels.end());
                capacity_levels.erase(std::unique(capacity_levels.begin(), capacity_levels.end()),
                                      capacity_levels.end());
                double ratio = p.arcs.empty() ? 0 : p.arcs[0].cost / p.arcs[0].length;
                transport_length_ratio = ratio;
                for (const auto &arc : p.arcs)
                    proportional_costs = proportional_costs && std::abs(arc.cost - ratio * arc.length) <=
                                                                   1e-12 * std::max(1., arc.cost);
                for (int i = 0; i < (int)p.nodes.size(); ++i)
                    if (p.nodes[i].yard)
                        yards.push_back(i);
                group_yards = yards;
                std::vector<double> importance(p.nodes.size());
                for (const auto &d : p.demands) {
                    importance[d.from] += d.quantity * d.penalty;
                    importance[d.to] += d.quantity * d.penalty;
                }
                std::stable_sort(group_yards.begin(), group_yards.end(),
                                 [&](int a, int b) { return importance[a] > importance[b]; });
                bundle_yards = group_yards;
                if (group_yards.size() > std::size_t(options.shared_hubs_per_sweep))
                    group_yards.resize(options.shared_hubs_per_sweep);
            }
            void scheduleEvent(const char *stage, int completed_rounds = 0, int stalls = 0) {
                schedule_history.push_back({{"stage", stage}, {"seconds", budget.elapsed()},
                                            {"objective", out.objective}, {"cycle", search_cycles},
                                            {"completed_rounds", completed_rounds}, {"stalls", stalls}});
                for (Json *trace : {&schedule_history, &group_history, &pool_model_history,
                                    &capacity_row_history, &path_generation_history})
                    while (trace->size() > std::size_t(options.schedule_trace_limit))
                        trace->erase(trace->begin());
                std::clog << "SCHEDULE stage=" << stage << " seconds=" << budget.elapsed()
                          << " completed_rounds=" << completed_rounds << " stalls=" << stalls
                          << " best=" << out.objective << '\n';
            }

            std::vector<int> selectReleasedDemands() {
                if (!options.benchmark_targeted_destroy) return {};
                const double total_limit = budget.seconds, begin = budget.elapsed();
                budget.seconds = std::min(total_limit, begin + options.destroy_trial_max_seconds);
                std::vector<int> released;
                try {
                    State current(p);
                    current.plan = out.plan;
                    for (int b = 0; b < int(p.blocks.size()); ++b)
                        if (!current.plan.physical[b].empty())
                            for (int r = 0; r < int(pool[b].size()); ++r)
                                if (pool[b][r].arcs == current.plan.physical[b]) current.route[b] = r;
                    rebuild(current);
                    std::vector<int> opened;
                    for (int b = 0; b < int(p.blocks.size()); ++b)
                        if (current.route[b] >= 0) opened.push_back(b);
                    std::stable_sort(opened.begin(), opened.end(), [&](int a, int b) {
                        return p.blocks[a].opening / std::max(Tol, current.load[a]) >
                               p.blocks[b].opening / std::max(Tol, current.load[b]);
                    });
                    repair_total = opened.size();
                    const std::size_t limit = stale_repairs >= options.expand_after_stalls
                                                  ? options.large_group_size : options.small_group_size;
                    for (std::size_t attempt = 0; attempt < opened.size(); ++attempt) {
                        budget.check();
                        const int b = opened[destroy_round++ % opened.size()];
                        State trial = current;
                        destroy(trial, b);
                        std::vector<int> changed;
                        for (int k = 0; k < int(p.demands.size()); ++k)
                            if (!current.plan.logical[k].empty() && trial.plan.logical[k].empty())
                                changed.push_back(k);
                        if (changed.empty() || changed.size() > limit) continue;
                        released = std::move(changed);
                        ++destroys;
                        ++repair_index;
                        for (int k : released)
                            if (std::find(guided[k].begin(), guided[k].end(), current.plan.logical[k]) ==
                                guided[k].end()) guided[k].push_back(current.plan.logical[k]);
                        break;
                    }
                } catch (const LimitReached &) {
                }
                budget.seconds = total_limit;
                repair_time += budget.elapsed() - begin;
                return released;
            }

            bool localSearchStage(double total_limit) {
                if (options.benchmark_local_repair) ++component_local_calls; else ++component_local_skips;
                if (!options.benchmark_local_repair) {
                    scheduleEvent("local_skipped");
                    return true;
                }
                const double begin = budget.elapsed();
                budget.seconds = std::min(total_limit, begin + options.joint_phase_max_seconds);
                stale_repairs = 0;
                int complete_calls = 0;
                scheduleEvent("local_start");
                try {
                    for (int round = 0; round < int(p.demands.size()) &&
                                        budget.remaining() > options.min_joint_seconds; ++round) {
                        const double before = out.objective;
                        auto released = selectReleasedDemands();
                        if (budget.remaining() <= options.min_joint_seconds) break;
                        const auto history_before = group_history.size();
                        improveDemandGroup(released);
                        if (group_history.size() == history_before) break;
                        ++complete_calls;
                        if (options.scheduleImproved(before, out.objective, Tol)) stale_repairs = 0;
                        if (stale_repairs >= options.joint_after_stalls) break;
                    }
                } catch (const LimitReached &) {
                }
                budget.seconds = total_limit;
                group_phase_seconds += budget.elapsed() - begin;
                scheduleEvent("local_return");
                return complete_calls > 0;
            }

            void run(bool construct = true) {
                const double restore_begin = budget.elapsed();
                std::clog << "PHASE_START phase=incumbent_restore seconds=" << restore_begin << '\n';
                State best(p);
                best.plan = out.plan;
                for (int b = 0; b < int(p.blocks.size()); ++b) {
                    if (out.plan.physical[b].empty()) continue;
                    progressLog("incumbent_restore");
                    ensureRoutes(b);
                    const auto &path = out.plan.physical[b];
                    int selected = -1;
                    for (int j = 0; j < int(pool[b].size()); ++j)
                        if (pool[b][j].arcs == path) selected = j;
                    if (selected < 0) {
                        Route route;
                        route.arcs = path;
                        route.unit = p.blocks[b].interchange;
                        route.capacity = p.blocks[b].upper;
                        std::map<int, int> links;
                        for (int a : path) {
                            route.unit += p.transport(b, a);
                            ++links[p.arcs[a].link];
                        }
                        for (auto [e, n] : links) {
                            route.links.emplace_back(e, n);
                            route.capacity = std::min(route.capacity, p.capacities[e] / n);
                        }
                        selected = int(pool[b].size());
                        pool[b].push_back(std::move(route));
                    }
                    best.route[b] = selected;
                }
                rebuild(best);
                for (int k = 0; k < int(p.demands.size()); ++k)
                    if (!out.plan.logical[k].empty()) guided[k].push_back(out.plan.logical[k]);
                incumbent_restore_seconds = budget.elapsed() - restore_begin;
                std::clog << "PHASE_END phase=incumbent_restore seconds=" << incumbent_restore_seconds << '\n';
                const double total_limit = budget.seconds;
                incremental_begin = last_global_improvement = budget.elapsed();
                incremental_active = true;
                std::clog << "PHASE_START phase=incremental seconds=" << incremental_begin
                          << " relative_tolerance=" << options.construction_relative_tolerance << '\n';
                try {
                    if (construct) descend(best, "construction");
                    else incremental_stop_reason = "supplied_incumbent";
                } catch (const LimitReached &) { /* out.plan always holds the last validated
                                                    improvement. */
                }
                incremental_active = false;
                incremental_seconds = construct ? budget.elapsed() - incremental_begin : 0;
                if (budget.remaining() <= 0)
                    incremental_stop_reason = "time_limit";
                std::clog << "PHASE_END phase=incremental seconds=" << incremental_seconds
                          << " reason=" << incremental_stop_reason << " best=" << out.objective
                          << " sweeps=" << sweeps << " repairs=" << destroys << '\n';
                if (options.benchmark_record_initial_pool) {
                    PathPool logical = guided, physical(p.blocks.size());
                    for (int k = 0; k < int(p.demands.size()); ++k)
                        if (!out.plan.logical[k].empty() &&
                            std::find(logical[k].begin(), logical[k].end(), out.plan.logical[k]) == logical[k].end())
                            logical[k].push_back(out.plan.logical[k]);
                    for (int b = 0; b < int(p.blocks.size()); ++b)
                        for (const auto &route : pool[b]) physical[b].push_back(route.arcs);
                    initial_candidate_pool = {{"scope", "After construction/restore; before local, PG and enrichment"},
                                              {"incumbent", out.plan.json()}, {"incumbent_objective", out.objective},
                                              {"logical_paths", logical}, {"physical_paths", physical},
                                              {"demand_driven", demand_driven}, {"original_block_ids", original_blocks}};
                }
                component_ctor_complete_seconds = budget.elapsed();
                if (component_ctor_observer) component_ctor_observer(*this);
                const bool use_patience = options.finite_stall_cycles > 0 &&
                                          !options.continuous_search && !options.pool_capture_only;
                int finite_stalls = 0;
                std::uint64_t joint_epoch = 0, local_epoch = 0, exposure_epoch = 0;
                bool pending_joint_return = false, patience_stopped = false;
                double pending_cycle_cost = out.objective;
                do {
                    ++search_cycles;
                    double cycle_cost = out.objective;
                    const int solves_before = mip_calls + pool_mip_calls + guidance_lp_calls;
                    const int global_solves_before = pool_mip_calls + guidance_lp_calls;
                    std::clog << "SEARCH_CYCLE iteration=" << search_cycles
                              << " seconds=" << budget.elapsed() << " best=" << out.objective << '\n';
                    const bool local_completed = localSearchStage(total_limit);
                    if (use_patience && pending_joint_return && local_completed) {
                        local_epoch = joint_epoch;
                        finite_stalls = out.objective < pending_cycle_cost - Tol ? 0 : finite_stalls + 1;
                        pending_joint_return = false;
                        scheduleEvent("post_joint_local_return");
                        cycle_cost = out.objective;
                    }
                    bool settle = use_patience && local_epoch == joint_epoch &&
                                  finite_stalls >= options.finite_stall_cycles;
                    const double settle_cost = out.objective;
                    if (settle) scheduleEvent("finite_patience_armed");
                    int overall_stalls = 0, completed_rounds = 0;
                    while (budget.remaining() > options.min_pool_seconds &&
                           overall_stalls < options.overall_stall_rounds) {
                        const double before = out.objective;
                        scheduleEvent("pricing_start", completed_rounds, overall_stalls);
                        if (options.benchmark_path_generation) {
                            ++component_pg_calls;
                            solvePathModel();
                        } else ++component_pg_skips;
                        scheduleEvent("pricing_return", completed_rounds, overall_stalls);
                        if (budget.remaining() <= options.min_joint_seconds ||
                            !localSearchStage(total_limit)) break;
                        ++completed_rounds;
                        ++pricing_rounds;
                        overall_stalls = options.scheduleImproved(before, out.objective, Tol) ? 0 : overall_stalls + 1;
                        scheduleEvent("pricing_round_complete", completed_rounds, overall_stalls);
                        if (settle) {
                            exposure_epoch = joint_epoch;
                            if (out.objective < settle_cost - Tol) {
                                finite_stalls = 0;
                                settle = false;
                            } else {
                                patience_stopped = true;
                                scheduleEvent("finite_patience_stop", completed_rounds, overall_stalls);
                                break;
                            }
                        }
                    }
                    if (patience_stopped) break;
                    if (overall_stalls < options.overall_stall_rounds ||
                        budget.remaining() <= options.min_pool_seconds) break;
                    budget.seconds = std::min(total_limit, budget.elapsed() + options.joint_repair_max_seconds);
                    scheduleEvent("joint_start", completed_rounds, overall_stalls);
                    const int joint_calls_before = pool_mip_calls;
                    bool enrichment_changed = false;
                    try {
                        if (options.benchmark_joint_repair) {
                            ++component_joint_calls;
                            enrichment_changed = improveAllDemands();
                        } else ++component_joint_skips;
                    } catch (const LimitReached &) {
                    }
                    scheduleEvent("joint_return", completed_rounds, overall_stalls);
                    budget.seconds = total_limit;
                    if (use_patience && pool_mip_calls > joint_calls_before) {
                        ++joint_epoch;
                        pending_joint_return = true;
                        pending_cycle_cost = cycle_cost;
                    }
                    if (options.pool_capture_only) break;
                    if (enrichment_changed) continue; // Expose new nondual candidates to local repair.
                    if (!options.continuous_search &&
                        pool_mip_calls + guidance_lp_calls == global_solves_before) break;
                    if (mip_calls + pool_mip_calls + guidance_lp_calls == solves_before) {
                        const double idle = std::min(options.continuous_idle_seconds, budget.remaining());
                        std::clog << "SEARCH_IDLE seconds=" << idle << " reason=no_new_subproblem\n";
                        std::this_thread::sleep_for(std::chrono::duration<double>(idle));
                    }
                } while (budget.remaining() > options.min_pool_seconds);
                std::size_t paths = 0, arcs = 0;
                for (const auto &rows : pool)
                    for (const auto &r : rows) {
                        ++paths;
                        arcs += r.arcs.size();
                    }
                out.profile = {{"schedule", "paper-nested-efficient-v1"},
                               {"non_cg_physical_search", "spp"},
                               {"schedule_history", schedule_history},
                               {"completed_pricing_rounds", pricing_rounds},
                               {"open_path_searches", open_path_searches},
                               {"open_paths_added", open_paths_added},
                               {"open_path_seconds", open_path_seconds},
                               {"open_path_cache_hits", open_path_cache_hits},
                               {"open_path_interruptions", open_path_interruptions},
                               {"open_path_skipped", open_path_skipped},
                               {"open_path_k_reached", open_path_k_reached},
                               {"open_path_exhausted", open_path_exhausted},
                               {"finite_stalls", finite_stalls},
                               {"finite_patience_stopped", patience_stopped},
                               {"joint_return_epoch", joint_epoch},
                               {"post_joint_local_epoch", local_epoch},
                               {"post_joint_exposure_epoch", exposure_epoch},
                               {"search_cycles", search_cycles},
                               {"trace_cycle", search_cycles},
                               {"pool_repeat_skips", pool_repeat_skips},
                               {"lp_repeat_skips", lp_repeat_skips},
                               {"incumbent_restore_seconds", incumbent_restore_seconds},
                               {"incremental_seconds", incremental_seconds},
                               {"incremental_stop_reason", incremental_stop_reason},
                               {"incremental_sweeps", sweeps},
                               {"group_phase_seconds", group_phase_seconds},
                               {"group_skipped_exhausted", exhausted_group_skips},
                               {"group_history", group_history},
                               {"pool_reduction", pool_reduction_audit},
                               {"fixing", fixed_assignment_audit},
                               {"pool_model_history", pool_model_history},
                               {"capacity_row_history", capacity_row_history},
                               {"seed_seconds", seed_time},
                               {"physical_searches", physical_searches},
                               {"physical_estimates", physical_estimates},
                               {"physical_route_recoveries", physical_route_recoveries},
                               {"physical_cache_hits", physical_cache_hits},
                               {"seed_blocks_completed", seed_blocks_completed},
                               {"move_seconds", move_time},
                               {"repair_seconds", repair_time},
                               {"evaluated_moves", evaluated},
                               {"evaluated_block_cells", evaluated_block_cells},
                               {"evaluated_yard_cells", evaluated_yard_cells},
                               {"evaluated_link_cells", evaluated_link_cells},
                               {"infeasible_moves", infeasible},
                               {"accepted_moves", accepted},
                               {"destroy_steps", destroys},
                               {"physical_paths", paths},
                               {"stored_path_arcs", arcs},
                               {"joint_mip_seconds", mip_seconds},
                               {"joint_mip_build_seconds", mip_build_seconds},
                               {"joint_mip_calls", mip_calls},
                               {"joint_mip_columns", mip_columns},
                               {"joint_mip_rows", mip_rows},
                               {"expanded_joint_repairs", expanded_repairs},
                               {"pool_lp_status", guidance_status},
                               {"pool_lp_objective", pool_objective},
                               {"guidance_seconds", guidance_seconds},
                               {"guidance_lp_calls", guidance_lp_calls},
                               {"path_generation_history", path_generation_history},
                               {"guidance_lp_seconds", guidance_lp_seconds},
                               {"guidance_build_seconds", guidance_build_seconds},
                               {"guidance_pricing_seconds", guidance_pricing_seconds},
                               {"rejected_candidate_checks", rejected_candidate_checks},
                               {"local_mip_integrality_focus", local_integrality_focus},
                               {"joint_mip_integrality_focus", joint_integrality_focus},
                               {"guided_logical_paths", guided_logical},
                               {"guided_physical_paths", 0},
                               {"support_generation_seconds", support_seconds},
                               {"support_paths_added", support_paths_added},
                               {"physical_alternative_seconds", physical_alternative_seconds},
                               {"physical_alternatives_added", physical_alternatives_added},
                               {"bridge_generation_seconds", bridge_generation_seconds},
                               {"joint_pool_logical_paths", joint_pool_logical_paths},
                               {"joint_pool_physical_paths", joint_pool_physical_paths},
                               {"pool_mip_solver_seconds", pool_mip_solver_seconds},
                               {"pool_mip_calls", pool_mip_calls},
                               {"pool_mip_continuations", pool_mip_continuations},
                               {"pool_mip_seconds", pool_mip_seconds},
                               {"pool_mip_build_seconds", pool_mip_build_seconds},
                               {"pool_mip_status", pool_mip_status},
                               {"pool_mip_conditional_bound", pool_mip_bound},
                               {"pool_mip_nodes", pool_mip_nodes}};
                if ((options.record_candidate_pool || options.pool_capture_only) && !candidate_pool.is_null())
                    out.profile["candidate_pool"] = std::move(candidate_pool);
                if (!initial_candidate_pool.is_null())
                    out.profile["initial_candidate_pool"] = std::move(initial_candidate_pool);
            }
        };
    } // namespace
    LocalSearchResult optimizeCandidatePaths(const Problem &p, const Plan &incumbent, const PathPool &logical,
                                             const PathPool &physical, Budget &budget,
                                             const LocalSearchOptions &options, bool path_generation_only) {
        if (options.pool_capture_only)
            throw std::invalid_argument("Candidate replay cannot capture another state");
        options.validate();
        LocalSearchResult out;
        out.plan = incumbent;
        out.objective = incumbent.validate(p);
        if (logical.size() != p.demands.size() || physical.size() != p.blocks.size())
            throw std::invalid_argument("Candidate path dimensions do not match");
        Search search(p, budget, out, options);
        search.guided = logical;
        for (int b = 0; b < (int)p.blocks.size(); ++b) {
            for (const auto &path : physical[b]) {
                Route route;
                route.arcs = path;
                route.unit = p.blocks[b].interchange;
                route.capacity = p.blocks[b].upper;
                std::map<int, int> links;
                int at = p.blocks[b].from;
                double length = 0;
                std::set<int> seen{at};
                for (int a : path) {
                    if (a < 0 || std::size_t(a) >= p.arcs.size() || p.arcs[a].from != at ||
                        !seen.insert(p.arcs[a].to).second)
                        throw std::invalid_argument("Invalid physical candidate");
                    at = p.arcs[a].to;
                    length += p.arcs[a].length;
                    route.unit += p.transport(b, a);
                    ++links[p.arcs[a].link];
                }
                if (at != p.blocks[b].to || length > p.blocks[b].length_limit + FeasTol)
                    throw std::invalid_argument("Invalid physical candidate endpoint or length");
                for (auto [e, n] : links) {
                    route.links.emplace_back(e, n);
                    route.capacity = std::min(route.capacity, p.capacities[e] / n);
                }
                search.pool[b].push_back(std::move(route));
            }
            if (!incumbent.physical[b].empty() &&
                std::find(physical[b].begin(), physical[b].end(), incumbent.physical[b]) == physical[b].end())
                throw std::invalid_argument("Incumbent physical candidate missing");
        }
        for (int k = 0; k < (int)p.demands.size(); ++k) {
            for (const auto &path : logical[k]) {
                int at = p.demands[k].from;
                std::set<int> seen{at};
                for (int b : path) {
                    if (b < 0 || std::size_t(b) >= p.blocks.size() || p.blocks[b].from != at ||
                        std::find(p.compatibleBlocks(k).begin(), p.compatibleBlocks(k).end(), b) ==
                            p.compatibleBlocks(k).end() ||
                        !seen.insert(p.blocks[b].to).second)
                        throw std::invalid_argument("Invalid logical candidate");
                    at = p.blocks[b].to;
                }
                if (at != p.demands[k].to)
                    throw std::invalid_argument("Invalid logical candidate endpoint");
            }
            if (incumbent.served[k] > Tol &&
                std::find(logical[k].begin(), logical[k].end(), incumbent.logical[k]) == logical[k].end())
                throw std::invalid_argument("Incumbent logical candidate missing");
        }
        const double input_seconds = budget.elapsed();
        if (path_generation_only) {
            search.solvePathModel(false, &logical);
            out.seconds = budget.elapsed();
            out.profile = {{"executed", search.guidance_lp_calls > 0}, {"status", search.guidance_status},
                           {"lp_calls", search.guidance_lp_calls}, {"lp_seconds", search.guidance_lp_seconds},
                           {"guidance_seconds", search.guidance_seconds},
                           {"guidance_build_seconds", search.guidance_build_seconds},
                           {"guidance_pricing_seconds", search.guidance_pricing_seconds},
                           {"path_generation_history", search.path_generation_history},
                           {"mip_calls", search.pool_mip_calls}, {"input_seconds", input_seconds},
                           {"candidate_pool", {{"logical_paths", search.guided}, {"physical_paths", physical},
                                               {"incumbent", incumbent.json()},
                                               {"incumbent_objective", out.objective}}}};
            return out;
        }
        search.solvePathModel(true, &logical);
        out.seconds = budget.elapsed();
        out.profile = {{"executed", search.pool_mip_calls > 0},
                       {"status", search.pool_mip_status},
                       {"columns", search.path_model_columns},
                       {"rows", search.path_model_rows},
                       {"conditional_bound", search.pool_mip_bound},
                       {"cutoff", incumbent.validate(p)},
                       {"pool_reduction", search.pool_reduction_audit},
                       {"fixing", search.fixed_assignment_audit},
                       {"pool_model_history", search.pool_model_history},
                       {"lp_seconds", search.guidance_seconds},
                       {"guidance_pricing_seconds", search.guidance_pricing_seconds},
                       {"rejected_candidate_checks", search.rejected_candidate_checks},
                       {"joint_mip_integrality_focus", search.joint_integrality_focus},
                       {"mip_build_seconds", search.pool_mip_build_seconds},
                       {"mip_solver_seconds", search.pool_mip_solver_seconds},
                       {"mip_calls", search.pool_mip_calls},
                       {"mip_continuations", search.pool_mip_continuations},
                       {"input_seconds", input_seconds}};
        if (options.record_candidate_pool)
            out.profile["candidate_pool"] = search.candidate_pool;
        const double validation_begin = budget.elapsed();
        if (!objectiveEqual(out.objective, out.plan.validate(p)))
            throw std::runtime_error("Candidate-path optimization validation failed");
        out.profile["validation_seconds"] = budget.elapsed() - validation_begin;
        return out;
    }
    namespace {
        // The original block universe is preserved. This is a temporary heuristic domain,
        // not an equivalent reduction or an optimality certificate.
        Problem proposalProblem(const Problem &p, Budget &budget, const LocalSearchOptions &options,
                                std::vector<int> &original_blocks, const Plan *incumbent) {
            Problem q;
            q.name = p.name;
            q.nodes = p.nodes;
            q.arcs = p.arcs;
            q.capacities = p.capacities;
            q.physical = p.physical;
            q.reverse_physical = p.reverse_physical;
            q.rejection = p.rejection;
            std::map<std::pair<int, int>, std::vector<int>> connections;
            auto between = [&](int k, int from, int to) {
                const auto key = std::make_pair(from, to);
                auto [it, added] = connections.emplace(key, std::vector<int>{});
                if (added)
                    for (int e = p.logical.offsets[from]; e < p.logical.offsets[from + 1]; ++e) {
                        int b = p.logical.edge_ids[e];
                        if (p.blocks[b].to == to)
                            it->second.push_back(b);
                    }
                std::vector<int> result;
                for (int b : it->second)
                    if (p.compatible(k, b))
                        result.push_back(b);
                return result;
            };
            std::vector<int> original_to_active(p.blocks.size(), -1);
            auto retain = [&](const std::vector<int> &ids) {
                for (int b : ids)
                    if (original_to_active[b] < 0) {
                        original_to_active[b] = int(q.blocks.size());
                        original_blocks.push_back(b);
                        q.blocks.push_back(p.blocks[b]);
                    }
            };
            if (incumbent)
                for (int b = 0; b < int(p.blocks.size()); ++b)
                    if (!incumbent->physical[b].empty()) retain({b});
            std::map<int, std::map<int, double>> hub_volume;
            for (int k = 0; k < int(p.demands.size()); ++k) {
                const auto &d = p.demands[k];
                int g = d.block_group < 0 ? -1 - k : d.block_group;
                hub_volume[g][d.from] += d.quantity;
                hub_volume[g][d.to] += d.quantity;
            }
            std::map<int, std::vector<int>> popular;
            for (const auto &[g, values] : hub_volume) {
                auto &hubs = popular[g];
                const auto &volumes = values;
                for (auto [h, volume] : values)
                    hubs.push_back(h);
                std::stable_sort(hubs.begin(), hubs.end(),
                                 [&](int a, int b) { return volumes.at(a) > volumes.at(b); });
                if (hubs.size() > std::size_t(options.shared_hubs_per_sweep))
                    hubs.resize(options.shared_hubs_per_sweep);
            }
            std::map<std::pair<int, int>, std::vector<int>> nearby;
            for (int k = 0; k < int(p.demands.size()); ++k) {
                budget.check();
                const auto &d = p.demands[k];
                retain(between(k, d.from, d.to));
                int g = d.block_group < 0 ? -1 - k : d.block_group;
                std::set<int> hubs(popular[g].begin(), popular[g].end());
                auto [near, new_origin] = nearby.emplace(std::make_pair(g, d.from), std::vector<int>{});
                if (new_origin) {
                    std::vector<std::pair<double, int>> ranked;
                    for (int e = p.logical.offsets[d.from]; e < p.logical.offsets[d.from + 1]; ++e) {
                        int b = p.logical.edge_ids[e];
                        if (p.compatible(k, b))
                            ranked.emplace_back(p.blocks[b].length_limit, p.blocks[b].to);
                    }
                    std::sort(ranked.begin(), ranked.end());
                    for (auto [length, h] : ranked)
                        if (std::find(near->second.begin(), near->second.end(), h) == near->second.end()) {
                            near->second.push_back(h);
                            if (near->second.size() >= std::size_t(options.proposal_hubs_per_demand))
                                break;
                        }
                }
                hubs.insert(near->second.begin(), near->second.end());
                for (int j = 0; j < int(p.demands.size()); ++j) {
                    const auto &other = p.demands[j];
                    if (other.from == d.from)
                        hubs.insert(other.to);
                    if (other.to == d.to)
                        hubs.insert(other.from);
                }
                struct Proposal {
                    double score;
                    int hub;
                    std::vector<int> blocks;
                };
                std::vector<Proposal> candidates;
                for (int h : hubs) {
                    if (h == d.from || h == d.to)
                        continue;
                    auto first = between(k, d.from, h), last = between(k, h, d.to);
                    if (first.empty() || last.empty())
                        continue;
                    double length =
                        p.blocks[first.front()].length_limit + p.blocks[last.front()].length_limit;
                    double support = d.quantity + hub_volume[g][h];
                    first.insert(first.end(), last.begin(), last.end());
                    candidates.push_back({support / std::max(1., length), h, std::move(first)});
                }
                std::stable_sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
                    return a.score > b.score || (a.score == b.score && a.hub < b.hub);
                });
                if (candidates.size() > std::size_t(options.proposal_hubs_per_demand))
                    candidates.resize(options.proposal_hubs_per_demand);
                for (const auto &candidate : candidates)
                    retain(candidate.blocks);
            }
            q.demands = p.demands;
            std::map<int, int> group_ids;
            for (int k = 0; k < int(p.demands.size()); ++k) {
                auto &d = q.demands[k];
                int original_group = d.block_group < 0 ? -1 - k : d.block_group;
                auto [it, added] = group_ids.emplace(original_group, int(q.compatibility_groups.size()));
                if (added) {
                    std::vector<int> members;
                    for (int a = 0; a < int(original_blocks.size()); ++a)
                        if (p.compatible(k, original_blocks[a]))
                            members.push_back(a);
                    q.compatibility_groups.push_back(std::move(members));
                }
                d.blocks.clear();
                d.block_group = it->second;
            }
            std::vector<int> from, to;
            for (const auto &block : q.blocks) {
                from.push_back(block.from);
                to.push_back(block.to);
            }
            q.logical.build(q.nodes.size(), from, to);
            q.indexCompatibility();
            budget.check();
            return q;
        }
    } // namespace
    LocalSearchResult solveLocalSearch(const Problem &p, Budget &budget, const LocalSearchOptions &options,
                                      const Plan *incumbent, bool construct) {
        options.validate();
        LocalSearchResult out;
        out.plan = incumbent ? *incumbent : Plan::empty(p);
        out.objective = incumbent ? incumbent->validate(p) : p.rejection;
        if (p.blocks.size() <= std::size_t(options.proposal_block_threshold)) {
            Search search(p, budget, out, options);
            search.run(construct);
        } else {
            const double begin = budget.elapsed();
            try {
                std::vector<int> original_blocks;
                Problem active = proposalProblem(p, budget, options, original_blocks, incumbent);
                const double proposal_seconds = budget.elapsed() - begin;
                std::clog << "ACTIVE_DOMAIN blocks=" << active.blocks.size() << '/' << p.blocks.size()
                          << " seconds=" << proposal_seconds << '\n';
                LocalSearchResult candidate;
                candidate.plan = Plan::empty(active);
                candidate.objective = out.objective;
                if (incumbent) {
                    std::map<int, int> active_ids;
                    for (int b = 0; b < int(original_blocks.size()); ++b) {
                        active_ids[original_blocks[b]] = b;
                        candidate.plan.physical[b] = incumbent->physical[original_blocks[b]];
                    }
                    candidate.plan.served = incumbent->served;
                    candidate.plan.logical = incumbent->logical;
                    for (auto &path : candidate.plan.logical)
                        for (int &b : path) b = active_ids.at(b);
                    if (!objectiveEqual(candidate.plan.validate(active), candidate.objective))
                        throw std::runtime_error("Active-domain incumbent mismatch");
                }
                Search search(active, budget, candidate, options, true, original_blocks);
                search.run(construct);
                out.objective = candidate.objective;
                out.history = std::move(candidate.history);
                out.plan.served = std::move(candidate.plan.served);
                out.plan.logical = std::move(candidate.plan.logical);
                for (auto &path : out.plan.logical)
                    for (int &b : path)
                        b = original_blocks[b];
                for (int b = 0; b < int(original_blocks.size()); ++b)
                    out.plan.physical[original_blocks[b]] = std::move(candidate.plan.physical[b]);
                out.profile = std::move(candidate.profile);
                if (out.profile.contains("candidate_pool")) {
                    auto &capture = out.profile["candidate_pool"];
                    for (auto &paths : capture["logical_paths"])
                        for (auto &path : paths)
                            for (auto &b : path)
                                b = original_blocks[b.get<int>()];
                    auto remap_physical = [&](Json &paths) {
                        Json expanded = Json::array();
                        for (std::size_t i = 0; i < p.blocks.size(); ++i)
                            expanded.push_back(Json::array());
                        for (int b = 0; b < int(original_blocks.size()); ++b)
                            expanded[original_blocks[b]] = std::move(paths[b]);
                        paths = std::move(expanded);
                    };
                    remap_physical(capture["physical_paths"]);
                    if (capture.contains("incumbent")) {
                        for (auto &path : capture["incumbent"]["logical_paths"])
                            for (auto &b : path)
                                b = original_blocks[b.get<int>()];
                        remap_physical(capture["incumbent"]["physical_paths"]);
                    }
                }
                out.profile["candidate_domain"] = {
                    {"scope", "Temporary demand-driven block candidates; not an exact reduction"},
                    {"original_blocks", p.blocks.size()},
                    {"active_blocks", active.blocks.size()},
                    {"original_block_ids", original_blocks},
                    {"proposal_seconds", proposal_seconds}};
            } catch (const LimitReached &) {
                out.profile["proposal_time_limit"] = true;
            }
        }
        out.seconds = budget.elapsed();
        if (!objectiveEqual(out.objective, out.plan.validate(p)))
            throw std::runtime_error("Local search final cost mismatch");
        return out;
    }
    LocalSearchResult solveLocalSearch(const Problem &p, Budget &budget) {
        return solveLocalSearch(p, budget, LocalSearchOptions{});
    }
} // namespace RouteOpt::Application::IRBP

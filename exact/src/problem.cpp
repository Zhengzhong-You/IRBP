/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/problem.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
namespace RouteOpt::Application::IRBP {
    void Graph::build(int nodes, const std::vector<int> &tails, const std::vector<int> &heads) {
        from = tails;
        to = heads;
        offsets.assign(nodes + 1, 0);
        edge_ids.resize(tails.size());
        for (int i : tails)
            ++offsets.at(i + 1);
        for (int i = 1; i <= nodes; ++i)
            offsets[i] += offsets[i - 1];
        auto cur = offsets;
        for (int a = 0; a < (int)tails.size(); ++a)
            edge_ids[cur[tails[a]]++] = a;
    }
    double Problem::transport(int b, int a) const {
        auto it = blocks[b].arc_cost.find(a);
        return it == blocks[b].arc_cost.end() ? blocks[b].transport_factor * arcs[a].cost : it->second;
    }
    Problem Problem::read(const std::string &path) {
        std::ifstream f(path);
        if (!f)
            throw std::runtime_error("Cannot open instance: " + path);
        Json j;
        f >> j;
        Problem p;
        p.name = j.at("name");
        for (auto &n : j.at("nodes"))
            p.nodes.push_back({n.at("yard"), n.value("tracks", 0.), n.value("handling_capacity", 0.),
                               n.value("handling_cost", 0.)});
        p.capacities = j.at("link_capacities").get<std::vector<double>>();
        for (auto &a : j.at("arcs"))
            p.arcs.push_back({a.at("from"), a.at("to"), a.at("link"), a.at("length"), a.at("cost")});
        for (auto &b : j.at("blocks")) {
            Block q{b.at("from"),
                    b.at("to"),
                    b.at("slots"),
                    b.at("lower"),
                    b.at("upper"),
                    b.at("length_limit"),
                    b.at("opening"),
                    b.at("interchange"),
                    b.value("transport_factor", 1.),
                    {}};
            if (b.contains("arc_cost"))
                for (auto it = b["arc_cost"].begin(); it != b["arc_cost"].end(); ++it)
                    q.arc_cost.emplace(std::stoi(it.key()), it.value());
            p.blocks.push_back(std::move(q));
        }
        for (auto &d : j.at("demands"))
            p.demands.push_back({d.at("from"), d.at("to"), d.at("quantity"), d.at("penalty"),
                                 d.at("blocks").get<std::vector<int>>()});
        auto nonneg = [](double x) {
            if (!std::isfinite(x) || x < 0)
                throw std::runtime_error("Expected finite nonnegative parameter");
        };
        auto node = [&](int i) {
            if (i < 0 || i >= (int)p.nodes.size())
                throw std::runtime_error("Invalid node");
        };
        for (auto &n : p.nodes) {
            nonneg(n.tracks);
            nonneg(n.handling_capacity);
            nonneg(n.handling_cost);
        }
        for (double c : p.capacities)
            nonneg(c);
        std::vector<std::pair<int, int>> links(p.capacities.size(), {-1, -1});
        std::vector<int> tails, heads;
        for (auto &a : p.arcs) {
            node(a.from);
            node(a.to);
            nonneg(a.cost);
            nonneg(a.length);
            if (a.from == a.to || a.length <= 0 || a.link < 0 || a.link >= (int)links.size())
                throw std::runtime_error("Invalid physical arc");
            std::pair<int, int> ends = std::minmax(a.from, a.to);
            if (links[a.link].first >= 0 && links[a.link] != ends)
                throw std::runtime_error("A physical link must connect the same two nodes");
            links[a.link] = ends;
            tails.push_back(a.from);
            heads.push_back(a.to);
        }
        p.physical.build(p.nodes.size(), tails, heads);
        p.reverse_physical.build(p.nodes.size(), heads, tails);
        tails.clear();
        heads.clear();
        for (auto &b : p.blocks) {
            node(b.from);
            node(b.to);
            if (!p.nodes[b.from].yard || !p.nodes[b.to].yard || b.from == b.to || b.lower <= 0 ||
                b.upper < b.lower || b.slots < 0 || b.slots > 1)
                throw std::runtime_error("Invalid block");
            for (double v : {b.lower, b.upper, b.length_limit, b.opening, b.interchange, b.transport_factor})
                nonneg(v);
            for (auto [a, c] : b.arc_cost) {
                if (a < 0 || a >= (int)p.arcs.size())
                    throw std::runtime_error("Invalid cost override");
                nonneg(c);
            }
            tails.push_back(b.from);
            heads.push_back(b.to);
        }
        p.logical.build(p.nodes.size(), tails, heads);
        p.block_demands.resize(p.blocks.size());
        for (int k = 0; k < (int)p.demands.size(); ++k) {
            auto &d = p.demands[k];
            node(d.from);
            node(d.to);
            if (!p.nodes[d.from].yard || !p.nodes[d.to].yard || d.from == d.to || d.quantity <= 0)
                throw std::runtime_error("Invalid demand");
            nonneg(d.quantity);
            nonneg(d.penalty);
            p.rejection += d.quantity * d.penalty;
            std::sort(d.blocks.begin(), d.blocks.end());
            if (std::adjacent_find(d.blocks.begin(), d.blocks.end()) != d.blocks.end())
                throw std::runtime_error("Duplicate compatibility");
            for (int b : d.blocks) {
                if (b < 0 || b >= (int)p.blocks.size())
                    throw std::runtime_error("Invalid compatible block");
                p.block_demands[b].push_back(k);
            }
        }
        return p;
    }
    Plan Plan::empty(const Problem &p) {
        return {std::vector<std::vector<int>>(p.demands.size()),
                std::vector<std::vector<int>>(p.blocks.size()), std::vector<double>(p.demands.size(), 0)};
    }
    double Plan::validate(const Problem &p) const {
        if (logical.size() != p.demands.size() || served.size() != p.demands.size() ||
            physical.size() != p.blocks.size())
            throw std::runtime_error("Plan dimensions");
        std::vector<double> loads(p.blocks.size()), links(p.capacities.size()), handles(p.nodes.size()),
            tracks(p.nodes.size());
        double obj = 0;
        auto bounded = [](double x, double u) {
            if (!std::isfinite(x) || x < -1e-6 || x > u + 1e-6 * std::max(1., u))
                throw std::runtime_error("Plan capacity violation");
        };
        auto traverse = [&](const std::vector<int> &edges, const Graph &g, int from, int to) {
            std::vector<bool> seen(p.nodes.size());
            seen[from] = true;
            int at = from;
            for (int a : edges) {
                if (a < 0 || a >= (int)g.from.size() || g.from[a] != at || seen[g.to[a]])
                    throw std::runtime_error("Plan path violation");
                at = g.to[a];
                seen[at] = true;
            }
            if (at != to)
                throw std::runtime_error("Plan disconnected path");
        };
        for (int k = 0; k < (int)p.demands.size(); ++k) {
            auto &d = p.demands[k];
            double q = served[k];
            bounded(q, d.quantity);
            obj += d.penalty * (d.quantity - q);
            if (logical[k].empty()) {
                if (q > 1e-6)
                    throw std::runtime_error("Volume without logical path");
                continue;
            }
            traverse(logical[k], p.logical, d.from, d.to);
            for (int b : logical[k]) {
                if (!std::binary_search(d.blocks.begin(), d.blocks.end(), b) || physical[b].empty())
                    throw std::runtime_error("Compatibility/activation violation");
                loads[b] += q;
                int i = p.blocks[b].from;
                if (i != d.from) {
                    handles[i] += q;
                    obj += q * p.nodes[i].handling_cost;
                }
            }
        }
        for (int b = 0; b < (int)p.blocks.size(); ++b) {
            auto &x = p.blocks[b];
            if (physical[b].empty()) {
                if (loads[b] > 1e-6)
                    throw std::runtime_error("Closed loaded block");
                continue;
            }
            traverse(physical[b], p.physical, x.from, x.to);
            if (loads[b] + 1e-6 < x.lower)
                throw std::runtime_error("Minimum block load");
            bounded(loads[b], x.upper);
            double length = 0;
            tracks[x.from] += x.slots;
            obj += x.opening + x.interchange * loads[b];
            for (int a : physical[b]) {
                length += p.arcs[a].length;
                links[p.arcs[a].link] += loads[b];
                obj += p.transport(b, a) * loads[b];
            }
            bounded(length, x.length_limit);
        }
        for (int i = 0; i < (int)p.nodes.size(); ++i) {
            bounded(handles[i], p.nodes[i].handling_capacity);
            bounded(tracks[i], p.nodes[i].tracks);
        }
        for (int e = 0; e < (int)links.size(); ++e)
            bounded(links[e], p.capacities[e]);
        return obj;
    }
    Json Plan::json() const {
        return {{"logical_paths", logical}, {"physical_paths", physical}, {"served", served}};
    }
    Json Stats::json() const {
        return {{"labels", labels},
                {"root_complete", root_complete},
                {"dominated", dominated},
                {"searches", searches},
                {"heuristic_searches", heuristic_searches},
                {"branch_opening", branches[0]},
                {"branching_seconds", branching_seconds},
                {"branch_copy_seconds", branch_copy_seconds},
                {"master_primal_solves", master_primal_solves},
                {"master_dual_solves", master_dual_solves},
                {"cut_dual_solves", cut_dual_solves},
                {"lp_barrier_solves", lp_barrier_solves},
                {"bkf_nodes", bkf_nodes},
                {"bkf_lp_requested", bkf_lp_requested},
                {"bkf_lp_selected", bkf_lp_tested},
                {"branch_lp_seconds", branch_lp_seconds},
                {"branch_lp_candidates", branch_lp_candidates},
                {"branch_lp_reused", branch_lp_reused},
                {"phase_one_nodes", phase_one_nodes},
                {"phase_one_infeasible", phase_one_infeasible},
                {"branch_physical", branches[1]},
                {"branch_logical", branches[2]},
                {"physical_reuse", reuse},
                {"cg_iterations", cg_iterations},
                {"nodes", nodes},
                {"activation_rows", activations},
                {"cover_rows", covers},
                {"flow_cover_rows", flow_covers},
                {"link_flow_cover_rows", flow_link_covers},
                {"yard_flow_cover_rows", flow_yard_covers},
                {"peak_labels", peak_labels},
                {"peak_label_bytes", peak_label_bytes},
                {"paths", paths},
                {"pool_cleanups", pool_cleanups},
                {"paths_removed", paths_removed},
                {"paths_generated", paths_generated},
                {"paths_regenerated", paths_regenerated},
                {"volume_bound_rows", volume_bound_rows},
                {"pricing_candidates", pricing_candidates},
                {"pricing_candidates_measure", bounded_candidate_collection ? "raw_negative_terminal_callbacks" : "legacy_counter"},
                {"candidate_retained_for_global_selection", candidate_retained},
                {"retained_candidate_duplicate_hits", candidate_duplicate_hits},
                {"max_paths_per_round", max_paths_per_round},
                {"peak_candidate_bytes", peak_candidate_bytes},
                {"columns", columns},
                {"rows", rows},
                {"pricing_seconds", pricing_seconds},
                {"label_work_seconds", label_work_seconds},
                {"master_seconds", master_seconds},
                {"last_master_status", last_master_status},
                {"build_seconds", build_seconds},
                {"heuristic_seconds", heuristic_seconds},
                {"heuristic_calls", heuristic_calls},
                {"heuristic_improvements", heuristic_improvements},
                {"heuristic_seed_paths", heuristic_seed_paths},
                {"heuristic_initial_objective", heuristic_initial_objective},
                {"separation_seconds", separation_seconds},
                {"root_bound", root_bound},
                {"root_seconds", root_seconds},
                {"time_best_solution", first_best}};
    }
    Json Result::json(const Problem &p) const {
        return {{"instance", p.name},
                {"method", method},
                {"status", status},
                {"objective", objective},
                {"lower_bound", bound},
                {"seconds", seconds},
                {"gap", (objective - bound) / std::max(1., std::abs(objective))},
                {"validated_objective", plan.validate(p)},
                {"stats", stats.json()},
                {"lp_certificate", lp_certificate},
                {"solution", plan.json()},
                {"feasibility_tolerance", FeasTol},
                {"pricing_relative_tolerance", RelativeRcTol},
                {"integrality_tolerance", IntTol}};
    }
} // namespace RouteOpt::Application::IRBP

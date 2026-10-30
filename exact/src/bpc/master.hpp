/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "irbp/model.hpp"
#include "irbp/pricing.hpp"
#include "irbp/cuts.hpp"
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace RouteOpt::Application::IRBP::BPC {
    struct Fix {
        int family{}, owner{}, item{}, value{};
    }; // opening, physical arc, logical block
    struct ExtraRow {
        int kind{}, owner{}, item{}, row{};
        std::vector<int> members;
        double excess{};
    }; // activation, cover, branches
    struct Aggregate {
        std::vector<double> open;
        std::vector<std::unordered_map<int, double>> logical, physical, links;
        std::vector<std::unordered_map<int, double>> link_volume, yard_selection, yard_volume;
    };
    inline std::uint64_t fingerprint(const Path &p) {
        std::uint64_t h = 1469598103934665603ULL;
        auto mix = [&](int v) {
            h ^= static_cast<std::uint64_t>(v);
            h *= 1099511628211ULL;
        };
        mix(p.physical);
        mix(p.owner);
        for (int e : p.edges)
            mix(e);
        return h;
    }
    inline bool uses(const Path &p, int e) {
        return std::find(p.edges.begin(), p.edges.end(), e) != p.edges.end();
    }
    class Master {
      public:
        const Problem &p;
        Model model;
        std::vector<Path> paths;
        std::vector<std::vector<int>> demand_paths, block_paths;
        std::vector<int> balance, link, handling, track, conv_d, conv_b;
        std::vector<ExtraRow> extras;
        std::vector<std::unordered_map<int, int>> activation;
        std::vector<std::vector<int>> d_extra, b_extra;
        std::map<std::tuple<int, int, int>, int> branch_rows;
        std::vector<int> active_branch;
        std::unordered_multimap<std::uint64_t, int> known;
        std::unordered_multimap<std::uint64_t, Path> removed_history;
        std::function<void(const Path &)> report_regenerated;
        Stats &stats;
        bool cover_cuts{true};
        double previous_lp_objective = std::numeric_limits<double>::quiet_NaN();
        explicit Master(const Problem &problem, int threads, Stats &s)
            : p(problem), model(threads, "IRBP-DW"), stats(s) {
            model.constant(p.rejection);
            model.var(1, 1); // Permanent RHS artificial column, index zero.
            auto coupling = [&](char sense, double rhs) {
                Sparse a;
                a.add(0, rhs);
                return model.row(a, sense, rhs);
            };
            for (std::size_t b = 0; b < p.blocks.size(); ++b)
                balance.push_back(coupling('=', 0));
            for (double c : p.capacities)
                link.push_back(coupling('<', c));
            for (auto &n : p.nodes) {
                handling.push_back(coupling('<', n.handling_capacity));
                track.push_back(coupling('<', n.tracks));
            }
            for (std::size_t k = 0; k < p.demands.size(); ++k)
                conv_d.push_back(coupling('<', 1));
            for (std::size_t b = 0; b < p.blocks.size(); ++b)
                conv_b.push_back(coupling('<', 1));
            demand_paths.resize(p.demands.size());
            block_paths.resize(p.blocks.size());
            activation.resize(p.demands.size());
            d_extra.resize(p.demands.size());
            b_extra.resize(p.blocks.size());
        }
        double extraCoefficient(const ExtraRow &r, const Path &c, bool volume = false) const {
            if (r.kind == 6 || r.kind == 7) {
                if (c.physical != (r.kind == 6) ||
                    !std::binary_search(r.members.begin(), r.members.end(), c.owner))
                    return 0;
                double incidence = 0;
                if (c.physical) {
                    for (int a : c.edges)
                        incidence += p.arcs[a].link == r.owner;
                } else if (p.demands[c.owner].from != r.owner) {
                    for (int b : c.edges)
                        incidence += p.blocks[b].from == r.owner;
                }
                const double upper = c.physical ? p.blocks[c.owner].upper : p.demands[c.owner].quantity;
                return incidence * (volume ? 1. : -std::max(0., upper - r.excess));
            }
            if (volume)
                return 0;
            if (r.kind == 0)
                return c.physical ? (c.owner == r.item ? -1. : 0.)
                                  : (c.owner == r.owner && uses(c, r.item) ? 1. : 0.);
            if (r.kind == 1) {
                if (!c.physical || !std::binary_search(r.members.begin(), r.members.end(), c.owner))
                    return 0;
                double v = 0;
                for (int a : c.edges)
                    v += p.arcs[a].link == r.owner;
                return v;
            }
            if (r.kind == 2)
                return c.physical && c.owner == r.owner ? 1 : 0;
            if (r.kind == 3)
                return c.physical && c.owner == r.owner && uses(c, r.item) ? 1 : 0;
            return !c.physical && c.owner == r.owner && uses(c, r.item) ? 1 : 0;
        }
        Sparse branchConstraint(const Fix &f, int value) const {
            Sparse row;
            row.add(0, value);
            ExtraRow test{f.family + 2, f.owner, f.item, 0, {}};
            for (const auto &path : paths) row.add(path.x, extraCoefficient(test, path));
            return row;
        }
        void prepareProbe(Model &, double) const {} // The DW master already has its penalty artificial.
        int addExtra(ExtraRow r, char sense, double rhs) {
            Sparse row;
            row.add(0, rhs);
            for (auto &path : paths) {
                row.add(path.x, extraCoefficient(r, path));
                row.add(path.v, extraCoefficient(r, path, true));
            }
            r.row = model.row(row, sense, rhs);
            int idx = extras.size();
            extras.push_back(r);
            if (r.kind == 0) {
                d_extra[r.owner].push_back(idx);
                b_extra[r.item].push_back(idx);
                activation[r.owner][r.item] = idx;
                ++stats.activations;
            } else if (r.kind == 1) {
                for (int b : r.members)
                    b_extra[b].push_back(idx);
                ++stats.covers;
            } else if (r.kind == 6 || r.kind == 7) {
                for (int owner : r.members)
                    (r.kind == 6 ? b_extra[owner] : d_extra[owner]).push_back(idx);
                ++stats.flow_covers;
                if (r.kind == 6)
                    ++stats.flow_link_covers;
                else
                    ++stats.flow_yard_covers;
            } else if (r.kind == 4)
                d_extra[r.owner].push_back(idx);
            else
                b_extra[r.owner].push_back(idx);
            return idx;
        }
        void activate(const std::vector<Fix> &fixes) {
            for (int r : active_branch) {
                auto id = extras[r].row;
                char sense = '<';
                double rhs = 1;
                check(model.solver.setRhs(id, 1, &sense, &rhs));
                int col = 0;
                check(model.solver.changeCoeffs(1, &id, &col, &rhs));
            }
            active_branch.clear();
            for (auto f : fixes) {
                auto key = std::make_tuple(f.family, f.owner, f.item);
                auto it = branch_rows.find(key);
                int r;
                if (it == branch_rows.end()) {
                    r = addExtra({f.family + 2, f.owner, f.item, 0, {}}, '=', f.value);
                    branch_rows[key] = r;
                } else {
                    r = it->second;
                    int id = extras[r].row, col = 0;
                    char sense = '=';
                    double rhs = f.value;
                    check(model.solver.setRhs(id, 1, &sense, &rhs));
                    check(model.solver.changeCoeffs(1, &id, &col, &rhs));
                }
                active_branch.push_back(r);
            }
        }
        void coefficients(const Path &c, Sparse &x, Sparse &v, double &cx, double &cv) const {
            cx = 0;
            cv = 0;
            if (c.physical) {
                int b = c.owner;
                auto &d = p.blocks[b];
                cx = d.opening;
                cv = d.interchange;
                x.add(conv_b[b], 1);
                x.add(track[d.from], d.slots);
                v.add(balance[b], 1);
                for (int a : c.edges) {
                    cv += p.transport(b, a);
                    v.add(link[p.arcs[a].link], 1);
                }
                for (int i : b_extra[b]) {
                    x.add(extras[i].row, extraCoefficient(extras[i], c));
                    v.add(extras[i].row, extraCoefficient(extras[i], c, true));
                }
            } else {
                int k = c.owner;
                auto &d = p.demands[k];
                cv = -d.penalty;
                x.add(conv_d[k], 1);
                for (int b : c.edges) {
                    v.add(balance[b], -1);
                    int i = p.blocks[b].from;
                    if (i != d.from) {
                        cv += p.nodes[i].handling_cost;
                        v.add(handling[i], 1);
                    }
                }
                for (int i : d_extra[k]) {
                    x.add(extras[i].row, extraCoefficient(extras[i], c));
                    v.add(extras[i].row, extraCoefficient(extras[i], c, true));
                }
            }
        }
        double jrc(const Path &c, double t, const std::vector<double> &dual) const {
            Sparse x, v;
            double cx, cv;
            coefficients(c, x, v, cx, cv);
            double ans = cx + t * cv;
            for (std::size_t j = 0; j < x.ind.size(); ++j)
                ans -= dual[x.ind[j]] * x.val[j];
            for (std::size_t j = 0; j < v.ind.size(); ++j)
                ans -= t * dual[v.ind[j]] * v.val[j];
            return ans;
        }
        void checkNewPath(const Path &c) const {
            auto range = known.equal_range(fingerprint(c));
            for (auto it = range.first; it != range.second; ++it) {
                const auto &old = paths[it->second];
                if (c.physical == old.physical && c.owner == old.owner && c.edges == old.edges) {
                    std::string message = "Duplicate " + std::string(c.physical ? "physical" : "logical") +
                        " path: owner=" + std::to_string(c.owner) + ", existing=" + std::to_string(it->second) + ", edges=";
                    for (int edge : c.edges) message += std::to_string(edge) + ",";
                    throw std::runtime_error(message);
                }
            }
        }
        bool addPath(Path c) {
            checkNewPath(c);
            const auto hash = fingerprint(c);
            Sparse x, v;
            double cx, cv;
            coefficients(c, x, v, cx, cv);
            c.x = model.var(cx, SOLVER_INFINITY, SOLVER_CONTINUOUS, x);
            c.v = model.var(cv, SOLVER_INFINITY, SOLVER_CONTINUOUS, v);
            const double upper = c.physical ? p.blocks[c.owner].upper : p.demands[c.owner].quantity;
            Sparse bound; bound.add(c.v, 1); bound.add(c.x, -upper);
            c.upper_row = model.row(bound, '<', 0);
            c.lower_row = -1;
            ++stats.volume_bound_rows;
            if (c.physical) {
                Sparse lower; lower.add(c.x, p.blocks[c.owner].lower); lower.add(c.v, -1);
                c.lower_row = model.row(lower, '<', 0);
                ++stats.volume_bound_rows;
            }
            auto range = removed_history.equal_range(hash);
            for (auto it = range.first; it != range.second; ++it) {
                const auto &old = it->second;
                if (c.physical == old.physical && c.owner == old.owner && c.edges == old.edges) {
                    ++stats.paths_regenerated;
                    if (report_regenerated) report_regenerated(c);
                    break;
                }
            }
            int idx = paths.size();
            (c.physical ? block_paths[c.owner] : demand_paths[c.owner]).push_back(idx);
            paths.push_back(std::move(c));
            known.emplace(hash, idx);
            stats.paths = paths.size();
            ++stats.paths_generated;
            return true;
        }
        int cleanPool(const std::vector<double> &dual) {
            const double objective = model.objective();
            const double previous = previous_lp_objective;
            previous_lp_objective = objective;
            if (!std::isfinite(previous)) return 0;
            if (paths.size() <= 10000) return 0;
            // Compare adjacent LP solves; small changes must not accumulate between cleanups.
            if (std::abs(objective - previous) <= pricingTolerance(objective)) return 0;
            const auto values = model.values();
            std::vector<std::pair<double, int>> ranked;
            for (int i = 0; i < (int)paths.size(); ++i) {
                const auto &c = paths[i];
                // Zero-valued basic columns may also be removed without changing primal feasibility.
                if (values[c.x] != 0 || values[c.v] != 0) continue;
                const double lower = c.physical ? p.blocks[c.owner].lower : 0;
                const double upper = c.physical ? p.blocks[c.owner].upper : p.demands[c.owner].quantity;
                ranked.emplace_back(std::min(jrc(c, lower, dual), jrc(c, upper, dual)), i);
            }
            const int count = std::min(paths.size() * 33 / 100, ranked.size());
            if (!count) return 0;
            std::partial_sort(ranked.begin(), ranked.begin() + count, ranked.end(),
                [](const auto &a, const auto &b) {
                    return a.first == b.first ? a.second < b.second : a.first > b.first;
                });
            std::vector<int> cols, rows, col_map(model.nvars), row_map(model.nrows);
            std::vector<char> removed(paths.size());
            for (int j = 0; j < count; ++j) {
                const int i = ranked[j].second;
                const auto &c = paths[i];
                removed[i] = true;
                const auto hash = fingerprint(c);
                auto range = removed_history.equal_range(hash);
                const bool recorded = std::any_of(range.first, range.second, [&](const auto &entry) {
                    const auto &old = entry.second;
                    return c.physical == old.physical && c.owner == old.owner && c.edges == old.edges;
                });
                if (!recorded) removed_history.emplace(hash, Path{c.physical, c.owner, c.edges});
                cols.push_back(c.x); cols.push_back(c.v);
                rows.push_back(c.upper_row);
                if (c.lower_row >= 0) rows.push_back(c.lower_row);
            }
            auto index_map = [](auto &map, const auto &deleted) {
                for (int i : deleted) map[i] = -1;
                int next = 0;
                for (int &i : map) if (i != -1) i = next++;
                return next;
            };
            model.nvars = index_map(col_map, cols);
            model.nrows = index_map(row_map, rows);
            check(model.solver.delVars(cols.size(), cols.data()));
            if (!rows.empty()) check(model.solver.delConstraints(rows.size(), rows.data()));
            check(model.solver.updateModel());
            for (auto *ids : {&balance, &link, &handling, &track, &conv_d, &conv_b})
                for (int &id : *ids) id = row_map[id];
            for (auto &r : extras) r.row = row_map[r.row];
            for (auto &ids : demand_paths) ids.clear();
            for (auto &ids : block_paths) ids.clear();
            known.clear();
            int next = 0;
            for (int i = 0; i < (int)paths.size(); ++i) {
                if (removed[i]) continue;
                auto &c = paths[i];
                c.x = col_map[c.x]; c.v = col_map[c.v];
                c.upper_row = row_map[c.upper_row];
                if (c.lower_row >= 0) c.lower_row = row_map[c.lower_row];
                (c.physical ? block_paths[c.owner] : demand_paths[c.owner]).push_back(next);
                known.emplace(fingerprint(c), next);
                if (i != next) paths[next] = std::move(c);
                ++next;
            }
            paths.resize(next);
            ++stats.pool_cleanups;
            stats.paths_removed += count;
            stats.paths = paths.size();
            return count;
        }
        Aggregate aggregate(const std::vector<double> &x) const {
            Aggregate a;
            a.open.resize(p.blocks.size());
            a.logical.resize(p.demands.size());
            a.physical.resize(p.blocks.size());
            a.links.resize(p.capacities.size());
            a.link_volume.resize(p.capacities.size());
            a.yard_selection.resize(p.nodes.size());
            a.yard_volume.resize(p.nodes.size());
            for (auto &c : paths) {
                double v = x[c.x];
                if (std::abs(v) < 1e-12)
                    continue;
                if (c.physical) {
                    a.open[c.owner] += v;
                    for (int e : c.edges) {
                        a.physical[c.owner][e] += v;
                        a.links[p.arcs[e].link][c.owner] += v;
                        a.link_volume[p.arcs[e].link][c.owner] += x[c.v];
                    }
                } else {
                    for (int b : c.edges) {
                        a.logical[c.owner][b] += v;
                        const int i = p.blocks[b].from;
                        if (i != p.demands[c.owner].from) {
                            a.yard_selection[i][c.owner] += v;
                            a.yard_volume[i][c.owner] += x[c.v];
                        }
                    }
                }
            }
            return a;
        }
        Plan plan(const std::vector<double> &x) const {
            Plan out = Plan::empty(p);
            for (auto &c : paths)
                if (x[c.x] > .5) {
                    if (c.physical)
                        out.physical[c.owner] = c.edges;
                    else {
                        out.logical[c.owner] = c.edges;
                        out.served[c.owner] = x[c.v];
                    }
                }
            return out;
        }
        int separate(const Aggregate &a, Budget &budget) {
            int added = 0;
            for (int k = 0; k < (int)p.demands.size(); ++k)
                for (auto [b, x] : a.logical[k])
                    if (x > a.open[b] + FeasTol && !activation[k].count(b)) {
                        addExtra({0, k, b, 0, {}}, '<', 0);
                        ++added;
                    }
            if (added || !cover_cuts) return added;
            {
                for (int e = 0; e < (int)p.capacities.size(); ++e) {
                    budget.check();
                    std::vector<int> ids;
                    for (auto [b, x] : a.links[e])
                        if (x > FeasTol)
                            ids.push_back(b);
                    std::sort(ids.begin(), ids.end(), [&](int i, int j) {
                        double x = a.links[e].at(i), y = a.links[e].at(j);
                        return x == y ? i < j : x > y;
                    });
                    std::vector<double> suffix(ids.size() + 1);
                    for (int j = (int)ids.size() - 1; j >= 0; --j)
                        suffix[j] = suffix[j + 1] + p.blocks[ids[j]].lower;
                    std::vector<int> chosen, cover;
                    std::uint64_t calls = 0;
                    std::function<bool(int, double, double)> dfs = [&](int j, double load, double cost) {
                        if ((++calls & 1023) == 0)
                            budget.check();
                        if (cost >= 1 - FeasTol)
                            return false;
                        if (load > p.capacities[e] + FeasTol) {
                            cover = chosen;
                            return true;
                        }
                        if (j == (int)ids.size() || load + suffix[j] <= p.capacities[e] + FeasTol)
                            return false;
                        int b = ids[j];
                        chosen.push_back(b);
                        if (dfs(j + 1, load + p.blocks[b].lower, cost + std::max(0., 1 - a.links[e].at(b))))
                            return true;
                        chosen.pop_back();
                        return dfs(j + 1, load, cost);
                    };
                    if (dfs(0, 0, 0)) {
                        std::sort(cover.begin(), cover.end());
                        bool exists = false;
                        for (auto &r : extras)
                            if (r.kind == 1 && r.owner == e && r.members == cover) {
                                exists = true;
                                break;
                            }
                        if (exists)
                            throw std::runtime_error("Enforced cover violated");
                        double rhs = cover.size() - 1;
                        addExtra({1, e, 0, 0, cover}, '<', rhs);
                        ++added;
                    }
                }
            }
            {
                auto flow_cut = [&](int kind, int resource, const auto &selection, const auto &volumes,
                                    double capacity) {
                    std::vector<FlowItem> items;
                    for (auto [owner, x] : selection)
                        if (x > FeasTol) {
                            const double upper = kind == 6 ? p.blocks[owner].upper
                                                           : p.demands[owner].quantity;
                            items.push_back({owner, upper, x, volumes.at(owner)});
                        }
                    auto cut = separateFlowCover(items, capacity, budget);
                    if (!cut)
                        return;
                    for (const auto &r : extras)
                        if (r.kind == kind && r.owner == resource && r.members == cut->members)
                            throw std::runtime_error("Enforced flow cover violated");
                    double rhs = capacity;
                    for (int owner : cut->members) {
                        const double upper = kind == 6 ? p.blocks[owner].upper
                                                       : p.demands[owner].quantity;
                        rhs -= std::max(0., upper - cut->excess);
                    }
                    addExtra({kind, resource, 0, 0, cut->members, cut->excess}, '<', rhs);
                    ++added;
                };
                    for (int e = 0; e < (int)p.capacities.size(); ++e)
                        flow_cut(6, e, a.links[e], a.link_volume[e], p.capacities[e]);
                    for (int i = 0; i < (int)p.nodes.size(); ++i)
                        if (p.nodes[i].yard)
                            flow_cut(7, i, a.yard_selection[i], a.yard_volume[i], p.nodes[i].handling_capacity);
            }
            return added;
        }
        Plan heuristic(Budget &budget, double seconds) {
            check(model.solver.updateModel());
            Model copy(1, "IRBP-incumbent");
            copy.copyFrom(model);
            check(copy.solver.setColUpper(0, 0));
            char binary = SOLVER_BINARY;
            for (auto &c : paths) check(copy.solver.setVTypeArray(c.x, 1, &binary));
            for (int k = 0; k < (int)p.demands.size(); ++k) {
                std::unordered_set<int> used;
                for (int pi : demand_paths[k])
                    for (int b : paths[pi].edges) used.insert(b);
                for (int b : used) if (!activation[k].count(b)) {
                    Sparse row;
                    for (int pi : demand_paths[k])
                        if (uses(paths[pi], b)) row.add(paths[pi].x, 1);
                    for (int pi : block_paths[b]) row.add(paths[pi].x, -1);
                    copy.row(row, '<', 0);
                }
            }
            check(copy.solver.updateModel());
            check(copy.solver.setEnvMethod(SOLVER_AUTOMATIC));
            check(copy.solver.setEnvTimeLimit(std::min(seconds, budget.remaining())));
            check(copy.solver.mipOptimize());
            int count;
            check(copy.solver.getSolCount(&count));
            Plan out = Plan::empty(p);
            if (count) { out = plan(copy.values()); out.validate(p); }
            return out;
        }

    };
}

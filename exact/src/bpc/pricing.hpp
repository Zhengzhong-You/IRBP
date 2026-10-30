/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "master.hpp"
#include "irbp/pricing_workers.hpp"
#include "candidate_pool.hpp"
#include "physical_spp.hpp"
#include <chrono>
#include <tuple>

namespace RouteOpt::Application::IRBP::BPC {
    struct PairCosts {
        double initial_x{}, initial_v{};
        std::vector<double> x, v;
        std::vector<char> allowed;
        std::vector<double> at(double t) const {
            std::vector<double> w(x.size());
            for (std::size_t i = 0; i < w.size(); ++i)
                w[i] = x[i] + t * v[i];
            return w;
        }
    };
    class Pricing {
        const Problem &p;
        Master &m;
        Budget &budget;
        Stats &stats;
        std::vector<double> length;
        struct Worker {
            Labeling labeling;
            Stats stats;
            std::unordered_map<int, std::vector<double>> bounds;
            std::map<std::pair<int,double>, PhysicalCostTree> cost_trees;
            PricingCandidatePool candidates;
            std::size_t peak_bytes{}, peak_labels{};
            explicit Worker(std::size_t bytes) : labeling(bytes) {}
        };
        struct BlockResult {
            double best{1e100};
        };
        std::vector<Worker> workers;
        std::vector<BlockResult> block_results;
        PricingWorkers team;

      public:
        Pricing(const Problem &problem, Master &master, Budget &b, Stats &s, std::size_t bytes,
                int worker_count)
            : p(problem), m(master), budget(b), stats(s), block_results(p.blocks.size()), team(worker_count) {
            stats.bounded_candidate_collection = true;
            workers.reserve(worker_count);
            for (int i = 0; i < worker_count; ++i)
                workers.emplace_back(bytes / worker_count);
            for (auto &a : p.arcs)
                length.push_back(a.length);
        }
        const std::vector<double> &completion(Worker &worker, int dest) {
            auto it = worker.bounds.find(dest);
            if (it != worker.bounds.end())
                return it->second;
            auto result = shortestLengths(p, dest, budget);
            return worker.bounds.emplace(dest, std::move(result)).first->second;
        }
        const PhysicalCostTree *nonnegativeCompletion(Worker &worker, int dest, double load,
                                                      const std::vector<double> &weights) {
            if (!std::all_of(weights.begin(), weights.end(), [](double v) { return v>=0; })) return nullptr;
            auto key=std::make_pair(dest,load);
            auto it=worker.cost_trees.find(key);
            if (it!=worker.cost_trees.end() && it->second.weights==weights) return &it->second;
            auto tree=physicalCostTree(p,dest,weights,budget);
            return &worker.cost_trees.insert_or_assign(key,std::move(tree)).first->second;
        }
        PairCosts costs(bool physical, int owner, const std::vector<double> &dual) const {
            PairCosts c;
            const int n = physical ? p.arcs.size() : p.blocks.size();
            c.x.resize(n);
            c.v.resize(n);
            if (!physical) {
                const auto &d = p.demands[owner];
                c.allowed.resize(n);
                c.initial_x = -dual[m.conv_d[owner]];
                c.initial_v = -d.penalty;
                for (int b : d.blocks) {
                    c.allowed[b] = 1;
                    c.v[b] = dual[m.balance[b]];
                    int i = p.blocks[b].from;
                    if (i != d.from)
                        c.v[b] += p.nodes[i].handling_cost - dual[m.handling[i]];
                }
                for (int ri : m.d_extra[owner]) {
                    const auto &r = m.extras[ri];
                    if (r.kind == 7) {
                        if (r.owner != d.from)
                            for (int b : d.blocks)
                                if (p.blocks[b].from == r.owner) {
                                    c.x[b] += dual[r.row] * std::max(0., d.quantity - r.excess);
                                    c.v[b] -= dual[r.row];
                                }
                    } else
                        c.x[r.item] -= dual[r.row];
                }
            } else {
                const auto &b = p.blocks[owner];
                c.initial_x = b.opening - dual[m.conv_b[owner]] - b.slots * dual[m.track[b.from]];
                c.initial_v = b.interchange - dual[m.balance[owner]];
                for (int a = 0; a < n; ++a)
                    c.v[a] = p.transport(owner, a) - dual[m.link[p.arcs[a].link]];
                for (int ri : m.b_extra[owner]) {
                    const auto &r = m.extras[ri];
                    if (r.kind == 0)
                        c.initial_x += dual[r.row];
                    else if (r.kind == 2)
                        c.initial_x -= dual[r.row];
                    else if (r.kind == 3)
                        c.x[r.item] -= dual[r.row];
                    else if (r.kind == 1) {
                        for (int a = 0; a < n; ++a)
                            if (p.arcs[a].link == r.owner)
                                c.x[a] -= dual[r.row];
                    } else if (r.kind == 6) {
                        for (int a = 0; a < n; ++a)
                            if (p.arcs[a].link == r.owner) {
                                c.x[a] += dual[r.row] * std::max(0., b.upper - r.excess);
                                c.v[a] -= dual[r.row];
                            }
                    }
                }
            }
            return c;
        }
        std::pair<int, double> run(const std::vector<double> &dual, bool heuristic) {
            const auto began = std::chrono::steady_clock::now();
            const double lp_value = m.model.objective();
            const double rc_tolerance = pricingTolerance(lp_value);
            for (auto &worker : workers) {
                worker.stats = {};
                worker.candidates.clear();
                worker.cost_trees.clear();
            }
            auto account = [&] {
                std::size_t bytes = 0, labels = 0, candidate_bytes = 0;
                for (auto &worker : workers) {
                    const auto &s = worker.stats;
                    stats.labels += s.labels;
                    stats.dominated += s.dominated;
                    stats.searches += s.searches;
                    stats.heuristic_searches += s.heuristic_searches;
                    stats.reuse += s.reuse;
                    stats.label_work_seconds += s.pricing_seconds;
                    stats.pricing_candidates += worker.candidates.callbacks;
                    stats.candidate_duplicate_hits += worker.candidates.duplicate_hits;
                    candidate_bytes += worker.candidates.peak_bytes;
                    worker.peak_bytes = std::max(worker.peak_bytes, s.peak_label_bytes);
                    worker.peak_labels = std::max(worker.peak_labels, s.peak_labels);
                    bytes += worker.peak_bytes;
                    labels += worker.peak_labels;
                }
                stats.peak_label_bytes = std::max(stats.peak_label_bytes, bytes);
                stats.peak_labels = std::max(stats.peak_labels, labels);
                stats.peak_candidate_bytes = std::max(stats.peak_candidate_bytes, candidate_bytes);
                stats.pricing_seconds +=
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
            };
            try {
                int added = 0;
                double correction = 0;
                double logical_min = 1e100, physical_min = 1e100;
                std::vector<PricingCandidate> candidates;
                auto accept = [&](PricingCandidatePool &target, Path path, double t, double val) {
                    if (val >= -rc_tolerance || path.edges.empty()) return;
                    const double actual = m.jrc(path, t, dual);
                    if (std::abs(actual - val) > 1e-6 * std::max(1., std::abs(val)))
                        throw std::runtime_error("Pricing coefficient mismatch");
                    if (actual >= -rc_tolerance) return;
                    m.checkNewPath(path);
                    target.retain(std::move(path), t, actual);
                };
                for (int k = 0; k < (int)p.demands.size(); ++k) {
                    budget.check();
                    const auto &d = p.demands[k];
                    const auto c = costs(false, k, dual);
                    auto &worker = workers[0];
                    auto search = [&](double t) {
                        const double initial = c.initial_x + t * c.initial_v;
                        auto best = worker.labeling.run(p.logical, d.from, d.to, c.at(t), {}, 1e100, {}, c.allowed,
                            budget, worker.stats, heuristic, -rc_tolerance - initial, [&](PricedPath path) {
                                const double value = initial + path.cost;
                                accept(worker.candidates, {false, k, std::move(path.edges)}, t, value);
                            });
                        return initial + best.cost;
                    };
                    double best = search(d.quantity);
                    bool zero = c.initial_x < 0;
                    for (int b : d.blocks) zero = zero || c.x[b] < 0;
                    best = std::min(best, zero ? search(0) : c.initial_x);
                    logical_min = std::min(logical_min, best);
                    correction += std::min(0., best);
                }
                team.run(p.blocks.size(), [&](int worker_id, int b) {
                    budget.check();
                    auto &worker = workers[worker_id];
                    auto &result = block_results[b];
                    result.best = 1e100;
                    const auto &d = p.blocks[b];
                    const auto c = costs(true, b, dual);
                    const auto &dist = completion(worker, d.to);
                    if (dist[d.from] > d.length_limit + 1e-10) return;
                    const bool reuse = std::all_of(c.x.begin(), c.x.end(), [](double v) { return v == 0; });
                    if (reuse) {
                        const double lo_initial = c.initial_x + d.lower * c.initial_v;
                        const double hi_initial = c.initial_x + d.upper * c.initial_v;
                        const auto weights=c.at(d.lower);
                        if (!heuristic) {
                            const auto *tree=nonnegativeCompletion(worker,d.to,d.lower,weights);
                            if (tree && tree->next[d.from]>=0 && tree->length[d.from]<=d.length_limit) {
                                double path_cost=0,path_length=0;int at=d.from;
                                for(std::size_t hops=0;at!=d.to && hops<p.nodes.size();++hops) {
                                    int a=tree->next[at];if(a<0) break;
                                    path_cost+=weights[a];path_length+=p.arcs[a].length;at=p.arcs[a].to;
                                }
                                const double best=std::min(lo_initial+path_cost,
                                    hi_initial+d.upper/d.lower*path_cost);
                                // A length-feasible unconstrained SPP is also constrained-optimal.
                                // Negative batches retain the original exact terminal collector.
                                if (at==d.to && path_length<=d.length_limit && best>=-rc_tolerance) {
                                    result.best=best;
                                    if (d.lower!=d.upper) ++worker.stats.reuse;
                                    return;
                                }
                            }
                        }
                        // The shared search must also collect paths negative only at the upper load.
                        const double threshold = std::max(-rc_tolerance - lo_initial,
                            d.lower / d.upper * (-rc_tolerance - hi_initial));
                        auto best = worker.labeling.run(p.physical, d.from, d.to, weights, length,
                            d.length_limit, dist, {}, budget, worker.stats, heuristic, threshold, [&](PricedPath path) {
                                const double lo = lo_initial + path.cost;
                                const double hi = hi_initial + d.upper / d.lower * path.cost;
                                accept(worker.candidates, {true, b, std::move(path.edges)},
                                       lo <= hi ? d.lower : d.upper, std::min(lo, hi));
                            });
                        result.best = std::min(lo_initial + best.cost, hi_initial + d.upper / d.lower * best.cost);
                        if (d.lower != d.upper) ++worker.stats.reuse;
                    } else {
                        for (int round = 0; round < (d.lower == d.upper ? 1 : 2); ++round) {
                            const double t = round ? d.upper : d.lower;
                            const double initial = c.initial_x + t * c.initial_v;
                            auto best = worker.labeling.run(p.physical, d.from, d.to, c.at(t), length,
                                d.length_limit, dist, {}, budget, worker.stats, heuristic,
                                -rc_tolerance - initial, [&](PricedPath path) {
                                    const double value = initial + path.cost;
                                    accept(worker.candidates, {true, b, std::move(path.edges)}, t, value);
                                });
                            result.best = std::min(result.best, initial + best.cost);
                        }
                    }
                });
                // Merge worker output deterministically before the global batch selection.
                for (auto &result : block_results) {
                    physical_min = std::min(physical_min, result.best);
                    correction += std::min(0., result.best);
                }
                std::size_t candidate_bytes = 0;
                for (auto &worker : workers) {
                    candidate_bytes += worker.candidates.peak_bytes;
                    worker.candidates.appendTo(candidates);
                }
                stats.candidate_retained += candidates.size();
                candidate_bytes += candidates.capacity() * sizeof(PricingCandidate);
                for (const auto &candidate : candidates) candidate_bytes += candidate.path.edges.capacity() * sizeof(int);
                auto identity = [](const PricingCandidate &c) { return candidateIdentity(c.path); };
                std::sort(candidates.begin(), candidates.end(), [&](const PricingCandidate &a, const PricingCandidate &b) {
                    return identity(a) == identity(b) ? a.cost < b.cost : identity(a) < identity(b);
                });
                candidates.erase(std::unique(candidates.begin(), candidates.end(), [&](const PricingCandidate &a, const PricingCandidate &b) {
                    return identity(a) == identity(b);
                }), candidates.end());
                stats.peak_candidate_bytes = std::max(stats.peak_candidate_bytes, candidate_bytes);
                std::sort(candidates.begin(), candidates.end(), [&](const PricingCandidate &a, const PricingCandidate &b) {
                    return a.cost == b.cost ? identity(a) < identity(b) : a.cost < b.cost;
                });
                if (candidates.size() > PricingBatchSize) candidates.resize(PricingBatchSize);
                for (auto &c : candidates) added += m.addPath(std::move(c.path));
                stats.max_paths_per_round = std::max(stats.max_paths_per_round, std::size_t(added));
                account();
                if (budget.report_pricing)
                    budget.report_pricing({stats.cg_iterations, stats.nodes, m.paths.size(), stats.paths_removed,
                                           stats.paths_regenerated, heuristic, added, int(m.extras.size()), lp_value,
                                           logical_min, physical_min, rc_tolerance, correction, budget.elapsed()});
                return {added, correction};
            } catch (...) {
                account();
                throw;
            }
        }
    };
}

/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#include "bpc/branching.hpp"
#include "single_cg/heuristic.hpp"
#include <queue>

namespace RouteOpt::Application::IRBP {
    namespace {
        struct SingleNode {
            double bound{};
            std::uint64_t id{};
            std::vector<BPC::Fix> fixes;
            BPC::Fix source{-1, 0, 0, 0};
            double parent_bound{};
            int parent_quota{1};
        };
        struct SingleOrder {
            bool operator()(const SingleNode &a, const SingleNode &b) const {
                return std::tie(a.bound, a.id) > std::tie(b.bound, b.id);
            }
        };
    } // namespace
    Result solveSingleCg(const Problem &p, Budget &budget, int threads, std::size_t label_budget,
                         const Plan *incumbent, const BpcOptions &options) try {
        if (options.pricing_workers < 1 || options.branch_workers < 1 || options.branch_workers > 64)
            throw std::runtime_error("Invalid pricing or branching worker count");
        Result result;
        result.method = "single-cg";
        result.plan = Plan::empty(p);
        result.objective = p.rejection;
        if (incumbent && incumbent->validate(p) < result.objective) {
            result.plan = *incumbent;
            result.objective = incumbent->validate(p);
        }
        SingleCG::SearchMaster master(p, budget, threads);
        SingleCG::Pricing pricing(master, budget, options.pricing_workers, label_budget);
        SingleCG::IncumbentSearch heuristic(master);
        BPC::Branching branching(master, budget, result.stats, options);
        std::priority_queue<SingleNode, std::vector<SingleNode>, SingleOrder> queue;
        queue.push({});
        std::uint64_t serial = 1;
        bool processing = false;
        double current = 0, retired = std::numeric_limits<double>::infinity();
        auto tolerance = [&] { return pricingTolerance(result.objective); };
        auto improve = [&](Plan plan) {
            double value = plan.validate(p);
            if (value < result.objective - 1e-7) {
                result.objective = value;
                result.plan = std::move(plan);
                result.stats.first_best = budget.elapsed();
            }
        };
        auto report = [&](const std::string &stage, double lp) {
            double lower = queue.empty() ? current : std::min(current, queue.top().bound);
            lower = std::min({lower, retired, result.objective});
            budget.emit({stage, result.stats.cg_iterations, result.stats.nodes, master.paths.size(),
                         master.m.nvars, master.m.nrows, lp, lower, result.objective, 0,
                         result.stats.master_seconds, pricing.stats.pricing_seconds});
        };
        auto solve = [&] {
            double start = budget.elapsed();
            int status = master.m.optimize(budget, false, SOLVER_BARRIER);
            result.stats.master_seconds += budget.elapsed() - start;
            return status;
        };
        auto price = [&](std::uint64_t node) {
            ++result.stats.cg_iterations;
            double lp = master.m.objective();
            auto [added, minimum, correction] = pricing.run();
            if (budget.report_pricing)
                budget.report_pricing({result.stats.cg_iterations, node, master.paths.size(), 0, 0, false,
                                       added, 0, lp, 0, minimum, pricingTolerance(lp), correction,
                                       budget.elapsed()});
            return std::make_tuple(added, minimum, correction);
        };
        try {
            pricing.seed();
            if (incumbent)
                for (int b = 0; b < (int)p.blocks.size(); ++b) {
                    const auto &edges = incumbent->physical[b];
                    if (edges.empty())
                        continue;
                    bool exists =
                        std::any_of(master.paths.begin(), master.paths.end(),
                                    [&](const Path &path) { return path.owner == b && path.edges == edges; });
                    if (!exists)
                        master.add(b, edges);
                }
            if (!options.root_only) {
                const double start = budget.elapsed();
                result.stats.heuristic_seed_paths = heuristic.seed(budget, 2.);
                improve(heuristic.construct(result.plan, budget));
                const double old = result.objective;
                improve(master.heuristic(budget, 2., result.plan, {}, true));
                ++result.stats.heuristic_calls;
                result.stats.heuristic_improvements += result.objective < old - 1e-7;
                result.stats.heuristic_initial_objective = result.objective;
                result.stats.heuristic_seconds += budget.elapsed() - start;
                report("INITIAL_UB", std::numeric_limits<double>::quiet_NaN());
            }
            result.stats.build_seconds = budget.elapsed();
            while (!queue.empty()) {
                budget.check();
                auto node = queue.top();
                queue.pop();
                current = node.bound;
                processing = true;
                if (current >= result.objective - tolerance()) {
                    retired = std::min(retired, current);
                    processing = false;
                    continue;
                }
                double node_start = budget.elapsed();
                master.activate(node.fixes);
                ++result.stats.nodes;
                bool infeasible = false;
                while (true) {
                    int status = solve();
                    if (status == SOLVER_MIP_INFEASIBLE || status == SOLVER_INF_OR_UNBD) {
                        master.phaseOne(true);
                        ++result.stats.phase_one_nodes;
                        while (true) {
                            status = solve();
                            if (status != SOLVER_OPTIMAL)
                                throw LimitReached(status == SOLVER_TIME_LIMIT ? "time_limit"
                                                                               : "phase_one_LP_unfinished");
                            double value = master.m.objective();
                            if (value <= 1e-8)
                                break;
                            auto [added, minimum, correction] = price(node.id);
                            report("FEASIBILITY", value);
                            if (added)
                                continue;
                            if (value + correction > FeasTol) {
                                infeasible = true;
                                ++result.stats.phase_one_infeasible;
                                break;
                            }
                            throw LimitReached("phase_one_pricing_tolerance");
                        }
                        master.phaseOne(false);
                        if (infeasible)
                            break;
                        continue;
                    }
                    if (status != SOLVER_OPTIMAL)
                        throw LimitReached(status == SOLVER_TIME_LIMIT  ? "time_limit"
                                           : status == SOLVER_MEM_LIMIT ? "master_memory_limit"
                                                                        : "LP_unfinished");
                    double lp = master.m.objective();
                    auto [added, minimum, correction] = price(node.id);
                    current = std::max(current, std::max(0., lp + correction));
                    report(node.id == 0 ? "ROOT_CG" : "TREE_CG", lp);
                    if (current >= result.objective - tolerance())
                        break;
                    if (added)
                        continue;
                    if (lp - current > tolerance())
                        throw LimitReached("pricing_tolerance");
                    break;
                }
                if (node.id == 0) {
                    result.stats.root_bound = current;
                    result.stats.root_seconds = budget.elapsed();
                    result.stats.root_complete = !infeasible && master.m.objective() - current <= tolerance();
                }
                if (infeasible) {
                    branching.close(result.objective - current);
                    processing = false;
                    continue;
                }
                if (options.root_only) {
                    result.status = result.stats.root_complete ? "root_complete" : "root_pricing_tolerance";
                    result.bound = std::min(current, result.objective);
                    processing = false;
                    break;
                }
                if (node.source.family >= 0)
                    branching.observe(node.source,
                                      std::max(0., std::min(current, result.objective) - node.parent_bound),
                                      node.fixes.size(), node.parent_quota);
                if (current >= result.objective - tolerance()) {
                    retired = std::min(retired, current);
                    branching.close(result.objective - current);
                    processing = false;
                    continue;
                }
                if (node.id == 0 || result.stats.nodes % 32 == 0) {
                    double start = budget.elapsed();
                    const double old = result.objective;
                    improve(node.id == 0 ? master.heuristic(budget, 2., result.plan)
                                         : heuristic.improve(result.plan, budget, 1.));
                    ++result.stats.heuristic_calls;
                    result.stats.heuristic_improvements += result.objective < old - 1e-7;
                    result.stats.heuristic_seconds += budget.elapsed() - start;
                    if (current >= result.objective - tolerance()) {
                        retired = std::min(retired, current);
                        branching.close(result.objective - current);
                        processing = false;
                        continue;
                    }
                }
                auto values = master.m.values();
                auto aggregate = master.aggregate(values);
                auto split = branching.choose(aggregate, result.objective, node.id, node.fixes.size(),
                                              budget.elapsed() - node_start);
                if (split.fix.family < 0) {
                    improve(master.recover(values));
                    if (current < result.objective - tolerance())
                        throw LimitReached("pricing_tolerance");
                    retired = std::min(retired, current);
                    branching.close(result.objective - current);
                } else {
                    ++result.stats.branches[split.fix.family];
                    for (int value : {0, 1}) {
                        auto fixes = node.fixes;
                        split.fix.value = value;
                        fixes.push_back(split.fix);
                        queue.push({current, serial++, std::move(fixes), split.fix, current, split.lp_quota});
                    }
                }
                processing = false;
            }
            if (!options.root_only) {
                result.status = "optimal";
                result.bound = std::min(result.objective, retired);
            }
        } catch (const LimitReached &e) {
            result.status = e.what();
            double lower = processing ? current : result.objective;
            if (!queue.empty())
                lower = std::min(lower, queue.top().bound);
            result.bound = std::min({lower, retired, result.objective});
        }
        auto &s = result.stats;
        const auto &ps = pricing.stats;
        s.searches = ps.searches;
        s.labels = ps.labels;
        s.dominated = ps.dominated;
        s.heuristic_searches = ps.heuristic_searches;
        s.pricing_seconds = ps.pricing_seconds;
        s.label_work_seconds = ps.label_work_seconds;
        s.peak_labels = ps.peak_labels;
        s.peak_label_bytes = ps.peak_label_bytes;
        s.peak_candidate_bytes = ps.peak_candidate_bytes;
        s.paths_generated = ps.paths_generated;
        s.max_paths_per_round = ps.max_paths_per_round;
        s.paths = master.paths.size();
        s.columns = master.m.nvars;
        s.rows = master.m.nrows;
        result.seconds = budget.elapsed();
        return result;
    } catch (const LimitReached &e) {
        Result result;
        result.method = "single-cg";
        result.status = e.what();
        result.plan = incumbent ? *incumbent : Plan::empty(p);
        result.objective = result.plan.validate(p);
        result.seconds = budget.elapsed();
        return result;
    }
} // namespace RouteOpt::Application::IRBP

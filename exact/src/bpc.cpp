/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "bpc/master.hpp"
#include "bpc/pricing.hpp"
#include "bpc/branching.hpp"
#include <queue>

namespace RouteOpt::Application::IRBP {
    using namespace BPC;
    namespace {
        struct SearchNode {
            double bound{};
            std::uint64_t serial{};
            std::vector<Fix> fixes;
            Fix source{-1, 0, 0, 0};
            double parent_bound{};
            int parent_lp_quota{1};
        };
        struct NodeOrder {
            bool operator()(const SearchNode &a, const SearchNode &b) const {
                return a.bound == b.bound ? a.serial > b.serial : a.bound > b.bound;
            }
        };
    } // namespace
    Result solveBpc(const Problem &p, Budget &budget, int threads, std::size_t label_budget,
                    const Plan *incumbent, const BpcOptions &options) {
        if (options.pricing_workers < 1)
            throw std::runtime_error("Positive pricing worker count required");
        if (options.branch_workers < 1 || options.branch_workers > 64)
            throw std::runtime_error("Use one to 64 LP-testing workers");
        Result result;
        result.method = "bpc";
        result.plan = Plan::empty(p);
        result.objective = p.rejection;
        if (incumbent) {
            const double value = incumbent->validate(p);
            if (value < result.objective) {
                result.objective = value;
                result.plan = *incumbent;
            }
        }
        Master master(p, threads, result.stats);
        master.cover_cuts = !options.dw_root;
        master.report_regenerated = budget.report_regenerated;
        Pricing pricing(p, master, budget, result.stats, label_budget, options.pricing_workers);
        Branching branching(master, budget, result.stats, options);
        result.stats.build_seconds = budget.elapsed();
        auto report = [&](Progress progress) {
            progress.lp_seconds = result.stats.master_seconds;
            progress.pricing_seconds = result.stats.pricing_seconds;
            budget.emit(progress);
        };
        std::priority_queue<SearchNode, std::vector<SearchNode>, NodeOrder> queue;
        queue.push({0, 0, {}});
        std::uint64_t serial = 1;
        bool processing = false;
        double current_bound = 0;
        auto improve = [&](Plan plan) {
            double obj = plan.validate(p);
            if (obj < result.objective - 1e-6) {
                result.objective = obj;
                result.plan = std::move(plan);
                result.stats.first_best = budget.elapsed();
            }
        };
        try {
            while (!queue.empty()) {
                budget.check();
                auto node = queue.top();
                queue.pop();
                current_bound = node.bound;
                processing = true;
                if (current_bound >= result.objective - 1e-6) {
                    processing = false;
                    continue;
                }
                const double node_begin = budget.elapsed();
                master.activate(node.fixes);
                ++result.stats.nodes;
                bool closed = false, branched = false, observed = false, after_cut = false;
                int next_method = node.serial == 0 ? SOLVER_PRIMAL_SIMPLEX : SOLVER_DUAL_SIMPLEX;
                while (!closed) {
                    budget.check();
                    double begin = budget.elapsed();
                    int status = master.model.optimize(budget, false, next_method);
                    result.stats.last_master_status = status;
                    const bool cut_reoptimization = after_cut;
                    if (next_method == SOLVER_PRIMAL_SIMPLEX) ++result.stats.master_primal_solves;
                    else ++result.stats.master_dual_solves;
                    if (after_cut) ++result.stats.cut_dual_solves;
                    next_method = options.dual_after_columns ? SOLVER_DUAL_SIMPLEX : SOLVER_PRIMAL_SIMPLEX;
                    after_cut = false;
                    result.stats.master_seconds += budget.elapsed() - begin;
                    ++result.stats.cg_iterations;
                    if (status != SOLVER_OPTIMAL)
                        throw LimitReached(
                            status == SOLVER_TIME_LIMIT
                                ? "time_limit"
                                : (status == SOLVER_MEM_LIMIT ? "master_memory_limit" : "master_not_optimal"));
                    const double lp_value = master.model.objective();
                    if (budget.report) {
                        const double global_lb = queue.empty() ? current_bound
                                                               : std::min(current_bound, queue.top().bound);
                        const char *stage = cut_reoptimization ? "CUT_LP" : node.serial == 0 ? "ROOT_CG" : "TREE";
                        report({stage, result.stats.cg_iterations,
                                     result.stats.nodes, master.paths.size(), master.model.nvars,
                                     master.model.nrows, lp_value, global_lb, result.objective});
                    }
                    auto dual = master.model.dual();
                    // Keep all generated paths during the convergence diagnostic.
                    auto heuristic_result = pricing.run(dual, true);
                    if (heuristic_result.first)
                        continue;
                    auto [added, correction] = pricing.run(dual, false);
                    if (added)
                        continue;
                    current_bound = std::max(current_bound, master.model.objective() + correction);
                    current_bound = std::max(0., current_bound);
                    if (node.serial == 0) {
                        result.stats.root_bound = current_bound;
                        result.stats.root_seconds = budget.elapsed();
                    }
                    if (current_bound >= result.objective - 1e-6) {
                        if (node.serial == 0)
                            result.stats.root_complete = true;
                        if (node.serial == 0 && options.root_only) {
                            Json logical = Json::array(), physical = Json::array();
                            std::vector<double> loads(p.blocks.size());
                            for (int k = 0; k < (int)p.demands.size(); ++k) {
                                if (result.plan.logical[k].empty()) continue;
                                logical.push_back({{"owner",k},{"path",result.plan.logical[k]},
                                                   {"x",1.},{"v",result.plan.served[k]}});
                                for (int b : result.plan.logical[k]) loads[b] += result.plan.served[k];
                            }
                            for (int b = 0; b < (int)p.blocks.size(); ++b)
                                if (!result.plan.physical[b].empty())
                                    physical.push_back({{"owner",b},{"path",result.plan.physical[b]},
                                                       {"x",1.},{"v",loads[b]}});
                            result.lp_certificate = {{"objective",result.objective},{"logical",logical},
                                {"physical",physical},{"extra_cover_cuts",master.cover_cuts},
                                {"proof_kind","feasible_upper_bound_equality"}};
                            result.status = "root_complete";
                            result.bound = current_bound;
                            result.seconds = budget.elapsed();
                            result.stats.paths = master.paths.size();
                            result.stats.columns = master.model.nvars;
                            result.stats.rows = master.model.nrows;
                            return result;
                        }
                        closed = true;
                        break;
                    }
                    auto x = master.model.values();
                    if (x[0] > 1e-6)
                        throw std::runtime_error("Artificial mass below incumbent");
                    auto aggregate = master.aggregate(x);
                    double separation_begin = budget.elapsed();
                    int new_rows = master.separate(aggregate, budget);
                    result.stats.separation_seconds += budget.elapsed() - separation_begin;
                    if (new_rows) {
                        next_method = SOLVER_DUAL_SIMPLEX;
                        after_cut = true;
                        continue;
                    }
                    if (node.serial == 0) {
                        result.stats.root_bound = current_bound;
                        result.stats.root_seconds = budget.elapsed();
                        result.stats.root_complete = master.model.objective() <= current_bound + 1e-6;
                    }
                    if (node.serial == 0 && options.root_only) {
                        if (options.dw_root) {
                            Json logical = Json::array(), physical = Json::array();
                            for (const auto &path : master.paths) if (std::abs(x[path.x]) > 1e-12 || std::abs(x[path.v]) > 1e-12) {
                                Json item = {{"owner",path.owner},{"path",path.edges},{"x",x[path.x]},{"v",x[path.v]}};
                                (path.physical ? physical : logical).push_back(item);
                            }
                            result.lp_certificate = {{"objective",master.model.objective()-x[0]},
                                {"logical",logical},{"physical",physical},{"extra_cover_cuts",false}};
                        }
                        result.status = result.stats.root_complete ? "root_complete" : "root_pricing_tolerance";
                        result.bound = current_bound;
                        result.seconds = budget.elapsed();
                        result.stats.paths = master.paths.size();
                        result.stats.columns = master.model.nvars;
                        result.stats.rows = master.model.nrows;
                        return result;
                    }
                    if (node.source.family >= 0) {
                        branching.observe(node.source,
                                          std::max(0., std::min(current_bound, result.objective) - node.parent_bound),
                                          node.fixes.size(), node.parent_lp_quota);
                        observed = true;
                    }
                    BranchCandidate split = branching.choose(aggregate, result.objective, node.serial,
                                                               node.fixes.size(), budget.elapsed() - node_begin);
                    if (split.fix.family < 0) {
                        improve(master.plan(x));
                        if (current_bound < result.objective - 1e-6)
                            throw LimitReached("pricing_tolerance");
                        closed = true;
                        break;
                    }
                    if (node.serial == 0 && !incumbent) {
                        double heuristic_begin = budget.elapsed();
                        improve(master.heuristic(budget, std::min(3., budget.remaining() * .1)));
                        result.stats.heuristic_seconds += budget.elapsed() - heuristic_begin;
                        if (current_bound >= result.objective - 1e-6) {
                            closed = true;
                            break;
                        }
                    }
                    branched = true;
                    ++result.stats.branches[split.fix.family];
                    for (int value : {0, 1}) {
                        auto fixes = node.fixes;
                        split.fix.value = value;
                        fixes.push_back(split.fix);
                        queue.push({current_bound, serial++, std::move(fixes), split.fix, current_bound,
                                    split.lp_quota});
                    }
                    closed = true;
                }
                if (node.source.family >= 0 && !observed)
                    branching.observe(node.source,
                                      std::max(0., std::min(current_bound, result.objective) - node.parent_bound),
                                      node.fixes.size(), node.parent_lp_quota);
                if (!branched)
                    branching.close(result.objective - current_bound);
                processing = false;
                const double global_lb = queue.empty() ? result.objective : queue.top().bound;
                report({"TREE", result.stats.cg_iterations, result.stats.nodes, master.paths.size(),
                             master.model.nvars, master.model.nrows,
                             std::numeric_limits<double>::quiet_NaN(), global_lb, result.objective});
            }
            result.status = "optimal";
            result.bound = result.objective;
        } catch (const LimitReached &e) {
            result.status = e.what();
            double lb = processing ? current_bound : result.objective;
            if (!queue.empty())
                lb = std::min(lb, queue.top().bound);
            result.bound = std::min(result.objective, lb);
        }
        result.seconds = budget.elapsed();
        result.stats.paths = master.paths.size();
        result.stats.columns = master.model.nvars;
        result.stats.rows = master.model.nrows;
        return result;
    }
} // namespace RouteOpt::Application::IRBP

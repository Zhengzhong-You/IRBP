/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "master.hpp"
#include "irbp/pricing_workers.hpp"
#include "irbp/branching_quota.hpp"
#include "bkf_controller.hpp"
#include "candidate_selector_controller.hpp"
#include <atomic>
#include <memory>
#include <numeric>

namespace RouteOpt::Application::IRBP::BPC {
    struct BranchCandidate {
        Fix fix{-1, 0, 0, 0};
        double fraction{}, score{};
        int lp_quota{1};
        std::array<double, 2> gains{};
    };
    template<class MasterType = Master> class Branching {
        using Key = std::tuple<int, int, int>;
        struct KeyHash {
            std::size_t operator()(const Key &k) const {
                auto [family, owner, item] = k;
                return (std::size_t(owner) * 1000003 + item) * 3 + family;
            }
        };
        using NativeHistory = RouteOpt::Branching::BranchingHistory<Key, KeyHash>;
        using NativeData = RouteOpt::Branching::BranchingDataShared<Key, KeyHash>;
        using NativeTester = RouteOpt::Branching::CandidateSelector::BranchingTesting<Branching, Key, KeyHash>;
        NativeHistory native_history;
        NativeData native_data{0};
        RouteOpt::Branching::BKF::BKFDataShared bkf_data;
        std::vector<RouteOpt::Branching::BKF::BKFController> bkf{1};
        MasterType &m;
        Budget &budget;
        Stats &stats;
        const BpcOptions &options;
        std::vector<std::unique_ptr<Model>> probes;
        std::unique_ptr<PricingWorkers> probe_team;
        static Key key(const Fix &f) { return {f.family, f.owner, f.item}; }
        std::vector<BranchCandidate> candidates(const Aggregate &a) const {
            std::vector<BranchCandidate> result;
            auto add = [&](int family, int owner, int item, double value) {
                if (std::min(value, 1 - value) > IntTol)
                    result.push_back({{family, owner, item, 0}, value, 0});
            };
            for (int family : std::array<int, 3>{0, 1, 2}) {
                if (family == 0) {
                    for (int b = 0; b < (int)a.open.size(); ++b)
                        add(0, b, 0, a.open[b]);
                } else {
                    const auto &values = family == 1 ? a.physical : a.logical;
                    for (int owner = 0; owner < (int)values.size(); ++owner)
                        for (auto [item, value] : values[owner])
                            add(family, owner, item, value);
                }
            }
            return result;
        }
        void testCandidates(std::vector<BranchCandidate> &all, double ub,
                            std::map<Key, std::array<double, 2>> &gains) {
            if (!probe_team) probe_team = std::make_unique<PricingWorkers>(options.branch_workers);
            // Identical rows have identical restricted-LP scores. Future columns may
            // distinguish them, so only testing is shared, never the actual branches.
            std::map<std::vector<std::pair<int, double>>, std::size_t> row_ids;
            std::vector<std::size_t> representatives, source(all.size());
            for (std::size_t j = 0; j < all.size(); ++j) {
                auto row = m.branchConstraint(all[j].fix, 0);
                row.canonicalize();
                std::vector<std::pair<int, double>> signature;
                for (std::size_t k = 0; k < row.ind.size(); ++k)
                    signature.emplace_back(row.ind[k], row.val[k]);
                auto [it, inserted] = row_ids.emplace(std::move(signature), j);
                source[j] = it->second;
                if (inserted) representatives.push_back(j);
            }
            stats.branch_lp_reused += all.size() - representatives.size();
            const int slots = std::min(options.branch_workers, 2 * (int)representatives.size());
            check(m.model.solver.updateModel());
            const double parent = m.model.objective(), copy_begin = budget.elapsed();
            const int lp_threads = branchLpThreads(m.model.threads(), options.branch_workers);
            std::vector<double> solve_seconds(slots);
            std::vector<std::uint64_t> solves(slots);
            for (int slot = 0; slot < slots; ++slot) {
                budget.check();
                if (!probes[slot]) probes[slot] = std::make_unique<Model>(lp_threads, "IRBP-probe");
                probes[slot]->copyFrom(m.model);
                auto &solver = probes[slot]->solver;
                check(solver.setEnvThreads(lp_threads, false));
                check(solver.setEnvCrossOver(SOLVER_CROSSOVER_DOWN));
                check(solver.setEnvLPWarmStart(0));
                m.prepareProbe(*probes[slot], ub);
            }
            stats.branch_copy_seconds += budget.elapsed() - copy_begin;
            std::atomic<std::size_t> next{0};
            std::atomic<bool> failed{false};
            std::exception_ptr interrupted;
            try {
                probe_team->run(slots, [&](int, int slot) {
                    auto &copy = *probes[slot];
                    try {
                        while (!failed.load(std::memory_order_relaxed)) {
                            const auto task = next.fetch_add(1, std::memory_order_relaxed);
                            if (task >= 2 * representatives.size()) break;
                            auto &candidate = all[representatives[task / 2]];
                            const int direction = task % 2;
                            const auto &f = candidate.fix;
                            budget.check();
                            Sparse row = m.branchConstraint(f, direction);
                            int row_id = copy.row(row, '=', direction);
                            const double begin = budget.elapsed();
                            const int status = copy.optimize(budget, false, SOLVER_BARRIER);
                            solve_seconds[slot] += budget.elapsed() - begin;
                            ++solves[slot];
                            if (status != SOLVER_OPTIMAL)
                                throw LimitReached(status == SOLVER_TIME_LIMIT ? "time_limit" :
                                                   "branch_lp_status_" + std::to_string(status));
                            candidate.gains[direction] = std::max(0., std::min(ub, copy.objective()) - parent);
                            check(copy.solver.delConstraints(1, &row_id));
                            --copy.nrows;
                            check(copy.solver.updateModel());
                        }
                    } catch (...) { failed.store(true, std::memory_order_relaxed); throw; }
                });
            } catch (...) { interrupted = std::current_exception(); }
            for (int slot = 0; slot < slots; ++slot) check(probes[slot]->solver.freeModel());
            stats.branch_lp_seconds += std::accumulate(solve_seconds.begin(), solve_seconds.end(), 0.);
            stats.lp_barrier_solves += std::accumulate(solves.begin(), solves.end(), std::uint64_t{0});
            if (interrupted) std::rethrow_exception(interrupted);
            stats.branch_lp_candidates += representatives.size();
            for (std::size_t j = 0; j < all.size(); ++j)
                all[j].gains = all[source[j]].gains;
            for (const auto &candidate : all) gains[key(candidate.fix)] = candidate.gains;
        }
        BranchCandidate routeOptChoice(const Aggregate &aggregate, double ub, std::uint64_t node,
                                        int depth, double node_seconds) {
            using Phase = RouteOpt::Branching::CandidateSelector::TestingPhase;
            const auto all = candidates(aggregate);
            if (all.empty())
                return {};
            std::map<Key, BranchCandidate> by_key;
            for (const auto &c : all)
                by_key.emplace(key(c.fix), c);
            std::array<std::vector<Key>, 3> screened;
            BranchingProgress report;
            report.node = node;
            report.depth = depth;
            // RouteOpt screens each family before BKF allocates the total testing budget.
            for (int family : std::array<int, 3>{0, 1, 2}) {
                auto &map = native_data.refCandidateMap();
                map.clear();
                for (const auto &c : all)
                    if (c.fix.family == family)
                        map.emplace(key(c.fix), c.fraction);
                native_history.initialScreen(native_data,
                    std::max(RootLpCandidates, MaxLpCandidatesPerWorker * options.branch_workers));
                screened[family] = native_data.getBranchPair();
                report.screened[family] = screened[family].size();
            }
            report.bkf_k = bkf[0].getBestK(bkf_data, ub, m.model.objective());
            report.lp_requested = lpTestingBudget(report.bkf_k, options.branch_workers, node == 0);
            report.lp = branchingQuota(report.screened, report.lp_requested);
            std::vector<BranchCandidate> lp_candidates;
            for (int family : std::array<int, 3>{0, 1, 2})
                for (int j = 0; j < report.lp[family]; ++j)
                    lp_candidates.push_back(by_key.at(screened[family][j]));
            auto &pairs = native_data.refBranchPair();
            pairs.clear();
            for (const auto &c : lp_candidates)
                pairs.push_back(key(c.fix));
            std::map<Key, std::array<double, 2>> lp_gains;
            NativeTester tester(RootLpCandidates, 1, 1, 1,
                [&](Branching *, const Key &key, double &down, double &up) {
                    const auto &g = lp_gains.at(key);
                    down = std::max(RouteOpt::TOLERANCE, g[0]);
                    up = std::max(RouteOpt::TOLERANCE, g[1]);
                }, {}, {});
            double begin = budget.elapsed();
            if (lp_candidates.size() > 1)
                testCandidates(lp_candidates, ub, lp_gains);
            tester.testing(this, native_history, native_data, Phase::LP);
            report.lp_seconds = budget.elapsed() - begin;
            tester.refLPTimeCnt() = {report.lp_seconds, lp_candidates.size() > 1 ? 2 * (int)lp_candidates.size() : 0};
            tester.updateBKFtime(node_seconds, bkf);
            bkf[0].updateTimeMeasure(bkf_data);
            bkf_data.increaseNodeB4();
            auto winner = by_key.at(pairs.front());
            winner.lp_quota = lp_candidates.size();
            if (!tester.getEdgeInfo().empty()) {
                for (const auto &info : tester.getEdgeInfo())
                    if (info.brc == pairs.front())
                        winner.score = info.score;
            }
            ++native_history.branch_choice[pairs.front()];
            report.winner = winner.fix.family;
            report.score = std::sqrt(std::max(0., winner.score));
            report.r_best = bkf_data.getCurrentRBest();
            ++stats.bkf_nodes;
            stats.bkf_lp_requested += report.lp_requested;
            for (int family = 0; family < 3; ++family) {
                stats.bkf_lp_tested[family] += report.lp[family];
            }
            if (budget.report_testing)
                budget.report_testing(report);
            return winner;
        }

      public:
        Branching(MasterType &master, Budget &time, Stats &s, const BpcOptions &config)
            : m(master), budget(time), stats(s), options(config), probes(config.branch_workers) {
            bkf[0].setMN(MaxLpCandidatesPerWorker, RootLpCandidates);
            bkf[0].setIfB4(true);
        }
        ~Branching() {
            if (probe_team) probe_team->stop();
        }
        void close(double gap) { bkf_data.updateF(gap); }
        void observe(const Fix &f, double gain, int depth, int parent_quota) {
            native_history.recordExactPerScore(key(f), 0, gain, f.value, depth);
            bkf[0].updateOptK(parent_quota, 0);
            bkf_data.calculateRStar(gain, depth, f.value, 0, bkf[0]);
        }
        BranchCandidate choose(const Aggregate &a, double ub, std::uint64_t node, int depth, double node_seconds) {
            const double begin = budget.elapsed();
            try {
                auto selected = routeOptChoice(a, ub, node, depth, node_seconds);
                stats.branching_seconds += budget.elapsed() - begin;
                return selected;
            } catch (...) {
                stats.branching_seconds += budget.elapsed() - begin;
                throw;
            }
        }
    };
}

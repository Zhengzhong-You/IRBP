/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#pragma once
#include "../bpc/master.hpp"
#include "irbp/single_cg.hpp"

namespace RouteOpt::Application::IRBP::SingleCG {
    using BPC::Aggregate;
    using BPC::Fix;
    class SearchMaster : public Master {
        struct BranchRow {
            Fix fix;
            int row;
        };
        std::vector<BranchRow> branches;
        std::map<std::tuple<int, int, int>, int> branch_ids;
        std::vector<int> active;
        std::vector<double> original_objective;

      public:
        Model &model;
        int artificial{-1};
        SearchMaster(const Problem &p, Budget &budget, int threads) : Master(p, budget, threads), model(m) {
            for (int b = 0; b < (int)p.blocks.size(); ++b) {
                if (p.blocks[b].lower <= 0)
                    continue;
                Sparse row;
                row.add(y[b], 1);
                for (int k : p.block_demands[b])
                    row.add(xd[k][b], -std::min(1., p.demands[k].quantity / p.blocks[b].lower));
                m.row(row, '<', 0);
            }
            for (int k = 0; k < (int)p.demands.size(); ++k)
                for (int b : p.demands[k].blocks) {
                    Sparse upper;
                    upper.add(g[k][b], 1);
                    upper.add(v[k], -1);
                    m.row(upper, '<', 0);
                    Sparse lower = upper;
                    lower.add(z[k], p.demands[k].quantity);
                    lower.add(xd[k][b], -p.demands[k].quantity);
                    m.row(lower, '>', 0);
                }
            check(m.solver.updateModel());
            std::vector<double> rhs(m.nrows);
            check(m.solver.getRhs(0, m.nrows, rhs.data()));
            Sparse column;
            for (int r = 0; r < m.nrows; ++r)
                column.add(r, rhs[r]);
            artificial = m.var(0, 0, SOLVER_CONTINUOUS, column);
            check(m.solver.setEnvCrossOver(0));
            check(m.solver.setEnvLPWarmStart(0));
            check(m.solver.setEnvDualReductions(0));
        }
        Sparse branchConstraint(const Fix &f, int value) const {
            Sparse row;
            row.add(artificial, value);
            if (f.family == 0)
                row.add(y.at(f.owner), 1);
            else if (f.family == 2)
                row.add(xd.at(f.owner).at(f.item), 1);
            else
                for (const auto &path : paths)
                    if (path.owner == f.owner && BPC::uses(path, f.item))
                        row.add(path.x, 1);
            return row;
        }
        void activate(const std::vector<Fix> &fixes) {
            if (phase_one)
                throw std::runtime_error("Cannot change nodes during feasibility pricing");
            for (int index : active) {
                int row = branches[index].row;
                char sense = '<';
                double rhs = 2;
                check(m.solver.setRhs(row, 1, &sense, &rhs));
                check(m.solver.changeCoeffs(1, &row, &artificial, &rhs));
            }
            active.clear();
            std::fill(closed_blocks.begin(), closed_blocks.end(), false);
            for (auto fix : fixes) {
                auto key = std::make_tuple(fix.family, fix.owner, fix.item);
                auto [it, inserted] = branch_ids.emplace(key, branches.size());
                if (inserted) {
                    int row = m.row(branchConstraint(fix, fix.value), '=', fix.value);
                    branches.push_back({fix, row});
                    if (fix.family == 1)
                        incidence_rows[fix.owner].emplace_back(fix.item, row);
                } else {
                    int row = branches[it->second].row;
                    char sense = '=';
                    double rhs = fix.value;
                    check(m.solver.setRhs(row, 1, &sense, &rhs));
                    check(m.solver.changeCoeffs(1, &row, &artificial, &rhs));
                }
                active.push_back(it->second);
                if (fix.family == 0 && fix.value == 0)
                    closed_blocks[fix.owner] = true;
            }
        }
        void phaseOne(bool enabled) {
            if (enabled == phase_one)
                return;
            check(m.solver.updateModel());
            if (enabled) {
                original_objective.resize(m.nvars);
                check(m.solver.getObj(0, m.nvars, original_objective.data()));
                std::vector<double> costs(m.nvars);
                costs[artificial] = 1;
                check(m.solver.changeObj(0, m.nvars, costs.data()));
                m.constant(0);
                check(m.solver.setColUpper(artificial, 1));
            } else {
                const int previous = original_objective.size();
                original_objective.resize(m.nvars, 0);
                for (const auto &path : paths)
                    if (path.v >= previous)
                        for (int a : path.edges)
                            original_objective[path.v] += p.transport(path.owner, a);
                check(m.solver.changeObj(0, m.nvars, original_objective.data()));
                m.constant(p.rejection);
                check(m.solver.setColUpper(artificial, 0));
            }
            phase_one = enabled;
        }
        void prepareProbe(Model &probe, double ub) const {
            // This penalty is used only to rank branches, never to certify a node bound.
            check(probe.solver.setColUpper(artificial, 1));
            double penalty = std::max(1., ub);
            check(probe.solver.changeObj(artificial, 1, &penalty));
        }
        Aggregate aggregate(const std::vector<double> &values) const {
            Aggregate a;
            a.open.resize(y.size());
            a.logical.resize(p.demands.size());
            a.physical.resize(p.blocks.size());
            for (int b = 0; b < (int)y.size(); ++b)
                a.open[b] = values[y[b]];
            for (int k = 0; k < (int)p.demands.size(); ++k)
                for (int b : p.demands[k].blocks)
                    if (values[xd[k][b]] > IntTol)
                        a.logical[k][b] = values[xd[k][b]];
            for (const auto &path : paths)
                if (values[path.x] > 1e-10)
                    for (int arc : path.edges)
                        a.physical[path.owner][arc] += values[path.x];
            return a;
        }
        Plan heuristic(Budget &budget, double seconds, const Plan &incumbent,
                       const std::vector<char> &free = {}, bool direct_only = false) {
            check(m.solver.updateModel());
            Model copy(1, "IRBP-single-incumbent");
            copy.copyFrom(m);
            // The incumbent neighborhood is global, independent of the current tree node.
            for (int index : active) {
                char sense = '<';
                double rhs = 2;
                check(copy.solver.setRhs(branches[index].row, 1, &sense, &rhs));
            }
            if (!free.empty()) {
                auto fix = [&](int col, double value) {
                    check(copy.solver.setColLower(col, value));
                    check(copy.solver.setColUpper(col, value));
                };
                for (int k = 0; k < (int)p.demands.size(); ++k)
                    if (!free[k]) {
                        fix(v[k], incumbent.served[k]);
                        fix(z[k], !incumbent.logical[k].empty());
                        for (int b : p.demands[k].blocks)
                            fix(xd[k][b], std::find(incumbent.logical[k].begin(), incumbent.logical[k].end(),
                                                    b) != incumbent.logical[k].end());
                    }
            }
            if (direct_only)
                for (int k = 0; k < (int)p.demands.size(); ++k)
                    for (int b : p.demands[k].blocks)
                        if (p.blocks[b].from != p.demands[k].from || p.blocks[b].to != p.demands[k].to)
                            check(copy.solver.setColUpper(xd[k][b], 0));
            char type = SOLVER_BINARY;
            for (int col : binary)
                check(copy.solver.setVTypeArray(col, 1, &type));
            check(copy.solver.setColUpper(artificial, 0));
            check(copy.solver.clearMIPStarts());
            check(copy.solver.setEnvCutoff(incumbent.validate(p)));
            check(copy.solver.setEnvMethod(SOLVER_AUTOMATIC));
            check(copy.solver.setEnvTimeLimit(std::min(seconds, budget.remaining())));
            check(copy.solver.mipOptimize());
            int count = 0;
            check(copy.solver.getSolCount(&count));
            return count ? recover(copy.values()) : incumbent;
        }
    };
} // namespace RouteOpt::Application::IRBP::SingleCG

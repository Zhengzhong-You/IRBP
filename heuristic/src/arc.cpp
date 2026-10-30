/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/model.hpp"
#include <cmath>
namespace RouteOpt::Application::IRBP {
    namespace {
        struct CallbackData {
            Budget *budget;
            double best;
            double first_best{};
        };
        void incumbentCallback(double objective, void *user) {
            auto &d = *static_cast<CallbackData *>(user);
            if (objective < d.best - 1e-6) {
                d.best = objective;
                d.first_best = d.budget->elapsed();
            }
        }
        std::vector<int> recover(const Graph &g, int source, int sink, const std::vector<int> &indices,
                                 const std::vector<double> &x) {
            std::vector<int> path;
            int cur = source;
            for (int step = 0; step < g.size() && cur != sink; ++step) {
                int found = -1;
                for (int e = g.offsets[cur]; e < g.offsets[cur + 1]; ++e) {
                    int a = g.edge_ids[e];
                    if (indices[a] >= 0 && x[indices[a]] > .5) {
                        found = a;
                        break;
                    }
                }
                if (found < 0)
                    throw std::runtime_error("Cannot recover integral arc path");
                path.push_back(found);
                cur = g.to[found];
            }
            if (cur != sink)
                throw std::runtime_error("Cyclic integral arc path");
            return path;
        }
    } // namespace
    Result solveArc(const Problem &p, Budget &budget, int threads, const Plan *incumbent) {
        Result res;
        res.method = "arc";
        res.plan = Plan::empty(p);
        res.objective = p.rejection;
        res.bound = 0;
        if (incumbent) {
            res.plan = *incumbent;
            res.objective = incumbent->validate(p);
        }
        const int N = p.nodes.size(), A = p.arcs.size(), B = p.blocks.size(), K = p.demands.size();
        Model m(threads, "IRBP-ARC");
        m.constant(p.rejection);
        std::vector<int> y(B), f(B), z(K), v(K), h(N);
        std::vector<std::vector<int>> xd(K, std::vector<int>(B, -1)), g = xd, xb(B, std::vector<int>(A)),
                                                                      w = xb, ld(K, std::vector<int>(N, -1)),
                                                                      lb(B, std::vector<int>(N));
        for (int b = 0; b < B; ++b) {
            auto &c = p.blocks[b];
            y[b] = m.var(c.opening, 1, SOLVER_BINARY);
            f[b] = m.var(c.interchange, c.upper);
            for (int a = 0; a < A; ++a) {
                xb[b][a] = m.var(0, 1, SOLVER_BINARY);
                w[b][a] = m.var(p.transport(b, a), c.upper);
            }
            for (int i = 0; i < N; ++i)
                lb[b][i] = m.var(0, N - 1);
        }
        int Y = 0;
        for (auto &n : p.nodes)
            Y += n.yard;
        for (int k = 0; k < K; ++k) {
            auto &d = p.demands[k];
            z[k] = m.var(0, 1, SOLVER_BINARY);
            v[k] = m.var(-d.penalty, d.quantity);
            for (int b : p.compatibleBlocks(k)) {
                xd[k][b] = m.var(0, 1, SOLVER_BINARY);
                g[k][b] = m.var(0, d.quantity);
            }
            for (int i = 0; i < N; ++i)
                if (p.nodes[i].yard)
                    ld[k][i] = m.var(0, Y - 1);
        }
        for (int i = 0; i < N; ++i)
            h[i] = m.var(p.nodes[i].handling_cost, p.nodes[i].handling_capacity);
        auto row = [&](std::initializer_list<std::pair<int, double>> items, char sense, double rhs) {
            Sparse s;
            for (auto [i, c] : items)
                s.add(i, c);
            m.row(s, sense, rhs);
        };
        for (int k = 0; k < K; ++k) {
            budget.check();
            auto &d = p.demands[k];
            row({{v[k], 1}, {z[k], -d.quantity}}, '<', 0);
            for (int i = 0; i < N; ++i)
                if (p.nodes[i].yard) {
                    Sparse bal, flow, out, in;
                    for (int b : p.compatibleBlocks(k)) {
                        if (p.blocks[b].from == i) {
                            bal.add(xd[k][b], 1);
                            flow.add(g[k][b], 1);
                            out.add(xd[k][b], 1);
                        }
                        if (p.blocks[b].to == i) {
                            bal.add(xd[k][b], -1);
                            flow.add(g[k][b], -1);
                            in.add(xd[k][b], 1);
                        }
                    }
                    int delta = (i == d.from) - (i == d.to);
                    bal.add(z[k], -delta);
                    flow.add(v[k], -delta);
                    out.add(z[k], -1);
                    in.add(z[k], -1);
                    m.row(bal, '=', 0);
                    m.row(flow, '=', 0);
                    m.row(out, '<', 0);
                    m.row(in, '<', 0);
                }
            for (int b : p.compatibleBlocks(k)) {
                auto &c = p.blocks[b];
                row({{g[k][b], 1}, {xd[k][b], -d.quantity}}, '<', 0);
                row({{xd[k][b], 1}, {y[b], -1}}, '<', 0);
                row({{ld[k][c.to], 1}, {ld[k][c.from], -1}, {xd[k][b], -double(Y)}}, '>', 1 - Y);
            }
        }
        for (int b = 0; b < B; ++b) {
            budget.check();
            auto &c = p.blocks[b];
            Sparse vol;
            vol.add(f[b], 1);
            for (int k : p.compatibleDemands(b))
                vol.add(g[k][b], -1);
            m.row(vol, '=', 0);
            row({{f[b], 1}, {y[b], -c.lower}}, '>', 0);
            row({{f[b], 1}, {y[b], -c.upper}}, '<', 0);
            for (int i = 0; i < N; ++i) {
                Sparse bal, out, in;
                for (int a = 0; a < A; ++a) {
                    if (p.arcs[a].from == i) {
                        bal.add(xb[b][a], 1);
                        out.add(xb[b][a], 1);
                    }
                    if (p.arcs[a].to == i) {
                        bal.add(xb[b][a], -1);
                        in.add(xb[b][a], 1);
                    }
                }
                bal.add(y[b], -((i == c.from) - (i == c.to)));
                out.add(y[b], -1);
                in.add(y[b], -1);
                m.row(bal, '=', 0);
                m.row(out, '<', 0);
                m.row(in, '<', 0);
            }
            Sparse len;
            len.add(y[b], -c.length_limit);
            for (int a = 0; a < A; ++a) {
                auto &e = p.arcs[a];
                len.add(xb[b][a], e.length);
                row({{lb[b][e.to], 1}, {lb[b][e.from], -1}, {xb[b][a], -double(N)}}, '>', 1 - N);
                row({{w[b][a], 1}, {xb[b][a], -c.upper}}, '<', 0);
                row({{w[b][a], 1}, {f[b], -1}}, '<', 0);
                row({{w[b][a], 1}, {f[b], -1}, {y[b], c.upper}, {xb[b][a], -c.upper}}, '>', 0);
            }
            m.row(len, '<', 0);
        }
        for (int i = 0; i < N; ++i) {
            Sparse handle, track;
            handle.add(h[i], 1);
            for (int k = 0; k < K; ++k) {
                if (i == p.demands[k].from)
                    handle.add(v[k], 1);
                for (int b : p.compatibleBlocks(k))
                    if (p.blocks[b].from == i)
                        handle.add(g[k][b], -1);
            }
            for (int b = 0; b < B; ++b)
                if (p.blocks[b].from == i)
                    track.add(y[b], p.blocks[b].slots);
            m.row(handle, '=', 0);
            m.row(track, '<', p.nodes[i].tracks);
        }
        std::vector<Sparse> capacity(p.capacities.size());
        for (int b = 0; b < B; ++b)
            for (int a = 0; a < A; ++a)
                capacity[p.arcs[a].link].add(w[b][a], 1);
        for (int e = 0; e < (int)capacity.size(); ++e)
            m.row(capacity[e], '<', p.capacities[e]);
        check(m.solver.updateModel());
        check(m.solver.clearMIPStarts());
        check(m.solver.setEnvCutoff(res.objective));
        CallbackData cb{&budget, res.objective, 0};
        check(m.solver.setIncumbentCallback(incumbentCallback, &cb));
        double begin = budget.elapsed();
        res.stats.build_seconds = begin;
        res.stats.columns = m.nvars;
        res.stats.rows = m.nrows;
        int status = m.optimize(budget, true);
        res.stats.solver_seconds = budget.elapsed() - begin;
        res.seconds = budget.elapsed();
        res.stats.first_best = cb.first_best;
        double node_count = 0;
        check(m.solver.getNodeCount(&node_count));
        res.stats.nodes = node_count;
        int count;
        check(m.solver.getSolCount(&count));
        if (count) {
            res.plan = Plan::empty(p);
            auto x = m.values();
            for (int k = 0; k < K; ++k) {
                res.plan.served[k] = x[v[k]];
                if (x[z[k]] > .5)
                    res.plan.logical[k] = recover(p.logical, p.demands[k].from, p.demands[k].to, xd[k], x);
            }
            for (int b = 0; b < B; ++b) {
                if (x[y[b]] > .5) {
                    res.plan.physical[b] = recover(p.physical, p.blocks[b].from, p.blocks[b].to, xb[b], x);
                }
            }
            res.objective = res.plan.validate(p);
            if (std::abs(res.objective - m.objective()) > 1e-5 * std::max(1., res.objective))
                throw std::runtime_error("ARC objective replay mismatch");
        }
        double bound = 0;
        if (status == SOLVER_CUTOFF)
            bound = res.objective;
        else
            check(m.solver.getObjBound(&bound));
        res.bound = std::max(0., bound);
        res.status = (status == SOLVER_OPTIMAL || status == SOLVER_CUTOFF) ? "optimal"
                     : status == SOLVER_TIME_LIMIT                         ? "time_limit"
                     : status == SOLVER_MEM_LIMIT                          ? "memory_limit"
                                                  : "solver_status_" + std::to_string(status);
        return res;
    }
} // namespace RouteOpt::Application::IRBP

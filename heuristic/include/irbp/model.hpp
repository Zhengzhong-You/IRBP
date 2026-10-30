/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#pragma once
#include "irbp/problem.hpp"
#include "solver.hpp"
#include <algorithm>
#include <utility>
namespace RouteOpt::Application::IRBP {
    inline void check(int status) {
        if (status)
            throw std::runtime_error("Solver error " + std::to_string(status));
    }
    struct Sparse {
        std::vector<int> ind;
        std::vector<double> val;
        void add(int i, double v) {
            if (v != 0) {
                ind.push_back(i);
                val.push_back(v);
            }
        }
        void canonicalize() {
            std::vector<std::pair<int, double>> tmp;
            tmp.reserve(ind.size());
            for (std::size_t j = 0; j < ind.size(); ++j)
                tmp.emplace_back(ind[j], val[j]);
            std::sort(tmp.begin(), tmp.end());
            ind.clear();
            val.clear();
            for (auto [i, v] : tmp) {
                if (!ind.empty() && ind.back() == i)
                    val.back() += v;
                else
                    add(i, v);
            }
        }
    };
    class Model {
        double objective_constant{};
        std::vector<double> objective_coefficients;
        void configure(int threads) {
            check(solver.setEnvThreads(threads, false));
            check(solver.setEnvSeed(1));
            check(solver.setEnvFeasibilityTol(1e-8, false));
            check(solver.setEnvOptimalityTol(1e-8));
            check(solver.setEnvIntFeasTol(1e-8, false));
            check(solver.setEnvMIPGap(0, false));
            check(solver.setEnvMIPGapAbs(1e-7));
        }

      public:
        RouteOpt::Solver solver{};
        int nvars{}, nrows{};
        explicit Model(int threads, const char *name) {
            try {
                check(solver.createEnv());
                check(solver.newModel(name, 0, nullptr, nullptr, nullptr, nullptr, nullptr));
                configure(threads);
            } catch (...) {
                solver.freeModel();
                solver.freeEnv();
                throw;
            }
        }
        ~Model() {
            solver.freeModel();
            solver.freeEnv();
        }
        Model(const Model &) = delete;
        Model &operator=(const Model &) = delete;
        int var(double obj = 0, double upper = SOLVER_INFINITY, char type = SOLVER_CONTINUOUS,
                Sparse col = {}) {
            col.canonicalize();
            check(solver.addVar(static_cast<int>(col.ind.size()), col.ind.data(), col.val.data(), obj, 0,
                                upper, type, nullptr));
            objective_coefficients.push_back(obj);
            return nvars++;
        }
        int row(Sparse a, char sense, double rhs) {
            a.canonicalize();
            check(solver.addConstraint(static_cast<int>(a.ind.size()), a.ind.data(), a.val.data(), sense, rhs,
                                       nullptr));
            return nrows++;
        }
        void constant(double z) {
            check(solver.setObjConstant(z));
            objective_constant = z;
        }
        double objective(const std::vector<double> &values) const {
            if (values.size() != objective_coefficients.size())
                throw std::invalid_argument("Objective replay dimensions");
            CostSum value;
            value += objective_constant;
            for (std::size_t i = 0; i < values.size(); ++i)
                value += objective_coefficients[i] * values[i];
            return value.value;
        }
        int optimize(Budget &budget, bool mip = false, int method = SOLVER_DUAL_SIMPLEX) {
            budget.check();
            check(solver.updateModel());
            check(solver.setEnvTimeLimit(budget.remaining()));
            if (mip)
                check(solver.mipOptimize());
            else
                check(solver.reoptimize(method));
            int status;
            check(solver.getStatus(&status));
            return status;
        }
        std::vector<double> values() const {
            std::vector<double> x(nvars);
            check(solver.getX(0, nvars, x.data()));
            return x;
        }
        std::vector<double> dual() const {
            std::vector<double> x(nrows);
            check(solver.getDual(0, nrows, x.data()));
            return x;
        }
        double objective() const {
            double x;
            check(solver.getObjVal(&x));
            return x;
        }
    };
} // namespace RouteOpt::Application::IRBP

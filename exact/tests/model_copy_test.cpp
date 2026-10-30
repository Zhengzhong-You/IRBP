/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#include "irbp/model.hpp"
#include <cmath>
#include <filesystem>
using namespace RouteOpt::Application::IRBP;
int main() {
    Model source(3, "source");
    source.var(-2, 4);
    check(source.solver.updateModel());
    for (int expected : {1, 2, 1}) {
        Model worker(expected, "worker");
        for (int round = 0; round < 3; ++round) {
            worker.copyFrom(source);
            int actual, source_threads;
            check(worker.solver.getThreads(&actual));
            check(source.solver.getThreads(&source_threads));
            if (actual != expected || source_threads != 3 || worker.nvars != 1 || worker.nrows != 0)
                return 1;
            Budget budget;
            if (worker.optimize(budget) != RouteOpt::SOLVER_OPTIMAL || std::abs(worker.objective() + 8) > 1e-8)
                return 2;
            check(worker.solver.freeModel());
        }
    }
    Model scaled(1, "objective-scale");
    int x = scaled.var(16384, 1);
    Sparse minimum;
    minimum.add(x, 1);
    scaled.row(minimum, '>', .5);
    check(scaled.solver.setEnvObjScale(16384));
    Budget budget;
    if (scaled.optimize(budget) != RouteOpt::SOLVER_OPTIMAL || std::abs(scaled.objective() - 8192) > 1e-8 ||
        std::abs(scaled.dual()[0] - 16384) > 1e-8)
        return 3;
    const auto basis = std::filesystem::temp_directory_path() /
        ("irbp-basis-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".bas");
    check(scaled.solver.write(basis.string().c_str()));
    Model replay(1, "basis-replay");
    replay.var(16384, 1);
    replay.row(minimum, '>', .5);
    check(replay.solver.updateModel());
    check(replay.solver.read(basis.string().c_str()));
    std::filesystem::remove(basis);
    int source_v, source_c, replay_v, replay_c;
    check(scaled.solver.getVBasis(0, 1, &source_v));
    check(scaled.solver.getCBasis(0, 1, &source_c));
    check(replay.solver.getVBasis(0, 1, &replay_v));
    check(replay.solver.getCBasis(0, 1, &replay_c));
    if (source_v != replay_v || source_c != replay_c ||
        replay.optimize(budget) != RouteOpt::SOLVER_OPTIMAL || std::abs(replay.objective() - 8192) > 1e-8)
        return 6;
    char binary = RouteOpt::SOLVER_BINARY;
    check(scaled.solver.setVTypeArray(x, 1, &binary));
    const double warm_start = 1.;
    check(scaled.solver.setMIPStart(x, 1, &warm_start));
    check(scaled.solver.setEnvMIPFocus(3));
    check(scaled.solver.setEnvCuts(2));
    if (scaled.optimize(budget, true) != RouteOpt::SOLVER_OPTIMAL)
        return 4;
    double bound;
    check(scaled.solver.getObjBound(&bound));
    if (std::abs(bound - 16384) > 1e-8 || std::abs(scaled.objective() - 16384) > 1e-8)
        return 5;
}

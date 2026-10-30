/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */
#include "irbp/model.hpp"
#include <cmath>
#include <set>
using namespace RouteOpt::Application::IRBP;
int main() {
    Model model(1, "solution-pool-index");
    Sparse sum;
    for (int j = 0; j < 3; ++j)
        sum.add(model.var(1 << j, 1, RouteOpt::SOLVER_BINARY), 1);
    model.row(sum, '>', 1);
    check(model.solver.setEnvPoolSolutions(7));
    check(model.solver.setEnvPoolSearchMode(2));
    Budget budget;
    budget.seconds = 10;
    if (model.optimize(budget, true) != RouteOpt::SOLVER_OPTIMAL)
        return 1;
    int count;
    check(model.solver.getSolCount(&count));
    if (count != 7)
        return 2;
    // Read directly: selecting an objective first must not be necessary.
    for (bool reverse : {false, true}) {
        std::set<int> distinct;
        for (int position = 0; position < count; ++position) {
            const int index = reverse ? count - position - 1 : position;
            double x[3];
            check(model.solver.getSolFromPool(index, 0, 3, x));
            const double value = x[0] + 2 * x[1] + 4 * x[2];
            if (std::abs(value - (index + 1)) > 1e-8)
                return 3;
            distinct.insert(static_cast<int>(std::lround(value)));
        }
        if (distinct.size() != 7)
            return 4;
    }
}

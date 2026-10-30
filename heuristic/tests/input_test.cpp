/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/problem.hpp"
#include <iostream>
#include <tuple>
using namespace RouteOpt::Application::IRBP;
int main(int argc, char **argv) try {
    if (argc != 3)
        throw std::runtime_error("usage: input_test ORIGINAL PACKED");
    Budget clock;
    clock.seconds = 1e100;
    auto a = Problem::read(argv[1]);
    double first = clock.elapsed();
    auto b = Problem::read(argv[2]);
    double second = clock.elapsed() - first;
    auto equal = [](bool value) {
        if (!value)
            throw std::runtime_error("Input representation mismatch");
    };
    equal(a.name == b.name && a.rejection == b.rejection && a.nodes.size() == b.nodes.size() &&
          a.arcs.size() == b.arcs.size() && a.blocks.size() == b.blocks.size() &&
          a.demands.size() == b.demands.size() && a.capacities == b.capacities &&
          a.compatibility_groups == b.compatibility_groups);
    for (std::size_t i = 0; i < a.nodes.size(); ++i) {
        const auto &x = a.nodes[i], &y = b.nodes[i];
        equal(std::tie(x.yard, x.tracks, x.handling_capacity, x.handling_cost) ==
              std::tie(y.yard, y.tracks, y.handling_capacity, y.handling_cost));
    }
    for (std::size_t i = 0; i < a.arcs.size(); ++i) {
        const auto &x = a.arcs[i], &y = b.arcs[i];
        equal(std::tie(x.from, x.to, x.link, x.length, x.cost) ==
              std::tie(y.from, y.to, y.link, y.length, y.cost));
    }
    for (std::size_t i = 0; i < a.blocks.size(); ++i) {
        const auto &x = a.blocks[i], &y = b.blocks[i];
        equal(std::tie(x.from, x.to, x.slots, x.lower, x.upper, x.length_limit, x.opening, x.interchange,
                       x.transport_factor, x.arc_cost) == std::tie(y.from, y.to, y.slots, y.lower, y.upper,
                                                                   y.length_limit, y.opening, y.interchange,
                                                                   y.transport_factor, y.arc_cost));
    }
    for (std::size_t i = 0; i < a.demands.size(); ++i) {
        const auto &x = a.demands[i], &y = b.demands[i];
        equal(std::tie(x.from, x.to, x.quantity, x.penalty, x.block_group) ==
              std::tie(y.from, y.to, y.quantity, y.penalty, y.block_group));
    }
    std::cout << Json({{"passed", true},
                       {"first_input", argv[1]},
                       {"second_input", argv[2]},
                       {"first_read_seconds", first},
                       {"second_read_seconds", second},
                       {"nodes", a.nodes.size()},
                       {"arcs", a.arcs.size()},
                       {"blocks", a.blocks.size()},
                       {"demands", a.demands.size()},
                       {"all_rejected_objective", a.rejection},
                       {"source_json_sha256", b.source_json_sha256}})
                     .dump()
              << '\n';
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

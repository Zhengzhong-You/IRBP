/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <psapi.h>
#else
#include <sys/resource.h>
#endif
using namespace RouteOpt::Application::IRBP;
namespace {
    double wallSeconds() {
        return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
    double peakMemoryMiB() {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS memory{};
        if (!GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
            throw std::runtime_error("Cannot read peak process memory");
        return memory.PeakWorkingSetSize / (1024. * 1024.);
#else
        rusage usage{};
        if (getrusage(RUSAGE_SELF, &usage))
            throw std::runtime_error("Cannot read peak process memory");
#ifdef __APPLE__
        return usage.ru_maxrss / (1024. * 1024.);
#else
        return usage.ru_maxrss / 1024.;
#endif
#endif
    }
} // namespace
int main(int argc, char **argv) try {
    const double entry = wallSeconds();
    Budget budget;
    if (argc < 5) {
        std::cerr << "usage: irbp INPUT.json|INPUT.irbp|INPUT.zip arc|heuristic|pg SECONDS OUTPUT.json "
                     "[--threads N] [--config OPTIONS.json] [--incumbent PLAN.json] "
                     "[--pool-state CAPTURE.json] [--deadline UNIX] [--demand-multiplier FACTOR]\n";
        return 2;
    }
    const std::string method = argv[2];
    if (method != "arc" && method != "heuristic" && method != "pg")
        throw std::runtime_error("Method must be arc, heuristic or pg");
    const double requested = std::stod(argv[3]);
    if (!std::isfinite(requested) || requested < 0)
        throw std::runtime_error("Seconds must be nonnegative; zero removes the overall time limit");
    std::string config_file, incumbent_file, pool_state_file;
    int threads = 0;
    double deadline = 0, demand_multiplier = 1.;
    for (int i = 5; i < argc; ++i) {
        const std::string flag = argv[i];
        if (i + 1 == argc)
            throw std::runtime_error("Missing value for " + flag);
        if (flag == "--threads") {
            threads = std::stoi(argv[++i]);
            if (threads < 1)
                throw std::runtime_error("Threads must be positive");
        } else if (flag == "--config")
            config_file = argv[++i];
        else if (flag == "--incumbent")
            incumbent_file = argv[++i];
        else if (flag == "--pool-state")
            pool_state_file = argv[++i];
        else if (flag == "--deadline")
            deadline = std::stod(argv[++i]);
        else if (flag == "--demand-multiplier")
            demand_multiplier = std::stod(argv[++i]);
        else
            throw std::runtime_error("Unknown option: " + flag);
    }
    if (threads < 0 || !std::isfinite(deadline) || deadline < 0)
        throw std::runtime_error("Invalid threads or deadline");
    if (method == "arc" && !config_file.empty())
        throw std::runtime_error("--config is for the heuristic");
    if (!incumbent_file.empty() && !pool_state_file.empty())
        throw std::runtime_error("Use either --incumbent or --pool-state");
    if (method == "pg" && pool_state_file.empty())
        throw std::runtime_error("pg requires --pool-state");
    if (method == "arc" && !pool_state_file.empty())
        throw std::runtime_error("--pool-state is for the heuristic");
    LocalSearchOptions options;
    if (!config_file.empty()) {
        std::ifstream input(config_file);
        if (!input)
            throw std::runtime_error("Cannot read heuristic settings");
        Json data;
        input >> data;
        options = readLocalSearchOptions(data);
    }
    if (threads)
        options.solver_threads = threads;
    if (method == "heuristic" && requested == 0 && pool_state_file.empty() && !options.pool_capture_only)
        options.continuous_search = true;
    options.validate();
    budget.seconds = requested == 0 ? 1e100 : requested;
    if (deadline)
        budget.seconds =
            std::min(budget.seconds, std::max(0., deadline - entry - options.output_return_reserve));
    std::clog << "READ_START " << argv[1] << '\n';
    Problem p;
    try {
        p = Problem::read(argv[1], demand_multiplier, &budget);
    } catch (const LimitReached &) {
        Json result = {{"method", method},
                       {"status", "input_time_limit"},
                       {"seconds", budget.elapsed()},
                       {"requested_seconds", requested},
                       {"peak_rss_mib", peakMemoryMiB()},
                       {"solution", nullptr},
                       {"objective", nullptr},
                       {"gap", nullptr}};
        std::ofstream output(argv[4]);
        if (!output || !(output << result.dump(2) << '\n'))
            throw std::runtime_error("Cannot write input timeout");
        std::clog << "INPUT_TIME_LIMIT seconds=" << budget.elapsed() << '\n';
        return 0;
    }
    const double input_seconds = budget.elapsed();
    std::clog << "READ_DONE seconds=" << std::fixed << std::setprecision(1) << input_seconds
              << " blocks=" << p.blocks.size() << " demands=" << p.demands.size() << '\n';
    Plan initial = Plan::empty(p);
    if (!incumbent_file.empty()) {
        Json data;
        std::ifstream input(incumbent_file);
        if (!input)
            throw std::runtime_error("Cannot read incumbent");
        input >> data;
        if (data.contains("instance") && data["instance"] != p.name)
            throw std::runtime_error("Incumbent instance mismatch");
        if (data.contains("demand_multiplier") && data.at("demand_multiplier") != demand_multiplier)
            throw std::runtime_error("Incumbent demand multiplier mismatch");
        if (data.contains("source_json_sha256") && data.at("source_json_sha256") != p.source_json_sha256)
            throw std::runtime_error("Incumbent input hash mismatch");
        const auto &s = data.contains("solution") ? data["solution"] : data;
        initial.logical = s.at("logical_paths").get<std::vector<std::vector<int>>>();
        initial.physical = s.at("physical_paths").get<std::vector<std::vector<int>>>();
        initial.served = s.at("served").get<std::vector<double>>();
        if (initial.validate(p) > p.rejection)
            initial = Plan::empty(p);
    }
    Json out;
    std::clog << "SEARCH_START remaining=" << budget.remaining() << '\n';
    if (method == "arc") {
        Result result;
        try {
            result = solveArc(p, budget, options.solver_threads, &initial);
        } catch (const LimitReached &) {
            result.method = "arc";
            result.status = "time_limit";
            result.plan = initial;
            result.objective = initial.validate(p);
            result.bound = 0;
            result.seconds = budget.elapsed();
        }
        out = result.json(p);
    } else {
        LocalSearchResult result;
        if (pool_state_file.empty())
            result = solveLocalSearch(p, budget, options, &initial, incumbent_file.empty());
        else {
            std::ifstream input(pool_state_file);
            if (!input)
                throw std::runtime_error("Cannot read pool state");
            Json capture;
            input >> capture;
            if (capture.at("instance") != p.name)
                throw std::runtime_error("Pool state instance mismatch");
            const auto &state = capture.at("profile").at("candidate_pool");
            const auto &plan = state.at("incumbent");
            initial.logical = plan.at("logical_paths").get<std::vector<std::vector<int>>>();
            initial.physical = plan.at("physical_paths").get<std::vector<std::vector<int>>>();
            initial.served = plan.at("served").get<std::vector<double>>();
            if (!objectiveEqual(initial.validate(p), state.at("incumbent_objective").get<double>()))
                throw std::runtime_error("Pool state objective mismatch");
            result = optimizeCandidatePaths(p, initial, state.at("logical_paths").get<PathPool>(),
                                            state.at("physical_paths").get<PathPool>(), budget, options, method == "pg");
        }
        if (options.pool_capture_only && !result.profile.contains("candidate_pool"))
            throw std::runtime_error("Budget expired before the big MIP input could be captured");
        out = {{"instance", p.name},
               {"method", method},
               {"status", method == "pg" ? "paths_generated" :
                              options.pool_capture_only ? "pool_prepared" : "heuristic"},
               {"objective", result.objective},
               {"validated_objective", result.plan.validate(p)},
               {"lower_bound", 0.},
               {"gap", nullptr},
               {"seconds", result.seconds},
               {"history", result.history},
               {"profile", result.profile},
               {"solution", result.plan.json()},
               {"search_options", options},
               {"bound_scope", "No global optimality certificate"}};
    }
    std::uint64_t stored_compatibility = 0, logical_compatibility = 0;
    for (const auto &group : p.compatibility_groups)
        stored_compatibility += group.size();
    for (int k = 0; k < int(p.demands.size()); ++k)
        logical_compatibility += p.compatibleBlocks(k).size();
    out["compatibility_entries_stored"] = stored_compatibility;
    out["compatibility_entries_logical"] = logical_compatibility;
    out["source_json_sha256"] = p.source_json_sha256;
    out["input_seconds"] = input_seconds;
    out["demand_multiplier"] = demand_multiplier;
    if (out.contains("profile") && out["profile"].contains("initial_candidate_pool")) {
        auto &initial_pool = out["profile"]["initial_candidate_pool"];
        initial_pool["source_json_sha256"] = p.source_json_sha256;
        initial_pool["demand_multiplier"] = demand_multiplier;
    }
    out["block_used"] = Json::array();
    for (const auto &path : out["solution"]["physical_paths"])
        out["block_used"].push_back(!path.empty());
    out["peak_rss_mib"] = peakMemoryMiB();
    out["process_entry_unix"] = entry;
    out["process_before_write_unix"] = wallSeconds();
    out["requested_seconds"] = requested;
    std::ofstream output(argv[4]);
    if (!output || !(output << out.dump(2) << '\n'))
        throw std::runtime_error("Cannot write result");
    std::cout << "Instance Method Status Objective LB Seconds PeakMiB\n"
              << p.name << ' ' << method << ' ' << out["status"].get<std::string>() << std::fixed
              << std::setprecision(1) << ' ' << out["objective"].get<double>() << ' '
              << out["lower_bound"].get<double>() << ' ' << out["seconds"].get<double>() << ' '
              << out["peak_rss_mib"].get<double>() << '\n';
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

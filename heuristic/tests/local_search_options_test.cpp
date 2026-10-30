/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#include "irbp/local_search_options.hpp"
#include <fstream>
#include <iostream>
using namespace RouteOpt::Application::IRBP;
int main(int argc, char **argv) try {
    if (argc != 2)
        return 2;
    Json file;
    std::ifstream(argv[1]) >> file;
    const Json defaults = LocalSearchOptions{};
    const Json resolved = readLocalSearchOptions(file);
    if (LocalSearchOptions{}.construction_relative_tolerance != 1e-6 ||
        readLocalSearchOptions(Json{{"construction_relative_tolerance", 0.}}).construction_relative_tolerance != 0.)
        throw std::runtime_error("Construction tolerance default or zero override failed");
    if (LocalSearchOptions{}.search_relative_tolerance != 1e-4 ||
        readLocalSearchOptions(Json{{"search_relative_tolerance", 0.}}).search_relative_tolerance != 0. ||
        readLocalSearchOptions(Json{{"search_relative_tolerance", 1e-6}}).search_relative_tolerance != 1e-6)
        throw std::runtime_error("Recommended search-progress default or explicit overrides failed");
    if (LocalSearchOptions{}.joint_repair_max_demands != 250 || resolved.at("joint_repair_max_demands") != 250 ||
        readLocalSearchOptions(Json{{"joint_repair_max_demands", 0}}).joint_repair_max_demands != 0 ||
        readLocalSearchOptions(Json{{"joint_repair_max_demands", 300}}).joint_repair_max_demands != 300)
        throw std::runtime_error("Joint-repair cap default or full-demand override failed");
    if (LocalSearchOptions{}.benchmark_targeted_destroy || resolved.at("benchmark_targeted_destroy") != false ||
        !readLocalSearchOptions(Json{{"benchmark_targeted_destroy", true}}).benchmark_targeted_destroy)
        throw std::runtime_error("Related-demand default or block-targeted comparator override failed");
    if (LocalSearchOptions{}.final_mip_focus != 0 ||
        readLocalSearchOptions(Json{{"final_mip_focus", 1}}).final_mip_focus != 1)
        throw std::runtime_error("Joint MIP focus default or explicit override failed");
    if (LocalSearchOptions{}.joint_arc_max_model_size != 100000 ||
        readLocalSearchOptions(Json{{"joint_arc_max_model_size", 0}}).joint_arc_max_model_size != 0)
        throw std::runtime_error("Joint ARC size default or zero override failed");
    if (LocalSearchOptions{}.guidance_logical_paths_per_demand != 4 ||
        readLocalSearchOptions(Json{{"guidance_logical_paths_per_demand", 0}}).guidance_logical_paths_per_demand != 0 ||
        LocalSearchOptions{}.guidance_max_seconds != 1e100 ||
        LocalSearchOptions{}.guidance_max_iterations != 10 ||
        LocalSearchOptions{}.guidance_plateau_window != 3 ||
        readLocalSearchOptions(Json{{"guidance_plateau_relative_tolerance", 0.}})
            .guidance_plateau_relative_tolerance != 0.)
        throw std::runtime_error("Bounded guidance defaults or plateau override failed");
    if (LocalSearchOptions{}.guidance_iteration_seconds != 10. ||
        readLocalSearchOptions(Json{{"guidance_iteration_seconds", 10.}}).guidance_iteration_seconds != 10.)
        throw std::runtime_error("Per-iteration soft target default or explicit override failed");
    if (Json(readLocalSearchOptions(resolved)) != resolved ||
        Json(readLocalSearchOptions(Json::object())) != defaults)
        throw std::runtime_error("Options round-trip or default fallback failed");
    auto options = readLocalSearchOptions(Json{{"max_new_blocks", 4}, {"paths_per_group", 0}});
    if (options.max_new_blocks != 4 || options.paths_per_group != 0 ||
        options.small_group_size != LocalSearchOptions{}.small_group_size)
        throw std::runtime_error("Partial configuration did not preserve defaults");
    for (const Json bad : {Json{{"max_new_block", 4}}, Json{{"max_new_blocks", 4.5}},
                           Json{{"max_new_blocks", 0}}, Json{{"paths_per_group", -1}},
                           Json{{"guidance_budget_fraction", 2.}}, Json{{"large_group_size", 4}},
                           Json{{"record_candidate_pool", "yes"}}, Json{{"pool_mip_max_seconds", 0}},
                           Json{{"pool_mip_max_seconds", -1.}}, Json{{"pool_arc_demands", 0}},
                           Json{{"incremental_max_seconds", 180.}}, Json{{"incremental_stall_seconds", 60.}},
                           Json{{"max_descent_passes", 100}}, Json{{"incremental_budget_fraction", .5}},
                           Json{{"construction_relative_tolerance", -1.}},
                           Json{{"construction_relative_tolerance", 1.}},
                           Json{{"construction_relative_tolerance", std::numeric_limits<double>::infinity()}},
                           Json{{"construction_relative_tolerance", std::numeric_limits<double>::quiet_NaN()}},
                           Json{{"search_relative_tolerance", -1.}},
                           Json{{"search_relative_tolerance", 1.}},
                           Json{{"search_relative_tolerance", std::numeric_limits<double>::infinity()}},
                           Json{{"search_relative_tolerance", std::numeric_limits<double>::quiet_NaN()}},
                           Json{{"joint_phase_max_seconds", 0}}, Json{{"progress_log_seconds", -1}},
                           Json{{"random_seed", -1}}, Json{{"continuous_idle_seconds", 0}},
                           Json{{"overall_stall_rounds", 0}}, Json{{"schedule_trace_limit", 0}},
                           Json{{"destroy_trial_max_seconds", 0}}, Json{{"joint_repair_max_seconds", -1}},
                           Json{{"open_logical_paths_per_demand", -1}},
                           Json{{"open_logical_search_seconds", 0}}, Json{{"finite_stall_cycles", -1}},
                           Json{{"initial_physical_spp_paths", -1}},
                           Json{{"guidance_logical_paths_per_demand", -1}},
                           Json{{"logical_pricing_fraction", .5}},
                           Json{{"benchmark_resource_diversity", true}},
                           Json{{"guidance_iteration_seconds", -1.}},
                           Json{{"guidance_iteration_seconds", std::numeric_limits<double>::infinity()}},
                           Json{{"guidance_iteration_seconds", std::numeric_limits<double>::quiet_NaN()}},
                           Json{{"guidance_neighbors", -1}},
                           Json{{"guided_jrc_paths_per_demand", -1}},
                           Json{{"guided_diverse_paths_per_demand", -1}},
                           Json{{"guidance_target_seconds", 10}},
                           Json{{"guidance_max_iterations", 0}},
                           Json{{"guidance_plateau_window", 0}},
                           Json{{"guidance_plateau_relative_tolerance", -1.}},
                           Json{{"guidance_plateau_relative_tolerance", 1.}},
                           Json{{"guidance_plateau_relative_tolerance", std::numeric_limits<double>::quiet_NaN()}},
                           Json{{"joint_repair_max_demands", -1}},
                           Json{{"joint_arc_max_model_size", -1}},
                           Json{{"neighborhood_policy", "learned"}},
                           Json{{"physical_length_weights", {-2.}}},
                           Json{{"pool_fix_fraction", 1.1}}, Json{{"pool_extra_paths", -1}},
                           Json{{"pool_lp_weight_tolerance", 0}},
                           Json{{"pool_jrc_relative_tolerance", -1}},
                           Json{{"pool_capture_only", true}, {"pool_extra_paths", 4}}}) {
        bool rejected = false;
        try {
            readLocalSearchOptions(bad);
        } catch (const std::exception &) {
            rejected = true;
        }
        if (!rejected)
            throw std::runtime_error("Invalid options accepted");
    }
    const double schedule_tol = 1e-6;
    struct ProgressCase { double before, after; bool improved; };
    const std::vector<ProgressCase> progress_cases = {
        {1e9, 1e9, false}, {1e9, 1e9 + 1, false}, {1e9, 1e9 - 1, false},
        {1e9, 1e9 - 1000, false}, {1e9, 1e9 - 1001, true},
        {1e3, 1e3 - 1e-4, false}, {1e9, 1e9 - 100, false},
        {1e3, 1e3 - .01, true}, {1e9, 1e9 - 1e4, true},
        {1e-7, 0, false}, {1e-6, 0, false}, {2e-6, 0, true},
        {-1e9, -1e9 - 1000, false}, {-1e9, -1e9 - 1001, true}
    };
    const auto schedule = readLocalSearchOptions(Json{{"search_relative_tolerance", 1e-6}});
    for (const auto &test : progress_cases)
        if (schedule.scheduleImproved(test.before, test.after, schedule_tol) != test.improved)
            throw std::runtime_error("Relative schedule boundary or scale inconsistency");
    const auto absolute_schedule = readLocalSearchOptions(Json{{"search_relative_tolerance", 0.}});
    if (!absolute_schedule.scheduleImproved(1e9, 1e9 - 1, schedule_tol) ||
        absolute_schedule.scheduleImproved(1e-6, 0, schedule_tol))
        throw std::runtime_error("Zero search threshold lost its absolute floor");
    if (!(1e9 - 1 < 1e9 - schedule_tol) || schedule.scheduleImproved(1e9, 1e9 - 1, schedule_tol))
        throw std::runtime_error("Schedule classification became the incumbent-acceptance gate");
    for (double before : {1e-7, 1e3, 1e9, -1e9}) {
        const double boundary = before - schedule_tol;
        for (double after : {std::nextafter(boundary, -std::numeric_limits<double>::infinity()),
                             boundary, std::nextafter(boundary, std::numeric_limits<double>::infinity())})
            if (absolute_schedule.scheduleImproved(before, after, schedule_tol) != (after < before - schedule_tol))
                throw std::runtime_error("Default zero tolerance changed the original comparison");
    }
    std::cout << "RELATIVE_SCHEDULE_BOUNDARIES_OK " << progress_cases.size() + 3 << '\n';
    std::cout << "DEFAULT_ZERO_COMPARISON_PARITY_OK 12\n";
    const auto larger_schedule = readLocalSearchOptions(Json{{"search_relative_tolerance", 1e-4}});
    const double boundary = 1000. - .1;
    const std::vector<ProgressCase> larger_cases = {
        {1e9, 1e9 - 99999., false}, {1e9, 1e9 - 100000., false}, {1e9, 1e9 - 100001., true},
        {1000., boundary, false},
        {1000., std::nextafter(boundary, -std::numeric_limits<double>::infinity()), true},
        {-1e9, -1e9 - 100000., false}, {-1e9, -1e9 - 100001., true}, {1e-7, 0., false}
    };
    for (const auto &test : larger_cases)
        if (larger_schedule.scheduleImproved(test.before, test.after, 1e-8) != test.improved)
            throw std::runtime_error("1e-4 schedule boundary failed");
    if (!(1e9 - 1 < 1e9 - 1e-8) || larger_schedule.scheduleImproved(1e9, 1e9 - 1, 1e-8))
        throw std::runtime_error("1e-4 stage progress altered the separate acceptance rule");
    std::cout << "RELATIVE_LEVEL_1E4_BOUNDARIES_OK 9\n";
    std::cout << "LOCAL_SEARCH_OPTIONS_OK\n";
} catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
}

/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#pragma once
#include "irbp/problem.hpp"

namespace RouteOpt::Application::IRBP {
    struct LocalSearchOptions {
        // Unopened blocks in a candidate, relative to the current solution.
        int max_new_blocks{2};
        // Large block universes use a temporary demand-driven candidate domain.
        int proposal_block_threshold{10000};
        int proposal_hubs_per_demand{8};
        int shared_hubs_per_sweep{16};
        int physical_tree_cache_size{64};
        double joint_mip_max_seconds{1.0};
        // Zero retains all candidates in each group during diagnosis.
        int paths_per_group{4};
        int base_logical_paths{16};
        int small_group_size{8};
        int large_group_size{12};
        int expand_after_stalls{2};
        int joint_after_stalls{3};
        int overall_stall_rounds{2};
        int schedule_trace_limit{128};
        double destroy_trial_max_seconds{0.1};
        double joint_repair_max_seconds{30.0};
        int open_logical_paths_per_demand{0};
        double open_logical_search_seconds{0.01};
        int finite_stall_cycles{0};
        int physical_paths_per_block{3};
        int initial_physical_spp_paths{2};
        int pricing_label_memory_mib{16};
        int guided_paths_per_demand{12};
        int guided_jrc_paths_per_demand{8};
        int guided_diverse_paths_per_demand{2};
        int support_paths_per_demand{3};
        double support_search_seconds{0.1};
        int physical_alternative_paths_per_block{2};
        double physical_alternative_seconds{0.1};
        bool record_candidate_pool{false};
        bool pool_capture_only{false};
        // Private component controls and comparators; defaults select the main method.
        bool benchmark_path_generation{true};
        bool benchmark_joint_repair{true};
        bool benchmark_local_repair{true};
        bool benchmark_targeted_destroy{false};
        bool benchmark_record_initial_pool{false};
        // Experiment switches: repairs use only the incumbent and the shared pool, without new candidate paths.
        bool benchmark_pool_only_joint{false};
        bool benchmark_pool_only_local{false};
        // Maximum jointly adjustable demand assignments; zero disables the cap.
        int joint_repair_max_demands{250};
        // Estimated variables plus rows of joint repair with one implicit logical path; zero disables it.
        int joint_arc_max_model_size{100000};
        double pool_fix_fraction{0.0};
        int pool_extra_paths{0};
        double pool_lp_weight_tolerance{1e-6};
        double pool_jrc_relative_tolerance{1e-6};
        int solver_threads{1};
        int final_mip_focus{0};
        double construction_relative_tolerance{1e-6};
        double search_relative_tolerance{1e-4};
        double anchor_priority{10000.};
        double origin_priority{1000.};
        double endpoint_priority{500.};
        double joint_phase_max_seconds{30.0};
        double progress_log_seconds{10.0};
        bool continuous_search{false};
        int random_seed{1};
        double continuous_idle_seconds{0.1};
        double seed_load_multiple{2.0};
        double guidance_max_seconds{1e100};
        double guidance_iteration_seconds{10.};
        // Logical paths seeded in the scratch RMP; zero retains the full input pool.
        int guidance_logical_paths_per_demand{4};
        bool guidance_heuristic_pricing{true};
        int guidance_neighbors{32};
        int guidance_max_iterations{10};
        int guidance_plateau_window{3};
        double guidance_plateau_relative_tolerance{1e-5};
        int guidance_lp_method{2};
        bool guidance_crossover{true};
        bool guidance_solver_log{true};
        double guidance_barrier_tolerance{1e-10};
        double guidance_budget_fraction{0.2};
        double min_joint_seconds{0.04};
        double min_pool_seconds{0.1};
        double pool_mip_max_seconds{30.0};
        double joint_return_reserve{0.01};
        double final_return_reserve{0.02};
        double solver_start_reserve{0.005};
        double output_return_reserve{0.05};
        // -1 requests the shortest physical length rather than a weighted cost.
        std::vector<double> physical_length_weights{1.0, 4.0, -1.0};
        bool scheduleImproved(double before, double after, double absolute_tolerance) const {
            return after < before - std::max(absolute_tolerance,
                search_relative_tolerance * std::max(1., std::abs(before)));
        }
        bool guidancePlateau(const std::vector<double> &objectives) const {
            if (objectives.size() <= std::size_t(guidance_plateau_window)) return false;
            const double previous = objectives[objectives.size() - 1 - guidance_plateau_window];
            return std::abs(objectives.back() - previous) <=
                guidance_plateau_relative_tolerance * std::max(1., std::abs(previous));
        }
        void validate() const {
            if (open_logical_paths_per_demand < 0 || finite_stall_cycles < 0 || initial_physical_spp_paths < 0 ||
                !std::isfinite(open_logical_search_seconds) || open_logical_search_seconds <= 0)
                throw std::invalid_argument("Invalid open-path or finite-patience settings");
            if (random_seed < 0 || !std::isfinite(continuous_idle_seconds) || continuous_idle_seconds <= 0)
                throw std::invalid_argument("Invalid continuous-search seed or idle interval");
            if (!std::isfinite(pool_fix_fraction) || pool_fix_fraction < 0 || pool_fix_fraction > 1 ||
                pool_extra_paths < 0 || !std::isfinite(pool_lp_weight_tolerance) ||
                pool_lp_weight_tolerance <= 0 || pool_lp_weight_tolerance >= 1 ||
                !std::isfinite(pool_jrc_relative_tolerance) || pool_jrc_relative_tolerance <= 0)
                throw std::invalid_argument("Invalid pool reduction settings");
            if (pool_capture_only && (pool_fix_fraction > 0 || pool_extra_paths > 0))
                throw std::invalid_argument("Candidate capture requires unmodified path sets");
            for (int value : {proposal_block_threshold, proposal_hubs_per_demand, shared_hubs_per_sweep,
                              physical_tree_cache_size, max_new_blocks, base_logical_paths, small_group_size,
                              large_group_size, expand_after_stalls, joint_after_stalls,
                              overall_stall_rounds, schedule_trace_limit,
                              physical_paths_per_block, pricing_label_memory_mib,
                              guided_paths_per_demand, support_paths_per_demand,
                              physical_alternative_paths_per_block, solver_threads})
                if (value < 1)
                    throw std::invalid_argument("Local-search counts must be positive");
            if (!std::isfinite(guidance_barrier_tolerance) || guidance_barrier_tolerance <= 0 ||
                guidance_barrier_tolerance >= 1)
                throw std::invalid_argument("Invalid barrier convergence tolerance");
            if (guidance_logical_paths_per_demand < 0)
                throw std::invalid_argument("Guidance logical path limit must be nonnegative");
            if (guidance_neighbors < 0 || guided_jrc_paths_per_demand < 0 ||
                guided_diverse_paths_per_demand < 0)
                throw std::invalid_argument("Invalid logical pricing neighborhood or return quotas");
            if (guidance_max_iterations < 1 || guidance_plateau_window < 1 ||
                !std::isfinite(guidance_plateau_relative_tolerance) ||
                guidance_plateau_relative_tolerance < 0 || guidance_plateau_relative_tolerance >= 1)
                throw std::invalid_argument("Invalid guidance iteration or plateau settings");
            if (guidance_lp_method < 0 || guidance_lp_method > 2)
                throw std::invalid_argument("LP method must be 0 (primal), 1 (dual), or 2 (barrier)");
            if (joint_repair_max_demands < 0 || joint_arc_max_model_size < 0 || paths_per_group < 0 || large_group_size < small_group_size ||
                joint_after_stalls <= expand_after_stalls || final_mip_focus < 0 || final_mip_focus > 3)
                throw std::invalid_argument("Invalid local-search count, stall, or focus setting");
            if (!std::isfinite(construction_relative_tolerance) || construction_relative_tolerance < 0 ||
                construction_relative_tolerance >= 1)
                throw std::invalid_argument("Construction relative tolerance must be in [0,1)");
            if (!std::isfinite(search_relative_tolerance) || search_relative_tolerance < 0 ||
                search_relative_tolerance >= 1)
                throw std::invalid_argument("Search relative tolerance must be in [0,1)");
            for (double value : {guidance_budget_fraction})
                if (!std::isfinite(value) || value <= 0 || value > 1)
                    throw std::invalid_argument("Local-search budget fractions must be in (0,1]");
            for (double value : {joint_phase_max_seconds, destroy_trial_max_seconds, joint_repair_max_seconds})
                if (!std::isfinite(value) || value <= 0)
                    throw std::invalid_argument("Phase limits must be positive and finite");
            for (double value : {progress_log_seconds, guidance_max_seconds, guidance_iteration_seconds, support_search_seconds,
                                 physical_alternative_seconds, min_joint_seconds, min_pool_seconds, joint_return_reserve,
                                 final_return_reserve, solver_start_reserve, output_return_reserve})
                if (!std::isfinite(value) || value < 0)
                    throw std::invalid_argument("Local-search time settings must be finite and nonnegative");
            for (double value : {anchor_priority, origin_priority, endpoint_priority})
                if (!std::isfinite(value) || value < 0)
                    throw std::invalid_argument("Demand-group priorities must be nonnegative");
            if (!std::isfinite(joint_mip_max_seconds) || joint_mip_max_seconds <= 0)
                throw std::invalid_argument("Joint MIP time limit must be positive and finite");
            if (!std::isfinite(pool_mip_max_seconds) || pool_mip_max_seconds <= 0)
                throw std::invalid_argument("Pool MIP time limit must be positive and finite");
            if (!std::isfinite(seed_load_multiple) || seed_load_multiple < 1)
                throw std::invalid_argument("Seed load multiple must be at least one");
            for (double value : physical_length_weights)
                if (!std::isfinite(value) || (value < 0 && value != -1))
                    throw std::invalid_argument("Physical length weights must be nonnegative or -1");
        }
    };
    // A shared field list keeps the flat JSON format beyond the library macro limit.
#define IRBP_LOCAL_SEARCH_FIELDS(X) \
    X(proposal_block_threshold) \
    X(proposal_hubs_per_demand) \
    X(shared_hubs_per_sweep) \
    X(physical_tree_cache_size) \
    X(joint_mip_max_seconds) \
    X(max_new_blocks) \
    X(paths_per_group) \
    X(base_logical_paths) \
    X(small_group_size) \
    X(large_group_size) \
    X(expand_after_stalls) \
    X(joint_after_stalls) \
    X(physical_paths_per_block) \
    X(initial_physical_spp_paths) \
    X(pricing_label_memory_mib) \
    X(guided_paths_per_demand) \
    X(guided_jrc_paths_per_demand) \
    X(guided_diverse_paths_per_demand) \
    X(support_paths_per_demand) \
    X(support_search_seconds) \
    X(physical_alternative_paths_per_block) \
    X(physical_alternative_seconds) \
    X(record_candidate_pool) \
    X(pool_capture_only) \
    X(benchmark_path_generation) \
    X(benchmark_joint_repair) \
    X(benchmark_local_repair) \
    X(benchmark_targeted_destroy) \
    X(benchmark_record_initial_pool) \
    X(benchmark_pool_only_joint) \
    X(benchmark_pool_only_local) \
    X(joint_repair_max_demands) \
    X(joint_arc_max_model_size) \
    X(pool_fix_fraction) \
    X(pool_extra_paths) \
    X(pool_lp_weight_tolerance) \
    X(pool_jrc_relative_tolerance) \
    X(solver_threads) \
    X(final_mip_focus) \
    X(construction_relative_tolerance) \
    X(search_relative_tolerance) \
    X(anchor_priority) \
    X(origin_priority) \
    X(endpoint_priority) \
    X(joint_phase_max_seconds) \
    X(progress_log_seconds) \
    X(continuous_search) \
    X(random_seed) \
    X(continuous_idle_seconds) \
    X(seed_load_multiple) \
    X(guidance_max_seconds) \
    X(guidance_iteration_seconds) \
    X(guidance_logical_paths_per_demand) \
    X(guidance_heuristic_pricing) \
    X(guidance_neighbors) \
    X(guidance_max_iterations) \
    X(guidance_plateau_window) \
    X(guidance_plateau_relative_tolerance) \
    X(guidance_lp_method) \
    X(guidance_crossover) \
    X(guidance_solver_log) \
    X(guidance_barrier_tolerance) \
    X(guidance_budget_fraction) \
    X(min_joint_seconds) \
    X(min_pool_seconds) \
    X(pool_mip_max_seconds) \
    X(joint_return_reserve) \
    X(final_return_reserve) \
    X(solver_start_reserve) \
    X(output_return_reserve) \
    X(physical_length_weights) \
    X(overall_stall_rounds) \
    X(schedule_trace_limit) \
    X(destroy_trial_max_seconds) \
    X(joint_repair_max_seconds) \
    X(open_logical_paths_per_demand) \
    X(open_logical_search_seconds) \
    X(finite_stall_cycles)
    inline void to_json(Json &data, const LocalSearchOptions &options) {
        data = Json::object();
#define IRBP_WRITE_OPTION(name) data[#name] = options.name;
        IRBP_LOCAL_SEARCH_FIELDS(IRBP_WRITE_OPTION)
#undef IRBP_WRITE_OPTION
    }
    inline void from_json(const Json &data, LocalSearchOptions &options) {
        const LocalSearchOptions defaults;
#define IRBP_READ_OPTION(name) options.name = data.value(#name, defaults.name);
        IRBP_LOCAL_SEARCH_FIELDS(IRBP_READ_OPTION)
#undef IRBP_READ_OPTION
    }
#undef IRBP_LOCAL_SEARCH_FIELDS
    inline LocalSearchOptions readLocalSearchOptions(const Json &data) {
        const Json defaults = LocalSearchOptions{};
        if (!data.is_object())
            throw std::invalid_argument("Local-search options must be a JSON object");
        for (const auto &[key, value] : data.items()) {
            if (!defaults.contains(key))
                throw std::invalid_argument("Unknown local-search option: " + key);
            if (defaults[key].is_number_integer() && !value.is_number_integer())
                throw std::invalid_argument("Integer local-search option required: " + key);
        }
        auto options = data.get<LocalSearchOptions>();
        options.validate();
        return options;
    }
} // namespace RouteOpt::Application::IRBP

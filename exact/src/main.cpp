/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/problem.hpp"
#include "irbp/branching_quota.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include "solver.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/resource.h>
using namespace RouteOpt::Application::IRBP;
int main(int argc, char **argv) {
    try {
        if (argc < 5) {
            std::cerr << "usage: irbp INSTANCE.json arc|arc-lp|bpc|single-cg SECONDS RESULT.json [THREADS] [LABEL_MIB] "
                         "[--incumbent RESULT.json] [--pricing-workers N] [--branch-workers N] [--root-only]\n";
            return 2;
        }
        auto p = Problem::read(argv[1]);
        Budget budget;
        budget.seconds = std::stod(argv[3]);
        if (budget.seconds <= 0)
            throw std::runtime_error("Positive time limit required");
        int threads = argc > 5 ? std::stoi(argv[5]) : 1;
        if (threads < 1)
            throw std::runtime_error("Positive threads required");
        BpcOptions options;
        int pricing_workers = options.pricing_workers, branch_workers = options.branch_workers;
        bool root_only = false, dw_root = false;
        std::string incumbent_file;
        for (int i = 7; i < argc; ++i) {
            const std::string flag = argv[i];
            if (flag == "--pricing-workers" && i + 1 < argc) pricing_workers = std::stoi(argv[++i]);
            else if (flag == "--branch-workers" && i + 1 < argc) branch_workers = std::stoi(argv[++i]);
            else if (flag == "--root-only") root_only = true;
            else if (flag == "--dw-root") {root_only = true; dw_root = true;}
            else if (flag == "--dual-after-columns") options.dual_after_columns = true;
            else if (flag == "--incumbent" && i + 1 < argc) incumbent_file = argv[++i];
            else throw std::runtime_error("Unknown or incomplete option: " + flag);
        }
        Plan initial;
        const Plan *incumbent = nullptr;
        if (!incumbent_file.empty()) {
            std::ifstream input(incumbent_file);
            if (!input)
                throw std::runtime_error("Cannot read incumbent");
            Json saved;
            input >> saved;
            if (saved.contains("instance") && saved["instance"] != p.name)
                throw std::runtime_error("Incumbent instance mismatch");
            const auto &sol = saved.contains("solution") ? saved.at("solution") : saved;
            initial.logical = sol.at("logical_paths").get<std::vector<std::vector<int>>>();
            initial.physical = sol.at("physical_paths").get<std::vector<std::vector<int>>>();
            initial.served = sol.at("served").get<std::vector<double>>();
            initial.validate(p);
            incumbent = &initial;
        }
        std::string method = argv[2];
        if (pricing_workers < 1)
            throw std::runtime_error("Positive pricing worker count required");
        options.pricing_workers = pricing_workers;
        options.root_only = root_only;
        options.dw_root = dw_root;
        if (dw_root && method != "bpc") throw std::runtime_error("--dw-root requires bpc");
        if (options.dual_after_columns && !dw_root) throw std::runtime_error("--dual-after-columns requires --dw-root");
        options.branch_workers = branch_workers;
        std::ofstream testing_csv, pricing_csv, regenerated_csv;
        if (method == "bpc" || method == "single-cg") {
            pricing_csv.open(std::filesystem::path(argv[4]).replace_extension(".pricing.csv"));
            regenerated_csv.open(std::filesystem::path(argv[4]).replace_extension(".regenerated.csv"));
            if (!pricing_csv || !regenerated_csv)
                throw std::runtime_error("Cannot create pricing diagnostic logs");
            pricing_csv << "Iter,Node,Mode,ExtraRows,LP,MinJRC,LogicalMinJRC,PhysicalMinJRC,RCTolerance,Correction,Added,Paths,Removed,Regenerated,Time(s)\n";
            regenerated_csv << "Time(s),Family,Owner,Edges\n";
            pricing_csv.flush();
            regenerated_csv.flush();
            budget.report_pricing = [&](const PricingProgress &p) {
                pricing_csv << p.iteration << ',' << p.node << ',' << (p.heuristic ? "heuristic" : "exact")
                            << ',' << p.extra_rows << std::setprecision(17);
                for (double value : {p.lp, std::min(p.logical_min, p.physical_min), p.logical_min,
                                     p.physical_min, p.tolerance, p.correction}) {
                    pricing_csv << ',';
                    if (std::isfinite(value) && std::abs(value) < 1e90) pricing_csv << value;
                }
                pricing_csv << ',' << p.added << ',' << p.paths << ',' << p.removed << ',' << p.regenerated
                            << ',' << p.seconds << '\n';
                pricing_csv.flush();
            };
            budget.report_regenerated = [&](const Path &p) {
                regenerated_csv << std::setprecision(17) << budget.elapsed() << ','
                                << (p.physical ? "physical" : "logical") << ',' << p.owner << ',';
                for (std::size_t i = 0; i < p.edges.size(); ++i) {
                    if (i) regenerated_csv << ';';
                    regenerated_csv << p.edges[i];
                }
                regenerated_csv << '\n';
                regenerated_csv.flush();
            };
            if (method == "bpc" || method == "single-cg") {
                auto testing_path = std::filesystem::path(argv[4]).replace_extension(".branching.csv");
                testing_csv.open(testing_path);
                if (!testing_csv)
                    throw std::runtime_error("Cannot create branching stage log");
                testing_csv << "Node,Depth,ScreenO,ScreenP,ScreenL,BKFK,LPRequested,LPO,LPP,LPL,Winner,Score,LP(s),RBest\n";
                budget.report_testing = [&](const BranchingProgress &p) {
                    testing_csv << std::setprecision(17) << p.node << ',' << p.depth;
                    for (int x : p.screened) testing_csv << ',' << x;
                    testing_csv << ',' << p.bkf_k << ',' << p.lp_requested;
                    for (int x : p.lp) testing_csv << ',' << x;
                    testing_csv << ',' << "OPL"[p.winner] << ',' << p.score << ',' << p.lp_seconds
                                << ',' << p.r_best << '\n';
                    testing_csv.flush();
                };
            }
        }
        auto csv_path = std::filesystem::path(argv[4]);
        csv_path.replace_extension(".csv");
        std::ofstream csv(csv_path);
        if (!csv)
            throw std::runtime_error("Cannot write progress log");
        csv << "Stage,Iter,Nodes,Paths,Cols,Rows,LP,LB,UB,Gap (%),Time (s),LP Delta(s),Pricing Delta(s),LP Total(s),Pricing Total(s)\n";
        std::cout << p.name << "  " << method << '\n'
                  << "Stage         Iter  Nodes   Paths    Cols    Rows          LP          LB          UB   Gap%   Time(s)   LPdt(s)   PRdt(s)\n";
        Progress latest;
        std::string last_stage;
        double last_time = -2;
        double last_lp_seconds = 0, last_pricing_seconds = 0;
        budget.report = [&](const Progress &p) {
            latest = p;
            if (p.stage == last_stage && p.seconds < last_time + 2)
                return;
            last_stage = p.stage;
            last_time = p.seconds;
            const double lp_delta = p.lp_seconds - last_lp_seconds;
            const double pricing_delta = p.pricing_seconds - last_pricing_seconds;
            last_lp_seconds = p.lp_seconds;
            last_pricing_seconds = p.pricing_seconds;
            const double gap = 100 * std::max(0., p.upper - p.lower) / std::max(1e-10, std::abs(p.upper));
            std::ostringstream line;
            line << std::left << std::setw(12) << p.stage << std::right << std::setw(6) << p.iteration
                 << std::setw(7) << p.nodes << std::setw(8) << p.paths;
            for (int n : {p.columns, p.rows})
                line << std::setw(8) << (n < 0 ? "-" : std::to_string(n));
            line << std::fixed << std::setprecision(1);
            for (double v : {p.lp, p.lower, p.upper}) {
                if (std::isfinite(v))
                    line << std::setw(12) << v;
                else
                    line << std::setw(12) << "-";
            }
            line << std::setw(7) << gap << std::setw(10) << p.seconds
                 << std::setw(10) << lp_delta << std::setw(10) << pricing_delta;
            std::cout << line.str() << std::endl;
            csv << p.stage << ',' << p.iteration << ',' << p.nodes << ',' << p.paths << ',';
            if (p.columns >= 0)
                csv << p.columns;
            csv << ',';
            if (p.rows >= 0)
                csv << p.rows;
            csv << std::setprecision(17);
            for (double v : {p.lp, p.lower, p.upper, gap, p.seconds}) {
                csv << ',';
                if (std::isfinite(v))
                    csv << v;
            }
            csv << ',' << lp_delta << ',' << pricing_delta << ',' << p.lp_seconds << ',' << p.pricing_seconds << '\n';
            csv.flush();
        };
        budget.emit({"START", 0, 0, 0, -1, -1, std::numeric_limits<double>::quiet_NaN(), 0,
                     incumbent ? initial.validate(p) : p.rejection});
        Result result;
        if (method == "arc" || method == "arc-lp")
            result = solveArc(p, budget, threads, incumbent, method == "arc-lp", root_only);
        else if (method == "bpc")
            result = solveBpc(p, budget, threads, (argc > 6 ? std::stoull(argv[6]) : 0) * 1024 * 1024,
                              incumbent, options);
        else if (method == "single-cg")
            result = solveSingleCg(p, budget, threads, (argc > 6 ? std::stoull(argv[6]) : 0) * 1024 * 1024,
                                   incumbent, options);
        else
            throw std::runtime_error("Unknown method");
        latest.stage = result.status == "optimal" ? "OPTIMAL"
                       : result.status == "lp_optimal" ? "LP_DONE"
                       : result.status == "root_complete" ? "ROOT_DONE"
                       : result.status == "time_limit" ? "TIME_LIMIT"
                       : result.status == "path_pool_limit" ? "PATH_LIMIT" : "UNFINISHED";
        latest.iteration = result.stats.cg_iterations;
        latest.nodes = result.stats.nodes;
        latest.paths = result.stats.paths;
        latest.columns = result.stats.columns;
        latest.rows = result.stats.rows;
        latest.lp = std::numeric_limits<double>::quiet_NaN();
        latest.lower = result.bound;
        latest.upper = result.objective;
        latest.seconds = result.seconds;
        latest.lp_seconds = method == "arc" ? 0 : result.stats.master_seconds;
        latest.pricing_seconds = result.stats.pricing_seconds;
        budget.report(latest);
        Json out = result.json(p);
        out["progress_log"] = csv_path.string();
        if (method == "bpc" || method == "single-cg") {
            out["pricing_log"] = std::filesystem::path(argv[4]).replace_extension(".pricing.csv").string();
            out["regenerated_paths_log"] = std::filesystem::path(argv[4]).replace_extension(".regenerated.csv").string();
        }
        rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
#if defined(__APPLE__) || defined(_WIN32)
        out["peak_rss_mib"] = usage.ru_maxrss / (1024. * 1024.);
#else
        out["peak_rss_mib"] = usage.ru_maxrss / 1024.;
#endif
        const auto version = RouteOpt::Solver::version();
        out["solver"] = RouteOpt::Solver::backendName();
        out["solver_version"] = std::to_string(version[0]) + "." + std::to_string(version[1]) + "." + std::to_string(version[2]);
        out["initial_incumbent_objective"] = incumbent ? initial.validate(p) : p.rejection;
        out["threads"] = threads;
        out["pricing_workers"] = method == "bpc" || method == "single-cg" ? pricing_workers : 0;
        out["root_only"] = root_only || method == "arc-lp";
        out["dw_root"] = dw_root;
        out["extra_cover_cuts"] = method == "bpc" && !dw_root;
        out["benchmark_lp_method"] = method == "arc-lp" ? "barrier_without_crossover" : "original_default";
        out["integer_search"] = method == "arc" || ((method == "bpc" || method == "single-cg") && !root_only);
        out["search_scope"] = method == "arc-lp" ? "LP_relaxation" : root_only ? "root_LP" : "integer_model";
        out["algorithm"] = method == "arc-lp" ? "Gurobi_LP" : method == "arc" ? "Gurobi_default_MIP" : root_only ? "column_generation_root_LP" : "RouteOpt_branch_price_cut";
        if (method == "bpc" && !root_only) {
            out["branch_strategy"] = "routeopt_lp_bkf";
            out["branch_workers"] = method == "bpc" ? branch_workers : 0;
            out["branch_lp_threads"] = method == "bpc" ? branchLpThreads(threads, branch_workers) : 0;
            out["bkf_lp_m"] = MaxLpCandidatesPerWorker;
            out["bkf_lp_n"] = RootLpCandidates;
            out["lp_candidate_multiplier"] = branch_workers;
            out["root_lp_total"] = RootLpCandidates;
            out["branch_weights"] = BranchWeights;
            out["lp_methods"] = {{"cg", "primal"}, {"cuts", "dual"}, {"lp_testing", "barrier_without_crossover"}};
        } else if (method == "single-cg" && !root_only) {
            out["path_capacity_bounds"] = true;
            out["branch_strategy"] = "routeopt_lp_bkf";
            out["branch_workers"] = branch_workers;
            out["branch_lp_threads"] = branchLpThreads(threads, branch_workers);
            out["relative_optimality_tolerance"] = RelativeRcTol;
            out["lp_methods"] = {{"cg", "barrier_without_crossover"}};
        } else if (method == "bpc") {
            out["physical_volume_upper_bound"] = "global_block_upper";
            out["lp_methods"] = {{"initial_master", "primal"}, {"after_columns", options.dual_after_columns ? "dual" : "primal"}, {"coupling_rows", "dual"}};
            out["dual_after_columns"] = options.dual_after_columns;
        } else if (method == "single-cg") {
            out["path_capacity_bounds"] = true;
            out["lp_methods"] = {{"cg", "barrier_without_crossover"}};
        }
        if (method == "arc-lp") out["lp_methods"] = {{"LP_relaxation","barrier_without_crossover"}};
        out["incumbent_upper_bound"] = result.objective;
        out["objective_measure"] = "integer_incumbent_cost";
        out["gap_measure"] = method == "arc-lp" || root_only ? "incumbent_upper_bound_vs_dual_bound" : "integer_MIP_relative_gap";
        if (method == "arc-lp" || root_only) {
            Json primal = nullptr, relative_gap = nullptr;
            std::string certificate_kind = "incomplete";
            if (method == "arc-lp" && result.status == "lp_optimal") {
                primal = result.bound;
                relative_gap = 0.;
                certificate_kind = "Gurobi_LP_optimal_status";
            } else if ((result.status == "root_complete" || result.status == "root_pricing_tolerance" || result.status == "optimal") && !result.lp_certificate.is_null()) {
                const double value = result.lp_certificate.at("objective").get<double>();
                primal = value;
                relative_gap = std::max(0., (value-result.bound)/std::max(1.,std::abs(value)));
                certificate_kind = result.lp_certificate.value("proof_kind", std::string("exact_pricing"));
            } else if (result.stats.root_complete) {
                certificate_kind = "primal_witness_not_exported";
            }
            out["root_LP"] = {{"primal_objective",primal},{"dual_bound",result.bound},
                              {"relative_gap",relative_gap},{"certificate_kind",certificate_kind}};
        }
        out["time_limit"] = budget.seconds;
        out["dimensions"] = {{"nodes", p.nodes.size()},
                             {"arcs", p.arcs.size()},
                             {"links", p.capacities.size()},
                             {"blocks", p.blocks.size()},
                             {"demands", p.demands.size()}};
        std::ofstream f(argv[4]);
        if (!f)
            throw std::runtime_error("Cannot write result");
        f << out.dump(2) << '\n';
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
    return 0;
}

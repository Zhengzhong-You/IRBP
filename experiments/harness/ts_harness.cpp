/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
// Generated-instance experiment adapter (TS campaign); production optimizer remains unchanged.
// Derived from the 20261027-components150-60s adapter: budget, seed and parameters come from the
// command line and config; only the component switches are tied to the arm.
#include <functional>
#include <limits>
#include <cmath>
#include <sstream>
#include "local_search.cpp"
#include <fstream>
#include <sstream>

using namespace RouteOpt::Application::IRBP;
namespace {
constexpr const char *SourceCommit = "f519e48acfede662b6c7af28d7c964b47e3fa5d1";

Json readJson(const std::string &path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot read " + path);
    Json value;
    input >> value;
    return value;
}

Json snapshot(const Search &search, const char *boundary, bool complete) {
    PathPool physical(search.pool.size()), logical = search.guided;
    std::vector<std::vector<double>> route_unit_costs(search.pool.size());
    for (int b = 0; b < int(search.pool.size()); ++b)
        for (const auto &route : search.pool[b]) {
            physical[b].push_back(route.arcs);
            route_unit_costs[b].push_back(route.unit);
        }
    for (int k = 0; k < int(logical.size()); ++k) {
        const auto &path = search.out.plan.logical[k];
        if (!path.empty() && std::find(logical[k].begin(), logical[k].end(), path) == logical[k].end())
            logical[k].push_back(path);
    }
    std::ostringstream random;
    random << search.random;
    return {{"format", "external-post-construction-v1"}, {"source_commit", SourceCommit},
            {"boundary", boundary}, {"complete", complete}, {"instance", search.p.name},
            {"domain", {{"demand_driven", search.demand_driven},
                         {"original_block_ids", search.original_blocks}}},
            {"state", {{"guided", search.guided}, {"seeded", search.seeded},
                        {"route_unit_costs", route_unit_costs},
                        {"incumbent_generation", search.incumbent_generation}, {"random", random.str()}}},
            {"profile", {{"candidate_pool", {{"incumbent", search.out.plan.json()},
                         {"incumbent_objective", search.out.objective},
                         {"logical_paths", logical}, {"physical_paths", physical}}}}}};
}

bool constructOnly(Search &search) {
    State state(search.p);
    search.rebuild(state);
    search.incremental_begin = search.last_global_improvement = search.budget.elapsed();
    search.incremental_active = true;
    try { search.descend(state, "construction"); } catch (const LimitReached &) {}
    search.incremental_active = false;
    search.incremental_seconds = search.budget.elapsed() - search.incremental_begin;
    const bool complete = search.budget.remaining() > 0 &&
        (search.incremental_stop_reason == "no_improving_sweep" ||
         search.incremental_stop_reason == "relative_sweep_tailoff");
    if (!complete) search.incremental_stop_reason = "time_limit";
    return complete;
}

} // namespace
Json instrumentation(const Search &s) {
    return {{"PG_calls",s.component_pg_calls},{"PG_skips",s.component_pg_skips},
            {"local_calls",s.component_local_calls},{"local_skips",s.component_local_skips},
            {"joint_calls",s.component_joint_calls},{"joint_skips",s.component_joint_skips},
            {"joint_preparation_calls",s.component_prep_calls},{"joint_ARC_enabled_models",s.component_joint_arc_enabled},
            {"joint_ARC_disabled_models",s.component_joint_arc_disabled},
            {"guidance_LP_calls",s.guidance_lp_calls},{"local_MIP_calls",s.mip_calls},{"joint_MIP_calls",s.pool_mip_calls}};
}
void sameConstructor(const Json &actual,const Json &expected) {
    for(const char *key:{"domain","state"})
        if(actual.at(key)!=expected.at(key))throw std::runtime_error(std::string("Constructor state mismatch:")+key);
    if(actual.at("profile").at("candidate_pool")!=expected.at("profile").at("candidate_pool"))
        throw std::runtime_error("Constructor pool/incumbent mismatch");
}
int main(int argc,char **argv) try {
    Budget budget; // Single from-zero total clock, including configuration/input/construction/setup.
    const double entry_unix=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    if(argc!=8)throw std::runtime_error("ts_harness INPUT ARM SECONDS OUTPUT CONFIG 1 EXPECTED_C0|-");
    const std::string arm=argv[2];const bool ctorOnly=arm=="ConstructionOnly";
    const bool noPG=arm=="NoPG"||arm=="NoPGNoImplicit";
    const bool noImplicit=arm=="FullNoImplicit"||arm=="NoPGNoImplicit";
    if(!ctorOnly&&arm!="Full"&&arm!="NoPG"&&arm!="NoLocal"&&arm!="NoJoint"&&arm!="FullNoImplicit"&&arm!="NoPGNoImplicit")
        throw std::runtime_error("Unknown arm");
    budget.seconds=std::stod(argv[3]);
    if(!(budget.seconds>0)||!std::isfinite(budget.seconds)||std::stod(argv[6])!=1.)
        throw std::runtime_error("Positive finite total seconds and multiplier1 required");
    auto options=readLocalSearchOptions(readJson(argv[5]));options.validate();
    // Only the component switches are tied to the arm; method parameters come from the config file.
    if(options.benchmark_targeted_destroy||(noImplicit&&options.joint_arc_max_model_size!=0)||
       options.benchmark_path_generation!=!noPG||options.benchmark_local_repair!=(arm!="NoLocal")||
       options.benchmark_joint_repair!=(arm!="NoJoint")||
       options.record_candidate_pool||options.pool_capture_only||options.continuous_search)
        throw std::runtime_error("Arm/configuration switch mismatch");
    Problem original=Problem::read(argv[1],1.,&budget);
    const bool checkC0=std::string(argv[7])!="-";
    Json expected;
    if(checkC0){
        expected=readJson(argv[7]);
        if(expected.at("format")!="external-post-construction-v1"||!expected.at("complete").get<bool>()||
           expected.at("boundary")!="post_construction"||expected.at("instance")!=original.name||
           expected.at("source_json_sha256")!=original.source_json_sha256||expected.at("demand_multiplier").get<double>()!=1.)
            throw std::runtime_error("Locked complete C0/input mismatch");
    }
    const double setup_seconds=budget.elapsed();
    std::clog<<"COMPONENT_TOTAL_BUDGET "<<Json{{"arm",arm},{"total_seconds",budget.seconds},
        {"setup_elapsed_seconds",setup_seconds},{"remaining_seconds",budget.remaining()},
        {"from_zero",true},{"implicit_config_limit",options.joint_arc_max_model_size},
        {"PG_enabled",options.benchmark_path_generation},{"local_enabled",options.benchmark_local_repair},
        {"joint_enabled",options.benchmark_joint_repair}}.dump()<<std::endl;
    LocalSearchResult out;out.plan=Plan::empty(original);out.objective=original.rejection;
    Search search(original,budget,out,options,false,{});
    Json ctor_state;bool ctor_checked=false;bool complete=true;bool ctor_seen=false;
    double ctor_objective=std::numeric_limits<double>::quiet_NaN();std::string ctor_digest;
    search.component_ctor_observer=[&](const Search &s){
        Json current=snapshot(s,"post_construction",true);
        if(checkC0){sameConstructor(current,expected);ctor_checked=true;}
        ctor_seen=true;ctor_state=current.at("state");ctor_objective=s.out.objective;
        ctor_digest=std::to_string(std::hash<std::string>{}(current.at("profile").dump()+current.at("state").dump()));
        std::clog<<"COMPONENT_CTOR_BOUNDARY "<<Json{{"matched_locked_C0",ctor_checked},{"C0_objective",ctor_objective},
            {"elapsed_total_seconds",budget.elapsed()},{"constructor_stage_seconds",s.incremental_seconds},
            {"remaining_seconds",budget.remaining()},{"PG_calls",s.component_pg_calls},
            {"local_calls",s.component_local_calls},{"joint_calls",s.component_joint_calls}}.dump()<<std::endl;
    };
    Json result;
    if(ctorOnly){
        complete=constructOnly(search);result=snapshot(search,"post_construction",complete);
        if(checkC0&&complete){sameConstructor(result,expected);ctor_checked=true;}
        ctor_seen=complete;ctor_state=result.at("state");ctor_objective=out.objective;
        ctor_digest=std::to_string(std::hash<std::string>{}(result.at("profile").dump()+result.at("state").dump()));
    } else search.run(true); // No restore or budget reset; constructor and all stages share one total budget.
    const double algorithm_seconds=budget.elapsed();
    const double validated=out.plan.validate(original);
    if(!objectiveEqual(validated,out.objective))throw std::runtime_error("Final plan validation mismatch");
    if(checkC0&&ctor_seen&&!ctor_checked)throw std::runtime_error("C0 was reached but not checked");
    if(!ctorOnly)result={{"profile",out.profile},{"history",out.history}};
    result["instance"]=original.name;result["source_commit"]=SourceCommit;result["arm"]=arm;
    result["objective"]=out.objective;result["validated_objective"]=validated;result["solution"]=out.plan.json();
    result["source_json_sha256"]=original.source_json_sha256;result["demand_multiplier"]=1.;
    result["search_options"]=options;result["requested_total_seconds"]=budget.seconds;
    result["demands"]=original.demands.size();result["blocks"]=original.blocks.size();
    result["constructor_reached"]=ctor_seen;result["C0_objective"]=ctor_seen?Json(ctor_objective):Json(nullptr);
    result["C0_digest"]=ctor_digest;result["C0_checked_against_expected"]=ctor_checked;
    result["process_entry_unix"]=entry_unix;result["from_zero"]=true;result["budget_includes_input_constructor_and_setup"]=true;
    result["setup_elapsed_seconds"]=setup_seconds;result["constructor_stage_seconds"]=search.incremental_seconds;
    result["constructor_complete_elapsed_seconds"]=ctorOnly?Json(algorithm_seconds):Json(search.component_ctor_complete_seconds);
    result["algorithm_total_elapsed_seconds"]=algorithm_seconds;result["process_before_write_seconds"]=budget.elapsed();
    result["initial_constructor_state"]=ctor_state;result["matched_locked_C0"]=ctor_checked;
    result["component_instrumentation"]=instrumentation(search);result["complete_constructor"]=complete;
    if(ctorOnly&&(search.component_pg_calls||search.component_local_calls||search.component_joint_calls))
        throw std::runtime_error("ConstructionOnly entered search component");
    if(arm=="NoJoint"&&(search.component_joint_calls||search.component_prep_calls||search.pool_mip_calls))
        throw std::runtime_error("NoJoint entered joint component");
    if(noPG&&(search.component_pg_calls||search.guidance_lp_calls||search.guidance_seconds!=0))
        throw std::runtime_error("NoPG entered PG");
    if(arm=="NoLocal"&&(search.component_local_calls||search.mip_calls||search.group_phase_seconds!=0))
        throw std::runtime_error("NoLocal entered local component");
    if(noImplicit&&search.component_joint_arc_enabled)throw std::runtime_error("NoImplicit enabled joint ARC");
    std::ofstream output(argv[4]);if(!output||!(output<<result.dump()<<'\n'))throw std::runtime_error("Cannot write final result");
    return 0; // completeness is recorded in complete_constructor
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}

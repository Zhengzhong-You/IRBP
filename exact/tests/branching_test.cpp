/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/problem.hpp"
#include "irbp/branching_quota.hpp"
#include <cmath>
#include <iostream>
using namespace RouteOpt::Application::IRBP;
int main(int argc, char **argv) {
    if (argc != 2) return 1;
    for (int k = 1; k <= MaxLpCandidatesPerWorker; ++k)
        if (lpTestingBudget(k, 8, false) != 8 * k || lpTestingBudget(k, 1, false) != k ||
            lpTestingBudget(k, 8, true) != RootLpCandidates) return 9;
    if (branchingQuota({100,100,100},14) != std::array<int,3>{9,4,1} ||
        branchingQuota({100,100,100},10) != std::array<int,3>{6,3,1} ||
        branchingQuota({5,2,12},10) != std::array<int,3>{5,2,3} ||
        branchingQuota({0,4,100},10) != std::array<int,3>{0,4,6}) return 2;
    for (int a=0;a<6;++a) for(int b=0;b<6;++b) for(int c=0;c<6;++c) for(int k=0;k<20;++k) {
        auto q=branchingQuota({a,b,c},k);
        if(q[0]<0 || q[0]>a || q[1]<0 || q[1]>b || q[2]<0 || q[2]>c ||
           q[0]+q[1]+q[2]!=std::min(k,a+b+c)) return 3;
    }
    std::uint64_t tests=0,cuts=0;
    for(const auto *name:{"partial_service","parallel_blocks","syn_y4_n6_k8_s42"}) {
        auto p=Problem::read(std::string(argv[1])+"/"+name+".json");
        Budget arc_budget; arc_budget.seconds=20;
        auto exact=solveArc(p,arc_budget,1);
        if(exact.status!="optimal") return 4;
        for(int workers:{1,8}) {
            BpcOptions options; options.branch_workers=workers;
            Budget budget; budget.seconds=20;
            budget.report_testing=[&](const BranchingProgress &v) {
                if(v.lp!=branchingQuota(v.screened,v.lp_requested) || !v.lp[v.winner])
                    throw std::runtime_error("Invalid weighted allocation or winner");
                if(v.node==0 && v.lp_requested!=RootLpCandidates)
                    throw std::runtime_error("Wrong root quota");
                if(v.node && (v.bkf_k<1 || v.bkf_k>MaxLpCandidatesPerWorker ||
                              v.lp_requested!=v.bkf_k*workers))
                    throw std::runtime_error("Wrong BKF quota");
            };
            budget.report=[&](const Progress &v) {
                if(v.lower>exact.objective+1e-6 || v.upper<exact.objective-1e-6)
                    throw std::runtime_error("Invalid bound from testing");
            };
            auto r=solveBpc(p,budget,1,64*1024*1024,&exact.plan,options);
            if(r.status!="optimal" || std::abs(r.objective-exact.objective)>1e-6 ||
               std::abs(r.bound-exact.objective)>1e-6) return 5;
            if(!r.stats.master_primal_solves || r.stats.cut_dual_solves>r.stats.master_dual_solves ||
               r.stats.lp_barrier_solves!=2*r.stats.branch_lp_candidates) return 6;
            tests+=r.stats.lp_barrier_solves; cuts+=r.stats.cut_dual_solves;
        }
        Budget tiny; tiny.seconds=.005;
        auto interrupted=solveBpc(p,tiny,1,64*1024*1024,&exact.plan);
        if(interrupted.bound>exact.objective+1e-6 || interrupted.objective<exact.objective-1e-6) return 7;
    }
    if(!tests || !cuts) return 8;
    std::cout<<"Native RouteOpt, 9:4:1 quotas, parallel barrier tests, simplex routing and exact bounds passed\n";
}

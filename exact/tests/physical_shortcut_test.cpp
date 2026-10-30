/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#ifdef BASELINE_PRICING_HEADER
#include "../../baseline-pricing.hpp"
#else
#include "../src/bpc/pricing.hpp"
#endif
#include <functional>
#include <iostream>
#include <random>
using namespace RouteOpt::Application::IRBP;
using namespace RouteOpt::Application::IRBP::BPC;

Problem graph(int n, const std::vector<Arc>& arcs, double limit) {
    Problem p;p.name="Physical pricing shortcut oracle";
    p.nodes.assign(n,{false,0,1000000,0});p.nodes.front().yard=p.nodes.back().yard=true;
    p.nodes.front().tracks=p.nodes.back().tracks=10;p.arcs=arcs;
    p.capacities.assign(arcs.size(),1000000);std::vector<int> from,to;
    for(const auto &a:arcs){from.push_back(a.from);to.push_back(a.to);}
    p.physical.build(n,from,to);p.reverse_physical.build(n,to,from);
    p.blocks={{0,n-1,1,1,2,limit,10,0,1,{}}};p.logical.build(n,{0},{n-1});
    p.demands={{0,n-1,1,10000,{0}}};p.block_demands={{0}};p.rejection=10000;return p;
}
double trial(Problem p,const std::vector<double>& link_dual,bool expect_shortcut=false) {
    Budget budget;budget.seconds=30;Stats stats;Master master(p,1,stats);
    if(master.model.optimize(budget)!=RouteOpt::SOLVER_OPTIMAL)throw std::runtime_error("LP setup failed");
    std::vector<double> dual(master.model.nrows);
    for(int i=0;i<(int)link_dual.size();++i)dual[master.link[i]]=link_dual[i];
    double oracle=1e100;
    for(int b=0;b<(int)p.blocks.size();++b){
        std::vector<bool> seen(p.nodes.size());std::vector<int> path;seen[p.blocks[b].from]=true;
        std::function<void(int,double)> dfs=[&](int at,double length){
            if(at==p.blocks[b].to){for(double t:{p.blocks[b].lower,p.blocks[b].upper})oracle=std::min(oracle,master.jrc({true,b,path},t,dual));return;}
            for(int z=p.physical.offsets[at];z<p.physical.offsets[at+1];++z){
                int a=p.physical.edge_ids[z],next=p.arcs[a].to;
                if(seen[next]||length+p.arcs[a].length>p.blocks[b].length_limit)continue;
                seen[next]=true;path.push_back(a);dfs(next,length+p.arcs[a].length);path.pop_back();seen[next]=false;
            }
        };dfs(p.blocks[b].from,0);
    }
    double actual=1e100;budget.report_pricing=[&](const PricingProgress &r){actual=r.physical_min;};
    Pricing pricing(p,master,budget,stats,0,1);pricing.run(dual,false);
    if(std::abs(actual-oracle)>1e-7*std::max(1.,std::abs(oracle)))throw std::runtime_error("Physical owner minimum differs from enumeration");
    if(expect_shortcut && stats.searches!=1)throw std::runtime_error("Nonnegative no-column certificate still enumerated physical labels");
    return actual;
}
int main(int argc,char**argv){
    if(argc==2){
        const int layers=std::stoi(argv[1]),n=2*layers+2;std::vector<Arc> arcs;
        auto add=[&](int i,int j){arcs.push_back({i,j,(int)arcs.size(),1,1});};
        add(0,1);add(0,2);
        for(int l=1;l<layers;++l)for(int a=0;a<2;++a)for(int b=0;b<2;++b)add(2*l-1+a,2*l+1+b);
        add(2*layers-1,n-1);add(2*layers,n-1);
        auto begin=std::chrono::steady_clock::now();double answer=trial(graph(n,arcs,layers+1),{});
        std::cout<<"layers="<<layers<<" nodes="<<n<<" physical_min="<<answer<<" seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<'\n';return 0;
    }
    auto p=graph(3,{{0,2,0,1,5},{0,1,1,2,1},{1,2,2,2,1}},3);
    if(std::abs(trial(p,{})-15)>1e-8)return 1; // Cheapest unrestricted path is too long: exact fallback.
    p=graph(3,{{0,2,0,1,1},{0,1,1,1,1},{1,2,2,1,1}},3);
    if(std::abs(trial(p,{2,2,2})-6)>1e-8)return 2; // Synthetic dual makes arc weights negative.
    p=graph(4,{{0,1,0,2,0},{1,3,1,2,0},{0,2,2,1,0},{2,3,3,1,0}},3);
    if(std::abs(trial(p,{},true)-10)>1e-8)return 3; // Cost tie must choose shortest length.
    p=graph(2,{{0,1,0,1,1}},2);p.blocks.push_back(p.blocks[0]);p.blocks[1].transport_factor=2;p.blocks[1].opening=0;
    p.logical.build(2,{0,0},{1,1});p.demands[0].blocks={0,1};p.block_demands={{0},{0}};
    if(std::abs(trial(p,{},true)-2)>1e-8)return 4; // Same destination/load but different block weights.
    std::mt19937 rng(1941);
    for(int test=0;test<100;++test){
        int n=4+test%4;std::vector<Arc> arcs;std::vector<double> dual;
        for(int i=0;i<n;++i)for(int j=0;j<n;++j)if(i!=j&&(j==i+1||rng()%3==0)){
            arcs.push_back({i,j,(int)arcs.size(),double(1+rng()%4),double(rng()%8)});dual.push_back(double(int(rng()%10)-3));
        }
        trial(graph(n,arcs,4+test%13),dual);
    }
    std::cout<<"100 exhaustive physical owner minima plus length, negative-weight, zero-cost and cache-identity boundaries passed\n";
}

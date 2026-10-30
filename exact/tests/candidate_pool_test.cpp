/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../src/bpc/candidate_pool.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>
using namespace RouteOpt::Application::IRBP;
using namespace RouteOpt::Application::IRBP::BPC;
static std::vector<PricingCandidate> select(std::vector<PricingCandidate> all) {
    std::sort(all.begin(),all.end(),[](const auto &a,const auto &b){
        return candidateIdentity(a.path)==candidateIdentity(b.path)?a.cost<b.cost:candidateIdentity(a.path)<candidateIdentity(b.path);
    });
    all.erase(std::unique(all.begin(),all.end(),[](const auto &a,const auto &b){return candidateIdentity(a.path)==candidateIdentity(b.path);}),all.end());
    std::sort(all.begin(),all.end(),[](const auto &a,const auto &b){return candidateCostLess(a.path,a.cost,b.path,b.cost);});
    if(all.size()>PricingBatchSize)all.resize(PricingBatchSize);
    return all;
}
int main() {
    std::mt19937 random(20261023);
    for(int trial=0;trial<20;++trial) {
        std::vector<PricingCandidatePool> local(4);std::vector<PricingCandidate> all;
        for(int j=0;j<25000;++j) {
            int id=random()%7000;
            Path p{bool(id%2),id%19,{id,id/7,id%13}};
            double cost=double(int(random()%4000)-2000);
            all.push_back({p,double(j%3),cost});
            local[random()%local.size()].retain(std::move(p),double(j%3),cost);
            for(const auto &pool:local)assert(pool.size()<=PricingBatchSize);
        }
        std::vector<PricingCandidate> merged;
        for(const auto &pool:local)pool.appendTo(merged);
        auto expected=select(std::move(all)),actual=select(std::move(merged));
        assert(expected.size()==actual.size());
        for(std::size_t i=0;i<actual.size();++i) {
            assert(candidateIdentity(expected[i].path)==candidateIdentity(actual[i].path));
            assert(expected[i].cost==actual[i].cost);
        }
    }
    PricingCandidatePool pool;
    for(int j=0;j<2000;++j)pool.retain({true,0,{j}},1,j);
    pool.retain({true,0,{1999}},2,-20);pool.retain({true,0,{1999}},3,-10);
    std::vector<PricingCandidate> out;pool.appendTo(out);out=select(std::move(out));
    assert(out.front().path.edges==std::vector<int>{1999} && out.front().cost==-20);
    pool.clear();assert(pool.size()==0 && pool.callbacks==0 && pool.peak_bytes==0);
    std::cout<<"500000 streamed candidates: local unique top-1000 equals full global sort, including ties and duplicates\n";
}

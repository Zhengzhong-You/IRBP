/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
 * Software: RouteOpt. Application: IRBP.
 */
#pragma once
#include "irbp/problem.hpp"
#include <map>
#include <set>
#include <tuple>

namespace RouteOpt::Application::IRBP::BPC {
    constexpr std::size_t PricingBatchSize = 1000;
    struct PricingCandidate {
        Path path;
        double load{}, cost{};
    };
    inline auto candidateIdentity(const Path &path) {
        return std::tie(path.physical, path.owner, path.edges);
    }
    struct CandidateIdentityLess {
        bool operator()(const Path &a, const Path &b) const {
            return candidateIdentity(a) < candidateIdentity(b);
        }
    };
    inline bool candidateCostLess(const Path &a, double ca, const Path &b, double cb) {
        return ca == cb ? candidateIdentity(a) < candidateIdentity(b) : ca < cb;
    }
    class PricingCandidatePool {
        struct Values { double load{}, cost{}; };
        using Paths = std::map<Path, Values, CandidateIdentityLess>;
        using Iterator = Paths::iterator;
        struct CostLess {
            bool operator()(const Iterator &a, const Iterator &b) const {
                return candidateCostLess(a->first, a->second.cost, b->first, b->second.cost);
            }
        };
        Paths paths;
        std::set<Iterator, CostLess> order;
        std::size_t payload_bytes{};
        static std::size_t bytes(const Path &path) {
            return sizeof(PricingCandidate) + path.edges.capacity() * sizeof(int) + 8 * sizeof(void *);
        }
      public:
        std::uint64_t callbacks{}, duplicate_hits{};
        std::size_t peak_bytes{};
        void clear() {
            order.clear(); paths.clear(); payload_bytes = peak_bytes = callbacks = duplicate_hits = 0;
        }
        std::size_t size() const { return paths.size(); }
        void retain(Path path, double load, double cost) {
            ++callbacks;
            auto old = paths.find(path);
            if (old != paths.end()) {
                ++duplicate_hits;
                if (cost >= old->second.cost) return;
                order.erase(old);
                old->second = {load, cost};
                order.insert(old);
                return;
            }
            if (paths.size() == PricingBatchSize) {
                const auto worst = *order.rbegin();
                if (!candidateCostLess(path, cost, worst->first, worst->second.cost)) return;
            }
            auto inserted = paths.emplace(std::move(path), Values{load, cost}).first;
            order.insert(inserted);payload_bytes += bytes(inserted->first);
            if (paths.size() > PricingBatchSize) {
                auto last = std::prev(order.end());auto worst = *last;
                payload_bytes -= bytes(worst->first);order.erase(last);paths.erase(worst);
            }
            peak_bytes = std::max(peak_bytes, payload_bytes);
        }
        void appendTo(std::vector<PricingCandidate> &target) const {
            for (const auto &item : paths) target.push_back({item.first, item.second.load, item.second.cost});
        }
    };
}

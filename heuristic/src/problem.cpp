/*
 * Copyright (c) 2026 Zhengzhong (Ricky) You.
 * All rights reserved.
 * Software: RouteOpt
 * Application: IRBP
 */

#include "irbp/problem.hpp"
#include "miniz.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
namespace RouteOpt::Application::IRBP {
    namespace {
        // Inflate into a bounded stream buffer; never extract archive paths onto disk.
        class ZipInput final : public std::streambuf {
            mz_zip_archive archive{};
            mz_zip_reader_extract_iter_state *iterator{};
            std::array<char, 65536> buffer{};

          public:
            bool binary_member{};
            std::uint64_t member_size{};
            explicit ZipInput(const std::string &path) {
                if (!mz_zip_reader_init_file(&archive, path.c_str(), 0))
                    throw std::runtime_error("Cannot open ZIP instance: " + path);
                try {
                    int selected = -1;
                    for (mz_uint i = 0; i < mz_zip_reader_get_num_files(&archive); ++i) {
                        mz_zip_archive_file_stat stat{};
                        if (!mz_zip_reader_file_stat(&archive, i, &stat))
                            throw std::runtime_error("Invalid ZIP directory");
                        auto suffix = std::filesystem::path(stat.m_filename).extension().string();
                        std::transform(suffix.begin(), suffix.end(), suffix.begin(),
                                       [](unsigned char c) { return std::tolower(c); });
                        if (!stat.m_is_directory && (suffix == ".json" || suffix == ".irbp")) {
                            if (selected >= 0)
                                throw std::runtime_error(
                                    "Instance ZIP must contain exactly one JSON or IRBP binary file");
                            selected = static_cast<int>(i);
                            binary_member = suffix == ".irbp";
                            member_size = stat.m_uncomp_size;
                        }
                    }
                    if (selected < 0 || !(iterator = mz_zip_reader_extract_iter_new(&archive, selected, 0)))
                        throw std::runtime_error("Cannot read the instance member of ZIP");
                } catch (...) {
                    mz_zip_reader_end(&archive);
                    throw;
                }
            }
            ZipInput(const ZipInput &) = delete;
            ZipInput &operator=(const ZipInput &) = delete;
            ~ZipInput() override {
                if (iterator)
                    mz_zip_reader_extract_iter_free(iterator);
                mz_zip_reader_end(&archive);
            }
            void verify() {
                const bool valid = mz_zip_reader_extract_iter_free(iterator);
                iterator = nullptr;
                if (!valid)
                    throw std::runtime_error("Instance ZIP decompression or CRC validation failed");
            }

          protected:
            int_type underflow() override {
                const auto n = mz_zip_reader_extract_iter_read(iterator, buffer.data(), buffer.size());
                if (!n)
                    return traits_type::eof();
                setg(buffer.data(), buffer.data(), buffer.data() + n);
                return traits_type::to_int_type(*gptr());
            }
        };
        // Portable little-endian arrays; no pointers, ABI-dependent structs, or serialized code.
        class PackedInput {
            std::istream &input;
            Budget *budget;
            std::uint64_t remaining, reads{};

          public:
            PackedInput(std::istream &input, Budget *budget, std::uint64_t size)
                : input(input), budget(budget), remaining(size) {}
            void bytes(char *out, std::size_t n) {
                if (n > remaining)
                    throw std::runtime_error("Truncated binary instance");
                if (budget && ((++reads & 4095) == 0))
                    budget->check();
                if (!input.read(out, n))
                    throw std::runtime_error("Truncated binary instance");
                remaining -= n;
            }
            std::uint32_t u32() {
                unsigned char b[4];
                bytes(reinterpret_cast<char *>(b), 4);
                return std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8) | (std::uint32_t(b[2]) << 16) |
                       (std::uint32_t(b[3]) << 24);
            }
            int index() {
                auto n = u32();
                if (n > std::uint32_t(std::numeric_limits<int>::max()))
                    throw std::runtime_error("Binary index is too large");
                return int(n);
            }
            double real() {
                static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
                unsigned char b[8];
                bytes(reinterpret_cast<char *>(b), 8);
                std::uint64_t bits = 0;
                for (int i = 0; i < 8; ++i)
                    bits |= std::uint64_t(b[i]) << (8 * i);
                double value;
                std::memcpy(&value, &bits, sizeof(value));
                return value;
            }
            void require(std::uint64_t bytes) const {
                if (bytes > remaining)
                    throw std::runtime_error("Invalid binary array size");
            }
            std::string text() {
                auto n = u32();
                require(n);
                std::string s(n, '\0');
                bytes(s.data(), n);
                return s;
            }
            void finish() const {
                if (remaining || input.peek() != std::char_traits<char>::eof())
                    throw std::runtime_error("Trailing binary instance data");
            }
        };
        Json readPacked(std::istream &input, Budget *budget, std::uint64_t size, Problem &p) {
            PackedInput r(input, budget, size);
            char magic[8];
            r.bytes(magic, 8);
            if (std::string(magic, 8) != "IRBPBIN1")
                throw std::runtime_error("Unknown IRBP binary version");
            const auto flags = r.u32();
            if (flags > 1)
                throw std::runtime_error("Unknown IRBP binary flags");
            p.name = r.text();
            p.source_json_sha256 = r.text();
            if (p.source_json_sha256.size() != 64 ||
                p.source_json_sha256.find_first_not_of("0123456789abcdef") != std::string::npos)
                throw std::runtime_error("Invalid binary source fingerprint");
            const int N = r.index(), A = r.index(), E = r.index(), B = r.index(), G = r.index(),
                      K = r.index();
            r.require(28ull * N + 28ull * A + 8ull * E + 64ull * B + 4ull * G + 28ull * K);
            p.nodes.reserve(N);
            p.arcs.reserve(A);
            p.blocks.reserve(B);
            for (int i = 0; i < N; ++i) {
                auto yard = r.u32();
                if (yard > 1)
                    throw std::runtime_error("Invalid binary yard flag");
                double tracks = r.real(), capacity = r.real(), cost = r.real();
                p.nodes.push_back({bool(yard), tracks, capacity, cost});
            }
            for (int a = 0; a < A; ++a) {
                int from = r.index(), to = r.index(), link = r.index();
                double length = r.real(), cost = r.real();
                p.arcs.push_back({from, to, link, length, cost});
            }
            p.capacities.resize(E);
            for (double &c : p.capacities)
                c = r.real();
            for (int b = 0; b < B; ++b) {
                Block block;
                block.from = r.index();
                block.to = r.index();
                block.slots = r.index();
                block.lower = r.real();
                block.upper = r.real();
                block.length_limit = r.real();
                block.opening = r.real();
                block.interchange = r.real();
                block.transport_factor = r.real();
                int overrides = r.index();
                r.require(12ull * overrides);
                for (int j = 0; j < overrides; ++j) {
                    int a = r.index();
                    double cost = r.real();
                    if (!block.arc_cost.emplace(a, cost).second)
                        throw std::runtime_error("Duplicate binary cost override");
                }
                p.blocks.push_back(std::move(block));
            }
            p.compatibility_groups.resize(G);
            for (auto &group : p.compatibility_groups) {
                int count = r.index();
                r.require(4ull * count);
                group.resize(count);
                for (int &b : group)
                    b = r.index();
            }
            p.demands.reserve(K);
            for (int k = 0; k < K; ++k) {
                Demand d;
                d.from = r.index();
                d.to = r.index();
                d.quantity = r.real();
                d.penalty = r.real();
                d.block_group = r.index();
                if (d.block_group >= G)
                    throw std::runtime_error("Invalid binary compatibility group");
                p.demands.push_back(std::move(d));
            }
            r.finish();
            return {{"import_metadata",
                     {{"block_upper_bound", flags ? "sum_compatible_quantities" : "explicit"}}}};
        }
        Json readInput(const std::string &path, Budget *budget, Problem &problem, bool &packed) {
            std::size_t events = 0;
            std::string top_key;
            auto read_record = [&](int depth, Json::parse_event_t event, Json &record) {
                if (budget && (++events % 4096 == 0))
                    budget->check();
                if (depth == 1 && event == Json::parse_event_t::key)
                    top_key = record.get<std::string>();
                if (top_key != "blocks" && top_key != "nodes" && top_key != "arcs")
                    return true;
                if (depth == 1 && event == Json::parse_event_t::array_start) {
                    if (top_key == "blocks")
                        problem.blocks.clear();
                    if (top_key == "nodes")
                        problem.nodes.clear();
                    if (top_key == "arcs")
                        problem.arcs.clear();
                }
                if (depth == 2 && event == Json::parse_event_t::value)
                    throw std::runtime_error("Network record must be an object");
                if (depth != 2 || event != Json::parse_event_t::object_end)
                    return true;
                if (top_key == "nodes") {
                    problem.nodes.push_back({record.at("yard"), record.value("tracks", 0.),
                                             record.value("handling_capacity", 0.),
                                             record.value("handling_cost", 0.)});
                    return false;
                }
                if (top_key == "arcs") {
                    problem.arcs.push_back({record.at("from"), record.at("to"), record.at("link"),
                                            record.at("length"), record.at("cost")});
                    return false;
                }
                Block block{record.at("from"),
                            record.at("to"),
                            record.at("slots"),
                            record.at("lower"),
                            record.at("upper"),
                            record.at("length_limit"),
                            record.at("opening"),
                            record.at("interchange"),
                            record.value("transport_factor", 1.),
                            {}};
                if (record.contains("arc_cost"))
                    for (auto it = record["arc_cost"].begin(); it != record["arc_cost"].end(); ++it)
                        block.arc_cost.emplace(std::stoi(it.key()), it.value());
                problem.blocks.push_back(std::move(block));
                if (problem.blocks.size() % 250000 == 0)
                    std::clog << "READ_BLOCKS count=" << problem.blocks.size() << '\n';
                // Release each JSON object immediately. Retaining it also makes the callback
                // parser scan an ever-growing parent array after every object.
                return false;
            };
            auto suffix = std::filesystem::path(path).extension().string();
            std::transform(suffix.begin(), suffix.end(), suffix.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            if (suffix == ".zip") {
                ZipInput buffer(path);
                std::istream input(&buffer);
                packed = buffer.binary_member;
                auto value = packed ? readPacked(input, budget, buffer.member_size, problem)
                                    : Json::parse(input, read_record);
                buffer.verify();
                return value;
            }
            std::ifstream input(path, std::ios::binary);
            if (!input)
                throw std::runtime_error("Cannot open instance: " + path);
            packed = suffix == ".irbp";
            return packed ? readPacked(input, budget, std::filesystem::file_size(path), problem)
                          : Json::parse(input, read_record);
        }
    } // namespace
    void Graph::build(int nodes, const std::vector<int> &tails, const std::vector<int> &heads) {
        from = tails;
        to = heads;
        offsets.assign(nodes + 1, 0);
        edge_ids.resize(tails.size());
        for (int i : tails)
            ++offsets.at(i + 1);
        for (int i = 1; i <= nodes; ++i)
            offsets[i] += offsets[i - 1];
        auto cur = offsets;
        for (int a = 0; a < (int)tails.size(); ++a)
            edge_ids[cur[tails[a]]++] = a;
    }
    double Problem::transport(int b, int a) const {
        auto it = blocks[b].arc_cost.find(a);
        return it == blocks[b].arc_cost.end() ? blocks[b].transport_factor * arcs[a].cost : it->second;
    }
    Problem Problem::read(const std::string &path, double demand_multiplier, Budget *budget) {
        if (!std::isfinite(demand_multiplier) || demand_multiplier <= 0)
            throw std::runtime_error("Demand multiplier must be positive and finite");
        Problem p;
        bool packed = false;
        Json j = readInput(path, budget, p, packed);
        if (!packed && !j.at("blocks").is_array())
            throw std::runtime_error("Blocks must be an array");
        const bool quantity_bounds =
            j.value("import_metadata", Json::object()).value("block_upper_bound", "") ==
            "sum_compatible_quantities";
        if (demand_multiplier != 1. && !quantity_bounds)
            throw std::runtime_error("Demand scaling requires imported quantity-derived block upper bounds");
        if (!packed) {
            p.name = j.at("name");
            if (!j.at("nodes").is_array() || !j.at("arcs").is_array())
                throw std::runtime_error("Network records must be arrays");
            p.capacities = j.at("link_capacities").get<std::vector<double>>();
            if (j.contains("compatibility_groups"))
                p.compatibility_groups = j.at("compatibility_groups").get<std::vector<std::vector<int>>>();
            for (auto &d : j.at("demands")) {
                if (d.contains("blocks") == d.contains("block_group"))
                    throw std::runtime_error("Demand requires either blocks or block_group");
                Demand demand{d.at("from"), d.at("to"), d.at("quantity"), d.at("penalty"), {}};
                if (d.contains("blocks"))
                    demand.blocks = d.at("blocks").get<std::vector<int>>();
                else {
                    demand.block_group = d.at("block_group").get<int>();
                    if (demand.block_group < 0 || demand.block_group >= int(p.compatibility_groups.size()))
                        throw std::runtime_error("Invalid compatibility group");
                }
                p.demands.push_back(std::move(demand));
            }
        }
        if (demand_multiplier != 1.) {
            for (auto &block : p.blocks)
                block.upper *= demand_multiplier;
            for (auto &demand : p.demands)
                demand.quantity *= demand_multiplier;
        }
        auto nonneg = [](double x) {
            if (!std::isfinite(x) || x < 0)
                throw std::runtime_error("Expected finite nonnegative parameter");
        };
        auto node = [&](int i) {
            if (i < 0 || i >= (int)p.nodes.size())
                throw std::runtime_error("Invalid node");
        };
        for (auto &n : p.nodes) {
            nonneg(n.tracks);
            nonneg(n.handling_capacity);
            nonneg(n.handling_cost);
        }
        for (double c : p.capacities)
            nonneg(c);
        std::vector<std::pair<int, int>> links(p.capacities.size(), {-1, -1});
        std::vector<int> tails, heads;
        for (auto &a : p.arcs) {
            node(a.from);
            node(a.to);
            nonneg(a.cost);
            nonneg(a.length);
            if (a.from == a.to || a.length <= 0 || a.link < 0 || a.link >= (int)links.size())
                throw std::runtime_error("Invalid physical arc");
            std::pair<int, int> ends = std::minmax(a.from, a.to);
            if (links[a.link].first >= 0 && links[a.link] != ends)
                throw std::runtime_error("A physical link must connect the same two nodes");
            links[a.link] = ends;
            tails.push_back(a.from);
            heads.push_back(a.to);
        }
        p.physical.build(p.nodes.size(), tails, heads);
        p.reverse_physical.build(p.nodes.size(), heads, tails);
        tails.clear();
        heads.clear();
        for (auto &b : p.blocks) {
            node(b.from);
            node(b.to);
            if (!p.nodes[b.from].yard || !p.nodes[b.to].yard || b.from == b.to || b.lower <= 0 ||
                (b.upper < b.lower && !quantity_bounds) || b.slots < 0 || b.slots > 1)
                throw std::runtime_error("Invalid block");
            for (double v : {b.lower, b.upper, b.length_limit, b.opening, b.interchange, b.transport_factor})
                nonneg(v);
            for (auto [a, c] : b.arc_cost) {
                if (a < 0 || a >= (int)p.arcs.size())
                    throw std::runtime_error("Invalid cost override");
                nonneg(c);
            }
            tails.push_back(b.from);
            heads.push_back(b.to);
        }
        p.logical.build(p.nodes.size(), tails, heads);

        CostSum rejection;
        for (int k = 0; k < (int)p.demands.size(); ++k) {
            auto &d = p.demands[k];
            node(d.from);
            node(d.to);
            if (!p.nodes[d.from].yard || !p.nodes[d.to].yard || d.from == d.to || d.quantity <= 0)
                throw std::runtime_error("Invalid demand");
            nonneg(d.quantity);
            nonneg(d.penalty);
            rejection += d.quantity * d.penalty;
        }
        p.rejection = rejection.value;
        p.indexCompatibility();
        if (budget)
            budget->check();
        return p;
    }
    void Problem::indexCompatibility() {
        // Explicit input remains valid; normalize it once, not once per consumer.
        for (auto &d : demands)
            if (d.block_group < 0) {
                d.block_group = int(compatibility_groups.size());
                compatibility_groups.push_back(std::move(d.blocks));
            }
        group_demands.assign(compatibility_groups.size(), {});
        for (int k = 0; k < int(demands.size()); ++k)
            group_demands.at(demands[k].block_group).push_back(k);
        std::vector<int> bs, gs;
        for (int g = 0; g < int(compatibility_groups.size()); ++g) {
            auto &members = compatibility_groups[g];
            if (!std::is_sorted(members.begin(), members.end()))
                std::sort(members.begin(), members.end());
            if (std::adjacent_find(members.begin(), members.end()) != members.end())
                throw std::runtime_error("Duplicate compatibility");
            for (int b : members) {
                if (b < 0 || b >= int(blocks.size()))
                    throw std::runtime_error("Invalid compatible block");
                bs.push_back(b);
                gs.push_back(g);
            }
        }
        block_groups.build(int(blocks.size()), bs, gs);
        block_demands.clear();
    }
    std::vector<int> Problem::compatibleDemands(int b) const {
        if (block_groups.offsets.empty())
            return block_demands.at(b);
        std::vector<int> result;
        for (int e = block_groups.offsets[b]; e < block_groups.offsets[b + 1]; ++e) {
            int g = block_groups.to[block_groups.edge_ids[e]];
            result.insert(result.end(), group_demands[g].begin(), group_demands[g].end());
        }
        return result;
    }
    Plan Plan::empty(const Problem &p) {
        return {std::vector<std::vector<int>>(p.demands.size()),
                std::vector<std::vector<int>>(p.blocks.size()), std::vector<double>(p.demands.size(), 0)};
    }
    double Plan::validate(const Problem &p) const {
        if (logical.size() != p.demands.size() || served.size() != p.demands.size() ||
            physical.size() != p.blocks.size())
            throw std::runtime_error("Plan dimensions");
        std::vector<double> loads(p.blocks.size()), links(p.capacities.size()), handles(p.nodes.size()),
            tracks(p.nodes.size());
        CostSum obj;
        auto bounded = [](double x, double u) {
            if (!std::isfinite(x) || x < -1e-6 || x > u + 1e-6 * std::max(1., u))
                throw std::runtime_error("Plan capacity violation");
        };
        auto traverse = [&](const std::vector<int> &edges, const Graph &g, int from, int to) {
            std::vector<bool> seen(p.nodes.size());
            seen[from] = true;
            int at = from;
            for (int a : edges) {
                if (a < 0 || a >= (int)g.from.size() || g.from[a] != at || seen[g.to[a]])
                    throw std::runtime_error("Plan path violation");
                at = g.to[a];
                seen[at] = true;
            }
            if (at != to)
                throw std::runtime_error("Plan disconnected path");
        };
        for (int k = 0; k < (int)p.demands.size(); ++k) {
            auto &d = p.demands[k];
            double q = served[k];
            bounded(q, d.quantity);
            obj += d.penalty * (d.quantity - q);
            if (logical[k].empty()) {
                if (q > 1e-6)
                    throw std::runtime_error("Volume without logical path");
                continue;
            }
            traverse(logical[k], p.logical, d.from, d.to);
            for (int b : logical[k]) {
                if (!p.compatible(k, b) || physical[b].empty())
                    throw std::runtime_error("Compatibility/activation violation");
                loads[b] += q;
                int i = p.blocks[b].from;
                if (i != d.from) {
                    handles[i] += q;
                    obj += q * p.nodes[i].handling_cost;
                }
            }
        }
        for (int b = 0; b < (int)p.blocks.size(); ++b) {
            auto &x = p.blocks[b];
            if (physical[b].empty()) {
                if (loads[b] > 1e-6)
                    throw std::runtime_error("Closed loaded block");
                continue;
            }
            traverse(physical[b], p.physical, x.from, x.to);
            if (loads[b] + 1e-6 < x.lower)
                throw std::runtime_error("Minimum block load");
            bounded(loads[b], x.upper);
            double length = 0;
            tracks[x.from] += x.slots;
            obj += x.opening + x.interchange * loads[b];
            for (int a : physical[b]) {
                length += p.arcs[a].length;
                links[p.arcs[a].link] += loads[b];
                obj += p.transport(b, a) * loads[b];
            }
            bounded(length, x.length_limit);
        }
        for (int i = 0; i < (int)p.nodes.size(); ++i) {
            bounded(handles[i], p.nodes[i].handling_capacity);
            bounded(tracks[i], p.nodes[i].tracks);
        }
        for (int e = 0; e < (int)links.size(); ++e)
            bounded(links[e], p.capacities[e]);
        return obj.value;
    }
    Json Plan::json() const {
        return {{"logical_paths", logical}, {"physical_paths", physical}, {"served", served}};
    }
    Json Stats::json() const {
        return {{"labels", labels},
                {"dominated", dominated},
                {"searches", searches},
                {"heuristic_searches", heuristic_searches},
                {"nodes", nodes},
                {"peak_labels", peak_labels},
                {"peak_label_bytes", peak_label_bytes},
                {"columns", columns},
                {"rows", rows},
                {"solver_seconds", solver_seconds},
                {"build_seconds", build_seconds},
                {"search_seconds", search_seconds},
                {"time_best_solution", first_best}};
    }
    Json Result::json(const Problem &p) const {
        return {{"instance", p.name},
                {"method", method},
                {"status", status},
                {"objective", objective},
                {"lower_bound", bound},
                {"seconds", seconds},
                {"gap", (objective - bound) / std::max(1., std::abs(objective))},
                {"validated_objective", plan.validate(p)},
                {"stats", stats.json()},
                {"solution", plan.json()},
                {"feasibility_tolerance", FeasTol},
                {"pricing_relative_tolerance", RelativeRcTol}};
    }
} // namespace RouteOpt::Application::IRBP

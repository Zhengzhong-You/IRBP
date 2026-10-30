# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""Generate the railroad node/link/demand/setting CSV instances.

Usage:
    python instance_generator.py --nodes 200 --seeds 20261014
    python instance_generator.py --nodes 200 600 1200 --seeds 211 1701 20261014
    python instance_generator.py --reference /path/to/L1 --output /path/to/instances

Dependencies: numpy==1.26.4, scipy==1.11.4, pandas==2.2.3, networkx==3.4.2,
NEMtropy==4.0.0, numba==0.61.2, llvmlite==0.44.0, powerlaw==1.5,
bicm==3.4.0, threadpoolctl.

The topology components are adapted from GenCAT at
https://github.com/seijimaekawa/GenCAT at
6f88444ce3ae60ab7fc72cf48b7b1100c63b385d (Maekawa et al., 2023).
Unused outputs and progress code have been removed, and names and comments
have been clarified. The preference adjustment and inverse-cumulative sampling
rules are retained.
Link insertion counts each undirected edge once, and yard allocations are checked
against every commodity-specific endpoint-level quota before demand sampling.
NEMtropy is imported as a library; the verified source commit is
b007d7bd8be674ec2e95bdfced0b9ee36fcdfa27 (Vallarano et al., 2021).
Geographical/yard assignment, OD selection and quantile calibration are local
railroad adapters. See Section EC.3 of the paper.
"""
from __future__ import annotations
import argparse
import contextlib
import io
import json
import math
import os
import random
import shutil
import tempfile
from pathlib import Path
from statistics import mean, stdev

os.environ.setdefault("NUMBA_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np
import pandas as pd
import networkx as nx
from numpy import linalg as la
from scipy import sparse
from scipy.cluster.vq import kmeans2
from scipy.optimize import linear_sum_assignment
from scipy.sparse.csgraph import dijkstra
from threadpoolctl import threadpool_limits
import NEMtropy as nem

HOME = Path(__file__).resolve().parent
BASE = HOME / "l1"  # pass --reference with the RAS L1 data set
TYPES = ["Merchandise", "Intermodal", "Coal", "Grain", "Automobile"]
REGIONS = 6
SPATIAL_NOISE = 0.035
FIT_TOLERANCE = 1e-5

# GenCAT topology components; source and commit are cited in the module docstring.
def latent_factor_gen(n, k, M, D, com_size):
    """Sample node classes and their normalized regional preferences."""
    preferences = np.zeros((n, k))
    classes = []
    for i in range(n):
        region = random.choices(list(range(k)), k=1, weights=com_size)[0]
        classes.append(region)
        for h in range(k):
            preferences[i, h] = np.random.normal(
                loc=M[region][h], scale=D[region][h], size=1
            )[0]

    below_zero = np.where(preferences < 0)
    for index in range(len(below_zero[0])):
        preferences[below_zero[0][index], below_zero[1][index]] = 0
    above_one = np.where(preferences > 1)
    for index in range(len(above_one[0])):
        preferences[above_one[0][index], above_one[1][index]] = 1
    for i in range(n):
        preferences[i] /= sum(preferences[i])
    return preferences, classes


def adjust(n, k, U, C, M):
    """Adjust latent preferences using the GenCAT temperature-grid rule."""
    endpoint_preferences = U.copy()
    partition = [[] for _ in range(k)]
    for i in range(len(C)):
        partition[C[i]].append(i)

    def freeze(values, temperature):
        return values ** (1 / temperature) / np.sum(values ** (1 / temperature))

    def inverse(values, own_region):
        transformed = 1 - values
        normalizer = sum(transformed) - values[own_region]
        for h in range(k):
            if h != own_region:
                transformed[h] = transformed[h] * values[own_region] / normalizer
        return transformed

    for region in range(k):
        best_loss = float("inf")
        assortative = M[region][region] >= 1 / k
        for temperature in np.arange(0.01, 1, 0.05):
            estimated = np.zeros(k)
            for i in partition[region]:
                frozen = freeze(U[i], temperature)
                endpoint = frozen if assortative else inverse(frozen, region)
                estimated += frozen * endpoint
            loss = la.norm(M[region] - estimated / len(partition[region]))
            if loss < best_loss:
                best_loss = loss
                best_temperature = temperature
        for i in partition[region]:
            U[i] = freeze(U[i], best_temperature)
            endpoint_preferences[i] = U[i] if assortative else inverse(U[i], region)
    return U, endpoint_preferences


def edge_construction(n, U, k, U_prime, step, theta, r):
    """Construct undirected links through bounded regional endpoint sampling."""
    adjacency = sparse.dok_matrix((n, n))
    degree_counters = np.zeros(n)
    for i in range(n):
        rounds = 0
        examined = {i}
        while rounds < r and degree_counters[i] < theta[i]:
            regions = random.choices(
                list(range(k)),
                k=int(theta[i] - degree_counters[i]),
                weights=U[i],
            )
            for region in regions:
                for _ in range(50):
                    j = U_prime[region][int(random.random() / step)]
                    if j not in examined:
                        examined.add(j)
                        break
                if i != j and degree_counters[j] < theta[j] and (i, j) not in adjacency:
                    adjacency[i, j] = 1
                    adjacency[j, i] = 1
                    degree_counters[i] += 1
                    degree_counters[j] += 1
            rounds += 1
    return adjacency


def ITS_U_prime(n, k, U_prime, step):
    """Build the discretized endpoint lookup tables used by GenCAT."""
    lookup_tables = []
    transposed = U_prime.transpose()
    for region in range(k):
        cumulative = transposed[region] / sum(transposed[region])
        for j in range(n - 1):
            cumulative[j + 1] += cumulative[j]

        lookup = []
        node_index = 0
        for probability in np.arange(0, 1, step):
            if node_index >= n - 1:
                lookup.append(n - 1)
            elif cumulative[node_index] > probability:
                lookup.append(node_index)
            else:
                node_index += 1
                lookup.append(node_index)
        lookup_tables.append(lookup)
    return lookup_tables


def class_reproduction(k, S, Label):
    """Estimate regional neighbor-share means, deviations, and class sizes."""
    preferences = np.zeros((len(Label), k))
    rows, columns = S.nonzero()
    for index in range(len(rows)):
        if rows[index] < columns[index]:
            preferences[rows[index]][Label[columns[index]]] += 1
            preferences[columns[index]][Label[rows[index]]] += 1
    for i in range(len(Label)):
        preferences[i] /= sum(preferences[i])

    partition = [[] for _ in range(k)]
    for i in range(len(Label)):
        partition[Label[i]].append(i)

    means = np.zeros((k, k))
    deviations = np.zeros((k, k))
    for region in range(k):
        regional_preferences = []
        for i in partition[region]:
            regional_preferences.append(preferences[i])
        regional_preferences = np.array(regional_preferences).transpose()
        for h in range(k):
            means[region, h] = mean(regional_preferences[h])
            deviations[region, h] = stdev(regional_preferences[h])

    class_sizes = []
    for members in partition:
        class_sizes.append(len(members))
    class_sizes = np.array(class_sizes) / sum(class_sizes)
    return means, deviations, class_sizes


# End GenCAT topology components.

def corridor_graph(node, link):
    """Contract transit chains while retaining yards, gates, and bottlenecks."""
    attrs = {int(r.node_id): r._asdict() for r in node.itertuples(index=False)}
    graph = nx.Graph()
    graph.add_nodes_from(attrs)
    for row in link.itertuples(index=False):
        u, v = int(row.from_node_id), int(row.to_node_id)
        length, capacity = float(row.length), float(row.capacity)
        if u == v:
            continue
        if graph.has_edge(u, v):
            if length < graph[u][v]["length"]:
                graph[u][v].update(length=length, capacity=capacity)
        else:
            graph.add_edge(u, v, length=length, capacity=capacity)
    components = sorted(nx.connected_components(graph), key=len, reverse=True)
    graph = graph.subgraph(components[0]).copy()
    retained = {
        u for u in graph
        if attrs[u]["node_type"] != "micro" or graph.degree(u) != 2
    }
    result = nx.Graph()
    result.add_nodes_from((u, attrs[u]) for u in sorted(retained))
    seen = set()
    for start in sorted(retained):
        for neighbor in graph.neighbors(start):
            key = tuple(sorted((start, neighbor)))
            if key in seen:
                continue
            seen.add(key)
            previous, current = start, neighbor
            length = graph[start][neighbor]["length"]
            capacity = graph[start][neighbor]["capacity"]
            while current not in retained:
                nxt = next(v for v in graph.neighbors(current) if v != previous)
                seen.add(tuple(sorted((current, nxt))))
                length += graph[current][nxt]["length"]
                capacity = min(capacity, graph[current][nxt]["capacity"])
                previous, current = current, nxt
            if current == start:
                continue
            if result.has_edge(start, current):
                # Keep the shortest corridor and its own bottleneck capacity.
                if length < result[start][current]["length"]:
                    result[start][current].update(length=length, capacity=capacity)
            else:
                result.add_edge(start, current, length=length, capacity=capacity)
    return result, graph, len(components)


def hamilton(weights, n):
    """Allocate an integer total by proportional floors and largest remainders."""
    p = np.asarray(weights, float)
    p /= p.sum()
    result = np.floor(n * p).astype(int)
    rem = n - result.sum()
    result[np.argsort(-(n * p - result))[:rem]] += 1
    return result


def sparse_dist(g, sources=None, predecessors=False):
    """Return physical shortest-path distances in graph node order."""
    a = nx.to_scipy_sparse_array(g, nodelist=list(g), weight="length", format="csr")
    a.indices = a.indices.astype(np.int32)
    a.indptr = a.indptr.astype(np.int32)
    return dijkstra(a, directed=False, indices=sources, return_predecessors=predecessors)


def reference(base=BASE):
    """Load the reference statistics used by topology, facilities, and demands."""
    node = pd.read_csv(base / "node.csv").fillna("")
    link = pd.read_csv(base / "link.csv").fillna("")
    demand = pd.read_csv(base / "demand.csv").fillna("")
    g, _, _ = corridor_graph(node, link)
    order = list(g)
    idx = {u: i for i, u in enumerate(order)}
    nd = node.set_index("node_id").loc[order].copy()
    # Local equirectangular coordinates in miles, retained as an explicit adapter.
    xy = np.column_stack([(nd.x_coord.to_numpy(float) + 98) * 69.172 * np.cos(np.deg2rad(39)),
                          (nd.y_coord.to_numpy(float) - 39) * 69.0])
    np.random.seed(20261014)
    _, labels = kmeans2(xy, REGIONS, minit="++", iter=50)
    s = nx.to_scipy_sparse_array(g, nodelist=order, weight=None, format="csr")
    m, dev, sizes = class_reproduction(REGIONS, s, labels.tolist())
    yards = [u for u in order if g.nodes[u]["node_type"] == "yard"]
    d = sparse_dist(g, [idx[u] for u in yards])
    yi = {u: i for i, u in enumerate(yards)}
    # Cache exact distances for all distinct yard pairs.
    yd = d[:, [idx[u] for u in yards]]
    pairdist = yd[np.triu_indices(len(yards), 1)]
    return dict(node=node, demand=demand, g=g, nd=nd, xy=xy, labels=labels,
                M=m, D=dev, sizes=sizes,
                degrees=np.array([g.degree(u) for u in order]), yards=yards,
                pairdist=pairdist, yarddist=yd, yi=yi)


def demand_quotas(ref, count):
    """Allocate demand counts by commodity and endpoint yard levels."""
    rd = ref["demand"].copy()
    nodes = ref["node"].set_index("node_id")
    rd["ol"] = rd.origin_yard_id.map(nodes.yard_level).astype(int)
    rd["dl"] = rd.dest_yard_id.map(nodes.yard_level).astype(int)
    out = []
    commodity_counts = hamilton(
        rd.block_type.value_counts().reindex(TYPES, fill_value=0), count
    )
    for typ, quota in zip(TYPES, commodity_counts):
        groups = rd[rd.block_type == typ].groupby(["ol", "dl"]).size()
        for (ol, dl), q in zip(groups.index, hamilton(groups.to_numpy(), int(quota))):
            if q:
                out.append((typ, int(ol), int(dl), int(q)))
    return out


def validate_od_quotas(counts, quotas):
    """Check distinct ordered-pair capacity for every commodity and level pair."""
    for commodity, origin_level, destination_level, required in quotas:
        origins = int(counts[origin_level - 1])
        destinations = int(counts[destination_level - 1])
        available = origins * (
            origins - 1 if origin_level == destination_level else destinations
        )
        if required > available:
            raise ValueError(
                f"Insufficient distinct OD pairs for {commodity}, "
                f"yard levels {origin_level}->{destination_level}: "
                f"required {required}, available {available} "
                f"with yard counts {list(map(int, counts))}"
            )


def generate_topology(ref, n, seed, demand_count):
    """Generate physical links, joint yard attributes, and scaled yard distances."""
    random.seed(seed)
    np.random.seed(seed)
    u, classes = latent_factor_gen(n, REGIONS, ref["M"], ref["D"], ref["sizes"])
    u, uprime = adjust(n, REGIONS, u, classes, ref["M"])
    rng = np.random.default_rng(seed)
    donor = np.empty(n, dtype=int)
    # Evenly spaced empirical degree quantiles supply node records within regions.
    for c in range(REGIONS):
        at = np.where(np.asarray(classes) == c)[0]
        pool = np.where(ref["labels"] == c)[0]
        pool = pool[np.argsort(ref["degrees"][pool], kind="stable")]
        choices = np.minimum(
            len(pool) - 1,
            ((np.arange(len(at)) + .5) / len(at) * len(pool)).astype(int),
        )
        chosen = pool[choices]
        rng.shuffle(chosen)
        donor[at] = chosen
    target = ref["degrees"][donor].astype(int)
    if target.sum() % 2:
        target[np.argmax(target)] -= 1
    # GenCAT processes higher target degrees first.
    perm = np.argsort(-target, kind="stable")
    u, uprime, target, donor = u[perm], uprime[perm], target[perm], donor[perm]
    classes = np.asarray(classes)[perm]
    step = 1 / (n * 100)
    cdf = ITS_U_prime(n, REGIONS, uprime, step)
    s = edge_construction(n, u, REGIONS, cdf, step, target, 50)
    original = nx.from_scipy_sparse_array(s.tocsr())
    components = sorted(nx.connected_components(original), key=len, reverse=True)
    keep = sorted(components[0])
    g = nx.convert_node_labels_to_integers(original.subgraph(keep).copy(), ordering="sorted")
    classes, donor = classes[keep], donor[keep]
    # Local coordinate noise varies layouts within the sampled geographical regions.
    points = ref["xy"][donor].copy()
    for c in range(REGIONS):
        mask = classes == c
        sigma = np.std(ref["xy"][ref["labels"] == c], axis=0) * SPATIAL_NOISE
        points[mask] += rng.normal(size=(mask.sum(), 2)) * sigma
    attrs = ref["nd"].iloc[donor].copy().reset_index(drop=True)
    attrs["node_id"] = np.arange(1, len(g) + 1)
    attrs["name"] = [f"SYN_{i+1:04d}" for i in range(len(g))]
    attrs["datasource"] = "GenCAT topology; empirical conditional facility resampling"
    yards = np.where(attrs.node_type.to_numpy() == "yard")[0]
    if len(yards) < 8:
        raise ValueError("Too few yards in this topology/facility draw")
    real_yards = ref["nd"].loc[ref["yards"]]
    levels = real_yards.yard_level.to_numpy(float).astype(int)
    counts = hamilton([(levels == level).sum() for level in [1, 2, 3]], len(yards))
    quotas = demand_quotas(ref, demand_count)
    required = [0, 0, 0]
    # Same-level demands need n_level * (n_level - 1) distinct ordered pairs.
    for _, ol, dl, q in quotas:
        if ol == dl:
            required[ol - 1] = max(
                required[ol - 1], math.ceil((1 + math.sqrt(1 + 4 * q)) / 2)
            )
    for level in [1,2]:
        extra = max(0, required[level - 1] - counts[level - 1])
        counts[level - 1] += extra
        counts[2] -= extra
    if counts[2] < 0:
        raise ValueError("Requested demands require more high-level facilities than available yards")
    validate_od_quotas(counts, quotas)
    facility_rows = []
    for level, quota in enumerate(counts, 1):
        pool = np.where(
            (ref["nd"].node_type.to_numpy() == "yard")
            & (pd.to_numeric(ref["nd"].yard_level, errors="coerce").to_numpy() == level)
        )[0]
        facility_rows.extend(rng.choice(pool, size=int(quota), replace=True).tolist())
    real_degree = ref["degrees"][facility_rows]
    synth_degree = np.array([g.degree(int(i)) for i in yards])
    cost = np.abs(np.log1p(synth_degree[:, None]) - np.log1p(real_degree[None, :])) * 3
    cost += np.linalg.norm(
        points[yards, None, :] - ref["xy"][facility_rows][None, :, :], axis=2
    ) / 1500
    rr, cc = linear_sum_assignment(cost)
    fields = ["yard_type", "yard_level", "railroad_id", "num_tracks", "handling_capacity",
              "handling_cost", "is_interchange", "allowed_commodities", "allowed_traversal"]
    for a, b in zip(rr, cc):
        donorrow = ref["nd"].iloc[facility_rows[int(b)]]
        for f in fields:
            attrs.loc[int(yards[a]), f] = donorrow[f]
    for a, b in g.edges():
        g[a][b]["length"] = max(.01, float(np.linalg.norm(points[a] - points[b])))
    yd = sparse_dist(g, yards.tolist())[:, yards]
    scale = float(np.median(ref["pairdist"]) / np.median(yd[np.triu_indices(len(yards), 1)]))
    for a, b in g.edges():
        g[a][b]["length"] *= scale
    yd *= scale
    # Export longitude/latitude coordinates and physical link lengths in miles.
    points *= scale
    attrs["x_coord"] = points[:, 0] / (69.172 * np.cos(np.deg2rad(39))) - 98
    attrs["y_coord"] = points[:, 1] / 69 + 39
    edges = list(g.edges())
    cap = np.array([z["capacity"] for _, _, z in ref["g"].edges(data=True)])
    capacities = rng.choice(cap, size=len(edges))
    li = []
    for eid, ((a, b), capacity) in enumerate(zip(edges, capacities), 1):
        li.append(dict(link_id=eid, from_node_id=a + 1, to_node_id=b + 1,
                       length=g[a][b]["length"], capacity=capacity,
                       railroad_id=-1, free_speed=30, tracks=2, geometry=""))
    links = pd.DataFrame(li)
    return attrs, links, yards, yd


def template_demands(ref, attrs, yards, yd, count, seed):
    """Select distinct OD pairs by commodity, yard level, distance, and operator."""
    rng = np.random.default_rng(seed)
    rd = ref["demand"]
    yattrs = attrs.iloc[yards].reset_index(drop=True)
    refnodes = ref["node"].set_index("node_id")
    # A donor is mapped to a synthetic facility with a matching hierarchy level.
    # Pair selection also conditions on the reference OD distance and operator relation.
    selected = []
    used_by_type = {typ: set() for typ in TYPES}
    for typ, ol, dl, quota in demand_quotas(ref, count):
        source = rd[(rd.block_type == typ)
                    & (rd.origin_yard_id.map(refnodes.yard_level) == ol)
                    & (rd.dest_yard_id.map(refnodes.yard_level) == dl)]
        used = used_by_type[typ]
        for _ in range(quota):
            row = source.iloc[int(rng.integers(len(source)))]
            ro, rt = refnodes.loc[int(row.origin_yard_id)], refnodes.loc[int(row.dest_yard_id)]
            oi = np.where(yattrs.yard_level.to_numpy(float) == float(ro.yard_level))[0]
            ti = np.where(yattrs.yard_level.to_numpy(float) == float(rt.yard_level))[0]
            if not len(oi) or not len(ti):
                raise ValueError(f"Missing endpoint hierarchy level for {typ}")
            pairs = [(a, b) for a in oi for b in ti if a != b and (a, b) not in used]
            if not pairs:
                raise ValueError(f"Insufficient distinct OD pairs at required levels for {typ}")
            rp = ref["yi"][int(row.origin_yard_id)], ref["yi"][int(row.dest_yard_id)]
            desired = ref["yarddist"][rp]
            cross = str(ro.railroad_id) != str(rt.railroad_id)
            distance = np.array([yd[a, b] for a, b in pairs])
            penalty = np.array([float((str(yattrs.iloc[a].railroad_id) != str(yattrs.iloc[b].railroad_id)) != cross)
                                for a, b in pairs])
            weights = np.exp(-np.abs(np.log(np.maximum(distance, 1) / max(desired, 1))) * 3 - penalty * 1.5)
            choice = int(rng.choice(len(pairs), p=weights / weights.sum()))
            a, b = pairs[choice]
            used.add((a, b))
            selected.append(dict(origin=a, destination=b, volume=float(row.volume), block_type=typ))
    return selected


def fit_volume_model(weights):
    """Fit fixed-support CReMa rates and check relative yard-total residuals."""
    model = nem.DirectedGraph(
        degree_sequence=np.r_[(weights > 0).sum(1), (weights > 0).sum(0)],
        strength_sequence=np.r_[weights.sum(1), weights.sum(0)],
    )
    model.solve_tool(
        model="crema", method="quasinewton", initial_guess="strengths_minor",
        adjacency=(weights > 0).astype(float), max_steps=2000, tol=1e-8, eps=1e-10,
    )

    def residual():
        expected = np.zeros_like(weights)
        for a, b in zip(*np.nonzero(weights)):
            rate = model.b_out[a] + model.b_in[b]
            if not np.isfinite(rate) or rate <= 0:
                return math.inf
            expected[a, b] = 1 / rate
        return max(
            np.max(np.abs(expected.sum(0) - weights.sum(0))),
            np.max(np.abs(expected.sum(1) - weights.sum(1))),
        )

    norm = max(1, weights.sum(0).max(), weights.sum(1).max())
    if residual() / norm > FIT_TOLERANCE:
        model.solve_tool(
            model="crema", method="newton",
            initial_guess=np.r_[model.b_out, model.b_in],
            adjacency=(weights > 0).astype(float), max_steps=1000, tol=1e-9, eps=1e-12,
        )
    relative_error = float(residual() / norm)
    if not np.isfinite(relative_error) or relative_error > FIT_TOLERANCE:
        raise ValueError(f"CReMa relative yard-total residual exceeds tolerance: {relative_error}")
    return model, relative_error


def generate_demands(ref, attrs, yards, yd, count, seed, folder):
    """Generate final volumes and return the largest relative fitting residual."""
    templates = template_demands(ref, attrs, yards, yd, count, seed)
    raw = []
    max_relative_error = 0.0
    for typ in TYPES:
        rows = [r for r in templates if r["block_type"] == typ]
        if not rows:
            continue
        active = sorted({r["origin"] for r in rows} | {r["destination"] for r in rows})
        loc = {v: i for i, v in enumerate(active)}
        scale = float(np.mean([r["volume"] for r in rows]))
        w = np.zeros((len(active), len(active)))
        for r in rows:
            w[loc[r["origin"]], loc[r["destination"]]] = r["volume"] / scale
        with contextlib.redirect_stdout(io.StringIO()):
            model, relative_error = fit_volume_model(w)
            max_relative_error = max(max_relative_error, relative_error)
            # NEMtropy writes its ensemble sample to disk before we read the weights.
            sampledir = folder / typ
            sampledir.mkdir(parents=True, exist_ok=True)
            model.ensemble_sampler(1, cpu_n=1, output_dir=str(sampledir) + os.sep, seed=seed + TYPES.index(typ))
        sampled = np.loadtxt(sampledir / "0.txt", ndmin=2)
        for a, b, value in sampled:
            raw.append(dict(origin=int(yards[active[int(a)]]) + 1,
                            destination=int(yards[active[int(b)]]) + 1,
                            volume=float(value * scale), block_type=typ))
    out = pd.DataFrame(raw)
    out.insert(0, "demand_id", np.arange(1, len(out) + 1))
    out.rename(columns={"origin": "origin_yard_id", "destination": "dest_yard_id"}, inplace=True)
    # Raw CReMa weights set the rank; empirical quantiles set final shipment sizes.
    for typ in TYPES:
        at = out.index[out.block_type == typ].to_numpy()
        if not len(at):
            continue
        ranks = np.argsort(out.loc[at, "volume"].to_numpy(), kind="stable")
        q = np.quantile(ref["demand"].loc[ref["demand"].block_type == typ, "volume"],
                        (np.arange(len(at)) + .5) / len(at))
        out.loc[at[ranks], "volume"] = np.maximum(1, np.rint(q))
    return out, max_relative_error


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--reference', type=Path, default=BASE)
    parser.add_argument('--output', type=Path, default=HOME)
    parser.add_argument('--nodes', type=int, nargs='+', default=[200])
    parser.add_argument('--seeds', type=int, nargs='+', default=[20261014])
    parser.add_argument('--demands', type=int, help='Override the demand count for each requested node size')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    base = args.reference.resolve()
    output = args.output.resolve()
    filenames = ['node.csv', 'link.csv', 'demand.csv', 'setting.csv']
    for filename in filenames:
        if not (base / filename).exists():
            parser.error('Reference CSV missing: ' + str(base / filename))
    if args.demands is not None and args.demands <= 0:
        parser.error('--demands must be positive')
    scales = {200: 12, 600: 60, 1200: 150}
    for n in args.nodes:
        if n <= 0:
            parser.error('--nodes must be positive')
        if n not in scales and args.demands is None:
            parser.error('Supply --demands for a custom node count')
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='irbp-generation-') as temporary:
        with threadpool_limits(limits=2):
            ref = reference(base)
            for n in args.nodes:
                for seed in args.seeds:
                    name = f'v2_gencat_n{n}_s{seed}'
                    scratch = Path(temporary) / name
                    scratch.mkdir()
                    count = args.demands if args.demands is not None else scales[n]
                    attrs, links, yards, yd = generate_topology(ref, n, seed, count)
                    demand, fit_error = generate_demands(
                        ref, attrs, yards, yd, count, seed + 5000, scratch
                    )
                    csvs = scratch / 'csv'
                    csvs.mkdir()
                    attrs.to_csv(csvs / 'node.csv', index=False)
                    links.to_csv(csvs / 'link.csv', index=False)
                    demand.to_csv(csvs / 'demand.csv', index=False)
                    shutil.copy2(base / 'setting.csv', csvs / 'setting.csv')
                    target = output / name
                    for filename in filenames:
                        existing = target / filename
                        if existing.exists() and existing.read_bytes() != (csvs / filename).read_bytes() and not args.overwrite:
                            raise FileExistsError(
                                'Different existing CSV; choose another --output or pass --overwrite: '
                                + str(existing)
                            )
                    target.mkdir(parents=True, exist_ok=True)
                    for filename in filenames:
                        shutil.copy2(csvs / filename, target / filename)
                    print(json.dumps(dict(
                        instance=name, nodes=len(attrs),
                        node_types=attrs.node_type.value_counts().to_dict(),
                        links=len(links), demands=len(demand),
                        total_volume=float(demand.volume.sum()),
                        max_strength_residual_normalized=fit_error, output=str(target),
                    ), ensure_ascii=False))

if __name__ == '__main__':
    main()

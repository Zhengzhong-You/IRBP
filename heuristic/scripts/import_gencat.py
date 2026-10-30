#!/usr/bin/env python3
# Copyright (c) 2026 Zhengzhong (Ricky) You.
# All rights reserved.
# Software: RouteOpt
# Application: IRBP

"""Import CSV with one block per ordered yard pair and commodity type."""
import argparse
from array import array
import csv
import hashlib
import heapq
import json
import math
from pathlib import Path
import time

DIRECT = {'intermodal', 'automobile'}
CLASS_I = {'BNSF', 'CN', 'CSX', 'UP', 'NS', 'CPKC'}


def yard_distances(adj, yards):
    """Only retain yard-to-yard distances; parallel arcs use their minimum length."""
    try:
        import numpy as np
        from scipy.sparse import csr_matrix
        from scipy.sparse.csgraph import dijkstra
    except ImportError:
        np = None
    distances = []
    if np is not None:
        edges = {(i, j): length for i, row in enumerate(adj) for j, length in row.items()}
        tails, heads = zip(*edges) if edges else ([], [])
        graph = csr_matrix((list(edges.values()), (tails, heads)), shape=(len(adj), len(adj)))
        for start in range(0, len(yards), 32):
            d = dijkstra(graph, directed=True, indices=yards[start:start + 32])
            distances.extend(array('d', row[yards]) for row in d)
            print(f'DISTANCE {len(distances)} {len(yards)}', flush=True)
        return distances
    for source in yards:
        d = [math.inf] * len(adj)
        d[source] = 0.
        queue = [(0., source)]
        while queue:
            cost, i = heapq.heappop(queue)
            if cost != d[i]:
                continue
            for j, length in adj[i].items():
                value = cost + length
                if value < d[j]:
                    d[j] = value
                    heapq.heappush(queue, (value, j))
        distances.append(array('d', (d[i] for i in yards)))
        if len(distances) % 32 == 0 or len(distances) == len(yards):
            print(f'DISTANCE {len(distances)} {len(yards)}', flush=True)
    return distances


def convert(root, output, profile='gencat', max_blocks=100000):
    started = time.monotonic()
    if output.exists() or output.with_suffix('.json.partial').exists():
        raise ValueError(f'Refusing to overwrite converted data: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)
    names = ['node.csv', 'link.csv', 'demand.csv', 'setting.csv']
    hashes = {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in names}
    def read(name):
        with (root / name).open(encoding='utf-8-sig', newline='') as f:
            return list(csv.DictReader(f))
    nodes, links, demands = (read(name) for name in names[:3])
    params = {s['parameter']: float(s['value']) for s in read('setting.csv')}
    ids = {int(n['node_id']): i for i, n in enumerate(nodes)}
    if len(ids) != len(nodes):
        raise ValueError('Duplicate node IDs')
    yards = [i for i, n in enumerate(nodes) if n['node_type'] == 'yard']
    yard_index = {i: y for y, i in enumerate(yards)}
    types = [d['block_type'].strip().lower() for d in demands]
    origins = [ids[int(d['origin_yard_id'])] for d in demands]
    destinations = [ids[int(d['dest_yard_id'])] for d in demands]
    quantities = [float(d['volume']) * params.get('demand_multiplier', 1.) for d in demands]
    groups, group_keys, group_for_demand = [], {}, []
    for k, typ in enumerate(types):
        i, j = origins[k], destinations[k]
        if i not in yard_index or j not in yard_index or i == j or quantities[k] <= 0:
            raise ValueError(f'Invalid demand {demands[k]["demand_id"]}')
        key = (typ, i, j) if typ in DIRECT else (typ,)
        if key not in group_keys:
            group_keys[key] = len(groups)
            groups.append({'key': key, 'demands': []})
        g = group_keys[key]
        groups[g]['demands'].append(k)
        group_for_demand.append(g)
    count = sum(1 if len(g['key']) == 3 else len(yards) * (len(yards) - 1) for g in groups)
    preflight = {'instance': root.name, 'profile': profile, 'nodes': len(nodes), 'yards': len(yards),
                 'input_links': len(links), 'demands': len(demands), 'single_block_candidates': count,
                 'source_hashes': hashes}
    output.with_suffix('.preflight.json').write_text(json.dumps(preflight, indent=2) + '\n')
    print(json.dumps(preflight), flush=True)
    if count > max_blocks:
        raise ValueError(f'{count} candidates exceed --max-blocks {max_blocks}')
    adj = [dict() for _ in nodes]
    arcs, capacities = [], []
    for e, row in enumerate(links):
        i, j = ids[int(row['from_node_id'])], ids[int(row['to_node_id'])]
        length, capacity = float(row['length']), float(row['capacity'])
        cost = float(row['transportation_cost']) if row.get('transportation_cost') else length * params['transport_cost_coefficient']
        if i == j or not math.isfinite(length) or length <= 0 or not math.isfinite(capacity) or capacity < 0 or not math.isfinite(cost) or cost < 0:
            raise ValueError(f'Invalid link {row["link_id"]}')
        capacities.append(capacity)
        for u, v in ([(i, j)] if profile == 'ras' else [(i, j), (j, i)]):
            adj[u][v] = min(adj[u].get(v, math.inf), length)
            arcs.append({'from': u, 'to': v, 'link': e, 'length': length, 'cost': cost})
    # Preserve the established GenCAT arc order (all forward, then all reverse).
    if profile == 'gencat':
        arcs = arcs[::2] + arcs[1::2]
    distance = yard_distances(adj, yards)
    def dist(i, j):
        return distance[yard_index[i]][yard_index[j]]
    penalties = []
    for k, demand in enumerate(demands):
        value = float(demand['unserved_cost']) if demand.get('unserved_cost') else params['stress_penalty_M'] * dist(origins[k], destinations[k])
        if not math.isfinite(value) or value < 0:
            raise ValueError(f'No finite unserved penalty for demand {demand["demand_id"]}')
        penalties.append(value)
    records = []
    for node in nodes:
        yard = node['node_type'] == 'yard'
        records.append({'yard': yard, 'tracks': float(node['num_tracks']) if yard else 0,
                        'handling_capacity': float(node['handling_capacity']) if yard else 0,
                        'handling_cost': float(node['handling_cost']) if yard else 0})
    partial = output.with_suffix('.json.partial')
    compatible, block_groups = [], []
    written = removed = 0
    with partial.open('x', encoding='utf-8', buffering=1024 * 1024) as f:
        def field(name, value):
            f.write(',' + json.dumps(name) + ':')
            json.dump(value, f, separators=(',', ':'), allow_nan=False)
        f.write('{"name":' + json.dumps(root.name) + ',"nodes":')
        json.dump(records, f, separators=(',', ':'), allow_nan=False)
        field('arcs', arcs)
        field('link_capacities', capacities)
        f.write(',"blocks":[')
        for group in groups:
            typ = group['key'][0]
            pairs = [group['key'][1:]] if typ in DIRECT else ((i, j) for i in yards for j in yards if i != j)
            upper = sum(quantities[k] for k in group['demands'])
            start = written
            for i, j in pairs:
                length = dist(i, j)
                lower = params.get('min_block_vol_short(<100mi)', 350) if length < 100 else params.get('min_block_vol_med(100-500mi)', 700) if length <= 500 else params.get('min_block_vol_long(>500mi)', 1050)
                if not math.isfinite(length):
                    removed += 1
                    continue
                op_i = nodes[i]['railroad_id'].upper().replace('CSXT', 'CSX')
                op_j = nodes[j]['railroad_id'].upper().replace('CSXT', 'CSX')
                block = {'from': i, 'to': j, 'slots': int(typ not in DIRECT), 'lower': lower, 'upper': upper,
                         'length_limit': params.get('max_circuitous_ratio', 1.3) * length,
                         'opening': params['block_fixed_cost'],
                         'interchange': params['interchange_cost'] if op_i != op_j and op_i in CLASS_I and op_j in CLASS_I else 0,
                         'transport_factor': 1}
                if written:
                    f.write(',')
                f.write(json.dumps(block, separators=(',', ':'), allow_nan=False))
                written += 1
                if written % 250000 == 0:
                    print(f'BLOCKS {written} {time.monotonic()-started:.1f}s', flush=True)
            compatible.append(range(start, written))
            block_groups.append({'commodity': typ, 'first_block': start, 'stop_block': written})
        f.write('],"compatibility_groups":[')
        for g, indices in enumerate(compatible):
            if g:
                f.write(',')
            f.write('[')
            for n, b in enumerate(indices):
                if n:
                    f.write(',')
                f.write(str(b))
            f.write(']')
        f.write(']')
        field('demands', [{'from': origins[k], 'to': destinations[k], 'quantity': quantities[k],
                           'penalty': penalties[k], 'block_group': group_for_demand[k]} for k in range(len(demands))])
        field('source_node_ids', [int(n['node_id']) for n in nodes])
        field('source_link_ids', [int(row['link_id']) for row in links])
        field('source_demand_ids', [int(d['demand_id']) for d in demands])
        metadata = {'block_policy': 'one_per_ordered_yard_pair_and_commodity', 'block_groups': block_groups,
                    'block_upper_bound': 'sum_compatible_quantities',
                    'profile': profile, 'input_candidates': count, 'provably_closed_blocks_removed': removed,
                    'source_hashes': hashes,
                    'link_mapping': 'one directed input row per one-way link' if profile == 'ras' else 'two opposite arcs sharing each input link capacity'}
        field('import_metadata', metadata)
        f.write('}\n')
    for name, digest in hashes.items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'Source changed during import: {name}')
    partial.rename(output)
    summary = {**preflight, 'blocks': written, 'arcs': len(arcs), 'removed': removed,
               'compatibility_entries': sum(len(compatible[g]) for g in group_for_demand),
               'json_bytes': output.stat().st_size, 'conversion_seconds': time.monotonic()-started,
               'output': str(output)}
    output.with_suffix('.conversion.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary), flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--profile', choices=['gencat', 'ras'], default='gencat')
    parser.add_argument('--max-blocks', type=int, default=100000)
    args = parser.parse_args()
    convert(args.directory, args.output, args.profile, args.max_blocks)

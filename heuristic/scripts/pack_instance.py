# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""One-time lossless JSON -> portable binary packing; original input is never modified."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zipfile


def pack(source, output):
    if output.exists() or output.with_suffix(output.suffix + '.partial').exists():
        raise FileExistsError(output)
    archive = zipfile.ZipFile(source) if source.suffix.lower() == '.zip' else None
    member = None
    if archive:
        members = [n for n in archive.namelist() if n.lower().endswith('.json')]
        if len(members) != 1:
            raise ValueError('Expected exactly one JSON instance')
        member = members[0]
    def open_source():
        return archive.open(member) if archive else source.open('rb')
    with open_source() as f:
        source_sha = hashlib.file_digest(f, 'sha256').hexdigest()
    with open_source() as f:
        p = json.load(f)
    if archive:
        archive.close()
    groups = p.get('compatibility_groups', []).copy()
    group_ids = []
    for demand in p['demands']:
        if ('blocks' in demand) == ('block_group' in demand):
            raise ValueError('Demand requires exactly one compatibility representation')
        if 'blocks' in demand:
            group_ids.append(len(groups)); groups.append(demand['blocks'])
        else:
            g = demand['block_group']
            if not 0 <= g < len(groups):
                raise ValueError('Invalid compatibility group')
            group_ids.append(g)
    counts = [len(p[key]) for key in ['nodes', 'arcs', 'link_capacities', 'blocks']]
    counts += [len(groups), len(p['demands'])]
    quantity_bounds = p.get('import_metadata', {}).get('block_upper_bound') == 'sum_compatible_quantities'
    partial = output.with_suffix(output.suffix + '.partial')
    binary = partial if output.suffix == '.irbp' else output.with_suffix('.irbp.partial')
    with binary.open('xb', buffering=1024*1024) as f:
        f.write(b'IRBPBIN1'); f.write(struct.pack('<I', int(quantity_bounds)))
        for text in [p['name'], source_sha]:
            encoded = text.encode('utf-8'); f.write(struct.pack('<I', len(encoded))); f.write(encoded)
        f.write(struct.pack('<6I', *counts))
        for n in p['nodes']:
            f.write(struct.pack('<I3d', int(n['yard']), n.get('tracks', 0),
                                n.get('handling_capacity', 0), n.get('handling_cost', 0)))
        for a in p['arcs']:
            f.write(struct.pack('<3I2d', a['from'], a['to'], a['link'], a['length'], a['cost']))
        for c in p['link_capacities']:
            f.write(struct.pack('<d', c))
        for b in p['blocks']:
            overrides = sorted((int(a), c) for a,c in b.get('arc_cost', {}).items())
            f.write(struct.pack('<3I6dI', b['from'], b['to'], b['slots'], b['lower'], b['upper'],
                                b['length_limit'], b['opening'], b['interchange'],
                                b.get('transport_factor', 1), len(overrides)))
            for a,c in overrides:
                f.write(struct.pack('<Id', a,c))
        for group in groups:
            f.write(struct.pack('<I', len(group)))
            for start in range(0, len(group), 65536):
                chunk = group[start:start+65536]
                f.write(struct.pack('<' + str(len(chunk)) + 'I', *chunk))
        for d,g in zip(p['demands'], group_ids):
            f.write(struct.pack('<2I2dI', d['from'], d['to'], d['quantity'], d['penalty'], g))
    with binary.open('rb') as f:
        binary_sha = hashlib.file_digest(f, 'sha256').hexdigest()
    binary_bytes = binary.stat().st_size
    if output.suffix == '.zip':
        with zipfile.ZipFile(partial, 'x', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
            z.write(binary, output.stem + '.irbp')
        binary.unlink()
    elif output.suffix != '.irbp':
        raise ValueError('Output must end in .irbp or .zip')
    partial.replace(output)
    with output.open('rb') as f:
        output_sha = hashlib.file_digest(f, 'sha256').hexdigest()
    manifest = dict(format='IRBPBIN1', source=str(source), source_json_sha256=source_sha,
                    binary_sha256=binary_sha, binary_bytes=binary_bytes, output_sha256=output_sha,
                    output_bytes=output.stat().st_size, counts=dict(zip(['nodes','arcs','links','blocks','groups','demands'], counts)),
                    quantity_derived_bounds=quantity_bounds)
    output.with_suffix(output.suffix + '.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.suffix not in {'.irbp', '.zip'}:
        parser.error('Output must end in .irbp or .zip')
    pack(args.source, args.output)

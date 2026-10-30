# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""Format-only bridge to the pre-cleanup engine; never split commodity blocks."""
from pathlib import Path
import json,hashlib
def expand(source:Path,target:Path):
    p=json.loads(source.read_text());assert not target.exists()
    keep=[b for b,c in enumerate(p['blocks']) if c['upper']>=c['lower']]
    remap={b:i for i,b in enumerate(keep)}
    # lower*y <= f <= upper*y forces y=f=0 when lower > upper, also in the LP.
    result={k:p[k] for k in ['name','nodes','arcs','link_capacities']}
    result['blocks']=[p['blocks'][b] for b in keep]
    result['demands']=[]
    for d in p['demands']:
        q={k:d[k] for k in ['from','to','quantity','penalty']}
        allowed=d['blocks'] if 'blocks' in d else p['compatibility_groups'][d['block_group']]
        q['blocks']=[remap[b] for b in allowed if b in remap];result['demands'].append(q)
    target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(result,separators=(',',':'),allow_nan=False))
    meta=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),legacy_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),original_blocks=len(p['blocks']),kept_block_ids=keep,provably_closed_blocks_removed=len(p['blocks'])-len(keep),policy='same coefficients and compatibility; expand shared lists, eliminate only zero-cone blocks')
    target.with_suffix('.mapping.json').write_text(json.dumps(meta,indent=2));return meta

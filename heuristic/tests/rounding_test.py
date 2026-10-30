# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
import json
from pathlib import Path
import subprocess
import sys
import tempfile

code = Path(__file__).resolve().parents[1]
original = json.loads((code.parent / 'examples/partial_service.json').read_text())
original['arcs'][0]['cost'] = 1.5
original['blocks'][0]['transport_factor'] = 1.5
original['blocks'][0]['opening'] = 2.5
original['demands'][0]['penalty'] = 9.5
with tempfile.TemporaryDirectory() as tmp:
    source, output = Path(tmp)/'source', Path(tmp)/'rounded'
    source.mkdir()
    path = source/'case.json'
    path.write_text(json.dumps(original))
    before = path.read_bytes()
    subprocess.run([sys.executable, str(code/'scripts/round_costs.py'), '--source', str(source), '--output', str(output)], check=True, capture_output=True)
    rounded = json.loads((output/'case.json').read_text())
    assert path.read_bytes() == before
    assert rounded['link_capacities'] == original['link_capacities']
    for section, costs in [('nodes',{'handling_cost'}),('arcs',{'cost'}),('demands',{'penalty'}),('blocks',{'opening','interchange','transport_factor','arc_cost'})]:
        for old, new in zip(original[section],rounded[section]):
            assert {k:v for k,v in old.items() if k not in costs} == {k:v for k,v in new.items() if k not in costs}
    assert rounded['arcs'][0]['cost'] == 2
    assert rounded['blocks'][0]['opening'] == 3
    assert rounded['demands'][0]['penalty'] == 10
    assert rounded['blocks'][0].get('arc_cost',{}).get('0',rounded['arcs'][0]['cost']) == 2
    assert 'expected' not in rounded
print('Half-up rounding preserves resources and source data')

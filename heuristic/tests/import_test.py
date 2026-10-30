# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""CSV directions, unique blocks, shared compatibility, ZIP integrity and scaling."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

code = Path(__file__).resolve().parents[1]
exe = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as folder:
    root = Path(folder)
    def table(name, header, rows):
        with (root/name).open('w', newline='') as f:
            w = csv.writer(f); w.writerow(header); w.writerows(rows)
    table('node.csv', ['node_id','node_type','num_tracks','handling_capacity','handling_cost','railroad_id'],
          [[10,'yard',3,100,0,'AA'],[20,'yard',3,100,0,'AA']])
    table('link.csv', ['link_id','from_node_id','to_node_id','length','capacity'],
          [[101,10,20,1,6],[102,20,10,2,7]])
    table('demand.csv', ['demand_id','origin_yard_id','dest_yard_id','volume','block_type'],
          [[1,10,20,4,'Merchandise'],[2,10,20,4,'Merchandise'],[3,10,20,2,'Grain'],[4,20,10,3,'Intermodal']])
    table('setting.csv', ['parameter','value'], [['block_fixed_cost',2],['transport_cost_coefficient',1],
          ['interchange_cost',0],['stress_penalty_M',5],['min_block_vol_short(<100mi)',5],
          ['max_circuitous_ratio',1.3]])
    before = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('*.csv')}
    imported = root/'ras.json'
    command = [sys.executable, str(code/'scripts/import_gencat.py'), str(root), str(imported), '--profile', 'ras']
    subprocess.run(command, check=True, capture_output=True)
    p = json.loads(imported.read_text())
    assert len(p['blocks']) == 5 and len(p['arcs']) == 2
    assert p['arcs'][0]['link'] != p['arcs'][1]['link']
    signatures = []
    for group in p['import_metadata']['block_groups']:
        for b in p['blocks'][group['first_block']:group['stop_block']]:
            signatures.append((b['from'],b['to'],group['commodity']))
    assert len(signatures) == len(set(signatures))
    assert p['demands'][0]['block_group'] == p['demands'][1]['block_group']
    for b in p['blocks']:
        assert abs(b['length_limit'] - (1.3 if b['from']==0 else 2.6)) < 1e-10
    p['reader_test_padding'] = 'x' * (3 * 65536)
    imported.write_text(json.dumps(p))
    explicit = json.loads(imported.read_text())
    for d in explicit['demands']:
        d['blocks'] = explicit['compatibility_groups'][d.pop('block_group')]
    explicit.pop('compatibility_groups')
    flat = root/'flat.json'; flat.write_text(json.dumps(explicit))
    zipped = root/'ras.zip'
    with zipfile.ZipFile(zipped,'w',zipfile.ZIP_DEFLATED) as z:
        z.write(imported, 'case/ras.json')
        z.writestr('README.txt', 'one JSON member')
    packed = root/'ras.irbp'
    subprocess.run([sys.executable,str(code/'scripts/pack_instance.py'),str(imported),str(packed)],
                   check=True,capture_output=True)
    packed_zip = root/'binary.zip'
    with zipfile.ZipFile(packed_zip,'w',zipfile.ZIP_DEFLATED) as z:
        z.write(packed,'case/ras.irbp')
    objectives = []
    for i, source in enumerate([imported, flat, zipped, packed, packed_zip]):
        out = root/f'out{i}.json'
        subprocess.run([exe,str(source),'arc','5',str(out)], check=True, capture_output=True)
        result = json.loads(out.read_text()); assert result['status']=='optimal'
        objectives.append(result['objective'])
    assert max(objectives)-min(objectives)<1e-7
    for factor in [.125,.5,2.]:
        scaled = json.loads(json.dumps(explicit))
        for d in scaled['demands']: d['quantity'] *= factor
        for b in scaled['blocks']: b['upper'] *= factor
        manually = root/f'scaled{factor}.json'; manually.write_text(json.dumps(scaled))
        costs=[]
        for i, (source,args) in enumerate([(zipped,['--demand-multiplier',str(factor)]),(packed_zip,['--demand-multiplier',str(factor)]),(manually,[])]):
            out = root/f'scale{factor}-{i}.json'
            subprocess.run([exe,str(source),'arc','5',str(out)]+args,check=True,capture_output=True)
            result=json.loads(out.read_text()); assert result['status']=='optimal'
            costs.append(result['objective'])
        assert abs(costs[0]-costs[1])<1e-7
    # Corrupt the central CRC while retaining syntactically valid compressed JSON.
    corrupted=bytearray(zipped.read_bytes());central=corrupted.index(b'PK\x01\x02')
    corrupted[central+16] ^= 1
    bad=root/'bad.zip';bad.write_bytes(corrupted)
    out=root/'bad-result.json'
    assert subprocess.run([exe,str(bad),'heuristic','1',str(out)],capture_output=True).returncode != 0
    assert not out.exists()
    multi=root/'multi.zip'
    with zipfile.ZipFile(multi,'w',zipfile.ZIP_DEFLATED) as z:
        z.write(imported,'one.json');z.write(flat,'two.json')
    assert subprocess.run([exe,str(multi),'heuristic','1',str(out)],capture_output=True).returncode != 0
    truncated = root/'truncated.irbp'; truncated.write_bytes(packed.read_bytes()[:-1])
    assert subprocess.run([exe,str(truncated),'heuristic','1',str(out)],capture_output=True).returncode != 0
    future = root/'future.irbp'; future.write_bytes(b'IRBPBIN9'+packed.read_bytes()[8:])
    assert subprocess.run([exe,str(future),'heuristic','1',str(out)],capture_output=True).returncode != 0
    ambiguous = root/'mixed.zip'
    with zipfile.ZipFile(ambiguous,'w',zipfile.ZIP_DEFLATED) as z:
        z.write(imported,'one.json');z.write(packed,'two.irbp')
    assert subprocess.run([exe,str(ambiguous),'heuristic','1',str(out)],capture_output=True).returncode != 0
    # Old unshared JSON remains valid; malformed group references fail.
    broken=json.loads(imported.read_text());broken['demands'][0]['block_group']=999
    bad_json=root/'invalid.json';bad_json.write_text(json.dumps(broken))
    assert subprocess.run([exe,str(bad_json),'heuristic','1',str(out)],capture_output=True).returncode != 0
    assert subprocess.run(command,capture_output=True).returncode != 0
    assert before == {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in root.glob('*.csv')}
print('CSV/ZIP/explicit JSON parity, binary parity, scaling, integrity, unique blocks and source preservation passed')

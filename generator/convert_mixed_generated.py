# Copyright (c) 2026 Zhengzhong (Ricky) You. All rights reserved.
# Software: RouteOpt. Application: IRBP.
"""New mixed-commodity RAS input; never modify archived per-commodity data."""
from pathlib import Path
from collections import defaultdict
import csv,hashlib,importlib.util,json,math,time

def convert(raw,output,importer,expected_hashes):
    assert not output.exists() and not output.with_suffix('.json.partial').exists()
    spec=importlib.util.spec_from_file_location('native_import',importer);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    hashes={n:hashlib.sha256((raw/n).read_bytes()).hexdigest() for n in ['node.csv','link.csv','demand.csv','setting.csv']};assert hashes==expected_hashes
    read=lambda n:list(csv.DictReader((raw/n).open(encoding='utf-8-sig',newline='')))
    nodes,links,demands=read('node.csv'),read('link.csv'),read('demand.csv');params={s['parameter']:float(s['value']) for s in read('setting.csv')}
    ids={int(n['node_id']):i for i,n in enumerate(nodes)};yards=[i for i,n in enumerate(nodes) if n['node_type']=='yard'];yi={i:j for j,i in enumerate(yards)}
    typ=[d['block_type'].strip().lower() for d in demands];orig=[ids[int(d['origin_yard_id'])] for d in demands];dest=[ids[int(d['dest_yard_id'])] for d in demands];q=[float(d['volume'])*params.get('demand_multiplier',1) for d in demands]
    direct={};group=[];regular=0.;special=defaultdict(float)
    for k,t in enumerate(typ):
        assert orig[k] in yi and dest[k] in yi and orig[k]!=dest[k] and q[k]>0
        if t in mod.DIRECT:
            od=(orig[k],dest[k]);direct.setdefault(od,len(direct)+1);group.append(direct[od]);special[od]+=q[k]
        else:group.append(0);regular+=q[k]
    adj=[{} for _ in nodes];arcs=[];caps=[]
    for e,row in enumerate(links):
        i,j=ids[int(row['from_node_id'])],ids[int(row['to_node_id'])];length=float(row['length']);cap=float(row['capacity']);cost=float(row['transportation_cost']) if row.get('transportation_cost') else length*params['transport_cost_coefficient']
        assert i!=j and length>0 and cap>=0 and cost>=0
        caps.append(cap)
        for u,v in [(i,j),(j,i)]:
            arcs.append({'from':u,'to':v,'link':e,'length':length,'cost':cost});adj[u][v]=min(adj[u].get(v,math.inf),length)
    arcs=arcs[::2]+arcs[1::2]
    dist=mod.yard_distances(adj,yards);penalty=[]
    for k,d in enumerate(demands):
        value=float(d['unserved_cost']) if d.get('unserved_cost') else params['stress_penalty_M']*dist[yi[orig[k]]][yi[dest[k]]]
        assert math.isfinite(value) and value>=0;penalty.append(value)
    records=[dict(yard=n['node_type']=='yard',tracks=float(n['num_tracks']) if n['node_type']=='yard' else 0,handling_capacity=float(n['handling_capacity']) if n['node_type']=='yard' else 0,handling_cost=float(n['handling_cost']) if n['node_type']=='yard' else 0) for n in nodes]
    partial=output.with_suffix('.json.partial');blocks=0;direct_ids={};opened=time.perf_counter()
    with partial.open('x',encoding='utf8',buffering=1024*1024) as f:
        def field(name,value):f.write(','+json.dumps(name)+':');json.dump(value,f,separators=(',',':'),allow_nan=False)
        f.write('{"name":'+json.dumps(raw.name)+',"nodes":');json.dump(records,f,separators=(',',':'),allow_nan=False);field('arcs',arcs);field('link_capacities',caps);f.write(',"blocks":[')
        for a,i in enumerate(yards):
            op_i=nodes[i]['railroad_id'].upper().replace('CSXT','CSX')
            for b,j in enumerate(yards):
                if i==j:continue
                length=dist[a][b]
                if not math.isfinite(length):continue
                lower=params.get('min_block_vol_short(<100mi)',350) if length<100 else params.get('min_block_vol_med(100-500mi)',700) if length<=500 else params.get('min_block_vol_long(>500mi)',1050)
                op_j=nodes[j]['railroad_id'].upper().replace('CSXT','CSX');upper=regular+special.get((i,j),0.)
                block={'from':i,'to':j,'slots':1,'lower':lower,'upper':upper,'length_limit':params.get('max_circuitous_ratio',1.3)*length,'opening':params['block_fixed_cost'],'interchange':params['interchange_cost'] if op_i!=op_j and op_i in mod.CLASS_I and op_j in mod.CLASS_I else 0,'transport_factor':1}
                if blocks:f.write(',')
                json.dump(block,f,separators=(',',':'),allow_nan=False)
                if (i,j) in direct:direct_ids[direct[i,j]]=blocks
                blocks+=1
            if a%128==0:print('MIXED_BLOCKS',a,blocks,round(time.perf_counter()-opened,1),flush=True)
        assert len(direct_ids)==len(direct)
        f.write('],"compatibility_groups":[[')
        for start in range(0,blocks,65536):
            if start:f.write(',')
            f.write(','.join(map(str,range(start,min(start+65536,blocks)))))
        f.write(']')
        for g in range(1,len(direct)+1):f.write(',['+str(direct_ids[g])+']')
        f.write(']')
        field('demands',[{'from':orig[k],'to':dest[k],'quantity':q[k],'penalty':penalty[k],'block_group':group[k]} for k in range(len(demands))]);field('source_node_ids',[int(n['node_id']) for n in nodes]);field('source_link_ids',[int(a['link_id']) for a in links]);field('source_demand_ids',[int(d['demand_id']) for d in demands]);field('source_demand_types',typ)
        field('import_metadata',dict(block_policy='one_shared_block_per_ordered_yard_pair',direct_only_types=sorted(mod.DIRECT),track_policy='each_shared_block_consumes_one_departure_track',block_upper_bound='sum_compatible_quantities',profile='gencat',source_hashes=hashes,direct_group_blocks=direct_ids));f.write('}\n')
    assert hashes=={n:hashlib.sha256((raw/n).read_bytes()).hexdigest() for n in hashes};partial.rename(output)
    summary=dict(nodes=len(nodes),arcs=len(arcs),demands=len(demands),blocks=blocks,groups=len(direct)+1,ordinary_quantity=regular,base_rejection=math.fsum(v*c for v,c in zip(q,penalty)),source_hashes=hashes,policy='mixed cargo, direct-only retained, one track per shared block')
    output.with_suffix('.conversion.json').write_text(json.dumps(summary,indent=2));return summary

#!/usr/bin/env python3
# Copyright (c) 2026 Zhengzhong (Ricky) You.
# All rights reserved.
# Software: RouteOpt
# Application: IRBP

"""Round effective cost coefficients half up; preserve all physical/resource data."""
import argparse,csv,decimal,hashlib,heapq,json,pathlib
D=decimal.Decimal
def integer(x):return int(D(str(x)).quantize(D('1'),rounding=decimal.ROUND_HALF_UP))
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):
 with p.open(newline='') as f:r=csv.DictReader(f);return r.fieldnames,list(r)
def write(p,fields,rows):
 with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def shortest(adj,source):
 d={source:0.};q=[(0.,source)]
 while q:
  c,i=heapq.heappop(q)
  if c!=d[i]:continue
  for j,v in adj.get(i,[]):
   if c+v<d.get(j,float('inf')):d[j]=c+v;heapq.heappush(q,(c+v,j))
 return d
ap=argparse.ArgumentParser();ap.add_argument('--source',type=pathlib.Path,default=pathlib.Path('instance'));ap.add_argument('--output',type=pathlib.Path,default=pathlib.Path('instance/integer_costs'));args=ap.parse_args()
if args.source.resolve()==args.output.resolve():raise ValueError('Use a distinct output directory')
args.output.mkdir(parents=True,exist_ok=True);records=[]
for folder in sorted(args.source.glob('v2_*')):
 if not folder.is_dir():continue
 target=args.output/folder.name;target.mkdir(exist_ok=True)
 nf,nodes=read(folder/'node.csv');lf,links=read(folder/'link.csv');df,demands=read(folder/'demand.csv');sf,settings=read(folder/'setting.csv');params={r['parameter']:float(r['value']) for r in settings}
 for n in nodes:
  if n.get('handling_cost','').strip():n['handling_cost']=str(integer(n['handling_cost']))
 adj={}
 for l in links:
  l['transportation_cost']=str(integer(D(l['length'])*D(str(params['transport_cost_coefficient']))))
  i,j=int(l['from_node_id']),int(l['to_node_id']);length=float(l['length']);adj.setdefault(i,[]).append((j,length));adj.setdefault(j,[]).append((i,length))
 distances={}
 for d in demands:
  o,t=int(d['origin_yard_id']),int(d['dest_yard_id'])
  if o not in distances:distances[o]=shortest(adj,o)
  d['unserved_cost']=str(integer(D(str(distances[o][t]))*D(str(params['stress_penalty_M']))))
 # Multipliers remain unchanged: effective per-link and per-demand costs above are authoritative.
 for s in settings:
  if s['parameter'] in ('block_fixed_cost','interchange_cost'):s['value']=str(integer(s['value']))
 write(target/'node.csv',nf,nodes);write(target/'link.csv',lf+['transportation_cost'],links);write(target/'demand.csv',df+['unserved_cost'],demands);write(target/'setting.csv',sf,settings)
 records.append({'instance':folder.name,'format':'csv','source_sha256':{f:digest(folder/f) for f in ['node.csv','link.csv','demand.csv','setting.csv']},'nodes':len(nodes),'links':len(links),'demands':len(demands)})
for source in sorted(args.source.glob('*.json')):
 p=json.loads(source.read_text());original_cost=[a['cost'] for a in p['arcs']]
 for a in p['arcs']:a['cost']=integer(a['cost'])
 for n in p['nodes']:n['handling_cost']=integer(n.get('handling_cost',0))
 for d in p['demands']:d['penalty']=integer(d['penalty'])
 for b in p['blocks']:
  b['opening']=integer(b['opening']);b['interchange']=integer(b['interchange']);factor=b.get('transport_factor',1);old=b.get('arc_cost',{});overrides={}
  for a,c in enumerate(original_cost):
   effective=integer(old.get(str(a),D(str(factor))*D(str(c))))
   if effective!=p['arcs'][a]['cost']:overrides[str(a)]=effective
  b['transport_factor']=1
  if overrides:b['arc_cost']=overrides
  else:b.pop('arc_cost',None)
 p.pop('expected',None) # An optimum for the unrounded objective is not an oracle for this data.
 p['cost_rounding']={'rule':'nearest integer, half up','source_sha256':digest(source)}
 (args.output/source.name).write_text(json.dumps(p,indent=2)+'\n');records.append({'instance':source.stem,'format':'json','source_sha256':digest(source)})
(args.output/'manifest.json').write_text(json.dumps({'rule':'Round effective opening, transportation, handling, interchange and unserved-demand cost coefficients to nearest integer, half up.','unchanged':'Physical lengths, node/arc indices, demand volumes, capacities, minimum block loads and path-length budgets.','csv_authority':'link.transportation_cost and demand.unserved_cost override length-derived objective coefficients; original lengths still determine physical constraints.','instances':records},indent=2)+'\n')
print('Generated',len(records),'integer-cost instances in',args.output)

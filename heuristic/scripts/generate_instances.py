#!/usr/bin/env python3
# Copyright (c) 2026 Zhengzhong (Ricky) You.
# All rights reserved.
# Software: RouteOpt
# Application: IRBP

"""Deterministic, shareable instances; no competition records are used."""
import json, random, pathlib, sys
out=pathlib.Path(sys.argv[1] if len(sys.argv)>1 else 'instance');out.mkdir(parents=True,exist_ok=True)
def save(data):
    (out/(data['name']+'.json')).write_text(json.dumps(data,indent=2)+'\n')
def node(yard=True,tracks=2,handling=20):
    return dict(yard=yard,tracks=tracks if yard else 0,handling_capacity=handling if yard else 0,handling_cost=1 if yard else 0)
def block(i,j,lower,upper,limit,opening):
    return dict(from_=i,to=j,slots=1,lower=lower,upper=upper,length_limit=limit,opening=opening,interchange=0,transport_factor=1)
def normalize(d):
    for b in d['blocks']:b['from']=b.pop('from_')
    return d
save(normalize(dict(name='partial_service',nodes=[node(),node()],link_capacities=[6],arcs=[dict(**{'from':0,'to':1},link=0,length=1,cost=1)],blocks=[block(0,1,3,10,1,2)],demands=[dict(**{'from':0,'to':1},quantity=10,penalty=5,blocks=[0])],expected=28)))
save(normalize(dict(name='shared_link_cover',nodes=[node(),node()],link_capacities=[10],arcs=[dict(**{'from':0,'to':1},link=0,length=1,cost=1),dict(**{'from':1,'to':0},link=0,length=1,cost=1)],blocks=[block(0,1,6,8,1,2),block(1,0,6,8,1,2)],demands=[dict(**{'from':0,'to':1},quantity=8,penalty=10,blocks=[0]),dict(**{'from':1,'to':0},quantity=8,penalty=9,blocks=[1])],expected=82)))
for seed,Y,N,K in [(41,4,6,6),(42,4,6,8),(43,5,8,10),(44,5,8,12),(45,6,10,16),(46,6,10,20),(47,8,12,30),(48,10,16,40)]:
    r=random.Random(seed);nodes=[node(i<Y,2,18+r.randrange(8)) for i in range(N)]
    edges={(i,(i+1)%N) if i<(i+1)%N else ((i+1)%N,i) for i in range(N)}
    while len(edges)<N+N//2:
        i,j=sorted(r.sample(range(N),2));edges.add((i,j))
    arcs=[];caps=[];dist=[[float('inf')]*N for _ in range(N)]
    for i in range(N):dist[i][i]=0
    for e,(i,j) in enumerate(sorted(edges)):
        length=r.randint(1,5);caps.append(r.randint(10,22));dist[i][j]=dist[j][i]=length
        for u,v in [(i,j),(j,i)]:arcs.append(dict(**{'from':u,'to':v},link=e,length=length,cost=round(length*.8+r.random(),3)))
    for k in range(N):
        for i in range(N):
            for j in range(N):dist[i][j]=min(dist[i][j],dist[i][k]+dist[k][j])
    demands=[]
    for k in range(K):
        i,j=r.sample(range(Y),2);demands.append(dict(**{'from':i,'to':j},quantity=r.randint(5,10),penalty=r.randint(25,38),blocks=[]))
    total=sum(d['quantity'] for d in demands)
    blocks=[block(i,j,r.randint(3,7),total,dist[i][j]*1.7+1,r.randint(8,22)) for i in range(Y) for j in range(Y) if i!=j]
    for d in demands:d['blocks']=list(range(len(blocks)))
    save(normalize(dict(name=f'syn_y{Y}_n{N}_k{K}_s{seed}',nodes=nodes,link_capacities=caps,arcs=arcs,blocks=blocks,demands=demands,seed=seed)))
# The sufficient-multiplicity fixture has one parallel candidate per compatible demand.
import copy
base=json.loads((out/'syn_y4_n6_k6_s41.json').read_text())
base['name']='syn_y4_n6_k6_s41_full_candidates'
base['blocks']=[copy.deepcopy(b) for b in base['blocks'] for _ in base['demands']]
for demand in base['demands']:demand['blocks']=list(range(len(base['blocks'])))
save(base)
save(normalize(dict(name='parallel_blocks',nodes=[node(tracks=2),node(),node(False),node(False)],link_capacities=[6,6,6,6],arcs=[dict(**{'from':u,'to':v},link=e,length=1,cost=.5) for e,(u,v) in enumerate([(0,2),(2,1),(0,3),(3,1)])],blocks=[block(0,1,4,12,2,2),block(0,1,4,12,2,2)],demands=[dict(**{'from':0,'to':1},quantity=6,penalty=10,blocks=[0,1]) for _ in range(2)],expected=16)))

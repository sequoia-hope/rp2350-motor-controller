import sys, os, json, math, collections
sys.path.insert(0,'/home/sequoia/pcb/rp2350-motor-controller/hardware/tools/jiggle2')
from common import load_board, NM, quiet_stderr
import pcbnew
HW='/home/sequoia/pcb/rp2350-motor-controller/hardware'
S=os.path.dirname(os.path.abspath(__file__))
bd=load_board(HW+'/rp2350_driver.kicad_pcb')
pre=load_board(HW+'/tools/jiggle2/board_prerip_2238aca.kicad_pcb')
pre_refs={f.GetReference():(f.GetPosition().x*NM,f.GetPosition().y*NM) for f in pre.GetFootprints()}
bb=bd.GetBoardEdgesBoundingBox()
out=dict(edge=[bb.GetLeft()*NM,bb.GetTop()*NM,bb.GetRight()*NM,bb.GetBottom()*NM],fps=[])
for f in bd.GetFootprints():
    try: f.BuildCourtyardCaches()
    except AttributeError: pass
    cy={}
    for lay,name in ((pcbnew.F_CrtYd,'F'),(pcbnew.B_CrtYd,'B')):
        ps=f.GetCourtyard(lay)
        polys=[]
        for i in range(ps.OutlineCount()):
            o=ps.Outline(i); pts=[[round(o.CPoint(j).x*NM,4),round(o.CPoint(j).y*NM,4)] for j in range(o.PointCount())]
            if len(pts)>=3: polys.append(pts)
        if polys: cy[name]=polys
    pads=[]
    for p in f.Pads():
        pb=p.GetBoundingBox()
        try: lc=p.GetLocalClearance()
        except TypeError: lc=p.GetLocalClearance(None)
        try: flc=f.GetLocalClearance()
        except TypeError: flc=0
        lc=max(lc or 0, flc or 0)*NM
        pads.append(dict(num=p.GetNumber(),lc=lc if lc>0.001 else 0.0,net=p.GetNetname(),x=round(p.GetPosition().x*NM,4),y=round(p.GetPosition().y*NM,4),
            bbox=[round(pb.GetLeft()*NM,4),round(pb.GetTop()*NM,4),round(pb.GetRight()*NM,4),round(pb.GetBottom()*NM,4)],
            F=p.IsOnLayer(pcbnew.F_Cu),B=p.IsOnLayer(pcbnew.B_Cu),drill=p.GetDrillSize().x*NM if p.GetDrillSize().x else 0,
            npth=p.GetAttribute()==pcbnew.PAD_ATTRIB_NPTH))
    fb=f.GetBoundingBox(False,False)
    r=f.GetReference(); px,py=f.GetPosition().x*NM,f.GetPosition().y*NM
    status='new' if r not in pre_refs else ('moved' if math.hypot(px-pre_refs[r][0],py-pre_refs[r][1])>0.005 else 'pinned')
    out['fps'].append(dict(ref=r,value=f.GetValue(),fpid=f.GetFPIDAsString(),x=round(px,4),y=round(py,4),rot=f.GetOrientationDegrees(),
        layer='B' if f.GetLayer()==pcbnew.B_Cu else 'F',courtyard=cy,pads=pads,status=status,
        bbox=[round(fb.GetLeft()*NM,4),round(fb.GetTop()*NM,4),round(fb.GetRight()*NM,4),round(fb.GetBottom()*NM,4)],
        pre=pre_refs.get(r)))
tr=collections.Counter(); vi=0
for t in bd.GetTracks():
    if t.GetClass()=='PCB_VIA': vi+=1
    else: tr[bd.GetLayerName(t.GetLayer())]+=1
out['tracks']=dict(tr); out['vias']=vi
out['zones']=len(list(bd.Zones()))
json.dump(out,open(S+'/board.json','w'))
print('edge',out['edge'],'fps',len(out['fps']),'tracks',sum(tr.values()),tr,'vias',vi,'zones',out['zones'])
c=collections.Counter(f['status'] for f in out['fps']); print(c)
print('NEW:',sorted(f['ref'] for f in out['fps'] if f['status']=='new'))
print('MOVED:',sorted(f['ref'] for f in out['fps'] if f['status']=='moved'))
print('F side:',sum(1 for f in out['fps'] if f['layer']=='F'),'B side:',sum(1 for f in out['fps'] if f['layer']=='B'))

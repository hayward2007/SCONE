exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/geometry.py').read())
def run(_context):
 a,src,d=source();orig,parts,rois=generate(d)
 results={'source':src.name,'source_modified':src.isModified,'dimensions':{k:bounds(v) for k,v in parts.items()}}
 removed=T.copy(orig['L1 frame']);cut(removed,parts['L1 frame'])
 for r in rois:cut(removed,r)
 added=T.copy(parts['L1 frame']);cut(added,orig['L1 frame'])
 results['L1_outside_four_regions_removed_mm3']=removed.volume*1000 if removed.isSolid else 0
 results['L1_added_mm3']=added.volume*1000 if added.isSolid else 0
 assert results['L1_outside_four_regions_removed_mm3']<1e-5 and results['L1_added_mm3']<1e-5
 pd=a.documents.add(c.DocumentTypes.FusionDesignDocumentType);pd.name='C1 L1 revision preview';dd=f.Design.cast(a.activeProduct);dd.designType=f.DesignTypes.DirectDesignType
 new={}
 for name,q in parts.items():
  b=dd.rootComponent.bRepBodies.add(q);b.name=name;b.appearance=orig[name].appearance;new[name]=b
 # Round the lengthwise cable-mouth edges and all four roof transitions.
 b=new['C1 USB cable guide'];edges=c.ObjectCollection.create()
 for e in b.edges:
  if not isinstance(e.geometry,c.Line3D):continue
  p0=e.startVertex.geometry;p1=e.endVertex.geometry
  x0,y0,z0=[v*10 for v in p0.asArray()];x1,y1,z1=[v*10 for v in p1.asArray()]
  # Y-directed roof fold edges, including inside cable-contact bends.
  if abs(x0-x1)<1e-5 and abs(z0-z1)<1e-5 and abs(y1-y0)>20 and any(abs(x0-x)<1e-4 for x in(-3,9,62.5,74.5)):
   edges.add(e)
 if edges.count:
  fi=dd.rootComponent.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(edges,c.ValueInput.createByReal(.12),False);fil=dd.rootComponent.features.filletFeatures.add(fi);fil.name='C1 smooth height transitions R1.2'
  results['C1_transition_fillet_edges']=edges.count
 # Add context only to a disposable preview, keeping source visibility untouched.
 for o in d.rootComponent.allOccurrences:
  for ob in o.bRepBodies:
   if ob.name in('L2 lidar plate','H2 handle + arm rest') or (ob.isVisible and any(k in ob.name for k in('LID','lid'))):
    cb=dd.rootComponent.bRepBodies.add(T.copy(ob));cb.name='CONTEXT '+ob.name;cb.appearance=ob.appearance
 results['preview_bodies']=[b.name for b in dd.rootComponent.bRepBodies]
 cam=a.activeViewport.camera;cam.isSmoothTransition=False;cam.eye=P(165,215,290);cam.target=P(-8,0,174);cam.upVector=V(0,0,1);cam.viewExtents=21;a.activeViewport.camera=cam;a.activeViewport.refresh()
 (OUT/'candidate.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(results,ensure_ascii=False))

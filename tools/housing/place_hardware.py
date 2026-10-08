exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py').read())
app,doc,d=guard();root=d.rootComponent
records=[]
for name,arr in [('V01 NVIDIA official mechanical model:1',[0,1,0,5.4876875,-1,0,0,4.6,0,0,1,4.7900021523,0,0,0,1]),('V02 Waveshare official camera:1',[0,0,-1,16.705,-1,0,0,0,0,1,0,8.965,0,0,0,1])]:
 o=root.occurrences.itemByName(name);m=c.Matrix3D.create();m.setWithArray(arr);o.transform2=m
 for b in list(o.bRepBodies)+[b for q in root.allOccurrences if q.fullPathName.startswith(name+'+') for b in q.bRepBodies]:
  if not b.isSolid:continue
  rec={'name':b.name,'bbox':bb(b),'cylinders':[]}
  for face in b.faces:
   g=c.Cylinder.cast(face.geometry)
   if g and 1.0<g.radius*10<2.1 and abs(g.axis.z)>.99:rec['cylinders'].append({'r':g.radius*10,'origin':xyz(g.origin),'bbox':bb(face)})
  if rec['cylinders']:records.append(rec)
if d.snapshots.hasPendingSnapshot:d.snapshots.add()
(OUT/'hardware_placed_holes.json').write_text(json.dumps(records,indent=2,ensure_ascii=True))
print('HARDWARE POSITIONED',len(records))

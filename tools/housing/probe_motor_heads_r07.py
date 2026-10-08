exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
app,doc,d=guard();r=d.rootComponent;b=r.occurrences.itemByName('LEG 1:1').bRepBodies.item(0);res=[]
for face in b.faces:
 cy=c.Cylinder.cast(face.geometry)
 if cy and abs(cy.axis.z)>.9 and face.boundingBox.minPoint.z<.35:
  res.append(dict(origin=xyz(cy.origin),radius=cy.radius*10,bounds=bb(face)))
(OUT/'R07_DELIVERY/motor_head_surfaces.json').write_text(json.dumps(res,indent=2));print(res)

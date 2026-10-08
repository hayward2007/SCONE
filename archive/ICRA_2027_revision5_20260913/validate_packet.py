"""Check frozen measurements, PDF structure, and companion-media constraints."""
from pathlib import Path
import json,re,subprocess,hashlib,xml.etree.ElementTree as ET
from pypdf import PdfReader
P=Path(__file__).resolve().parent
raw=P/'evidence/stairs/trials.jsonl';rows=list(map(json.loads,raw.read_text().splitlines()))
protocol=json.loads((P/'evidence/stairs/protocol.json').read_text())
assert len(rows)==102 and sorted(r['id'] for r in rows)==list(range(102))
assert all(not r['git_dirty'] and r['git_revision']==protocol['git_revision'] for r in rows)
assert sum(r['halt_success'] for r in rows)==53
assert sum(r['failure']=='ascent-timeout' for r in rows)==49
for r in rows:
 j=next(j for j in protocol['jobs'] if j['id']==r['id'])
 assert all(r[k]==v for k,v in j.items())
for c in ['fixed','neutral','lookup']:
 for h,want in [(.1,6),(.15,6),(.2,0)]:
  a=[r for r in rows if r['suite']!='timestep' and r['geometry']=='decomposed-arc' and r['control']==c and r['h']==h]
  assert len(a)==6 and sum(r['halt_success'] for r in a)==want
w=[r for r in rows if r['geometry']=='closed-wheel' and r['control']=='fixed' and r['h']==.15 and r['d']==.35 and r['pose']==0]
assert {r['dt']:r['halt_success'] for r in w}=={.001:True,.002:False,.004:False}
flat=list(map(json.loads,(P/'evidence/joint_limit/raw.jsonl').read_text().splitlines()));assert len(flat)==176
pdfs=[]
for f,n in [('SCONE_ICRA2027_Manuscript_EN.pdf',6),('SCONE_ICRA2027_Review_KO.pdf',4)]:
 path=P/'output/pdf'/f;r=PdfReader(path);assert len(r.pages)==n
 fonts=subprocess.check_output(['pdffonts',str(path)],text=True)
 lines=fonts.strip().splitlines()[2:];assert lines and all(re.search(r'\s+yes\s+(?:yes|no)\s+(?:yes|no)\s+\d+\s+\d+\s*$',line) for line in lines),fonts
 assert all('Type 3' not in line for line in lines)
 text='\n'.join(p.extract_text() for p in r.pages);assert '??' not in text and '\ufffd' not in text
 bbox=P/'build'/f'{path.stem}.html';subprocess.run(['pdftotext','-bbox',str(path),str(bbox)],check=True)
 tree=ET.parse(bbox)
 for page in tree.iter('{http://www.w3.org/1999/xhtml}page'):
  pw,ph=float(page.attrib['width']),float(page.attrib['height'])
  for word in page.iter('{http://www.w3.org/1999/xhtml}word'):
   a=word.attrib;assert 0<=float(a['xMin'])<=float(a['xMax'])<=pw and 0<=float(a['yMin'])<=float(a['yMax'])<=ph
 pdfs.append({'name':f,'pages':n,'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'fonts_embedded':True,'page_bounds_ok':True})
assert 'Overfull' not in (P/'build/paper.log').read_text()
assert (P/'ieeeconf.cls').read_bytes()==(P.parent/'ICRA_2027_revision4_20260913/ieeeconf.cls').read_bytes()
video=json.loads((P/'evidence/video_validation.json').read_text());assert all(c['max_error']==0 and not c['git_dirty'] for c in video['replay_checks'])
assert video['size_bytes']<20000000 and video['duration_s']<180 and video['height']>=480
assert 'Ran 287 tests' in (P/'evidence/tests.log').read_text() and (P/'evidence/tests.log').read_text().rstrip().endswith('OK')
result={'pdfs':pdfs,'stairs_planned_and_recorded':102,'flat_retained_trials':176,'stairs_raw_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'tests':287,'figures':6,'tables':3,'cited_references':len(re.findall(r'\\bibitem', (P/'build/paper.bbl').read_text())),'video':{'bytes':video['size_bytes'],'duration_s':video['duration_s']},'template_unchanged':True,'source_revision':protocol['git_revision']}
(P/'evidence/final_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

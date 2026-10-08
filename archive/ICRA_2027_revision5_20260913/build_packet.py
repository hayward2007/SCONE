"""Rebuild the revised manuscript and internal review from preserved evidence."""
from pathlib import Path
import subprocess,sys
from pypdf import PdfReader,PdfWriter
P=Path(__file__).resolve().parent
subprocess.run([sys.executable,str(P/'analyze_stairs.py')],check=True)
subprocess.run(['tectonic','--keep-logs','--keep-intermediates','--outdir','build','paper.tex'],cwd=P,check=True)
r=PdfReader(P/'build/paper.pdf');assert len(r.pages)<=8,len(r.pages)
w=PdfWriter()
for page in r.pages:w.add_page(page)
w.add_metadata({'/Title':'Posture and Contact Geometry in Simple Hybrid Locomotion of an Articulated Arc-Leg Hexapod','/Author':'','/Creator':'LaTeX','/Producer':'Tectonic'})
with (P/'output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf').open('wb') as f:w.write(f)
if (P/'REVIEW_KO.md').exists():
 python=Path('/Users/hayward_kim/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3')
 subprocess.run([str(python) if python.exists() else sys.executable,str(P/'render_review.py')],check=True)
print('English pages:',len(r.pages))

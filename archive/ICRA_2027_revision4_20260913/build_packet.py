"""Rebuild PDFs from preserved measurements; does not rerun simulations."""
from pathlib import Path
import subprocess,sys
from pypdf import PdfReader,PdfWriter
P=Path(__file__).resolve().parent
subprocess.run([sys.executable,str(P/'analyze_joint_study.py')],check=True)
subprocess.run(['tectonic','--keep-logs','--keep-intermediates','--outdir','build','paper.tex'],cwd=P,check=True)
python=Path('/Users/hayward_kim/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3')
subprocess.run([str(python) if python.exists() else sys.executable,str(P/'render_review.py')],check=True)
r=PdfReader(P/'build/paper.pdf');assert len(r.pages)<=8
w=PdfWriter()
for page in r.pages:w.add_page(page)
w.add_metadata({'/Title':'Rewind-Aware Rolling Allocation for an Articulated Arc-Leg Hexapod','/Author':'','/Subject':'Anonymous ICRA 2027 manuscript','/Creator':'LaTeX','/Producer':'Tectonic'})
with (P/'output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf').open('wb') as f:w.write(f)
print('English pages:',len(r.pages))

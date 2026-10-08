"""Build the revised manuscript and internal review, preserving original results."""
from pathlib import Path
import json
import runpy
import shutil
import subprocess
import sys
from pypdf import PdfReader, PdfWriter

P = Path(__file__).resolve().parent
runpy.run_path(str(P/'reviewer_analysis.py'),run_name='__main__')
subprocess.run(['tectonic','--keep-logs','--keep-intermediates','--outdir','build','paper.tex'],cwd=P,check=True)
bundled_python=Path('/Users/hayward_kim/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3')
subprocess.run([str(bundled_python) if bundled_python.exists() else sys.executable,'render_review.py'],cwd=P,check=True)
reader=PdfReader(P/'build/paper.pdf')
assert len(reader.pages)<=8
writer=PdfWriter()
for page in reader.pages:writer.add_page(page)
writer.add_metadata({'/Title':'Rewind-Aware Rolling Allocation for an Articulated Arc-Leg Hexapod',
                     '/Author':'','/Subject':'Anonymous ICRA 2027 manuscript','/Creator':'LaTeX','/Producer':'Tectonic'})
with (P/'output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf').open('wb') as stream:writer.write(stream)
print(json.dumps({'english_pages':len(reader.pages),
                  'korean_pages':len(PdfReader(P/'output/pdf/SCONE_ICRA2027_Review_KO.pdf').pages)}))

"""Render the Korean internal review from its editable Markdown source."""
from pathlib import Path
import re,html
from functools import partial
from reportlab.pdfgen.canvas import Canvas
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,KeepTogether,Image,PageBreak
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase import pdfmetrics
from reportlab.lib.pagesizes import A4
P=Path(__file__).resolve().parent
pdfmetrics.registerFont(TTFont('Korean','/System/Library/Fonts/Supplemental/AppleGothic.ttf'))
pdfmetrics.registerFont(TTFont('Unicode','/System/Library/Fonts/Supplemental/Arial Unicode.ttf'))
pdfmetrics.registerFontFamily('Korean',normal='Korean',bold='Korean',italic='Korean',boldItalic='Korean')
ink=colors.HexColor('#193341');blue=colors.HexColor('#16667b');muted=colors.HexColor('#647581')
styles={
 'body':ParagraphStyle('body',fontName='Korean',fontSize=9.4,leading=15,spaceAfter=7,textColor=ink,wordWrap='CJK'),
 'title':ParagraphStyle('title',fontName='Korean',fontSize=24,leading=32,spaceAfter=15,textColor=ink,keepWithNext=True),
 'h2':ParagraphStyle('h2',fontName='Korean',fontSize=14,leading=20,spaceBefore=16,spaceAfter=8,textColor=blue,keepWithNext=True),
 'h3':ParagraphStyle('h3',fontName='Korean',fontSize=10.7,leading=17,spaceBefore=10,spaceAfter=5,textColor=blue,keepWithNext=True),
 'table':ParagraphStyle('table',fontName='Korean',fontSize=8,leading=12,wordWrap='CJK',textColor=ink),
 'tablehead':ParagraphStyle('tablehead',fontName='Korean',fontSize=8,leading=12,wordWrap='CJK',textColor=colors.white),
 'bullet':ParagraphStyle('bullet',fontName='Korean',fontSize=9.4,leading=15,spaceAfter=5,leftIndent=11,firstLineIndent=-9,textColor=ink,wordWrap='CJK')}
def fmt(t):
 t=html.escape(t)
 t=re.sub(r'\[([^]]+)\]\(([^)]+)\)',lambda m:'<link color="#16667b" href="'+m[2]+'">'+m[1]+'</link>',t)
 t=re.sub(r'\*\*(.+?)\*\*',r'<b>\1</b>',t)
 t=re.sub(r'`([^`]+)`',r'<font color="#536979">\1</font>',t)
 return t
story=[];lines=(P/'REVIEW_KO.md').read_text().splitlines();i=0
while i<len(lines):
 l=lines[i].strip()
 if not l:i+=1;continue
 if l=='<!-- pagebreak -->':story.append(PageBreak());i+=1;continue
 if l.startswith('!['):
  match=re.match(r'!\[([^]]*)\]\(([^)]+)\)',l)
  if match:
   im=Image(str(P/match[2]));scale=min(487/im.imageWidth,230/im.imageHeight)
   im.drawWidth=im.imageWidth*scale;im.drawHeight=im.imageHeight*scale;im.hAlign='LEFT';story.extend([im,Spacer(1,10)])
  i+=1;continue
 if l.startswith('|'):
  block=[]
  while i<len(lines) and lines[i].strip().startswith('|'):
   raw=lines[i].strip().strip('|').split('|');raw=[x.strip() for x in raw]
   if not all(re.fullmatch(r'[:\- ]+',x) for x in raw):block.append(raw)
   i+=1
  nc=len(block[0]);width=487
  weights={2:[.28,.72],3:[.23,.35,.42],4:[.16,.22,.39,.23],5:[.32,.17,.17,.17,.17]}.get(nc,[1/nc]*nc)
  # Numeric results get a wider first column and equal metric columns.
  if '0.18' in ''.join(block[0]):weights=[.26,.37,.37]
  data=[[Paragraph(fmt(x),styles['tablehead' if j==0 else 'table']) for x in r] for j,r in enumerate(block)]
  table=Table(data,colWidths=[w*width for w in weights],repeatRows=1,hAlign='LEFT')
  table.setStyle(TableStyle([('FONTNAME',(0,0),(-1,-1),'Korean'),('BACKGROUND',(0,0),(-1,0),blue),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.HexColor('#f0f5f6'),colors.white]),('LINEBELOW',(0,-1),(-1,-1),.5,colors.HexColor('#b9cbd1'))]))
  story.extend([table,Spacer(1,10)]);continue
 if l.startswith('# '):style='title';l=l[2:]
 elif l.startswith('## '):style='h2';l=l[3:]
 elif l.startswith('### '):style='h3';l=l[4:]
 elif l.startswith('- '):style='bullet';l='• '+l[2:]
 elif re.match(r'^\d+\. ',l):style='bullet'
 else:style='body'
 paragraph=Paragraph(fmt(l),styles[style]);story.append(KeepTogether([paragraph]) if style=='body' else paragraph);i+=1

def review_canvas(*args,**kwargs):
 kwargs['initialFontName']='Korean'
 return Canvas(*args,**kwargs)

def page(canvas,doc):
 w,h=A4;canvas.saveState();canvas.setStrokeColor(colors.HexColor('#d7e1e5'));canvas.line(54,40,w-54,40);canvas.setFont('Korean',8);canvas.setFillColor(muted);canvas.drawString(54,27,'SCONE · ICRA 2027 · 제출 전 내부 검토');canvas.drawRightString(w-54,27,str(doc.page));canvas.restoreState()
SimpleDocTemplate(str(P/'output/pdf/SCONE_ICRA2027_Review_KO.pdf'),pagesize=A4,rightMargin=54,leftMargin=54,topMargin=49,bottomMargin=55,title='SCONE ICRA 2027 제출 전 검토',author='',pageCompression=1).build(story,onFirstPage=page,onLaterPages=page,canvasmaker=review_canvas)
print('Korean review rendered')

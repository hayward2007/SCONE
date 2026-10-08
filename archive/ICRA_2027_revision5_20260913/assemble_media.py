"""Assemble a labeled companion video without adding performance trials."""
from pathlib import Path
import json,subprocess
from PIL import Image,ImageDraw,ImageFont
import numpy as np
P=Path(__file__).resolve().parent;ROOT=P.parents[1]
font='/System/Library/Fonts/Supplemental/Arial.ttf'
parts=[];segments=[]
def run(cmd):subprocess.run(cmd,check=True)
def card(name,lines,duration):
 im=Image.new('RGB',(1280,720),'#142a36');d=ImageDraw.Draw(im)
 for i,line in enumerate(lines):d.text((70,135+i*83),line,font=ImageFont.truetype(font,34 if i else 43),fill='white' if i==0 else '#bcdfe4')
 png=P/f'build/{name}.png';im.save(png);out=P/f'build/{name}.mp4'
 run(['ffmpeg','-y','-loglevel','error','-loop','1','-i',str(png),'-t',str(duration),'-r','25','-c:v','libx264','-pix_fmt','yuv420p','-an',str(out)])
 parts.append(out);segments.append({'source':name,'duration_s':duration,'kind':'explanation'})
def clip(source,name,start,duration,label=None):
 out=P/f'build/{name}.mp4';vf='scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,fps=25'
 cmd=['ffmpeg','-y','-loglevel','error','-ss',str(start),'-i',str(source)]
 if label:
  overlay=Image.new('RGBA',(1280,720),(0,0,0,0));draw=ImageDraw.Draw(overlay)
  draw.rectangle((0,0,1280,48),fill=(0,0,0,255));draw.text((16,12),label,font=ImageFont.truetype(font,24),fill='white')
  label_path=P/f'build/{name}_overlay.png';overlay.save(label_path)
  cmd+=['-loop','1','-i',str(label_path),'-filter_complex',f'[0:v]{vf}[base];[base][1:v]overlay=0:0:shortest=1']
 else:cmd+=['-vf',vf]
 if duration is not None:cmd+=['-t',str(duration)]
 run(cmd+['-c:v','libx264','-crf','23','-pix_fmt','yuv420p','-an',str(out)])
 parts.append(out);segments.append({'source':str(source.relative_to(ROOT)),'source_start_s':start,'requested_duration_s':duration,'label':label})
card('intro',['SCONE: simple hybrid locomotion','One articulated arc mechanism, two tested modes','Flat: periodic reindexing','Stairs: fixed posture and one common phase'],5)
clip(ROOT/'archive/videos/SCONEv2_stairs.mp4','archive',4,8,'PHYSICAL ARCHIVE | earlier controller | no synchronized trial logs | 1x')
clip(P.parent/'ICRA_2027_revision4_20260913/media/joint_P.mp4','flat',0,None)
card('separate',['Mode-specific tests','The next clips start after stair-pose acquisition.','They do not show an autonomous floor-to-stair transition.','All stair replays below use a 2 ms physics step.'],5)
for jid in [45,47,75]:clip(P/f'media/stair_{jid}.mp4',f'new{jid}',0,None)
card('limits',['What the evidence supports','Fixed arc settings retain 100-150 mm grid success.','200 mm: rear-bank transfer remains incomplete.','150 mm wheel result changes at 1 ms; no impossibility claim.','New quantitative hardware validation remains necessary.'],8)
listing=P/'build/concat.txt';listing.write_text(''.join(f"file '{x}'\n" for x in parts))
run(['ffmpeg','-y','-loglevel','error','-f','concat','-safe','0','-i',str(listing),'-c:v','libx264','-crf','24','-pix_fmt','yuv420p','-r','25','-an','-movflags','+faststart',str(P/'media/companion_video.mp4')])
probe=json.loads(subprocess.check_output(['ffprobe','-v','quiet','-show_streams','-show_format','-of','json',str(P/'media/companion_video.mp4')]))
s=next(s for s in probe['streams'] if s['codec_type']=='video')
assert s['height']>=480 and int(probe['format']['size'])<=20000000 and float(probe['format']['duration'])<=180 and s['r_frame_rate']=='25/1' and s.get('field_order')=='progressive'
rows={r['id']:r for r in map(json.loads,(P/'evidence/stairs/trials.jsonl').read_text().splitlines())};checks=[]
for r in json.loads((P/'evidence/rendered_stair_trials.json').read_text()):
 b=rows[r['id']];keys=['final_y','final_z','measurement_work_j','minimum_upright','total_time_s']
 err=max(abs(r[k]-b[k]) for k in keys);assert err<1e-10
 assert r['ascent_crossed']==b['ascent_crossed'] and r['halt_success']==b['halt_success']
 checks.append({'id':r['id'],'max_error':err,'excluded_from_trial_count':True,'git_dirty':r['git_dirty']})
(P/'evidence/video_validation.json').write_text(json.dumps({'size_bytes':int(probe['format']['size']),'duration_s':float(probe['format']['duration']),'width':s['width'],'height':s['height'],'fps':s['r_frame_rate'],'scan':s['field_order'],'segments':segments,'replay_checks':checks},indent=2))
print('Video assembled and three replays matched',probe['format']['duration'],probe['format']['size'])

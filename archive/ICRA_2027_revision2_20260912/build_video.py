"""Compose a labeled, normal-speed scientific companion video from source clips."""
from pathlib import Path
import subprocess,json
from PIL import Image,ImageDraw,ImageFont
P=Path(__file__).resolve().parent;ROOT=P.parents[1]
TMP=Path('/private/tmp/scone-icra-rev2/video');TMP.mkdir(parents=True,exist_ok=True)
font='/System/Library/Fonts/Supplemental/Arial.ttf'
def ff(args):subprocess.run(['ffmpeg','-v','error','-y',*args],check=True)
def encode_tail(dest):return ['-an','-c:v','libx264','-preset','medium','-crf','25','-pix_fmt','yuv420p','-r','25','-map_metadata','-1','-movflags','+faststart',str(dest)]
def card(name,title,lines,duration):
 im=Image.new('RGB',(1280,720),'#14242d');d=ImageDraw.Draw(im)
 d.text((70,85),'RESEARCH VIDEO | WORKING DRAFT',font=ImageFont.truetype(font,25),fill='#63c4d0')
 d.text((70,158),title,font=ImageFont.truetype(font,39),fill='white')
 for i,line in enumerate(lines):d.text((70,295+i*57),line,font=ImageFont.truetype(font,28),fill='#e4edf0')
 im.save(TMP/f'{name}.png');dest=TMP/f'{name}.mp4'
 ff(['-loop','1','-i',str(TMP/f'{name}.png'),'-t',str(duration),*encode_tail(dest)]);return dest
def archival(name,source,start,duration,header,note):
 # Keep the complete frame, cables and surrounding scene. Text sits above the image.
 headerfile=TMP/f'{name}_label.png'
 label=Image.new('RGB',(1280,100),'black');draw=ImageDraw.Draw(label)
 draw.text((25,16),header,fill='white',font=ImageFont.truetype(font,27))
 draw.text((25,57),note,fill='#d2e0e4',font=ImageFont.truetype(font,23));label.save(headerfile)
 dest=TMP/f'{name}.mp4'
 vf='[0:v]fps=25,scale=1280:620:force_original_aspect_ratio=decrease:force_divisible_by=2,pad=1280:720:(ow-iw)/2:100:black,setsar=1[base];[base][1:v]overlay=0:0[v]'
 ff(['-ss',str(start),'-i',str(source),'-loop','1','-i',str(headerfile),'-t',str(duration),'-filter_complex',vf,'-map','[v]',*encode_tail(dest)]);return dest
parts=[card('intro','Rewind-budgeted arc-leg locomotion',[
 'Historical prototype footage + new MuJoCo comparisons',
 'Hardware archive is qualitative; controller version is unverified.',
 'Simulation command: 0.45 m/s | phase zero | normal speed',
 'Full study: 268 fixed trials | proposed B: 68/68 completed'],6)]
parts.append(archival('physical_floor',ROOT/'archive/videos/SCONEv2.mp4',0,24,'ARCHIVAL HARDWARE | source 0-24 s | 1x playback','Historical controller; external cables visible; no calibrated speed measurement'))
for name in ['N','U','B']:
 dest=TMP/f'{name}.mp4';ff(['-i',str(P/f'media/sim_{name}.mp4'),'-vf','setsar=1',*encode_tail(dest)]);parts.append(dest)
parts.append(archival('physical_stairs',ROOT/'archive/videos/SCONEv2_stairs.mp4',18,12,'ARCHIVAL HARDWARE | stairs, source 18-30 s | 1x playback','Qualitative motion; not a trial of the new controller; cable and moving camera retained'))
parts.append(card('outro','Evidence and remaining validation',[
 'B: 0.283 m/s | N: 0.126 m/s (eight-phase mean).',
 'Peak sector rate: U 1517 to B 536 deg/s at high command.',
 '24-piece tire contacts preserve the inner cavity.',
 'Sector rate constrained; full joint and hardware validation remain.'],8))
lst=TMP/'concat.txt';lst.write_text('\n'.join("file '"+str(p)+"'" for p in parts))
out=P/'media/companion_video.mp4';ff(['-f','concat','-safe','0','-i',str(lst),'-c','copy','-map_metadata','-1','-movflags','+faststart',str(out)])
probe=json.loads(subprocess.check_output(['ffprobe','-v','quiet','-show_format','-show_streams','-of','json',str(out)]))
v=next(s for s in probe['streams'] if s['codec_type']=='video')
assert out.stat().st_size<20_000_000 and float(probe['format']['duration'])<=180 and v['height']>=480 and v['r_frame_rate']=='25/1' and v['field_order']=='progressive'
(P/'evidence/video_validation.json').write_text(json.dumps({'path':'media/companion_video.mp4','size_bytes':out.stat().st_size,'probe':probe,'playback_speed':1,'audio_removed':True,'notes':'Historical clips are downsampled from 30 to 25 fps without changing duration. Simulations rendered natively at 25 fps. No frame interpolation or generative alteration.'},indent=2))
print(out,probe['format']['duration'],out.stat().st_size,flush=True)

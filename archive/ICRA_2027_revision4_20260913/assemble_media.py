from pathlib import Path
import json, subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent
rows={r['id']:r for r in map(json.loads,(P/'evidence/joint_limit/raw.jsonl').read_text().splitlines())}
replays=json.loads((P/'evidence/joint_rendered_metrics.json').read_text());checks=[]
for r in replays:
 b=rows[r['id']]; errors={k:abs(r[k]-v) for k,v in b.items() if isinstance(v,(int,float)) and not isinstance(v,bool)}
 assert max(errors.values())<1e-10,(r['id'],errors)
 for k in ['issued_target_rate_peak_dps','inner_setpoint_rate_peak_dps','actual_joint_rate_peak_dps','joint_tracking_rms_deg']:
  assert np.allclose(r[k],b[k],rtol=0,atol=1e-10)
 checks.append({'trial_id':r['id'],'code':r['code'],'max_scalar_error':max(errors.values()),'speed_mps':r['mean_vx_mps'],'excluded_from_trial_count':True})
(P/'evidence/joint_replay_validation.json').write_text(json.dumps(checks,indent=2))
plt.rcParams.update({'font.family':'DejaVu Sans','pdf.fonttype':42})
fig,axs=plt.subplots(2,3,figsize=(7.05,3.05),layout='constrained')
for i,c in enumerate('BP'):
 for j in range(3):
  axs[i,j].imshow(plt.imread(P/f'figures/joint_replay_{c}_{j}.png'));axs[i,j].axis('off')
  axs[i,j].set_title(f'{c}, t = {[0,10,19.96][j]:g} s',fontsize=8,pad=3)
fig.savefig(P/'figures/joint_replay.pdf',dpi=170);fig.savefig(P/'figures/joint_replay.png',dpi=180);plt.close(fig)
inputs=[P.parent/'ICRA_2027_revision2_20260912/media/companion_video.mp4',P/'media/joint_B.mp4',P/'media/joint_P.mp4']
listing=P/'build/video_concat.txt';listing.write_text(''.join(f"file '{x}'\n" for x in inputs))
subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(listing),'-c:v','libx264','-crf','24','-pix_fmt','yuv420p','-r','25','-an','-movflags','+faststart',str(P/'media/companion_video.mp4')],check=True)
probe=json.loads(subprocess.check_output(['ffprobe','-v','quiet','-show_streams','-show_format','-of','json',str(P/'media/companion_video.mp4')]))
s=next(x for x in probe['streams'] if x['codec_type']=='video')
assert s['height']>=480 and float(probe['format']['duration'])<=180 and int(probe['format']['size'])<=20000000
(P/'evidence/current_video_validation.json').write_text(json.dumps({'size_bytes':int(probe['format']['size']),'duration_s':float(probe['format']['duration']),'width':s['width'],'height':s['height'],'frame_rate':s['r_frame_rate'],'field_order':s.get('field_order'),'audio':any(x['codec_type']=='audio' for x in probe['streams']),'segments':['0-74 s: prior labeled archival and baseline simulation','74-94 s: B, H 0.8, simulated 1x','94-114 s: P, H 0.8, simulated 1x']},indent=2))
print('Replays validated; companion video assembled')

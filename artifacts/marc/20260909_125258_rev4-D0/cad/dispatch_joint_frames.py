import sys, json, importlib
sys.path.insert(0, '/Users/hayward_kim/Developer/SCONE/artifacts/marc/20260909_125258_rev4-D0/evidence/tools_snapshot')
import fusion_common, rebuild_joint_frames
importlib.reload(fusion_common)
importlib.reload(rebuild_joint_frames)
def run(_context: str):
    ctx=json.loads(open('/Users/hayward_kim/Developer/SCONE/artifacts/marc/20260909_125258_rev4-D0/context.json').read())
    print(json.dumps(rebuild_joint_frames.run(ctx)))

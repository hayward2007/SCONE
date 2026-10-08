import sys, json, importlib
sys.path.insert(0, '/Users/hayward_kim/Developer/SCONE/artifacts/marc/20260909_125258_rev4-D0/evidence/tools_snapshot')
import fusion_common, build_03_sector
importlib.reload(fusion_common)
importlib.reload(build_03_sector)
def run(_context: str):
    ctx=json.loads(open('/Users/hayward_kim/Developer/SCONE/artifacts/marc/20260909_125258_rev4-D0/context.json').read())
    print(json.dumps(build_03_sector.run(ctx)))

import sys, json
sys.path.insert(0, '/Users/hayward_kim/Developer/SCONE/artifacts/marc/20260909_125258_rev4-D0/evidence/tools_snapshot')
import create_document, importlib
importlib.reload(create_document)
def run(_context: str):
    ctx=json.loads(open('/Users/hayward_kim/Developer/SCONE/artifacts/marc/20260909_125258_rev4-D0/context.json').read())
    print(json.dumps(create_document.run(ctx)))

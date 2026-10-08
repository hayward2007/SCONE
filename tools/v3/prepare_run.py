"""Create an isolated, source-locked SCONE v3 CAD run. No Fusion mutation."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import re
import subprocess
import sys


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.part')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temp.replace(path)


def run(ctx):
    root = Path(ctx['repo_root']).resolve()
    out = Path(ctx['run_dir']).resolve()
    out.mkdir(parents=True, exist_ok=False)
    for name in ('cad', 'evidence', 'paths', 'meshes'):
        (out / name).mkdir()
    source = root / 'docs/30-scone-v3-design-plan.md'
    text = source.read_text()
    def extract(marker, language):
        content = text.split('<!-- ' + marker + ':start -->', 1)[1].split('<!-- ' + marker + ':end -->', 1)[0]
        return re.search(r'^```' + language + r'\n(.*?)\n```\s*$', content, re.M | re.S).group(1)
    params = json.loads(extract('scone-v3-params', 'json'))
    write_json(out / 'params.json', params)
    reference = out / 'evidence/reference_calculation.py'
    reference.write_text(extract('scone-v3-reference-python', 'python') + '\n')
    result = subprocess.run([sys.executable, str(reference)], cwd=root, check=True, capture_output=True, text=True)
    write_json(out / 'calculations.json', json.loads(result.stdout))
    doc_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    ctx.update(document_id=None, creation_id=None, stage='D0', source_sha256=doc_hash,
               params_sha256=hashlib.sha256((out / 'params.json').read_bytes()).hexdigest())
    write_json(out / 'context.json', ctx)
    names = subprocess.check_output(['git', 'ls-files', '-m', '-o', '--exclude-standard', '-z'], cwd=root).decode().split('\0')
    protected = {n: hashlib.sha256((root/n).read_bytes()).hexdigest() for n in names
                 if n and (root/n).is_file() and not n.startswith(('artifacts/', 'tools/v3/'))}
    write_json(out / 'source_manifest.json', {
        'created_at': datetime.datetime.now().astimezone().isoformat(),
        'document_path': str(source), 'document_sha256': doc_hash,
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root).decode().strip(),
        'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=root).decode(),
        'protected_files': protected, 'scope': 'New Fusion D0 design; existing models and controllers preserved',
    })
    write_json(out / 'inputs.json', {f'I{i:02}': {'status': 'BLOCKED', 'value': None, 'evidence': []} for i in range(1,10)})
    write_json(out / 'checks.json', {f'G{i:02}': {'status': 'NOT_RUN', 'model_stage': 'D0'} for i in range(11)})
    write_json(out / 'parts_manifest.json', {'parts': [], 'missing': [], 'stage': 'D0'})
    return ctx


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo-root', default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument('--run-dir')
    args = parser.parse_args()
    run_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_rev3-D0')
    directory = args.run_dir or str(Path(args.repo_root)/'artifacts/scone_v3'/run_id)
    print(json.dumps(run({'repo_root': str(Path(args.repo_root).resolve()), 'run_dir': directory}), indent=2))

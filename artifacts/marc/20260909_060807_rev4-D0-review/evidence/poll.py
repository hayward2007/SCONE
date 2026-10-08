#!/usr/bin/env python3
"""Snapshot the MARC D0 run directory so an external reviewer can tell when GPT stopped writing."""
import json, os, subprocess, sys, time
from pathlib import Path

CAND = [Path.home()/"Developer/SCONE/artifacts/marc", Path.home()/"mnt/SCONE/artifacts/marc"]
RUN = next((c for c in CAND if c.exists()), CAND[0])
runs = sorted([p for p in RUN.iterdir() if p.is_dir()]) if RUN.exists() else []
if not runs:
    print(json.dumps({"error": "no run dir"})); sys.exit(0)
run = runs[-1]

files, newest, newest_path = 0, 0.0, ""
for p in run.rglob("*"):
    if p.is_file():
        files += 1
        m = p.stat().st_mtime
        if m > newest:
            newest, newest_path = m, str(p.relative_to(run))

checks = {}
cp = run / "checks.json"
if cp.exists():
    try:
        d = json.loads(cp.read_text())
        for k, v in d.items():
            s = v.get("status") if isinstance(v, dict) else str(v)
            checks[s] = checks.get(s, 0) + 1
    except Exception as e:
        checks = {"parse_error": str(e)}

now = time.time()
out = {
    "run": run.name,
    "now": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
    "files": files,
    "newest_file": newest_path,
    "newest_mtime": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(newest)),
    "idle_seconds": round(now - newest),
    "checks_status": checks,
    "handoff_exists": (run / "handoff.md").exists(),
    "parts_manifest_exists": (run / "parts_manifest.json").exists(),
}
print(json.dumps(out, ensure_ascii=False))

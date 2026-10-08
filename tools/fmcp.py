#!/usr/bin/env python3
"""Fusion MCP (streamable-http) client.
  fmcp.py list
  fmcp.py call <tool> '<json args>'
  fmcp.py script <path.py>          # run python inside Fusion
  fmcp.py shot <out.png> [direction] [width]
"""
import base64, json, sys, urllib.request

URL = "http://127.0.0.1:27182/mcp"
SID = None

def post(payload, notify=False):
    global SID
    req = urllib.request.Request(URL, data=json.dumps(payload).encode(), method="POST")
    req.add_header("Content-Type", "application/json")
    req.add_header("Accept", "application/json, text/event-stream")
    if SID:
        req.add_header("MCP-Session-Id", SID)
    with urllib.request.urlopen(req, timeout=300) as r:
        sid = r.headers.get("MCP-Session-Id")
        if sid:
            SID = sid
        body = r.read().decode("utf-8", "replace")
    if notify or not body.strip():
        return None
    if body.lstrip().startswith(("event:", "data:")):
        body = "\n".join(l[5:].strip() for l in body.splitlines() if l.startswith("data:"))
    return json.loads(body)

def init():
    post({"jsonrpc":"2.0","id":1,"method":"initialize","params":{
        "protocolVersion":"2025-06-18","capabilities":{},
        "clientInfo":{"name":"scone-v3","version":"1.0"}}})
    try:
        post({"jsonrpc":"2.0","method":"notifications/initialized"}, notify=True)
    except Exception:
        pass

def call(name, args):
    return post({"jsonrpc":"2.0","id":2,"method":"tools/call",
                 "params":{"name":name,"arguments":args}})

def text_of(r):
    out = []
    for c in (r.get("result", {}) or {}).get("content", []) or []:
        if c.get("type") == "text":
            out.append(c["text"])
    if not out:
        return json.dumps(r, ensure_ascii=False, indent=1)
    return "\n".join(out)

def main():
    init()
    cmd = sys.argv[1]
    if cmd == "list":
        print(json.dumps(post({"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}),
                         ensure_ascii=False, indent=1)); return
    if cmd == "call":
        print(text_of(call(sys.argv[2], json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}))); return
    if cmd == "script":
        src = open(sys.argv[2], encoding="utf-8").read()
        print(text_of(call("fusion_mcp_execute", {"featureType":"script","object":{"script":src}}))); return
    if cmd == "shot":
        out = sys.argv[2]
        args = {"queryType":"screenshot","transparentBackground":False}
        if len(sys.argv) > 3: args["direction"] = sys.argv[3]
        if len(sys.argv) > 4: args["width"] = int(sys.argv[4]); args["height"] = int(int(sys.argv[4])*0.72)
        r = call("fusion_mcp_read", args)
        b64 = None
        for c in (r.get("result", {}) or {}).get("content", []) or []:
            if c.get("type") == "image" and c.get("data"):
                b64 = c["data"]; break
            if c.get("type") == "text":
                try:
                    b64 = json.loads(c["text"]).get("base64Data")
                except Exception:
                    pass
        if not b64:
            print(json.dumps(r)[:600]); return
        open(out, "wb").write(base64.b64decode(b64))
        print("saved", out); return
    raise SystemExit("unknown cmd")

main()

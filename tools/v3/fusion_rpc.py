"""Side-effect-free Fusion MCP transport; scripts are never retried automatically."""
from pathlib import Path
import argparse
import base64
import json
import urllib.request


class Client:
    def __init__(self, timeout=60):
        self.url = 'http://127.0.0.1:27182/mcp'
        self.session = None
        self.timeout = timeout
        self.sequence = 0
        self.post('initialize', {'protocolVersion': '2025-06-18', 'capabilities': {},
                               'clientInfo': {'name':'scone-v3-D0', 'version':'1.0'}})
        self.post('notifications/initialized', {}, notify=True)

    def post(self, method, params, notify=False):
        payload = {'jsonrpc':'2.0','method':method,'params':params}
        if not notify:
            self.sequence += 1
            payload['id'] = self.sequence
        req = urllib.request.Request(self.url, data=json.dumps(payload).encode(), method='POST',
                                     headers={'Content-Type':'application/json','Accept':'application/json, text/event-stream'})
        if self.session:
            req.add_header('MCP-Session-Id', self.session)
        with urllib.request.urlopen(req, timeout=self.timeout) as response:
            self.session = response.headers.get('MCP-Session-Id', self.session)
            raw = response.read().decode('utf-8')
        if not raw.strip():
            return None
        if raw.lstrip().startswith(('event:', 'data:')):
            messages = [json.loads(line[5:].strip()) for line in raw.splitlines() if line.startswith('data:')]
            result = next(m for m in reversed(messages) if m.get('id') == payload.get('id'))
        else:
            result = json.loads(raw)
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result.get('result', result)

    def call(self, name, args):
        return self.post('tools/call', {'name': name, 'arguments': args})


def texts(result):
    return '\n'.join(c['text'] for c in result.get('content',[]) if c.get('type') == 'text')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['list','read','script','shot'])
    parser.add_argument('argument', nargs='?')
    parser.add_argument('--output')
    parser.add_argument('--timeout', type=int, default=60)
    args = parser.parse_args()
    client = Client(args.timeout)
    if args.operation == 'list':
        result = client.post('tools/list', {})
    elif args.operation == 'script':
        code = Path(args.argument).read_text()
        compile(code, args.argument, 'exec')
        result = client.call('fusion_mcp_execute', {'featureType':'script','object':{'script':code}})
    elif args.operation == 'shot':
        result = client.call('fusion_mcp_read', {'queryType':'screenshot','width':1600,'height':1100,
                            'transparentBackground':False,'direction':args.argument or 'current'})
    else:
        result = client.call('fusion_mcp_read', json.loads(args.argument))
    if args.output:
        if args.operation == 'shot':
            blocks = result.get('content', [])
            encoded = next((c['data'] for c in blocks if c.get('type') == 'image'), None)
            if encoded is None:
                encoded = json.loads(texts(result))['base64Data']
            Path(args.output).write_bytes(base64.b64decode(encoded))
        else:
            Path(args.output).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    if args.operation != 'shot':
        print(texts(result) or json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(json.dumps({'screenshot':args.output}))
    if result.get('isError'):
        raise SystemExit(1)

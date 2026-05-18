#!/usr/bin/env bash
set -euo pipefail
URL="${1:-http://127.0.0.1:18080}"

python3 - "$URL" <<'PY'
import sys
import urllib.request

base = sys.argv[1].rstrip('/')
for path in ('/healthz', '/latest'):
    url = base + path
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            data = r.read(500)
            print(f'{path}: {r.status} {len(data)} bytes')
            if path == '/healthz':
                print(data.decode('utf-8', 'replace'))
    except Exception as exc:
        print(f'{path}: FAIL {exc}')
        raise SystemExit(1)
PY

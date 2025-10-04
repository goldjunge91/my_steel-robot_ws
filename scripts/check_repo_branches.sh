#!/usr/bin/env bash
# Check that branches/versions configured in src/ros2.repos exist on GitHub
set -euo pipefail
REPOS=src/ros2.repos
OUT=repo_branch_check.txt
echo "Repo branch check generated: $(date)" > "$OUT"
python3 - <<'PY'
import yaml,re
from pathlib import Path
r=Path('src/ros2.repos').read_text()
data=yaml.safe_load(r)
if not data or 'repositories' not in data:
    print('No repositories found in ros2.repos');
    raise SystemExit(1)
for name,info in data['repositories'].items():
    url=info.get('url','')
    version=info.get('version','')
    print(f"Checking {name}: url={url} version={version}")
    m=re.search(r'github.com[:/](?P<owner>[^/]+)/(?P<repo>[^/.]+)', url)
    if not m:
        print(f"  Skipping {name}: not a github url: {url}")
        continue
    owner=m.group('owner'); repo=m.group('repo')
    # Call GitHub API unauthenticated; may be rate limited
    import requests
    api=f'https://api.github.com/repos/{owner}/{repo}/branches'
    resp=requests.get(api)
    if resp.status_code==200:
        branches=[b['name'] for b in resp.json()]
        ok=version in branches
        print(f"  branches: {branches[:5]}{'...' if len(branches)>5 else ''}")
        print(f"  version '{version}' present on remote: {ok}")
    else:
        print(f"  Failed to query {owner}/{repo}: {resp.status_code} {resp.text[:200]}")
PY

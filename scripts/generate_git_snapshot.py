#!/usr/bin/env python3
import subprocess
from pathlib import Path
from datetime import datetime
out=Path('git_snapshot_py_{}.txt'.format(datetime.now().strftime('%Y%m%d_%H%M%S')))
with out.open('w') as f:
    f.write(f'Snapshot generated: {__import__("datetime").datetime.now().isoformat()}\n')
    f.write('--- Top-level git branch and status ---\n')
    try:
        f.write(subprocess.check_output(['git','rev-parse','--abbrev-ref','HEAD'],encoding='utf-8'))
    except Exception as e:
        f.write('ERR: '+str(e)+'\n')
    try:
        f.write(subprocess.check_output(['git','status','--porcelain'],encoding='utf-8'))
    except Exception as e:
        f.write('ERR: '+str(e)+'\n')
    f.write('--- Submodules (if any) ---\n')
    try:
        f.write(subprocess.check_output(['git','submodule','status'],encoding='utf-8'))
    except Exception as e:
        f.write('ERR: '+str(e)+'\n')
    f.write('--- Per-src directory git remote and HEAD ---\n')
    for p in sorted(Path('src').iterdir()):
        if not p.is_dir():
            continue
        f.write(f'Directory: {p}\n')
        if (p/'.git').exists():
            try:
                f.write('Remote:\n')
                f.write(subprocess.check_output(['git','-C',str(p),'remote','-v'],encoding='utf-8'))
                f.write('HEAD:\n')
                f.write(subprocess.check_output(['git','-C',str(p),'rev-parse','--abbrev-ref','HEAD'],encoding='utf-8'))
                f.write('SHA:\n')
                f.write(subprocess.check_output(['git','-C',str(p),'rev-parse','--short','HEAD'],encoding='utf-8'))
            except Exception as e:
                f.write('ERR: '+str(e)+'\n')
        else:
            f.write('  Not a git repo\n')
print('Wrote',out)

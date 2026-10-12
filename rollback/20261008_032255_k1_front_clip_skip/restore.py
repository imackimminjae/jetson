from pathlib import Path
import hashlib,json,shutil,os
h=Path(__file__).resolve().parent
m=json.loads((h/'manifest.json').read_text())
for row in m['files']:
 p=Path(row['path']);saved=h/row['backup']
 if hashlib.sha256(p.read_bytes()).hexdigest()!=row['after_sha256']:
  raise SystemExit('Refusing restore: file changed since trial setup: '+str(p))
 if hashlib.sha256(saved.read_bytes()).hexdigest()!=row['before_sha256']:
  raise SystemExit('Backup integrity failure: '+str(saved))
for row in m['files']:
 p=Path(row['path']);tmp=p.with_name(p.name+'.k1_restore_tmp')
 if tmp.exists():raise SystemExit('Restore temporary path already exists: '+str(tmp))
 shutil.copy2(h/row['backup'],tmp);os.replace(tmp,p)
print('Restored pre-trial source, header, tests, config, and upper planner executable. No process restart or hardware action performed.')

2026-10-13 local cleanup. Every file in deleted_files.txt was verified byte-identical on branch logs/archive-20261006 (commit ec9fbc5; >100 MB files as .zst, verified after decompression).
Also removed: px4_rest_audit_20261002/.venv, __pycache__ dirs, followup_20261009/github_logs_upload (clone; HEAD beab5c0 is in the archive; its only unique file UPLOAD_ALL60_README.txt is kept here).
Restore: git restore --source=logs/archive-20261006 -- <path>  (paths are gitignored; or git show logs/archive-20261006:<path> > <path>)

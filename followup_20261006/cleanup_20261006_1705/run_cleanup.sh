#!/bin/bash
# Deletes bags (dirs_to_delete.txt) and finished-analysis leftovers (files_to_delete.txt). Figures/RESULT/scripts are kept.
set -e
cd /home/imac/ros2_ws
H=followup_20261006/cleanup_20261006_1705
df -h / | tail -1
xargs -d '\n' rm -f  < $H/files_to_delete.txt
xargs -d '\n' rm -rf < $H/dirs_to_delete.txt
find followup_20261005 followup_20261006 full_audit_20261003 -type d -empty -delete
echo "png kept: $(find followup_2026100* full_audit_20261003 -name '*.png' | wc -l)"
df -h / | tail -1

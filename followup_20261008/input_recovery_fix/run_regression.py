"""Load the staged runner for existing isolated tests, leaving installed symlinks untouched."""
from pathlib import Path
import os,sys,importlib.util
assert os.environ.get('ROS_DOMAIN_ID')=='177'
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
H=Path(__file__).resolve().parent;P=Path('/home/imac/ros2_ws/src/virtual_control')
sys.path.insert(0,str(P/'scripts'))
spec=importlib.util.spec_from_file_location('knu_batch_runner',H/'candidate/knu_batch_runner.py');module=importlib.util.module_from_spec(spec);sys.modules['knu_batch_runner']=module;spec.loader.exec_module(module)
import pytest
raise SystemExit(pytest.main(['-q',str(H/'test_recovery.py'),str(P/'test/test_knu_batch.py'),str(P/'test/test_repeat_drive_setup.py'),str(P/'test/test_repeat_drive_ros.py'),'-k','not real_recorder_discovery']))

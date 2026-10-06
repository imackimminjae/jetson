from pathlib import Path
import shlex,subprocess
root=Path('/home/imac/ros2_ws');build=root/'build/virtual_control';out=root/'drive_debug_20260924_042235_light/analysis'
flags={}
for line in (build/'CMakeFiles/test_lower_path_generation.dir/flags.make').read_text().splitlines():
 if ' = ' in line:
  k,v=line.split(' = ',1);flags[k]=shlex.split(v)
cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],'-std=gnu++17','-O0','-c',str(out/'snapshot_probe.cpp'),'-o',str(out/'snapshot_probe.o')]
subprocess.run(cmd,check=True)
link=shlex.split((build/'CMakeFiles/test_lower_path_generation.dir/link.txt').read_text())
link=[str(out/'snapshot_probe.o') if x=='CMakeFiles/test_lower_path_generation.dir/test/test_lower_path_generation.cpp.o' else str(out/'snapshot_probe') if x=='test_lower_path_generation' else x for x in link if not x.startswith('gtest/')]
subprocess.run(link,cwd=build,check=True)

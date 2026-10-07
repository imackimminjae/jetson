from pathlib import Path
import shlex
import subprocess

here = Path(__file__).resolve().parent
root = here.parents[1]
flags = []
for line in (root / 'build/virtual_control/CMakeFiles/lower_tracking_mpc_node.dir/flags.make').read_text().splitlines():
    if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
        flags += shlex.split(line.split('=', 1)[1])
flags = [x for x in flags if x != '-O3'] + ['-O2']
link = shlex.split((root / 'build/virtual_control/CMakeFiles/upper_planner_node.dir/link.txt').read_text())
libs = link[link.index('upper_planner_node') + 1:]
with (here / 'build.log').open('w') as log:
    for name in ('tracking_control_experiment', 'replay', 'sim'):
        subprocess.run(['/usr/bin/c++', *flags, '-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN',
                        '-c', str(here / (name + '.cpp')), '-o', str(here / (name + '.o'))],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    for name in ('replay', 'sim'):
        subprocess.run(['/usr/bin/c++', str(here / (name + '.o')),
                        str(here / 'tracking_control_experiment.o'), '-o', str(here / name), *libs],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
print('Built isolated executables; no node started')

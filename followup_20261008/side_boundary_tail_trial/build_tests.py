from pathlib import Path
import shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1];T=H/'trial/src/virtual_control';B=R/'build/virtual_control';flags=['-I'+str(T/'include')]
for line in (B/'CMakeFiles/test_lower_path_generation.dir/flags.make').read_text().splitlines():
 if line.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(line.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O1'];link=shlex.split((B/'CMakeFiles/test_lower_path_generation.dir/link.txt').read_text());libs=link[link.index('test_lower_path_generation')+1:];libs=[str(B/v) if v.startswith('gtest/') else v for v in libs]
with (H/'tests_build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*flags,'-c',str(T/'test/test_lower_path_generation.cpp'),'-o',str(H/'tests.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'tests.o'),str(H/'tracking_control.o'),'-o',str(H/'tests'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built isolated tests')

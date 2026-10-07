# Rebuild only the harness object; reuse planner_now.o (current production tracking_control.cpp).
exec(open('build.py').read().replace("subprocess.run(['/usr/bin/c++',*flags,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN'","(lambda *a,**k:None)(['/usr/bin/c++',*flags,'-DVIRTUAL_CONTROL_UPPER_PLANNER_NO_MAIN'"))

from pathlib import Path
import urllib.request
root=Path(__file__).resolve().parent/'reference_source';base='https://raw.githubusercontent.com/PX4/PX4-Autopilot/94cb2012792b2ae89f0b147cfee53ee31ae550be/'
for name in ['src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp','src/modules/ekf2/EKF/ekf.h','src/modules/ekf2/EKF/control.cpp','src/modules/land_detector/LandDetector.h']:
 try:
  data=urllib.request.urlopen(base+name,timeout=15).read();p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data);print(name,len(data),flush=True)
 except Exception as e:print(name,str(e),flush=True)

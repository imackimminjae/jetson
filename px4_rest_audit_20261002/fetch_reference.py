from pathlib import Path
import urllib.request,json,hashlib
root=Path(__file__).resolve().parent/'reference_source';root.mkdir(exist_ok=True)
paths=['src/modules/land_detector/RoverLandDetector.cpp','src/modules/land_detector/RoverLandDetector.h','src/modules/land_detector/LandDetector.cpp','src/modules/ekf2/EKF2.cpp','src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp','src/modules/logger/logged_topics.cpp','src/modules/logger/logger.cpp','msg/EstimatorGpsStatus.msg','msg/EstimatorStatusFlags.msg','src/systemcmds/topic_listener/listener_main.cpp']
base='https://raw.githubusercontent.com/PX4/PX4-Autopilot/94cb2012792b2ae89f0b147cfee53ee31ae550be/';manifest=[]
for path in paths:
 try:
  data=urllib.request.urlopen(base+path,timeout=15).read();p=root/path;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data);manifest.append({'path':path,'url':base+path,'sha256':hashlib.sha256(data).hexdigest()});print(path,len(data),flush=True)
 except Exception as e:print(path,str(e),flush=True)
(root/'manifest.json').write_text(json.dumps(manifest,indent=2))

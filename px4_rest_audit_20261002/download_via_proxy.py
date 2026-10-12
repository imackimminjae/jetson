"""Download a board file read-only through existing MAVProxy; no consumer-port bind.
Sends only MAVFTP protocol requests. Receives replies from MAVProxy's existing tlog.
"""
import argparse,os,time,json,hashlib,logging,struct
from pathlib import Path
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP

class TlogMaster:
 def __init__(self,port,tlog):
  self.sender=mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}',source_system=246,source_component=191,dialect='common');self.mav=self.sender.mav;self.source_system=246;self.source_component=191
  self.f=open(tlog,'rb',buffering=0);self.buf=bytearray();self.decoder=mavutil.mavlink.MAVLink(None);self.decoder.robust_parsing=True
  # Drain existing complete frames. Any partial final record stays buffered.
  self.buf.extend(self.f.read());self._drain(discard=True)
 def _drain(self,discard=False):
  start=0;result=None
  while len(self.buf)-start>=10:
   magic=self.buf[start+8];payloadlen=self.buf[start+9]
   if magic==253:
    if len(self.buf)-start<20:break
    length=payloadlen+12+(13 if self.buf[start+10]&1 else 0)
    msgid=int.from_bytes(self.buf[start+15:start+18],'little')
   elif magic==254:
    if len(self.buf)-start<14:break
    length=payloadlen+8;msgid=self.buf[start+13]
   else:start+=1;continue
   if len(self.buf)-start<length+8:break
   packet=bytes(self.buf[start+8:start+8+length]);start+=8+length
   if msgid!=110 or discard:continue
   try:m=self.decoder.parse_char(packet)
   except Exception:continue
   if m and m.get_type()=='FILE_TRANSFER_PROTOCOL' and m.get_srcSystem()==1 and m.target_system==246 and m.target_component==191:result=m;break
  if start:del self.buf[:start]
  return result
 def recv_match(self,type=None,blocking=False,timeout=.1):
  end=time.monotonic()+(timeout or .1)
  while True:
   m=self._drain()
   if m is not None:return m
   chunk=self.f.read(262144)
   if chunk:self.buf.extend(chunk);continue
   if time.monotonic()>=end:return None
   time.sleep(.01)
 def close(self):self.f.close();self.sender.close()

def main():
 a=argparse.ArgumentParser();a.add_argument('--proxy-port',type=int,required=True);a.add_argument('--remote',required=True);a.add_argument('--local',type=Path,required=True);a.add_argument('--tlog',default='/home/imac/ros2_ws/mav.tlog');p=a.parse_args()
 if p.local.exists():raise SystemExit('Refusing to overwrite an existing download')
 if not p.remote.startswith('/fs/microsd/log/') or not p.remote.endswith('.ulg'):raise SystemExit('Only ULog reads under /fs/microsd/log are allowed')
 logging.basicConfig(level=logging.INFO);master=TlogMaster(p.proxy_port,p.tlog)
 try:
  ftp=MAVFTP(master,1,1);ftp.temp_filename=str(p.local)+'.part';ftp.ftp_settings.burst_read_size=239
  last=[0]
  def progress(r):
   if r is not None and time.monotonic()-last[0]>10:print(f'download {r*100:.1f}%',flush=True);last[0]=time.monotonic()
  ftp.cmd_get([p.remote,str(p.local)],progress_callback=progress);result=ftp.process_ftp_reply('OpenFileRO',timeout=900)
  if not ftp.done or not p.local.exists():raise RuntimeError(f'Download incomplete: {result}')
  size=p.local.stat().st_size
  if size!=ftp.remote_file_size:raise RuntimeError(f'Size mismatch {size}/{ftp.remote_file_size}')
  data=p.local.read_bytes()
  if not data.startswith(b'ULog\x01\x12\x35'):raise RuntimeError('Not a ULog file')
  report={'remote':p.remote,'bytes':size,'sha256':hashlib.sha256(data).hexdigest(),'read_retries':ftp.read_retries,'remaining_gaps':len(ftp.read_gaps),'received_at':time.time()};p.local.with_suffix('.download.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
 finally:master.close()
if __name__=='__main__':main()

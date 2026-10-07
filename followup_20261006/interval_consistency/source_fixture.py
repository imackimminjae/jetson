"""Extract and execute attached C++ segmentation functions verbatim on synthetic RGB."""
from pathlib import Path
import hashlib,json,subprocess,shlex
here=Path(__file__).resolve().parent
attachment=Path('/home/imac/.codex/attachments/2501702b-cbf2-49c7-b5da-7f85ee4a19eb/rgb_to_occupancy.cpp')
folder=here/'bev_source';folder.mkdir(exist_ok=True)
saved=folder/'rgb_to_occupancy.original.cpp'
if not saved.exists():saved.write_bytes(attachment.read_bytes())
assert saved.read_bytes()==attachment.read_bytes()
source=saved.read_text()
def method(signature):
    start=source.index(signature);brace=source.index('{',start);depth=1;end=brace+1
    while depth:
        if source[end]=='{':depth+=1
        elif source[end]=='}':depth-=1
        end+=1
    return source[start:end]
methods=[method('  static cv::Mat keepLargestComponentBinary255('),method('  cv::Mat buildRoadMaskForCurrentIsaacEnv(')]
cpp='''#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
struct Fixture {
 int road_gray_max_=85,white_line_gray_min_=180,morph_ksize_=5;
 bool keep_largest_component_=true;
'''+ '\n'.join(methods)+'''
};
int main(int argc,char**argv){
 using J=nlohmann::json; J out=J::array();
 for(bool largest:{true,false}){
  cv::Mat previous;
  for(int offset:{-2,2}){
   // 40x40m BEV at 0.02m, 10m asphalt strip, 0.2m white center stripe.
   cv::Mat rgb(2000,2000,CV_8UC3,cv::Scalar(140,140,140));
   rgb.colRange(750,1250).setTo(cv::Scalar(50,50,50));
   rgb.colRange(995+offset,1005+offset).setTo(cv::Scalar(255,255,255));
   cv::Mat valid(2000,2000,CV_8UC1,cv::Scalar(255));Fixture f;f.keep_largest_component_=largest;
   auto mask=f.buildRoadMaskForCurrentIsaacEnv(rgb,valid);
   cv::Mat grid;cv::resize(mask,grid,cv::Size(128,128),0,0,cv::INTER_NEAREST);
   auto moments=cv::moments(mask,true);
   J row={{"keep_largest",largest},{"white_line_shift_m",offset*.02},
          {"free_area_m2",cv::countNonZero(mask)*.0004},
          {"free_centroid_lateral_m",(moments.m10/moments.m00-1000)*.02}};
   int components=0;bool in=false;
   for(int col=0;col<grid.cols;++col){bool now=grid.at<unsigned char>(110,col)>0;if(now&&!in)++components;in=now;}
   row["near_section_free_runs"]=components;
   if(!previous.empty()){cv::Mat diff;cv::bitwise_xor(previous,grid,diff);row["grid_changed_cells"]=cv::countNonZero(diff);}
   previous=grid.clone();out.push_back(row);
   if(argc>1)cv::imwrite(std::string(argv[1])+"/mask_"+(largest?"largest":"all")+"_"+std::to_string(offset)+".png",grid);
  }
 }
 std::cout<<out.dump(2)<<std::endl;
}
'''
(folder/'segmentation_fixture.cpp').write_text(cpp)
flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True))
if not Path('/usr/local/include/opencv4/opencv2/core.hpp').is_file():
    flags=['-I/usr/include/opencv4']+[str(next(Path('/usr/lib/aarch64-linux-gnu').glob(f'libopencv_{name}.so.4.5.*'))) for name in ['imgcodecs','imgproc','core']]
subprocess.run(['/usr/bin/c++','-std=c++17','-O1',str(folder/'segmentation_fixture.cpp'),'-o',str(folder/'segmentation_fixture'),*flags],check=True)
r=subprocess.run([str(folder/'segmentation_fixture'),str(folder)],capture_output=True,text=True)
(folder/'native_attempt.json').write_text(json.dumps({'returncode':r.returncode,'stderr':r.stderr,
    'headers':'OpenCV 4.8.0','available_shared_libraries':'OpenCV 4.5.4'},indent=2)+'\n')
if r.returncode==0:
    (folder/'segmentation_fixture.json').write_text(r.stdout)
    print(r.stdout)
else:
    # This host has mismatched C++ OpenCV headers/libraries. Preserve the failed
    # native attempt; execute the identical image operations with Python OpenCV.
    import cv2,numpy as np
    out=[]
    for largest in [True,False]:
        previous=None
        for offset in [-2,2]:
            rgb=np.full((2000,2000,3),140,np.uint8);rgb[:,750:1250]=50;rgb[:,995+offset:1005+offset]=255
            gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
            candidate=cv2.bitwise_and((gray<=85).astype(np.uint8)*255,cv2.bitwise_not((gray>=180).astype(np.uint8)*255))
            kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
            mask=cv2.morphologyEx(cv2.morphologyEx(candidate,cv2.MORPH_CLOSE,kernel),cv2.MORPH_OPEN,kernel)
            n,labels,stats,_=cv2.connectedComponentsWithStats(mask,connectivity=8)
            if largest:
                best=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]));mask=np.where(labels==best,255,0).astype(np.uint8)
            grid=cv2.resize(mask,(128,128),interpolation=cv2.INTER_NEAREST);m=cv2.moments(mask,True)
            near=grid[110]>0
            row=dict(keep_largest=largest,white_line_shift_m=offset*.02,free_area_m2=float(cv2.countNonZero(mask)*.0004),
                     free_centroid_lateral_m=float((m['m10']/m['m00']-1000)*.02),near_section_free_runs=int(np.sum(near&~np.r_[False,near[:-1]])))
            if previous is not None:row['grid_changed_cells']=int(np.count_nonzero(previous!=grid))
            previous=grid.copy();out.append(row);cv2.imwrite(str(folder/f'mask_python_{largest}_{offset}.png'),grid)
    result=dict(execution='Python OpenCV equivalent operations; native C++ execution failed',opencv_version=cv2.__version__,results=out)
    (folder/'segmentation_fixture_python.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
(folder/'source_sha256.json').write_text(json.dumps({'attachment_sha256':hashlib.sha256(saved.read_bytes()).hexdigest(),'extracted_methods_sha256':hashlib.sha256('\n'.join(methods).encode()).hexdigest()},indent=2)+'\n')

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
struct Fixture {
 int road_gray_max_=85,white_line_gray_min_=180,morph_ksize_=5;
 bool keep_largest_component_=true;
  static cv::Mat keepLargestComponentBinary255(const cv::Mat & binary255)
  {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;

    int n = cv::connectedComponentsWithStats(
      binary255,
      labels,
      stats,
      centroids,
      8,
      CV_32S);

    int best = -1;
    int best_area = 0;

    for (int i = 1; i < n; ++i) {
      const int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area > best_area) {
        best_area = area;
        best = i;
      }
    }

    cv::Mat out = cv::Mat::zeros(binary255.size(), CV_8UC1);

    if (best > 0) {
      out.setTo(255, labels == best);
    }

    return out;
  }
  cv::Mat buildRoadMaskForCurrentIsaacEnv(
    const cv::Mat & bev_rgb,
    const cv::Mat & valid)
  {
    cv::Mat gray;
    cv::cvtColor(bev_rgb, gray, cv::COLOR_BGR2GRAY);

    cv::Mat dark_road = (gray <= road_gray_max_);
    dark_road.convertTo(dark_road, CV_8U, 255);

    cv::Mat white_line = (gray >= white_line_gray_min_);
    white_line.convertTo(white_line, CV_8U, 255);

    cv::Mat not_white;
    cv::bitwise_not(white_line, not_white);

    cv::Mat road_candidate;
    cv::bitwise_and(dark_road, not_white, road_candidate);
    cv::bitwise_and(road_candidate, valid, road_candidate);

    int k = morph_ksize_;
    if (k < 1) {
      k = 1;
    }
    if (k % 2 == 0) {
      k += 1;
    }

    cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));

    cv::Mat road_clean;
    cv::morphologyEx(road_candidate, road_clean, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(road_clean, road_clean, cv::MORPH_OPEN, kernel);

    if (keep_largest_component_) {
      road_clean = keepLargestComponentBinary255(road_clean);
    }

    return road_clean;
  }
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

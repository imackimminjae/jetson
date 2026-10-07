"""Create a portable diagnostic-only revision of the attached publisher.
Image segmentation, occupancy data, TF selection and existing parameters are unchanged.
"""
from pathlib import Path
import difflib,hashlib,json
here=Path(__file__).resolve().parent;folder=here/'bev_source'
src=folder/'rgb_to_occupancy.original.cpp';s=src.read_text()
def one(a,b):
    global s
    assert s.count(a)==1,a
    s=s.replace(a,b)
one('#include <nav_msgs/msg/occupancy_grid.hpp>',
    '#include <nav_msgs/msg/occupancy_grid.hpp>\n#include <std_msgs/msg/float64_multi_array.hpp>')
one('    declare_parameter<bool>("publish_bev_images", false);',
    '''    declare_parameter<bool>("publish_bev_images", false);
    declare_parameter<bool>("publish_bev_source_diagnostics", true);''')
one('    // ---- read params ----', '''    if (get_parameter("publish_bev_source_diagnostics").as_bool()) {
      source_diagnostics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/debug/bev_source_timing", rclcpp::QoS(10).best_effort());
    }

    // ---- read params ----''')
one('  static cv::Mat keepLargestComponentBinary255(const cv::Mat & binary255)',
    '  cv::Mat keepLargestComponentBinary255(const cv::Mat & binary255)')
one('    int best = -1;\n    int best_area = 0;', '''    int best = -1;
    int best_area = 0;
    source_components_ = n - 1;
    source_total_area_ = 0;
    source_second_area_ = 0;''')
one('      if (area > best_area) {\n        best_area = area;\n        best = i;\n      }',
    '''      source_total_area_ += area;
      if (area > best_area) {
        source_second_area_ = best_area;
        best_area = area;
        best = i;
      } else if (area > source_second_area_) {
        source_second_area_ = area;
      }''')
one('    cv::Mat out = cv::Mat::zeros(binary255.size(), CV_8UC1);',
    '''    source_selected_area_ = best_area;
    source_selected_label_ = best;
    cv::Mat out = cv::Mat::zeros(binary255.size(), CV_8UC1);''')
one('    const cv::Matx44d T_cam_base = transformToMatrix(tf);',
    '''    source_tf_stamp_sec_ = rclcpp::Time(tf.header.stamp).seconds();
    const cv::Matx44d T_cam_base = transformToMatrix(tf);''')
one('    ++image_rx_count_;', '''    const auto source_started = std::chrono::steady_clock::now();
    const double source_callback_ros_sec = now().seconds();
    ++image_rx_count_;''')
one('    cv::Mat road_clean = buildRoadMaskForCurrentIsaacEnv(bev_rgb, valid);',
    '''    source_components_ = source_total_area_ = source_selected_area_ =
      source_second_area_ = source_selected_label_ = -1;
    cv::Mat road_clean = buildRoadMaskForCurrentIsaacEnv(bev_rgb, valid);''')
one('    if (publish_resized_) {\n      cv::Mat bev_rgb_resized;',
    '''    if (source_diagnostics_pub_) {
      std_msgs::msg::Float64MultiArray diagnostic;
      diagnostic.layout.dim.resize(1);
      diagnostic.layout.dim[0].label =
        "image_seq,image_stamp_sec,callback_ros_sec,after_grid_publish_ros_sec,"
        "work_ms_to_grid,tf_stamp_sec,components,total_area_px,selected_area_px,"
        "second_area_px,selected_label,keep_largest,road_gray_max,white_line_gray_min,"
        "morph_ksize,zero_grid_stamp,use_latest_tf,process_every_n_frames,use_sim_time,"
        "grid_enabled,roi_x_min,roi_x_max,roi_y_min,roi_y_max,bev_resolution,ground_z,"
        "occupancy_width,occupancy_height,rotate_ccw90,flip_lr,invalid_as_unknown";
      bool source_use_sim_time = false;
      get_parameter_or("use_sim_time", source_use_sim_time, false);
      diagnostic.data = {
        static_cast<double>(image_rx_count_), rclcpp::Time(msg->header.stamp).seconds(),
        source_callback_ros_sec, now().seconds(),
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - source_started).count(),
        source_tf_stamp_sec_, static_cast<double>(source_components_),
        static_cast<double>(source_total_area_), static_cast<double>(source_selected_area_),
        static_cast<double>(source_second_area_), static_cast<double>(source_selected_label_),
        keep_largest_component_ ? 1.0 : 0.0, static_cast<double>(road_gray_max_),
        static_cast<double>(white_line_gray_min_), static_cast<double>(morph_ksize_),
        occupancy_grid_zero_stamp_ ? 1.0 : 0.0, use_latest_tf_ ? 1.0 : 0.0,
        static_cast<double>(process_every_n_frames_), source_use_sim_time ? 1.0 : 0.0,
        publish_occupancy_grid_ ? 1.0 : 0.0, x_min_, x_max_, y_min_, y_max_, res_, ground_z_,
        static_cast<double>(occupancy_grid_width_), static_cast<double>(occupancy_grid_height_),
        occupancy_rotate_ccw_90_ ? 1.0 : 0.0, occupancy_flip_lr_ ? 1.0 : 0.0,
        occupancy_invalid_as_unknown_ ? 1.0 : 0.0};
      diagnostic.layout.dim[0].size = diagnostic.data.size();
      diagnostic.layout.dim[0].stride = diagnostic.data.size();
      source_diagnostics_pub_->publish(diagnostic);
    }

    if (publish_resized_) {
      cv::Mat bev_rgb_resized;''')
one('  tf2_ros::Buffer tf_buffer_;', '''  // Diagnostic state only; never used by image or occupancy decisions.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr source_diagnostics_pub_;
  double source_tf_stamp_sec_{0.0};
  int source_components_{-1};
  int source_total_area_{-1};
  int source_selected_area_{-1};
  int source_second_area_{-1};
  int source_selected_label_{-1};

  tf2_ros::Buffer tf_buffer_;''')
patched=folder/'rgb_to_occupancy.diagnostics.cpp';patched.write_text(s)
(folder/'diagnostics.patch').write_text(''.join(difflib.unified_diff(src.read_text().splitlines(True),s.splitlines(True),fromfile='rgb_to_occupancy.cpp',tofile='rgb_to_occupancy.cpp')))
(folder/'diagnostics_sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [src,patched,folder/'diagnostics.patch']},indent=2)+'\n')
print('Prepared diagnostic-only publisher copy; no external or operational file changed')

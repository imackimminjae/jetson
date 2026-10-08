#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <memory>
#include <cstdint>

using namespace std::chrono_literals;

class RgbToOccupancyNode : public rclcpp::Node
{
public:
  RgbToOccupancyNode()
  : Node("rgb_to_occupancy"),
    tf_buffer_(this->get_clock())
  {
    // ---------------------------
    // Topics
    // ---------------------------
    declare_parameter<std::string>("rgb_topic", "/rgb");
    declare_parameter<std::string>("camera_info_topic", "/camera_info");

    declare_parameter<std::string>("bev_rgb_topic", "/bev/rgb");
    declare_parameter<std::string>("bev_valid_topic", "/bev/valid");
    declare_parameter<std::string>("bev_road_topic", "/bev/road_mask");

    declare_parameter<bool>("publish_bev_images", false);

    // OccupancyGrid publish
    declare_parameter<bool>("publish_occupancy_grid", true);
    declare_parameter<std::string>("bev_occupancy_grid_topic", "/bev/occupancy_grid");
    declare_parameter<bool>("occupancy_invalid_as_unknown", false);

    // OccupancyGrid rotation / flip
    declare_parameter<bool>("occupancy_rotate_ccw_90", true);
    declare_parameter<bool>("occupancy_flip_lr", false);

    // RViz Map display friendly settings
    declare_parameter<bool>("occupancy_grid_reliable", true);
    declare_parameter<bool>("occupancy_grid_transient_local", true);
    declare_parameter<bool>("occupancy_grid_zero_stamp", true);
    declare_parameter<bool>("occupancy_grid_rviz_standard_values", true);

    // OccupancyGrid output size
    declare_parameter<int>("occupancy_grid_width", 64);
    declare_parameter<int>("occupancy_grid_height", 64);

    // Resized BEV debug image publish(optional)
    declare_parameter<bool>("publish_resized", false);
    declare_parameter<int>("resized_width", 128);
    declare_parameter<int>("resized_height", 128);
    declare_parameter<std::string>("bev_rgb_resized_topic", "/bev/rgb_resized");
    declare_parameter<std::string>("bev_valid_resized_topic", "/bev/valid_resized");
    declare_parameter<std::string>("bev_road_resized_topic", "/bev/road_mask_resized");

    // ---------------------------
    // Frames
    // ---------------------------
    declare_parameter<std::string>("base_frame", "Vehicle");
    declare_parameter<std::string>("camera_frame", "Camera_OmniVision_OV9782_Color");

    // ---------------------------
    // BEV ROI in base frame
    // x: forward, y: lateral
    // This physical region is directly converted to OccupancyGrid.
    // No additional crop is applied.
    // ---------------------------
    declare_parameter<double>("x_min", 0.0);
    declare_parameter<double>("x_max", 22.5);
    declare_parameter<double>("y_min", -11.25);
    declare_parameter<double>("y_max", 11.25);
    declare_parameter<double>("resolution", 0.02);

    // Road plane height in Vehicle frame
    declare_parameter<double>("ground_z", -0.098);

    // ---------------------------
    // QoS / TF
    // ---------------------------
    declare_parameter<bool>("pub_reliable", true);
    declare_parameter<bool>("sub_reliable", false);
    declare_parameter<bool>("use_latest_tf", true);
    declare_parameter<int>("tf_timeout_ms", 50);
    declare_parameter<bool>("enable_tf_listener", true);

    // ---------------------------
    // Output values for internal road_mask image
    // 1 = drivable
    // 0 = non-drivable
    // ---------------------------
    declare_parameter<int>("road_value", 1);
    declare_parameter<int>("nonroad_value", 0);
    declare_parameter<bool>("invalid_as_nonroad", true);

    // ---------------------------
    // Road segmentation
    // ---------------------------
    declare_parameter<int>("road_gray_max", 85);
    declare_parameter<int>("white_line_gray_min", 180);
    declare_parameter<int>("morph_ksize", 5);
    declare_parameter<bool>("keep_largest_component", true);

    // ---------------------------
    // Preview & Save
    // ---------------------------
    declare_parameter<bool>("enable_preview", false);
    declare_parameter<bool>("input_debug_log", false);
    declare_parameter<int>("process_every_n_frames", 1);
    declare_parameter<std::string>("save_dir", "/tmp/bev_captures");
    declare_parameter<std::string>("save_key", "s");

    // ---- read params ----
    rgb_topic_ = get_parameter("rgb_topic").as_string();
    info_topic_ = get_parameter("camera_info_topic").as_string();

    bev_rgb_topic_ = get_parameter("bev_rgb_topic").as_string();
    bev_valid_topic_ = get_parameter("bev_valid_topic").as_string();
    bev_road_topic_ = get_parameter("bev_road_topic").as_string();
    publish_bev_images_ = get_parameter("publish_bev_images").as_bool();

    publish_occupancy_grid_ = get_parameter("publish_occupancy_grid").as_bool();
    bev_occupancy_grid_topic_ = get_parameter("bev_occupancy_grid_topic").as_string();
    occupancy_invalid_as_unknown_ = get_parameter("occupancy_invalid_as_unknown").as_bool();
    occupancy_rotate_ccw_90_ = get_parameter("occupancy_rotate_ccw_90").as_bool();
    occupancy_flip_lr_ = get_parameter("occupancy_flip_lr").as_bool();

    occupancy_grid_reliable_ = get_parameter("occupancy_grid_reliable").as_bool();
    occupancy_grid_transient_local_ = get_parameter("occupancy_grid_transient_local").as_bool();
    occupancy_grid_zero_stamp_ = get_parameter("occupancy_grid_zero_stamp").as_bool();
    occupancy_grid_rviz_standard_values_ =
      get_parameter("occupancy_grid_rviz_standard_values").as_bool();

    occupancy_grid_width_ = get_parameter("occupancy_grid_width").as_int();
    occupancy_grid_height_ = get_parameter("occupancy_grid_height").as_int();

    if (occupancy_grid_width_ <= 0 || occupancy_grid_height_ <= 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid occupancy grid size: %dx%d",
        occupancy_grid_width_,
        occupancy_grid_height_);
      throw std::runtime_error("Invalid occupancy grid size");
    }

    publish_resized_ = get_parameter("publish_resized").as_bool();
    resized_width_ = get_parameter("resized_width").as_int();
    resized_height_ = get_parameter("resized_height").as_int();
    bev_rgb_resized_topic_ = get_parameter("bev_rgb_resized_topic").as_string();
    bev_valid_resized_topic_ = get_parameter("bev_valid_resized_topic").as_string();
    bev_road_resized_topic_ = get_parameter("bev_road_resized_topic").as_string();

    if (resized_width_ <= 0 || resized_height_ <= 0) {
      RCLCPP_FATAL(get_logger(), "Invalid resized size: %dx%d", resized_width_, resized_height_);
      throw std::runtime_error("Invalid resized size");
    }

    base_frame_ = get_parameter("base_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();

    x_min_ = get_parameter("x_min").as_double();
    x_max_ = get_parameter("x_max").as_double();
    y_min_ = get_parameter("y_min").as_double();
    y_max_ = get_parameter("y_max").as_double();
    res_ = get_parameter("resolution").as_double();
    ground_z_ = get_parameter("ground_z").as_double();

    pub_reliable_ = get_parameter("pub_reliable").as_bool();
    sub_reliable_ = get_parameter("sub_reliable").as_bool();
    use_latest_tf_ = get_parameter("use_latest_tf").as_bool();
    tf_timeout_ms_ = get_parameter("tf_timeout_ms").as_int();
    enable_tf_listener_ = get_parameter("enable_tf_listener").as_bool();

    road_value_ = clampU8(get_parameter("road_value").as_int());
    nonroad_value_ = clampU8(get_parameter("nonroad_value").as_int());
    invalid_as_nonroad_ = get_parameter("invalid_as_nonroad").as_bool();

    road_gray_max_ = get_parameter("road_gray_max").as_int();
    white_line_gray_min_ = get_parameter("white_line_gray_min").as_int();

    morph_ksize_ = get_parameter("morph_ksize").as_int();
    keep_largest_component_ = get_parameter("keep_largest_component").as_bool();

    enable_preview_ = get_parameter("enable_preview").as_bool();
    input_debug_log_ = get_parameter("input_debug_log").as_bool();
    process_every_n_frames_ =
      static_cast<int>(get_parameter("process_every_n_frames").as_int());
    process_every_n_frames_ = std::max(1, process_every_n_frames_);
    save_dir_ = get_parameter("save_dir").as_string();
    save_key_str_ = get_parameter("save_key").as_string();
    save_key_ = save_key_str_.empty() ? 's' : save_key_str_[0];

    bev_w_ = static_cast<int>(std::round((y_max_ - y_min_) / res_));
    bev_h_ = static_cast<int>(std::round((x_max_ - x_min_) / res_));

    if (bev_w_ <= 0 || bev_h_ <= 0) {
      RCLCPP_FATAL(
        get_logger(),
        "Invalid BEV size: %dx%d. Check x/y range and resolution.",
        bev_w_,
        bev_h_);
      throw std::runtime_error("Invalid BEV size");
    }

    rclcpp::QoS sub_qos(rclcpp::KeepLast(10));
    if (sub_reliable_) {
      sub_qos.reliable();
    } else {
      sub_qos.best_effort();
    }

    rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
    if (pub_reliable_) {
      pub_qos.reliable();
    } else {
      pub_qos.best_effort();
    }

    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_,
      sub_qos,
      std::bind(&RgbToOccupancyNode::onCameraInfo, this, std::placeholders::_1));

    sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_,
      sub_qos,
      std::bind(&RgbToOccupancyNode::onImage, this, std::placeholders::_1));

    if (enable_tf_listener_) {
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    }

    if (publish_bev_images_) {
      pub_bev_rgb_ = create_publisher<sensor_msgs::msg::Image>(bev_rgb_topic_, pub_qos);
      pub_valid_ = create_publisher<sensor_msgs::msg::Image>(bev_valid_topic_, pub_qos);
      pub_road_ = create_publisher<sensor_msgs::msg::Image>(bev_road_topic_, pub_qos);
    }

    if (publish_occupancy_grid_) {
      rclcpp::QoS map_qos(rclcpp::KeepLast(1));

      if (occupancy_grid_reliable_) {
        map_qos.reliable();
      } else {
        map_qos.best_effort();
      }

      if (occupancy_grid_transient_local_) {
        map_qos.transient_local();
      } else {
        map_qos.durability_volatile();
      }

      pub_occupancy_grid_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(
          bev_occupancy_grid_topic_,
          map_qos);
    }

    if (publish_resized_) {
      pub_bev_rgb_resized_ =
        create_publisher<sensor_msgs::msg::Image>(bev_rgb_resized_topic_, pub_qos);
      pub_valid_resized_ =
        create_publisher<sensor_msgs::msg::Image>(bev_valid_resized_topic_, pub_qos);
      pub_road_resized_ =
        create_publisher<sensor_msgs::msg::Image>(bev_road_resized_topic_, pub_qos);
    }

    if (enable_preview_) {
      cv::namedWindow("input_rgb", cv::WINDOW_NORMAL);
      cv::namedWindow("bev_rgb", cv::WINDOW_NORMAL);
      cv::namedWindow("road_mask", cv::WINDOW_NORMAL);
      cv::namedWindow("valid", cv::WINDOW_NORMAL);

      std::error_code ec;
      std::filesystem::create_directories(save_dir_, ec);
    }

    RCLCPP_INFO(
      get_logger(),
      "Node name: rgb_to_occupancy\n"
      "Subscribed: %s, %s | publish_bev_images=%s topics: %s, %s, %s | resized_publish=%s\n"
      "Publish OccupancyGrid=%s topic=%s size=%dx%d invalid_unknown=%s\n"
      "OccupancyGrid rotation/flip: ccw_90=%s flip_lr=%s\n"
      "OccupancyGrid RViz settings: reliable=%s transient_local=%s zero_stamp=%s rviz_standard_values=%s\n"
      "Frames: base=%s, camera=%s | BEV image: %dx%d (res=%.3f)\n"
      "ROI directly used for OccupancyGrid: x[%.2f, %.2f], y[%.2f, %.2f], ground_z=%.3f\n"
      "Crop: disabled. Full RGB image is warped to BEV. Full BEV is resized to OccupancyGrid.\n"
      "Internal road_mask values: %d=drivable, %d=non-drivable\n"
      "RViz OccupancyGrid standard values if enabled: 0=free/drivable, 100=occupied/non-drivable, -1=unknown\n"
      "QoS: sub=%s debug_image_pub=%s occupancy_pub=%s | TF listener=%s lookup=%s (timeout=%dms) | process_every_n_frames=%d\n"
      "Road threshold: gray<=%d, white_line_gray>=%d | morph=%d, largest=%s\n"
      "Preview=%s save_dir=%s save_key=%c",
      rgb_topic_.c_str(),
      info_topic_.c_str(),
      publish_bev_images_ ? "true" : "false",
      bev_rgb_topic_.c_str(),
      bev_valid_topic_.c_str(),
      bev_road_topic_.c_str(),
      publish_resized_ ? "true" : "false",
      publish_occupancy_grid_ ? "true" : "false",
      bev_occupancy_grid_topic_.c_str(),
      occupancy_grid_width_,
      occupancy_grid_height_,
      occupancy_invalid_as_unknown_ ? "true" : "false",
      occupancy_rotate_ccw_90_ ? "true" : "false",
      occupancy_flip_lr_ ? "true" : "false",
      occupancy_grid_reliable_ ? "true" : "false",
      occupancy_grid_transient_local_ ? "true" : "false",
      occupancy_grid_zero_stamp_ ? "true" : "false",
      occupancy_grid_rviz_standard_values_ ? "true" : "false",
      base_frame_.c_str(),
      camera_frame_.c_str(),
      bev_w_,
      bev_h_,
      res_,
      x_min_,
      x_max_,
      y_min_,
      y_max_,
      ground_z_,
      static_cast<int>(road_value_),
      static_cast<int>(nonroad_value_),
      sub_reliable_ ? "reliable" : "best_effort",
      pub_reliable_ ? "reliable" : "best_effort",
      occupancy_grid_reliable_ ? "reliable" : "best_effort",
      enable_tf_listener_ ? "true" : "false",
      use_latest_tf_ ? "latest(TimePointZero)" : "stamp(image)",
      tf_timeout_ms_,
      process_every_n_frames_,
      road_gray_max_,
      white_line_gray_min_,
      morph_ksize_,
      keep_largest_component_ ? "true" : "false",
      enable_preview_ ? "true" : "false",
      save_dir_.c_str(),
      save_key_);
  }

private:
  static uint8_t clampU8(int v)
  {
    if (v < 0) {
      return 0;
    }
    if (v > 255) {
      return 255;
    }
    return static_cast<uint8_t>(v);
  }

  static cv::Mat occupancyToVisWhiteDrivable(const cv::Mat & occupancy)
  {
    cv::Mat vis(occupancy.size(), CV_8UC1, cv::Scalar(0));
    vis.setTo(255, occupancy == 1);
    return vis;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    K_ = cv::Matx33d(
      msg->k[0], msg->k[1], msg->k[2],
      msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);

    cam_w_ = static_cast<int>(msg->width);
    cam_h_ = static_cast<int>(msg->height);
    have_K_ = true;
  }

  static cv::Matx44d transformToMatrix(const geometry_msgs::msg::TransformStamped & tf)
  {
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;

    const double x = q.x;
    const double y = q.y;
    const double z = q.z;
    const double w = q.w;

    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - z * w);
    const double r02 = 2.0 * (x * z + y * w);

    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - x * w);

    const double r20 = 2.0 * (x * z - y * w);
    const double r21 = 2.0 * (y * z + x * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    return cv::Matx44d(
      r00, r01, r02, t.x,
      r10, r11, r12, t.y,
      r20, r21, r22, t.z,
      0.0, 0.0, 0.0, 1.0);
  }

  cv::Point2f projectPoint(const cv::Vec3d & p_cam) const
  {
    const double X = p_cam[0];
    const double Y = p_cam[1];
    const double Z = p_cam[2];

    const double fx = K_(0, 0);
    const double fy = K_(1, 1);
    const double cx = K_(0, 2);
    const double cy = K_(1, 2);

    const float u = static_cast<float>(fx * (X / Z) + cx);
    const float v = static_cast<float>(fy * (Y / Z) + cy);

    return cv::Point2f(u, v);
  }

  bool computeHomography(
    const rclcpp::Time & stamp,
    cv::Mat & H_out)
  {
    if (!enable_tf_listener_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TF listener is disabled. Enable enable_tf_listener:=true for BEV/OccupancyGrid generation.");
      return false;
    }

    geometry_msgs::msg::TransformStamped tf;

    try {
      if (use_latest_tf_) {
        tf = tf_buffer_.lookupTransform(
          camera_frame_,
          base_frame_,
          tf2::TimePointZero,
          std::chrono::milliseconds(tf_timeout_ms_));
      } else {
        tf = tf_buffer_.lookupTransform(
          camera_frame_,
          base_frame_,
          stamp,
          std::chrono::milliseconds(tf_timeout_ms_));
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TF lookup failed (target=%s source=%s): %s",
        camera_frame_.c_str(),
        base_frame_.c_str(),
        e.what());
      return false;
    }

    const cv::Matx44d T_cam_base = transformToMatrix(tf);

    const std::array<cv::Vec4d, 4> corners_base_h = {
      cv::Vec4d(x_min_, y_min_, ground_z_, 1.0),
      cv::Vec4d(x_min_, y_max_, ground_z_, 1.0),
      cv::Vec4d(x_max_, y_max_, ground_z_, 1.0),
      cv::Vec4d(x_max_, y_min_, ground_z_, 1.0)
    };

    std::vector<cv::Point2f> src_pts;
    src_pts.reserve(4);

    const int image_w = cam_w_ > 0 ? cam_w_ : 1280;
    const int image_h = cam_h_ > 0 ? cam_h_ : 720;

    for (const auto & pb : corners_base_h) {
      cv::Vec4d pc4 = T_cam_base * pb;
      cv::Vec3d pc(pc4[0], pc4[1], pc4[2]);

      if (pc[2] <= 1e-6) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Projected BEV corner is behind camera. pc=(%.3f, %.3f, %.3f). "
          "Check camera optical frame or ground_z.",
          pc[0],
          pc[1],
          pc[2]);
        return false;
      }

      cv::Point2f p_img = projectPoint(pc);

      if (
        p_img.x < -0.5f * image_w || p_img.x > 1.5f * image_w ||
        p_img.y < -0.5f * image_h || p_img.y > 1.5f * image_h)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Projected BEV corner is far outside full image. "
          "p_img=(%.1f, %.1f), image=%dx%d. Check BEV ROI, camera FOV, TF, or ground_z.",
          p_img.x,
          p_img.y,
          image_w,
          image_h);
      }

      src_pts.push_back(p_img);
    }

    for (const auto & p : src_pts) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        return false;
      }
    }

    const std::vector<cv::Point2f> dst_pts = {
      cv::Point2f(0.0f,                            static_cast<float>(bev_h_ - 1)),
      cv::Point2f(static_cast<float>(bev_w_ - 1), static_cast<float>(bev_h_ - 1)),
      cv::Point2f(static_cast<float>(bev_w_ - 1), 0.0f),
      cv::Point2f(0.0f,                            0.0f)
    };

    H_out = cv::getPerspectiveTransform(src_pts, dst_pts);
    return true;
  }

  void publishMono8(
    const rclcpp::Time & stamp,
    const cv::Mat & mono8,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub)
  {
    if (!pub) {
      return;
    }

    cv_bridge::CvImage out;
    out.header.stamp = stamp;
    out.header.frame_id = base_frame_;
    out.encoding = "mono8";
    out.image = mono8;
    pub->publish(*out.toImageMsg());
  }

  void publishBgr8(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const cv::Mat & bgr,
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub)
  {
    if (!pub) {
      return;
    }

    cv_bridge::CvImage out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    out.encoding = "bgr8";
    out.image = bgr;
    pub->publish(*out.toImageMsg());
  }

  void publishOccupancyGrid(
    const rclcpp::Time & stamp,
    const cv::Mat & road_mask,
    const cv::Mat & valid)
  {
    if (!pub_occupancy_grid_) {
      return;
    }

    cv::Mat road_resized;
    cv::Mat valid_resized;

    cv::resize(
      road_mask,
      road_resized,
      cv::Size(occupancy_grid_height_, occupancy_grid_width_),
      0.0,
      0.0,
      cv::INTER_NEAREST);

    cv::resize(
      valid,
      valid_resized,
      cv::Size(occupancy_grid_height_, occupancy_grid_width_),
      0.0,
      0.0,
      cv::INTER_NEAREST);

    nav_msgs::msg::OccupancyGrid grid;

    if (occupancy_grid_zero_stamp_) {
      grid.header.stamp.sec = 0;
      grid.header.stamp.nanosec = 0;
    } else {
      grid.header.stamp = stamp;
    }

    grid.header.frame_id = base_frame_;

    const double x_range = x_max_ - x_min_;
    const double y_range = y_max_ - y_min_;

    const double res_x = x_range / static_cast<double>(occupancy_grid_width_);
    const double res_y = y_range / static_cast<double>(occupancy_grid_height_);

    grid.info.map_load_time.sec = 0;
    grid.info.map_load_time.nanosec = 0;
    grid.info.resolution = static_cast<float>(std::max(res_x, res_y));

    const int raw_w = occupancy_grid_width_;
    const int raw_h = occupancy_grid_height_;

    std::vector<int8_t> raw_data(
      static_cast<size_t>(raw_w) * static_cast<size_t>(raw_h),
      100);

    for (int my = 0; my < raw_h; ++my) {
      for (int mx = 0; mx < raw_w; ++mx) {
        const int img_r = raw_w - 1 - mx;
        const int img_c = my;

        const bool is_valid = valid_resized.at<uint8_t>(img_r, img_c) != 0;
        const uint8_t road_value_at_cell = road_resized.at<uint8_t>(img_r, img_c);
        const bool is_road = (road_value_at_cell == road_value_);

        int8_t out_value = 0;

        if (!is_valid && occupancy_invalid_as_unknown_) {
          out_value = -1;
        } else {
          if (occupancy_grid_rviz_standard_values_) {
            out_value = is_road ? 0 : 100;
          } else {
            out_value = is_road ? 1 : 0;
          }
        }

        const size_t raw_idx =
          static_cast<size_t>(mx) +
          static_cast<size_t>(my) * static_cast<size_t>(raw_w);

        raw_data[raw_idx] = out_value;
      }
    }

    if (occupancy_rotate_ccw_90_) {
      const int rot_w = raw_h;
      const int rot_h = raw_w;

      grid.info.width = static_cast<uint32_t>(rot_w);
      grid.info.height = static_cast<uint32_t>(rot_h);

      grid.data.assign(
        static_cast<size_t>(rot_w) * static_cast<size_t>(rot_h),
        100);

      for (int y = 0; y < raw_h; ++y) {
        for (int x = 0; x < raw_w; ++x) {
          const int new_x = y;
          const int new_y = raw_w - 1 - x;

          const size_t src_idx =
            static_cast<size_t>(x) +
            static_cast<size_t>(y) * static_cast<size_t>(raw_w);

          const size_t dst_idx =
            static_cast<size_t>(new_x) +
            static_cast<size_t>(new_y) * static_cast<size_t>(rot_w);

          grid.data[dst_idx] = raw_data[src_idx];
        }
      }
    } else {
      grid.info.width = static_cast<uint32_t>(raw_w);
      grid.info.height = static_cast<uint32_t>(raw_h);
      grid.data = raw_data;
    }

    if (occupancy_flip_lr_) {
      const int w = static_cast<int>(grid.info.width);
      const int h = static_cast<int>(grid.info.height);

      std::vector<int8_t> flipped(
        static_cast<size_t>(w) * static_cast<size_t>(h),
        100);

      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const int new_x = w - 1 - x;
          const int new_y = y;

          const size_t src_idx =
            static_cast<size_t>(x) +
            static_cast<size_t>(y) * static_cast<size_t>(w);

          const size_t dst_idx =
            static_cast<size_t>(new_x) +
            static_cast<size_t>(new_y) * static_cast<size_t>(w);

          flipped[dst_idx] = grid.data[src_idx];
        }
      }

      grid.data = std::move(flipped);
    }

    grid.info.origin.position.x = x_min_;
    grid.info.origin.position.y = y_min_;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    const size_t expected_size =
      static_cast<size_t>(grid.info.width) * static_cast<size_t>(grid.info.height);

    if (grid.data.size() != expected_size) {
      RCLCPP_ERROR(
        get_logger(),
        "OccupancyGrid data size mismatch. width=%u height=%u data.size=%zu expected=%zu. Drop this grid.",
        grid.info.width,
        grid.info.height,
        grid.data.size(),
        expected_size);
      return;
    }

    pub_occupancy_grid_->publish(grid);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Published occupancy grid %ux%u data.size=%zu expected=%zu x[%.2f, %.2f] y[%.2f, %.2f] res=%.4f",
      grid.info.width,
      grid.info.height,
      grid.data.size(),
      expected_size,
      x_min_,
      x_max_,
      y_min_,
      y_max_,
      grid.info.resolution);
  }

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

  void saveBEV(
    const cv::Mat & rgb,
    const cv::Mat & valid,
    const cv::Mat & road,
    const rclcpp::Time & stamp)
  {
    std::error_code ec;
    std::filesystem::create_directories(save_dir_, ec);

    const auto ns = stamp.nanoseconds();
    const std::string base = save_dir_ + "/bev_" + std::to_string(ns);

    cv::imwrite(base + "_rgb.png", rgb);
    cv::imwrite(base + "_valid.png", valid);
    cv::imwrite(base + "_road_raw.png", road);

    cv::Mat road_vis = occupancyToVisWhiteDrivable(road);
    cv::imwrite(base + "_road_vis.png", road_vis);

    RCLCPP_INFO(
      get_logger(),
      "Saved: %s_[rgb/valid/road_raw/road_vis].png",
      base.c_str());
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

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++image_rx_count_;

    if (process_every_n_frames_ > 1 &&
      (image_rx_count_ % static_cast<uint64_t>(process_every_n_frames_)) != 0)
    {
      return;
    }

    if (input_debug_log_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Input image callback OK: count=%lu encoding=%s size=%ux%u step=%u data=%zu",
        static_cast<unsigned long>(image_rx_count_),
        msg->encoding.c_str(),
        msg->width,
        msg->height,
        msg->step,
        msg->data.size());
    }

    if (!have_K_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for CameraInfo: %s",
        info_topic_.c_str());
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat bgr_image;

    try {
      if (msg->encoding == "bgr8") {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        bgr_image = cv_ptr->image;
      } else if (msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
        cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "bgra8") {
        cv_ptr = cv_bridge::toCvShare(msg, "bgra8");
        cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_BGRA2BGR);
      } else if (msg->encoding == "rgba8") {
        cv_ptr = cv_bridge::toCvShare(msg, "rgba8");
        cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGBA2BGR);
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Unsupported image encoding: %s",
          msg->encoding.c_str());
        return;
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge conversion failed: %s",
        e.what());
      return;
    }

    cv::Mat H;
    if (!computeHomography(msg->header.stamp, H)) {
      return;
    }

    cv::Mat bev_rgb;
    cv::warpPerspective(
      bgr_image,
      bev_rgb,
      H,
      cv::Size(bev_w_, bev_h_),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT,
      cv::Scalar(0, 0, 0));

    cv::Mat ones(bgr_image.rows, bgr_image.cols, CV_8UC1, cv::Scalar(255));
    cv::Mat valid;

    cv::warpPerspective(
      ones,
      valid,
      H,
      cv::Size(bev_w_, bev_h_),
      cv::INTER_NEAREST,
      cv::BORDER_CONSTANT,
      cv::Scalar(0));

    cv::Mat road_clean = buildRoadMaskForCurrentIsaacEnv(bev_rgb, valid);

    cv::Mat road_mask(bev_h_, bev_w_, CV_8UC1, cv::Scalar(nonroad_value_));
    road_mask.setTo(road_value_, road_clean);

    if (invalid_as_nonroad_) {
      road_mask.setTo(nonroad_value_, valid == 0);
    }

    if (publish_bev_images_) {
      publishBgr8(base_frame_, msg->header.stamp, bev_rgb, pub_bev_rgb_);
      publishMono8(msg->header.stamp, valid, pub_valid_);
      publishMono8(msg->header.stamp, road_mask, pub_road_);
    }

    if (publish_occupancy_grid_) {
      publishOccupancyGrid(msg->header.stamp, road_mask, valid);
    }

    if (publish_resized_) {
      cv::Mat bev_rgb_resized;
      cv::Mat valid_resized;
      cv::Mat road_resized;

      cv::resize(
        bev_rgb,
        bev_rgb_resized,
        cv::Size(resized_width_, resized_height_),
        0.0,
        0.0,
        cv::INTER_LINEAR);

      cv::resize(
        valid,
        valid_resized,
        cv::Size(resized_width_, resized_height_),
        0.0,
        0.0,
        cv::INTER_NEAREST);

      cv::resize(
        road_mask,
        road_resized,
        cv::Size(resized_width_, resized_height_),
        0.0,
        0.0,
        cv::INTER_NEAREST);

      publishBgr8(base_frame_, msg->header.stamp, bev_rgb_resized, pub_bev_rgb_resized_);
      publishMono8(msg->header.stamp, valid_resized, pub_valid_resized_);
      publishMono8(msg->header.stamp, road_resized, pub_road_resized_);
    }

    if (enable_preview_) {
      cv::imshow("input_rgb", bgr_image);
      cv::imshow("bev_rgb", bev_rgb);
      cv::imshow("valid", valid);

      cv::Mat road_vis = occupancyToVisWhiteDrivable(road_mask);
      cv::imshow("road_mask", road_vis);

      int key = cv::waitKey(1) & 0xFF;

      if (key == static_cast<int>(save_key_)) {
        saveBEV(bev_rgb, valid, road_mask, msg->header.stamp);
      }
    }
  }

private:
  std::string rgb_topic_;
  std::string info_topic_;
  std::string bev_rgb_topic_;
  std::string bev_valid_topic_;
  std::string bev_road_topic_;
  std::string base_frame_;
  std::string camera_frame_;

  bool publish_bev_images_{false};

  bool publish_occupancy_grid_{true};
  std::string bev_occupancy_grid_topic_{"/bev/occupancy_grid"};
  bool occupancy_invalid_as_unknown_{false};
  bool occupancy_rotate_ccw_90_{true};
  bool occupancy_flip_lr_{false};

  bool occupancy_grid_reliable_{true};
  bool occupancy_grid_transient_local_{true};
  bool occupancy_grid_zero_stamp_{true};
  bool occupancy_grid_rviz_standard_values_{true};

  int occupancy_grid_width_{64};
  int occupancy_grid_height_{64};

  bool publish_resized_{false};
  int resized_width_{128};
  int resized_height_{128};
  std::string bev_rgb_resized_topic_;
  std::string bev_valid_resized_topic_;
  std::string bev_road_resized_topic_;

  bool pub_reliable_{true};
  bool sub_reliable_{false};
  bool use_latest_tf_{true};
  int tf_timeout_ms_{50};
  bool enable_tf_listener_{true};

  double x_min_{0.0};
  double x_max_{1.5};
  double y_min_{-0.75};
  double y_max_{0.75};
  double res_{0.02};
  double ground_z_{-0.098};

  int bev_w_{0};
  int bev_h_{0};

  uint8_t road_value_{1};
  uint8_t nonroad_value_{0};
  bool invalid_as_nonroad_{true};

  int road_gray_max_{85};
  int white_line_gray_min_{180};

  int morph_ksize_{5};
  bool keep_largest_component_{true};

  bool enable_preview_{false};
  bool input_debug_log_{false};
  int process_every_n_frames_{1};
  uint64_t image_rx_count_{0};
  std::string save_dir_{"/tmp/bev_captures"};
  std::string save_key_str_{"s"};
  char save_key_{'s'};

  cv::Matx33d K_{};
  bool have_K_{false};
  int cam_w_{0};
  int cam_h_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_bev_rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_valid_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_road_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_bev_rgb_resized_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_valid_resized_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_road_resized_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbToOccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
